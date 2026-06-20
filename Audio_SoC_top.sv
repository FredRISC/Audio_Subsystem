`timescale 1ns/1ps

import dsp_pkg::*;

/*
The System Architecture

Imagine this subsystem sitting between 4 High-Speed ADCs and the main SoC DDR Memory Interconnect.

    Clocking & Reset (The Foundation)

        RDC (Reset Domain Crossing): The external asynchronous pad_rst_n arrives and is safely synchronized into two separate reset trees: sys_rst_n and adc_rst_n.

        Clock Divider: A high-speed 500 MHz PLL clock is divided down. We need a Divide-by-5 circuit with a 50% duty cycle to generate the 100 MHz sys_clk for the AXI bus.

    Data Ingestion & CDC (The Analog Boundary)

        4x I2S Receivers: Four independent interfaces capture serialized audio from the ADCs.

        4x Async FIFOs: The deserialized data is pushed into four independent Asynchronous FIFOs to cross safely from the ADC clock domains into the sys_clk domain.

    Arbitration (The Traffic Cop)

        Round-Robin Arbiter: The 4 FIFOs will assert a not_empty request flag. The arbiter uses a Round-Robin scheme to grant access to one channel at a time, preventing any single high-speed ADC from starving the others.

    The AXI4 DMA Engine (The SoC Interface)

        The DMA engine takes the granted audio sample and formats it into an AXI4 Memory-Mapped transaction.

        It manages the AW (Address Write) channel to specify the DDR memory destination, the W (Write Data) channel to burst the audio payload, and the B (Write Response) channel to confirm the data was stored safely.
*/

module Audio_SoC_top #(    
    //parameter DMA_DATA_WIDTH      = 32,
    //parameter IS2_WORD_LENGTH       = 16,
    //parameter CALIBRATED_SAMPLES    = 16,
    //parameter ASYNC_FIFO_DEPTH      = 16
)(
    input  logic pll_clk,     // High-speed main clock (e.g., 500MHz)
    input  logic pad_rst_n,   // Asynchronous external reset
    
    // External I2S Pins from 4 Black Box ADCs
    input  logic i2s_bclk,    // Shared I2S Bit Clock (e.g., 3.072 MHz)
    input  logic [NUM_OF_ADCs-1:0] i2s_ws,
    input  logic [NUM_OF_ADCs-1:0] i2s_sdata,
    
    // Calibration Control (Shared or separate, shown shared here for simplicity)
    input  logic [NUM_OF_ADCs-1:0] start_cal,
    output logic [NUM_OF_ADCs-1:0] cal_done,
    output logic [NUM_OF_ADCs-1:0] adc_mux_gnd,   // Tells external ADCs to short inputs to ground
    
    // AXI4 Master Interface (to SoC Memory)
    input  logic AWREADY,
    output logic AWVALID,
    output logic [3:0]  AWID,    
    output logic [DMA_ADDR_WIDTH-1:0] AWADDR,
    output logic [7:0]  AWLEN,
    output logic [2:0] AWSIZE,
    
    input  logic WREADY,
    output logic WVALID,
    output logic [31:0] WDATA,
    output logic WLAST,
    
    input  logic [3:0] BID,
    input  logic BVALID,
    output logic BREADY,
    input  logic [1:0] BRESP,

    // Error handling
    output logic ERR_ID_mismatch,
    output logic [1:0] ERR_type,
    input  logic ERR_release
);

    // =========================================================================
    // 1. Clocking & Reset Architecture
    // =========================================================================
    logic rst_power_n, rst_bus_n, rst_core_n, rst_adc_n;
    
    // All resets are synced to pll_clk
    Reset_Sequencer u_rst_seq (
        .pll_clk(pll_clk),
        .rst_n(pad_rst_n),
        .rst_power_n(rst_power_n),
        .rst_bus_n(rst_bus_n),
        .rst_core_n(rst_core_n),
        .rst_adc_n(rst_adc_n)
    );

    // Freq Divider
    logic sys_clk;
    Freq_Divider #(
        .N(5)
    ) u_clk_div (
        .pll_clk(pll_clk),
        .rst_pll_n(rst_power_n),
        .sys_clk(sys_clk) // Fast domain for DMA/Bus
    );

    // Sync resets to corresponding subdomains (i2s_bclk, sys_clk)
    logic rst_sys_n, rst_bclk_adc_n;
    Reset_Synchronizer u_rst_sync (
        .clk(i2s_bclk),
        .rst_n_in(rst_adc_n),
        .rst_n_out(rst_bclk_adc_n)
    );
    Reset_Synchronizer u_rst_sync (
        .clk(sys_clk),
        .rst_n_in(rst_core_n),
        .rst_n_out(rst_sys_n)
    );

    // =========================================================================
    // 2. Data Ingestion, Calibration, CDC, DSP, Buffering (Instantiated 4 Times)
    // =========================================================================
    logic [NUM_OF_ADCs-1:0] sync_fifo_read_burst_ready;
    logic [NUM_OF_ADCs-1:0] sync_fifo_read_ready;
    logic [NUM_OF_ADCs-1:0] sync_fifo_read_valid;
    logic [DMA_DATA_WIDTH-1:0] sync_fifo_read_data [NUM_OF_ADCs-1:0];

    genvar i;
    generate
        for (i = 0; i < NUM_OF_ADCs; i++) begin : GEN_AUDIO_CHANNELS
            logic [IS2_WORD_LENGTH-1:0] raw_left_data, raw_right_data;
            logic left_valid, right_valid;
            
        // A. I2S Deserializers
            I2S_Deserializer #(
                .WORD_LENGTH(IS2_WORD_LENGTH)
            ) u_i2s_rx (
                .i2s_bclk(i2s_bclk),
                .rst_n(rst_bclk_adc_n),
                .i2s_ws(i2s_ws[i]),
                .i2s_sdata(i2s_sdata[i]),
                .left_channel_data(raw_left_data),   // 32-bit zero padded left channel data
                .left_valid(left_valid),
                .right_channel_data(raw_right_data), // 32-bit zero padded right channel data
                .right_valid(right_valid)
            );

        // B. Calibration
            // Left Channel DC Offset Calibrators
            logic signed [IS2_WORD_LENGTH-1:0] calibrated_left;
            logic calibrated_left_valid, mux_gnd_left, cal_done_left;
            
            // Right Channel DC Offset Calibrators
            logic signed [IS2_WORD_LENGTH-1:0] calibrated_right;
            logic calibrated_right_valid, mux_gnd_right, cal_done_right;

            ADC_DC_Offset_Calibrator #(
                .NUMBER_OF_SAMPLE(CALIBRATED_SAMPLES)
            ) u_dc_cal_left (
                .clk(i2s_bclk),
                .rst_n(rst_bclk_adc_n),
                .start_cal(start_cal[i]),                           // from ADC tp start calibration
                .adc_data_in(raw_left_data[IS2_WORD_LENGTH-1:0]),   // Grab the lower 16 bits
                .adc_data_valid_in(left_valid),
                .adc_mux_gnd(mux_gnd_left),                         // to ground ADC
                .cal_done(cal_done_left),                           // left calibration is done 
                .calibrated_data_out(calibrated_left),
                .calibrated_data_valid_out(calibrated_left_valid)
            );

            ADC_DC_Offset_Calibrator #(
                .NUMBER_OF_SAMPLE(CALIBRATED_SAMPLES)
            ) u_dc_cal_right (
                .clk(i2s_bclk),
                .rst_n(rst_bclk_adc_n),
                .start_cal(start_cal[i]),                            // from ADC tp start calibration
                .adc_data_in(raw_right_data[IS2_WORD_LENGTH-1:0]),   // Grab the lower 16 bits 
                .adc_data_valid_in(right_valid),
                .adc_mux_gnd(mux_gnd_right),                         // to ground ADC
                .cal_done(cal_done_right),                           // right calibration is done
                .calibrated_data_out(calibrated_right), 
                .calibrated_data_valid_out(calibrated_right_valid)
            );

            // Merge the control flags
            assign adc_mux_gnd[i] = mux_gnd_left | mux_gnd_right; // both left and right channel come from same ADC
            assign cal_done[i] = cal_done_left & cal_done_right;  // to tell ADC that both left and right calibrations are done (debug signal)



            // C. DSP filtering (AFIFO -> Biquad 0 -> Biquad 1 -> SFIFO)
            logic [IS2_WORD_LENGTH:0] async_fifo_write_data;
            logic async_fifo_write_valid;
            assign async_fifo_write_valid = calibrated_left_valid | calibrated_right_valid;
            assign async_fifo_write_data  = calibrated_left_valid ? calibrated_left : calibrated_right;

            DSP_Channel_Processor #(
                .DATA_WIDTH(IS2_WORD_LENGTH),
                .ASYNC_FIFO_DEPTH(ASYNC_FIFO_DEPTH),
                .SYNC_FIFO_DEPTH(SYNC_FIFO_DEPTH),
                .DMA_DATA_WIDTH(DMA_DATA_WIDTH)
            ) u_dsp (
                .sys_clk(sys_clk),
                .rst_sys_n(rst_sys_n),
                .i2s_bclk(i2s_bclk),
                // write interface
                .async_fifo_write_valid_in(async_fifo_write_valid),
                .async_fifo_write_data_in(async_fifo_write_data),
                .async_fifo_write_ready_out(async_fifo_write_ready), 

                // Sync_FIFO DMA interface
                .sync_fifo_read_valid_in(sync_fifo_read_valid[i]),                     
                .sync_fifo_read_ready_out(sync_fifo_read_ready[i]),                    
                .sync_fifo_read_data_out(sync_fifo_read_data[i]),
                .sync_fifo_read_burst_ready_out(sync_fifo_read_burst_ready[i]) 
            );
        end
    endgenerate

    // =========================================================================
    // 3. Arbitration and AXI4 DMA Engine (System Domain)
    // =========================================================================
    AXI4_DMA_Master u_axi4_dma (
        .sys_clk(sys_clk),
        .rst_core_n(rst_sys_n),
        // DSP Processor SYNC FIFO interface
        .read_burst_ready_in(sync_fifo_read_burst_ready),
        .fifo_read_ready_in(sync_fifo_read_ready),
        .fifo_read_valid_out(sync_fifo_read_valid),
        .fifo_read_data_in(sync_fifo_read_data),
        // AXI4 DMA write interface
        .AWREADY(AWREADY),
        .AWVALID(AWVALID),
        .AWID(AWID),
        .AWADDR(AWADDR),
        .AWLEN(AWLEN),
        .AWSIZE(AWSIZE),
        .WREADY(WREADY),
        .WVALID(WVALID),
        .WDATA(WDATA),
        .WLAST(WLAST),
        .BID(BID),
        .BVALID(BVALID),
        .BREADY(BREADY),
        .BRESP(BRESP),
        .ERR_ID_mismatch(ERR_ID_mismatch),
        .ERR_type(ERR_type),
        .ERR_release(ERR_release)
    );


/* AXIS Interface
logic [$clog2(BURST_LENGTH)-1:0] m_axis_burst_counter;
logic m_axis_handshake;
assign m_axis_handshake = m_axis_tready && m_axis_tvalid;
assign axis_stall = m_axis_tvalid && !m_axis_tready;

always_ff @(posedge sys_clk, negedge rst_n) begin : M_AXIS_INTERFACE
    if(~sys_clk_rst_n) begin
        m_axis_burst_counter <= '0;
        m_axis_tvalid <= 1'b0;
        m_axis_tdata_out <= '0;
        m_axis_tlast <= 1'b0;
    end
    else begin
        // Clear TVALID upon successful handshake
        if(m_axis_handshake) begin
            m_axis_tvalid <= 1'b0;
            
            // Advance the burst counter on handshake
            if (m_axis_burst_counter == BURST_LENGTH - 1) begin
                m_axis_burst_counter <= '0;
            end 
            else begin
                m_axis_burst_counter <= m_axis_burst_counter + 1'b1;
            end
        end

        // If new valid data exits the pipeline and we aren't stalled
        if (y_valid_out_2 && !axis_stall) begin
            m_axis_tvalid    <= 1'b1;
            m_axis_tdata_out <= y_out_2;
        end

        m_axis_tlast <= (m_axis_burst_counter == BURST_LENGTH - 1);
    end
end
*/

endmodule