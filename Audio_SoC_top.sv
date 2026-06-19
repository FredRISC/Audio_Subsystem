`timescale 1ns/1ps

import dsp_pkg::*;

module Audio_SoC_top #(    
    //parameter SYSTEM_BUS_WIDTH      = 32,
    //parameter IS2_WORD_LENGTH       = 16,
    //parameter CALIBRATED_SAMPLES    = 16,
    //parameter ASYNC_FIFO_DEPTH      = 16
)(
    input  logic pll_clk,     // High-speed main clock (e.g., 500MHz)
    input  logic pad_rst_n,   // Asynchronous external reset
    
    // External I2S Pins from 4 Black Box ADCs
    input  logic i2s_bclk,    // Shared I2S Bit Clock (e.g., 3.072 MHz)
    input  logic [3:0] i2s_ws,
    input  logic [3:0] i2s_sdata,
    
    // Calibration Control (Shared or separate, shown shared here for simplicity)
    input  logic [3:0] start_cal,
    output logic [3:0] cal_done,
    output logic [3:0] adc_mux_gnd,   // Tells external ADCs to short inputs to ground
    
    // AXI4 Master Interface (to SoC Memory)
    input  logic AWREADY,
    output logic AWVALID,
    output logic [3:0]  AWID,    
    output logic [31:0] AWADDR,
    output logic [7:0]  AWLEN,
    output logic [31:0] AWSIZE,
    
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
    
    reset_sequencer u_rst_seq (
        .pll_clk(pll_clk),
        .rst_n(pad_rst_n),
        .rst_power_n(rst_power_n),
        .rst_bus_n(rst_bus_n),
        .rst_core_n(rst_core_n),
        .rst_adc_n(rst_adc_n)
    );

    logic sys_clk;
    freq_divider_5 u_clk_div (
        .pll_clk(pll_clk),
        .rst_pll_n(rst_power_n),
        .sys_clk(sys_clk) // Fast domain for DMA/Bus
    );

    // Interconnects
    logic [3:0] fifo_write_ready, fifo_read_valid, fifo_read_ready, read_burst_ready;
    logic [31:0] fifo_read_data [3:0];

    // =========================================================================
    // 2. Data Ingestion, Calibration, and CDC (Instantiated 4 Times)
    // =========================================================================
    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : GEN_AUDIO_CHANNELS
            logic [31:0] raw_left_data, raw_right_data;
            logic left_valid, right_valid;
            
            // A. four I2S Deserializers
            I2S_DESERIALIZER #(
                SYSTEM_BUS_WIDTH = SYSTEM_BUS_WIDTH,
                WORD_LENGTH = IS2_WORD_LENGTH
            ) u_i2s_rx (
                .i2s_bclk(i2s_bclk),
                .rst_n(rst_adc_n),
                .i2s_ws(i2s_ws[i]),
                .i2s_sdata(i2s_sdata[i]),
                .left_channel_data(raw_left_data),   // 32-bit zero padded left channel data
                .left_valid(left_valid),
                .right_channel_data(raw_right_data), // 32-bit zero padded right channel data
                .right_valid(right_valid)
            );

            // B. four DC Offset Calibrators (Left Channel)
            logic signed [15:0] calibrated_left;
            logic calibrated_left_valid, mux_gnd_left, cal_done_left;

            ADC_DC_OFFSET_CALIBRATION #(
                NUMBER_OF_SAMPLE = CALIBRATED_SAMPLES
            ) u_dc_cal_left (
                .clk(i2s_bclk),
                .rst_n(rst_adc_n),
                .start_cal(start_cal[i]),                           // from ADC tp start calibration
                .adc_data_in(raw_left_data[NUMBER_OF_SAMPLE-1:0]),  // Grab the lower 16 bits
                .adc_data_valid_in(left_valid),
                .adc_mux_gnd(mux_gnd_left),                         // to ground ADC
                .cal_done(cal_done_left),                           // left calibration is done 
                .calibrated_data_out(calibrated_left),
                .calibrated_data_valid_out(calibrated_left_valid)
            );

            // C. four DC Offset Calibrators (Right Channel)
            logic signed [15:0] calibrated_right;
            logic calibrated_right_valid, mux_gnd_right, cal_done_right;

            ADC_DC_OFFSET_CALIBRATION #(
                NUMBER_OF_SAMPLE = CALIBRATED_SAMPLES // 16
            ) u_dc_cal_right (
                .clk(i2s_bclk),
                .rst_n(rst_adc_n),
                .start_cal(start_cal[i]),                               // from ADC tp start calibration
                .adc_data_in(raw_right_data[NUMBER_OF_SAMPLE-1:0]),     // Grab the lower 16 bits 
                .adc_data_valid_in(right_valid),
                .adc_mux_gnd(mux_gnd_right),                            // to ground ADC
                .cal_done(cal_done_right),                              // right calibration is done
                .calibrated_data_out(calibrated_right), 
                .calibrated_data_valid_out(calibrated_right_valid)
            );

            // Merge the control flags
            assign adc_mux_gnd[i] = mux_gnd_left | mux_gnd_right; // both left and right channel come from same ADC
            assign cal_done[i] = cal_done_left & cal_done_right;  // to tell ADC that both left and right calibrations are done (debug signal)

            // D. Multiplex and Sign Extend to align 16-bit cleaned audio to 32-bit bus
            logic [31:0] fifo_write_data;
            logic fifo_write_valid;
            
            // Because left_valid and right_valid occur on mutually exclusive clock cycles
            // we can safely merge them here. The FIFO will naturally interleave L, R, L, R...
            assign fifo_write_valid = calibrated_left_valid | calibrated_right_valid;
            assign fifo_write_data  = calibrated_left_valid ? {{IS2_WORD_LENGTH{calibrated_left[IS2_WORD_LENGTH-1]}}, calibrated_left} :
                                                           {{IS2_WORD_LENGTH{calibrated_right[IS2_WORD_LENGTH-1]}}, calibrated_right}; // sign-extended

            // E. Clock Domain Crossing & Buffering
            data_count_aware_async_fifo #(
                DATA_WIDTH = SYSTEM_BUS_WIDTH,  // 32-bit data
                FIFO_DEPTH = ASYNC_FIFO_DEPTH
            ) u_fifo (
                .read_clk(sys_clk),
                .write_clk(i2s_bclk),
                .rst_bus_n(rst_bus_n),
                .read_valid_in(fifo_read_valid[i]),
                .read_ready_out(fifo_read_ready[i]),
                .read_data_out(fifo_read_data[i]),
                .write_valid_in(fifo_write_valid), 
                .write_data_in(fifo_write_data),
                .write_ready_out(fifo_write_ready[i]),
                .read_burst_ready_out(read_burst_ready[i]) // Signals Arbiter when 4 beats are ready
            );
        end
    endgenerate

    // =========================================================================
    // 3. Arbitration and AXI4 DMA Engine (System Domain)
    // =========================================================================
    AXI4_DMA_MASTER u_dma (
        .sys_clk(sys_clk),
        .rst_core_n(rst_core_n),
        .read_burst_ready_in(read_burst_ready),
        .fifo_read_ready_in(fifo_read_ready),
        .fifo_read_valid_out(fifo_read_valid),
        .fifo_read_data_in(fifo_read_data),
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

endmodule