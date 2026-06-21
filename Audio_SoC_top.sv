`timescale 1ns/1ps

import dsp_pkg::*;

/*
 * Audio SoC Top — Multi-Channel ADC RX Subsystem with Inline DSP
 *
 * ┌─────────────────────────────────────────────────────────────────────────────────────────┐
 * │                              Audio_SoC_top                                              │
 * │                                                                                         │
 * │  ┌──────────┐   ┌────────────┐   ┌─────────────────────────────────────┐                │
 * │  │ Reset    │   │  Freq      │   │  Reset_Synchronizer ×2              │                │
 * │  │ Sequencer│──▶│  Divider   │   │  (pll → sys_clk, pll → i2s_bclk)   │                │
 * │  └──────────┘   │  ÷5        │   └─────────────────────────────────────┘                │
 * │  pad_rst_n       └──────┬────┘                                                          │
 * │  pll_clk                │sys_clk                                                        │
 * │                         ▼                                                               │
 * │  ┌────────────────────────────────────────────────────────────────┐  ×4 (generate)       │
 * │  │  GEN_AUDIO_CHANNELS[i]                                        │                      │
 * │  │                                                                │                      │
 * │  │  I2S ──▶ DC_Cal_L ──┐                                         │                      │
 * │  │  ADC     DC_Cal_R ──┤ MUX ──▶ DSP_Channel_Processor           │                      │
 * │  │                     │         ┌──────────────────────────────┐ │                      │
 * │  │                     └────────▶│ Async   Biquad  Biquad  Sync│ │                      │
 * │  │                               │ FIFO ──▶ IIR ──▶ IIR ──▶FIFO│──▶ burst_ready        │
 * │  │                               │(CDC)    Stage0  Stage1      │ │                      │
 * │  │                               └──────────────────────────────┘ │                      │
 * │  └────────────────────────────────────────────────────────────────┘                      │
 * │                                          │                                               │
 * │                                          ▼                                               │
 * │                         ┌──────────────────────────────┐                                 │
 * │                         │  AXI4_DMA_Master             │                                 │
 * │                         │  (Round-Robin Arbiter + AXI4 │──▶ AW/W/B → DDR Memory         │
 * │                         │   Write Engine)              │                                 │
 * │                         └──────────────────────────────┘                                 │
 * └─────────────────────────────────────────────────────────────────────────────────────────┘
 */

module Audio_SoC_top (
    input  logic pll_clk,                       // High-speed PLL clock (e.g., 500 MHz)
    input  logic pad_rst_n,                     // Asynchronous external reset (active-low)

    // I2S interface from external ADCs
    input  logic                      i2s_bclk, // Shared I2S bit clock (e.g., 3.072 MHz)
    input  logic [NUM_CHANNELS-1:0]   i2s_ws,
    input  logic [NUM_CHANNELS-1:0]   i2s_sdata,

    // Calibration control
    input  logic [NUM_CHANNELS-1:0]   start_cal,
    output logic [NUM_CHANNELS-1:0]   cal_done,
    output logic [NUM_CHANNELS-1:0]   adc_mux_gnd,

    // AXI4 Write Master interface (to SoC memory interconnect)
    input  logic                       AWREADY,
    output logic                       AWVALID,
    output logic [3:0]                 AWID,
    output logic [DMA_ADDR_WIDTH-1:0]  AWADDR,
    output logic [7:0]                 AWLEN,
    output logic [2:0]                 AWSIZE,

    input  logic                       WREADY,
    output logic                       WVALID,
    output logic [DMA_DATA_WIDTH-1:0]  WDATA,
    output logic                       WLAST,

    input  logic [3:0]                 BID,
    input  logic                       BVALID,
    output logic                       BREADY,
    input  logic [1:0]                 BRESP,

    // Error reporting
    output logic                       ERR_ID_mismatch,
    output logic [1:0]                 ERR_type,
    input  logic                       ERR_release
);

    // =========================================================================
    // 1. Clock & Reset
    // =========================================================================
    logic rst_power_n, rst_bus_n, rst_core_n, rst_adc_n;

    Reset_Sequencer u_rst_seq (
        .pll_clk    (pll_clk),
        .rst_n      (pad_rst_n),
        .rst_power_n(rst_power_n),
        .rst_bus_n  (rst_bus_n),
        .rst_core_n (rst_core_n),
        .rst_adc_n  (rst_adc_n)
    );

    logic sys_clk;
    Freq_Divider #(.N(5)) u_clk_div (
        .pll_clk  (pll_clk),
        .rst_pll_n(rst_power_n),
        .sys_clk  (sys_clk)
    );

    // Synchronize resets to their respective clock domains
    logic rst_sys_n, rst_bclk_n;

    Reset_Synchronizer u_rst_sync_bclk (
        .clk      (i2s_bclk),
        .rst_n_in (rst_adc_n),
        .rst_n_out(rst_bclk_n)
    );

    Reset_Synchronizer u_rst_sync_sys (
        .clk      (sys_clk),
        .rst_n_in (rst_core_n),
        .rst_n_out(rst_sys_n)
    );

    // =========================================================================
    // 2. Per-Channel: I2S → Calibration → DSP → Sync FIFO
    // =========================================================================
    logic [NUM_CHANNELS-1:0]         sfifo_burst_ready;
    logic [NUM_CHANNELS-1:0]         sfifo_rd_ready;
    logic [NUM_CHANNELS-1:0]         sfifo_rd_valid;
    logic [DMA_DATA_WIDTH-1:0]       sfifo_rd_data [NUM_CHANNELS-1:0];

    genvar i;
    generate
        for (i = 0; i < NUM_CHANNELS; i++) begin : ch

            // ---- A. I2S Deserializer ----
            logic [DATA_WIDTH-1:0] raw_left, raw_right;
            logic                  left_valid, right_valid;

            I2S_Deserializer #(.DATA_WIDTH(DATA_WIDTH)) u_i2s (
                .i2s_bclk  (i2s_bclk),
                .rst_n     (rst_bclk_n),
                .i2s_ws    (i2s_ws[i]),
                .i2s_sdata (i2s_sdata[i]),
                .left_data (raw_left),
                .left_valid(left_valid),
                .right_data(raw_right),
                .right_valid(right_valid)
            );

            // ---- B. DC Offset Calibration (Left & Right) ----
            logic signed [DATA_WIDTH-1:0] cal_left, cal_right;
            logic cal_left_valid, cal_right_valid;
            logic mux_gnd_left,  mux_gnd_right;
            logic done_left,     done_right;

            ADC_DC_Offset_Calibrator #(
                .NUMBER_OF_SAMPLE(CAL_SAMPLES),
                .DATA_WIDTH      (DATA_WIDTH)
            ) u_cal_left (
                .clk                    (i2s_bclk),
                .rst_n                  (rst_bclk_n),
                .start_cal              (start_cal[i]),
                .adc_data_in            (raw_left),
                .adc_data_valid_in      (left_valid),
                .adc_mux_gnd            (mux_gnd_left),
                .cal_done               (done_left),
                .calibrated_data_out    (cal_left),
                .calibrated_data_valid_out(cal_left_valid)
            );

            ADC_DC_Offset_Calibrator #(
                .NUMBER_OF_SAMPLE(CAL_SAMPLES),
                .DATA_WIDTH      (DATA_WIDTH)
            ) u_cal_right (
                .clk                    (i2s_bclk),
                .rst_n                  (rst_bclk_n),
                .start_cal              (start_cal[i]),
                .adc_data_in            (raw_right),
                .adc_data_valid_in      (right_valid),
                .adc_mux_gnd            (mux_gnd_right),
                .cal_done               (done_right),
                .calibrated_data_out    (cal_right),
                .calibrated_data_valid_out(cal_right_valid)
            );

            assign adc_mux_gnd[i] = mux_gnd_left | mux_gnd_right;
            assign cal_done[i]    = done_left & done_right;

            // ---- C. Interleave L/R into DSP pipeline ----
            // L and R valid pulses are mutually exclusive → simple MUX
            logic [DATA_WIDTH-1:0] dsp_data;
            logic                  dsp_valid;
            assign dsp_valid = cal_left_valid | cal_right_valid;
            assign dsp_data  = cal_left_valid ? cal_left : cal_right;

            DSP_Channel_Processor #(
                .DATA_WIDTH      (DATA_WIDTH),
                .ASYNC_FIFO_DEPTH(ASYNC_FIFO_DEPTH),
                .SYNC_FIFO_DEPTH (SYNC_FIFO_DEPTH),
                .DMA_DATA_WIDTH  (DMA_DATA_WIDTH)
            ) u_dsp (
                .sys_clk         (sys_clk),
                .i2s_bclk        (i2s_bclk),
                .rst_sys_n       (rst_sys_n),
                .rst_bclk_n      (rst_bclk_n),
                .afifo_wr_data   (dsp_data),
                .afifo_wr_valid  (dsp_valid),
                .afifo_wr_ready  (),            // Backpressure not used (FIFO sized for headroom)
                .sfifo_rd_valid  (sfifo_rd_valid[i]),
                .sfifo_rd_ready  (sfifo_rd_ready[i]),
                .sfifo_rd_data   (sfifo_rd_data[i]),
                .sfifo_burst_ready(sfifo_burst_ready[i])
            );
        end
    endgenerate

    // =========================================================================
    // 3. AXI4 DMA Engine (sys_clk domain)
    // =========================================================================
    AXI4_DMA_Master #(
        .AXI_BURST_LEN (AXI_BURST_LEN),
        .AXI_BURST_SIZE(AXI_BURST_SIZE),
        .DMA_DATA_WIDTH(DMA_DATA_WIDTH),
        .DMA_ADDR_WIDTH(DMA_ADDR_WIDTH),
        .NUM_CHANNELS  (NUM_CHANNELS)
    ) u_dma (
        .sys_clk        (sys_clk),
        .rst_n          (rst_sys_n),
        .fifo_rd_valid  (sfifo_rd_valid),
        .burst_ready_in (sfifo_burst_ready),
        .fifo_rd_ready  (sfifo_rd_ready),
        .fifo_rd_data   (sfifo_rd_data),
        .AWREADY        (AWREADY),
        .AWVALID        (AWVALID),
        .AWID           (AWID),
        .AWADDR         (AWADDR),
        .AWLEN          (AWLEN),
        .AWSIZE         (AWSIZE),
        .WREADY         (WREADY),
        .WVALID         (WVALID),
        .WDATA          (WDATA),
        .WLAST          (WLAST),
        .BID            (BID),
        .BVALID         (BVALID),
        .BREADY         (BREADY),
        .BRESP          (BRESP),
        .ERR_ID_mismatch(ERR_ID_mismatch),
        .ERR_type       (ERR_type),
        .ERR_release    (ERR_release)
    );

endmodule