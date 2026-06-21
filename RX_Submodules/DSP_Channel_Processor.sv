`timescale 1ns/1ps

// DSP Channel Processor — per-channel inline processing block.
// Data flow: Async_FIFO → Biquad_0 → Biquad_1 → Sync_FIFO
// The Sync_FIFO output presents burst-ready + FWFT interface to the DMA engine.
module DSP_Channel_Processor #(
    parameter DATA_WIDTH      = 16,
    parameter ASYNC_FIFO_DEPTH = 32,
    parameter SYNC_FIFO_DEPTH  = 32,
    parameter DMA_DATA_WIDTH   = 32
)(
    // Clocks & reset
    input  sys_clk,
    input  i2s_bclk,
    input  rst_sys_n,       // Synchronized to sys_clk
    input  rst_bclk_n,      // Synchronized to i2s_bclk

    // Async FIFO write interface (from calibrator, i2s_bclk domain)
    input  logic [DATA_WIDTH-1:0] afifo_wr_data,
    input  logic                  afifo_wr_valid,
    output logic                  afifo_wr_ready,

    // Sync FIFO read interface (to DMA, sys_clk domain)
    input  logic                      sfifo_rd_valid,
    output logic                      sfifo_rd_ready,
    output logic [DMA_DATA_WIDTH-1:0] sfifo_rd_data,
    output logic                      sfifo_burst_ready
);

    // ---- Internal wires ----
    // Async FIFO → Biquad 0
    logic                  afifo_rd_valid;
    logic                  afifo_rd_ready;
    logic [DATA_WIDTH-1:0] afifo_rd_data;

    // Biquad 0 → Biquad 1
    logic [DATA_WIDTH-1:0] bq0_y;
    logic                  bq0_y_valid;
    logic                  bq0_y_ready;

    // Biquad 1 → Sync FIFO
    logic [DATA_WIDTH-1:0] bq1_y;
    logic                  bq1_y_valid;
    logic                  bq1_y_ready;

    // ---- Async FIFO (CDC: i2s_bclk → sys_clk) ----
    Async_FIFO #(
        .DATA_WIDTH (DATA_WIDTH),
        .FIFO_DEPTH (ASYNC_FIFO_DEPTH)
    ) u_async_fifo (
        .read_clk       (sys_clk),
        .write_clk      (i2s_bclk),
        .rst_rd_n       (rst_sys_n),
        .rst_wr_n       (rst_bclk_n),
        .read_valid_in  (afifo_rd_valid),
        .read_ready_out (afifo_rd_ready),
        .read_data_out  (afifo_rd_data),
        .write_valid_in (afifo_wr_valid),
        .write_data_in  (afifo_wr_data),
        .write_ready_out(afifo_wr_ready)
    );

    // ---- Biquad Stage 0 ----
    Biquad_IIR_Filter u_bq0 (
        .sys_clk       (sys_clk),
        .rst_n         (rst_sys_n),
        .read_data_in  (afifo_rd_data),
        .read_ready_in (afifo_rd_ready),
        .read_valid_out(afifo_rd_valid),
        .y_out         (bq0_y),
        .y_valid_out   (bq0_y_valid),
        .y_ready_in    (bq0_y_ready)
    );

    // ---- Biquad Stage 1 ----
    Biquad_IIR_Filter u_bq1 (
        .sys_clk       (sys_clk),
        .rst_n         (rst_sys_n),
        .read_data_in  (bq0_y),
        .read_ready_in (bq0_y_valid),
        .read_valid_out(bq0_y_ready),
        .y_out         (bq1_y),
        .y_valid_out   (bq1_y_valid),
        .y_ready_in    (bq1_y_ready)
    );

    // ---- Sign-extend 16-bit filter output to 32-bit DMA width ----
    logic [DMA_DATA_WIDTH-1:0] sign_ext_data;
    assign sign_ext_data = {{(DMA_DATA_WIDTH - DATA_WIDTH){bq1_y[DATA_WIDTH-1]}}, bq1_y};

    // ---- Sync FIFO (post-DSP buffering, sys_clk domain) ----
    Sync_FIFO #(
        .FIFO_DEPTH     (SYNC_FIFO_DEPTH),
        .DMA_DATA_WIDTH (DMA_DATA_WIDTH)
    ) u_sync_fifo (
        .sys_clk             (sys_clk),
        .rst_n               (rst_sys_n),
        .fifo_read_valid_in  (sfifo_rd_valid),
        .fifo_read_ready_out (sfifo_rd_ready),
        .fifo_read_data_out  (sfifo_rd_data),
        .read_burst_ready_out(sfifo_burst_ready),
        .fifo_write_data_in  (sign_ext_data),
        .fifo_write_valid_in (bq1_y_valid),
        .fifo_write_ready_out(bq1_y_ready)
    );

endmodule