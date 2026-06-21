`timescale 1ns/1ps

// Synchronous FIFO with FWFT read and burst-ready signaling for DMA.
// FIFO_DEPTH must be a power of 2.
module Sync_FIFO #(
    parameter FIFO_DEPTH     = 32,
    parameter DMA_DATA_WIDTH = 32
)(
    input  sys_clk,
    input  rst_n,

    // DMA read interface (FWFT)
    input  logic                      fifo_read_valid_in,
    output logic                      fifo_read_ready_out,
    output logic [DMA_DATA_WIDTH-1:0] fifo_read_data_out,
    output logic                      read_burst_ready_out,

    // Biquad filter write interface
    input  logic [DMA_DATA_WIDTH-1:0] fifo_write_data_in,
    input  logic                      fifo_write_valid_in,
    output logic                      fifo_write_ready_out
);

    localparam PTR_WIDTH = $clog2(FIFO_DEPTH);

    logic [DMA_DATA_WIDTH-1:0] mem [FIFO_DEPTH-1:0];
    logic [PTR_WIDTH:0] wr_ptr, rd_ptr;

    logic full, empty;
    assign full  = (wr_ptr[PTR_WIDTH] != rd_ptr[PTR_WIDTH]) &&
                   (wr_ptr[PTR_WIDTH-1:0] == rd_ptr[PTR_WIDTH-1:0]);
    assign empty = (wr_ptr == rd_ptr);

    assign fifo_read_ready_out  = ~empty;
    assign fifo_write_ready_out = ~full;

    logic rd_handshake, wr_handshake;
    assign rd_handshake = fifo_read_valid_in  & fifo_read_ready_out;
    assign wr_handshake = fifo_write_valid_in & fifo_write_ready_out;

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
        end else begin
            if (rd_handshake)
                rd_ptr <= rd_ptr + 1;
            if (wr_handshake) begin
                mem[wr_ptr[PTR_WIDTH-1:0]] <= fifo_write_data_in;
                wr_ptr <= wr_ptr + 1;
            end
        end
    end

    // FWFT read output
    assign fifo_read_data_out = mem[rd_ptr[PTR_WIDTH-1:0]];

    // Burst-ready: at least 4 words available for DMA
    wire [PTR_WIDTH:0] data_count = wr_ptr - rd_ptr;
    assign read_burst_ready_out = (data_count >= PTR_WIDTH'(4));

endmodule