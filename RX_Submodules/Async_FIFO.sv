`timescale 1ns/1ps

// Async FIFO with separate read/write domain resets and Gray-code pointer CDC.
// Uses distributed RAM (FWFT read). FIFO_DEPTH must be a power of 2.
module Async_FIFO #(
    parameter DATA_WIDTH = 16,
    parameter FIFO_DEPTH = 32
)(
    input  read_clk,
    input  write_clk,
    input  rst_rd_n,        // Reset synchronized to read_clk domain
    input  rst_wr_n,        // Reset synchronized to write_clk domain

    // Read interface (FWFT)
    input  logic read_valid_in,
    output logic read_ready_out,
    output logic [DATA_WIDTH-1:0] read_data_out,

    // Write interface
    input  logic write_valid_in,
    input  logic [DATA_WIDTH-1:0] write_data_in,
    output logic write_ready_out
);

    localparam PTR_WIDTH = $clog2(FIFO_DEPTH);

    // Internal FIFO memory
    logic [DATA_WIDTH-1:0] mem [FIFO_DEPTH-1:0];

    // Binary and Gray-code pointers (extra MSB for full/empty detection)
    logic [PTR_WIDTH:0] rd_bin, wr_bin;
    logic [PTR_WIDTH:0] rd_gray, wr_gray;
    assign rd_gray = rd_bin ^ (rd_bin >> 1);
    assign wr_gray = wr_bin ^ (wr_bin >> 1);

    // Synchronized Gray-code pointers
    (* async_reg = "true" *) logic [PTR_WIDTH:0] wr_gray_sync1, wr_gray_sync2;
    (* async_reg = "true" *) logic [PTR_WIDTH:0] rd_gray_sync1, rd_gray_sync2;

    // Full / Empty flags
    logic full, empty;
    assign full  = (rd_gray_sync2 == {~wr_gray[PTR_WIDTH:PTR_WIDTH-1],
                                        wr_gray[PTR_WIDTH-2:0]});
    assign empty = (wr_gray_sync2 == rd_gray);

    // Handshake signals
    logic rd_handshake, wr_handshake;
    assign rd_handshake = read_valid_in  && read_ready_out;
    assign wr_handshake = write_valid_in && write_ready_out;

    // ---- Write domain ----
    always_ff @(posedge write_clk or negedge rst_wr_n) begin
        if (~rst_wr_n)
            wr_bin <= '0;
        else if (wr_handshake) begin
            mem[wr_bin[PTR_WIDTH-1:0]] <= write_data_in;
            wr_bin <= wr_bin + 1;
        end
    end

    // Sync read Gray pointer into write domain
    always_ff @(posedge write_clk or negedge rst_wr_n) begin
        if (~rst_wr_n) begin
            rd_gray_sync1 <= '0;
            rd_gray_sync2 <= '0;
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

    // ---- Read domain ----
    always_ff @(posedge read_clk or negedge rst_rd_n) begin
        if (~rst_rd_n)
            rd_bin <= '0;
        else if (rd_handshake)
            rd_bin <= rd_bin + 1;
    end

    // Sync write Gray pointer into read domain
    always_ff @(posedge read_clk or negedge rst_rd_n) begin
        if (~rst_rd_n) begin
            wr_gray_sync1 <= '0;
            wr_gray_sync2 <= '0;
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
        end
    end

    // Output assignments (FWFT — data available combinationally)
    assign read_data_out  = mem[rd_bin[PTR_WIDTH-1:0]];
    assign read_ready_out = ~empty;
    assign write_ready_out = ~full;

endmodule
