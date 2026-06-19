`timescale 1ps/1ps

import general_pkg::*

module async_fifo
(
    input write_clk,
    input read_clk,
    input write_rst_n,
    input read_rst_n,

    output write_ready_out,
    output read_ready_out,

    // Write Interface
    input  logic write_valid_in,
    input  logic [DATA_WIDTH-1:0] write_data_in,

    // Read Interface
    input  logic read_valid_in,
    output logic [DATA_WIDTH-1:0] read_data_out
);

    localparam ADDR_WIDTH = $clog2(FIFO_DEPTH);

    // Internal FIFO MEMORY
    logic [DATA_WIDTH-1:0] FIFO [FIFO_DEPTH-1:0];

    // Pointers with an extra MSB bit for full/empty conditions
    logic [ADDR_WIDTH:0] wptr_bin, wptr_gray;
    logic [ADDR_WIDTH:0] rptr_bin, rptr_gray;
    assign wptr_gray = (wptr_bin) ^ (wptr_bin >> 1);
    assign rptr_gray = (rptr_bin) ^ (rptr_bin >> 1);

    // Synchronized pointers
    logic [ADDR_WIDTH:0] wptr_gray_sync1, wptr_gray_sync2;
    logic [ADDR_WIDTH:0] rptr_gray_sync1, rptr_gray_sync2;

    logic write_handshake, read_handshake;

    assign write_handshake = write_valid_in && write_ready_out; // write_ready_out is asserted when FIFO is deemed full
    assign read_handshake  = read_valid_in && read_ready_out;   //

    // ---------------------------------------------------------
    // WRITE DOMAIN
    // ---------------------------------------------------------
    always_ff @(posedge write_clk or negedge write_rst_n) begin : FIFO_WRITE_PTR
        if (!write_rst_n) begin
            wptr_bin  <= '0;
        end 
        else if (write_handshake) begin
            wptr_bin  <= wptr_bin + 1'b1;
            FIFO[wptr_bin[ADDR_WIDTH-1:0]] <= write_data_in;
        end
    end


    // Synchronize Read Pointer to Write Domain
    always_ff @(posedge write_clk or negedge write_rst_n) begin : READ_PTR_SYNC
        if (!write_rst_n) begin
            rptr_gray_sync1 <= '0;
            rptr_gray_sync2 <= '0;
        end 
        else begin
            rptr_gray_sync1 <= rptr_gray;
            rptr_gray_sync2 <= rptr_gray_sync1;
        end
    end

    // Full Flag Logic (MSB & 2nd MSB inverted, others matching, because in binary only MSB is different)
    // Pessimistic Full, a read ptr advance takes two cycle to reflect on write side
    logic FIFO_FULL;
    assign FIFO_FULL = (wptr_gray == {~rptr_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1], rptr_gray_sync2[ADDR_WIDTH-2:0]});
    assign write_ready_out = !FIFO_FULL;

    // ---------------------------------------------------------
    // READ DOMAIN
    // ---------------------------------------------------------
    always_ff @(posedge read_clk or negedge read_rst_n) begin : FIFO_READ_PTR
        if (!read_rst_n) begin
            rptr_bin  <= '0;
        end 
        else if (read_handshake) begin
            rptr_bin  <= rptr_bin + 1'b1;
        end
    end

    // Read Data Output (First-Word Fall-Through / FWFT)
    // Data is combinationally available as long as the FIFO is not empty.
    // BRAM cannot be inferred with this approach, but for small depth like 100 it synthesizes 
    // efficiently into Distributed RAM (LUTs/Registers).
    assign read_data_out = FIFO[rptr_bin[ADDR_WIDTH-1:0]];

    // Synchronize Write Pointer to Read Domain
    always_ff @(posedge read_clk or negedge read_rst_n) begin : WRITE_PTR_SYNC
        if (!read_rst_n) begin
            wptr_gray_sync1 <= '0;
            wptr_gray_sync2 <= '0;
        end else begin
            wptr_gray_sync1 <= wptr_gray;
            wptr_gray_sync2 <= wptr_gray_sync1;
        end
    end

    // Empty Flag Logic (Pointers match exactly)
    logic FIFO_EMPTY;
    assign FIFO_EMPTY = (rptr_gray == wptr_gray_sync2);
    assign read_ready_out = ~FIFO_EMPTY;
endmodule