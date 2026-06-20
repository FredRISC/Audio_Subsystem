`timescale 1ns/1ps

module data_count_aware_async_fifo #(
    parameter DATA_WIDTH = 32,  // 32-bit data
    parameter FIFO_DEPTH = 16
)
(
    input read_clk,
    input write_clk,
    input rst_bus_n,

    // read interface
    input read_valid_in,
    output read_ready_out,
    output [DATA_WIDTH-1:0] read_data_out,
    // write interface
    input write_valid_in,
    input [DATA_WIDTH-1:0] write_data_in,
    output write_ready_out, 

    // Report to the fifo arbiter that there is at least 4 data words in the FIFO
    output read_burst_ready_out
);

// RDC
logic read_rst_n_sync1, read_rst_n_sync2;

always_ff @(posedge read_clk or negedge rst_bus_n) begin
    if(~rst_bus_n) begin
        read_rst_n_sync1 <= 1'b0;
        read_rst_n_sync2 <= 1'b0;
    end
    else begin
        read_rst_n_sync1 <= 1'b1;
        read_rst_n_sync2 <= read_rst_n_sync1;
    end
end

(* async_reg = "true" *) logic write_rst_n_sync1, write_rst_n_sync2;
always_ff @(posedge write_clk or negedge rst_bus_n) begin
    if(~rst_bus_n) begin
        write_rst_n_sync1 <= 1'b0;
        write_rst_n_sync2 <= 1'b0;
    end
    else begin
        write_rst_n_sync1 <= 1'b1;
        write_rst_n_sync2 <= write_rst_n_sync1;
    end
end


logic [DATA_WIDTH-1:0] FIFO [FIFO_DEPTH-1:0];
logic [$clog2(FIFO_DEPTH):0] read_bin_ptr, write_bin_ptr, read_gray_ptr, write_gray_ptr;

assign read_gray_ptr = read_bin_ptr ^ (read_bin_ptr >> 1);      // to be sycned to write_clk
assign write_gray_ptr = write_bin_ptr ^ (write_bin_ptr >> 1);   // to be sycned to read_clk

(* async_reg = "true" *)  logic [$clog2(FIFO_DEPTH):0] read_gray_ptr_sync1, read_gray_ptr_sync2, write_gray_ptr_sync1, write_gray_ptr_sync2;
logic FIFO_FULL, FIFO_EMPTY;
assign FIFO_FULL = (read_gray_ptr_sync2 == {~write_gray_ptr[$clog2(FIFO_DEPTH):$clog2(FIFO_DEPTH)-1], write_gray_ptr[$clog2(FIFO_DEPTH)-2:0]});
assign FIFO_EMPTY = (write_gray_ptr_sync2 == read_gray_ptr); 

// Sync the gray pointers
always_ff @(posedge read_clk or negedge read_rst_n_sync2) begin
    if(~read_rst_n_sync2) begin
        write_gray_ptr_sync1 <= 'd0;
        write_gray_ptr_sync2 <= 'd0;
    end
    else begin
        write_gray_ptr_sync1 <= write_gray_ptr;
        write_gray_ptr_sync2 <= write_gray_ptr_sync1;
    end
end

always_ff @(posedge write_clk or negedge write_rst_n_sync2) begin
    if(~write_rst_n_sync2) begin
        read_gray_ptr_sync1 <= 'd0;
        read_gray_ptr_sync2 <= 'd0;
    end
    else begin
        read_gray_ptr_sync1 <= read_gray_ptr;
        read_gray_ptr_sync2 <= read_gray_ptr_sync1;
    end
end

logic fifo_read_handshake, write_handshake;
assign fifo_read_handshake = read_valid_in && read_ready_out;
assign write_handshake = write_valid_in && write_ready_out;

always_ff @(posedge read_clk or negedge read_rst_n_sync2) begin
    if(~read_rst_n_sync2) begin
        read_bin_ptr <= '0;
    end
    else if(fifo_read_handshake) begin
        read_bin_ptr <= read_bin_ptr + 1;
    end
end

always_ff @(posedge write_clk or negedge write_rst_n_sync2) begin
    if(~write_rst_n_sync2) begin
        write_bin_ptr <= '0;
    end
    else if(write_handshake) begin
        FIFO[write_bin_ptr] <= write_data_in;
        write_bin_ptr <= write_bin_ptr + 1;
    end
end


// Driving output ports
assign read_data_out   = FIFO[read_bin_ptr];
assign read_ready_out  = ~FIFO_EMPTY;
assign write_ready_out = ~FIFO_FULL;


// Driving read_burst_ready_out
// transform the read-synced write gray pointer (write_gray_ptr_sync2) to binary
logic [$clog2(FIFO_DEPTH):0] write_bin_ptr_sync2;
logic [$clog2(FIFO_DEPTH):0] data_count;
always_comb begin
    write_bin_ptr_sync2 = 'd0;
    write_bin_ptr_sync2[$clog2(FIFO_DEPTH)] = write_gray_ptr_sync2[$clog2(FIFO_DEPTH)];
    for(int i = $clog2(FIFO_DEPTH)-1; i >= 0; i--) begin // Ignore the critical path problem
        write_bin_ptr_sync2[i] = write_bin_ptr_sync2[i+1] ^ write_gray_ptr_sync2[i];
    end
end
always_ff @(posedge read_clk or negedge read_rst_n_sync2) begin
    if(~read_rst_n_sync2) begin
        data_count <= 'd0;
    end
    else begin
        data_count <= write_bin_ptr_sync2 - read_bin_ptr;
    end
end

assign read_burst_ready_out = (data_count >= 'd4);

endmodule


