`timescale 1ns/1ps

module Sync_FIFO #(
    parameter FIFO_DEPTH = 32,
    parameter DMA_DATA_WIDTH = 32
)(
    input sys_clk,
    input rst_sfifo_n,
    
    // DMA interface
    input  fifo_read_valid_in,                     
    output fifo_read_ready_out,                    
    output [DMA_DATA_WIDTH-1:0] fifo_read_data_out,
    output read_burst_ready_out,  

    // Biquad filter interface
    input [DMA_DATA_WIDTH-1:0] fifo_write_data_in,
    input fifo_write_valid_in,
    output fifo_write_ready_out                  
);

logic [DMA_DATA_WIDTH-1:0] FIFO_inst[FIFO_DEPTH-1:0];
logic [$clog2(FIFO_DEPTH):0] wr_ptr, rd_ptr, data_count;

logic FULL, EMPTY;
assign FULL = (wr_ptr[$clog2(FIFO_DEPTH)] != rd_ptr[$clog2(FIFO_DEPTH)]) && (wr_ptr[$clog2(FIFO_DEPTH)-1:0] == rd_ptr[$clog2(FIFO_DEPTH)-1:0]);
assign EMPTY = (wr_ptr == rd_ptr);
assign fifo_read_ready_out  = ~EMPTY;
assign fifo_write_ready_out = ~FULL;

logic read_handshake, write_handshake;
assign read_handshake  = fifo_read_valid_in  & fifo_read_ready_out;
assign write_handshake = fifo_write_valid_in & fifo_write_ready_out;

always_ff @(posedge sys_clk or negedge rst_sfifo_n) begin
    if(~rst_sfifo_n) begin
        wr_ptr <= '0;
        rd_ptr <= '0;
    end
    else begin
        if(read_handshake) begin
            rd_ptr <= rd_ptr + 1;
        end

        if(write_handshake) begin
            wr_ptr <= wr_ptr + 1;
            FIFO_inst[wr_ptr[$clog2(FIFO_DEPTH)-1:0]] <= fifo_write_data_in;
        end
    end
end

assign fifo_read_data_out = FIFO_inst[rd_ptr[$clog2(FIFO_DEPTH)-1:0]]; // FWFT
assign data_count = wr_ptr - rd_ptr;
assign read_burst_ready_out = (data_count < 4)? 1'b0 : 1'b1; 

endmodule