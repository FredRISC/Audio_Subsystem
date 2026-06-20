`timescale 1ns/1ps

module DSP_Channel_Processor #(
    parameter DATA_WIDTH = 16
    parameter ASYNC_FIFO_DEPTH = 32,
    parameter SYNC_FIFO_DEPTH  = 32,
    parameter DMA_DATA_WIDTH = 32
)(    
// Async_FIFO IS2 interface
    input i2s_bclk,
    input async_fifo_write_valid_in,
    input [DATA_WIDTH-1:0] async_fifo_write_data_in,
    output async_fifo_write_ready_out, 

// Sync_FIFO DMA interface
    input sys_clk,
    input rst_sys_n,
    input  sync_fifo_read_valid_in,                     
    output sync_fifo_read_ready_out,                    
    output [DATA_WIDTH-1:0] sync_fifo_read_data_out,
    output sync_fifo_read_burst_ready_out                   
);

// Async_FIFO Biquad filter interface
logic async_fifo_read_valid;
logic async_fifo_read_ready;
logic [DATA_WIDTH-1:0] async_fifo_read_data;

// Sync_FIFO Biquad filter interface
logic [DATA_WIDTH-1:0] sync_fifo_write_data_in;
logic sync_fifo_write_valid_in;
logic sync_fifo_write_ready_out;

// Interconnect
logic [DATA_WIDTH-1:0] biq0_y_out, biq1_y_out;
logic biq0_y_valid, biq1_y_valid;               
logic biq0_y_ready, biq0_y_ready;                  

Async_FIFO #(
    .DATA_WIDTH(DATA_WIDTH),
    .FIFO_DEPTH(ASYNC_FIFO_DEPTH)
) u_async_fifo (
    .read_clk(sys_clk),
    .write_clk(i2s_bclk),
    .rst_afifo_n(rst_sys_n),
    .read_valid_in(async_fifo_read_valid),
    .read_ready_out(async_fifo_read_ready),
    .read_data_out(async_fifo_read_data),
    .write_valid_in(async_fifo_write_valid_in), 
    .write_data_in(async_fifo_write_data_in),
    .write_ready_out(async_fifo_write_ready_out)
);


Biquad_IIR_Filter u_biq0(
    .sys_clk(sys_clk),
    .sys_clk_rst_n(rst_sys_n),

    // Biquad filter input interface (read from Async_FIFO)
    .read_valid_out(async_fifo_read_valid),
    .read_ready_in(async_fifo_read_ready),
    .read_data_in(async_fifo_read_data),

    // Biquad filter output interface (write to Sync_FIFO)
    .y_valid_out(biq0_y_valid),
    .y_ready_in(biq0_y_ready),
    .y_out(biq0_y_out),
);

Biquad_IIR_Filter u_biq1(
    .sys_clk(sys_clk),
    .sys_clk_rst_n(rst_sys_n),
    // Biquad filter input interface (read from Async_FIFO)
    .read_valid_out(biq0_y_ready),
    .read_ready_in(biq0_y_valid),
    .read_data_in(biq0_y_out),

    // Biquad filter output interface (write to Sync_FIFO)
    .y_valid_out(biq1_y_valid),
    .y_ready_in(biq1_y_ready),
    .y_out(biq1_y_out),
);

localparam SIGN_EXT_BITS  = DMA_DATA_WIDTH - DATA_WIDTH;
logic [DMA_DATA_WIDTH-1:0] sign_ext_y_out;
assign sign_ext_y_out = {SIGN_EXT_BITS{biq1_y_out[DATA_WIDTH-1]}, biq1_y_out};

Sync_FIFO #(
    .FIFO_DEPTH(SYNC_FIFO_DEPTH),
    .DMA_DATA_WIDTH(DMA_DATA_WIDTH),
)(
    .sys_clk(sys_clk),
    .rst_sfifo_n(rst_sys_n),
    
    // DMA interface
    .fifo_read_valid_in(sync_fifo_read_valid_in),
    .fifo_read_ready_out(sync_fifo_read_ready_out),
    .fifo_read_data_out(sync_fifo_read_data_out),
    .read_burst_ready_out(sync_fifo_read_burst_ready_out),  

    // Biquad filter interface
    .fifo_write_data_in(sign_ext_y_out),
    .fifo_write_valid_in(biq1_y_valid),
    .fifo_write_ready_out(biq1_y_ready),
);

endmodule