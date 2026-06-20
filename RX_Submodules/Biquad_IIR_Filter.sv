/*
Filter Specifications & Q14 Fixed-Point Coefficients (Direct Form I):
    Sampling Rate (f_s): 48kHz
    Cutoff Frequency (f_c): (3kHz)
    Low-pass Scaling Factor: 2^{14} = 16384 times (Q14)

Difference Equation: y[n] = (B0 * x[n]) + (B1 * x[n-1]) + (B2 * x[n-2]) - (A1 * y[n-1) - (A2 * y[n-2])
                          = (mul_x0 + mul_x1 + mul_x2) - (mul_y1 + mul_y2) = add_x - add_y

*/


`timescale 1ns/1ps

module Biquad_IIR_Filter #(
    parameter DATA_WIDTH = 16,
    parameter SCALING_FACTOR = 14,
    parameter signed [15:0] B0 = 16'sd551,
    parameter signed [15:0] B1 = 16'sd1101,
    parameter signed [15:0] B2 = 16'sd551,
    parameter signed [31:0] A1 = 32'sd(-24959), // -1.5234 * 16384 (<<< 14)
    parameter signed [15:0] A2 = 16'sd10777,    //  0.6578 * 16384
)
(
    input sys_clk,
    input sys_clk_rst_n,

    input signed [DATA_WIDTH-1:0] read_data_in,
    input read_ready_in,       
    output logic read_valid_out,

    // Output Interface
    output [DATA_WIDTH-1:0] y_out, // sync_fifo_write_data_out
    output y_valid_out,                // sync_fifo_write_valid_out
    input  y_ready_in                  // sync_fifo_write_ready_in
);

localparam signed MAX_OUT = 'sd(2**(DATA_WIDTH-1)-1);
localparam signed MIN_OUT = -'sd(2**(DATA_WIDTH-1));

// current input, previous inputs, and previous outputs
logic signed [DATA_WIDTH-1:0] x_n, x_n_1, x_n_2, y_n_1, y_n_2; 
logic signed [DATA_WIDTH-1:0] left_x_n_1, left_x_n_2, left_y_n_1, left_y_n_2; 
logic signed [DATA_WIDTH-1:0] right_x_n_1, right_x_n_2, right_y_n_1, right_y_n_2; 
logic channel_sel;

always_comb begin
    x_n = read_data_in;
    x_n_1 = left_x_n_1;
    x_n_2 = left_x_n_2;
    y_n_1 = left_y_n_1;
    y_n_2 = left_y_n_2;   
    if(channel_sel) begin
        x_n_1 = right_x_n_1;
        x_n_2 = right_x_n_2;
        y_n_1 = right_y_n_1;
        y_n_2 = right_y_n_2;
    end
end

logic signed [2*DATA_WIDTH-1:0] mul_x0, mul_x1, mul_x2, mul_y1, mul_y2;
logic signed [2*DATA_WIDTH+1:0] add_x, add_y;
logic signed [2*DATA_WIDTH+1:0] y_out_scale;
logic signed [2*DATA_WIDTH+1-SCALING_FACTOR:0] y_out_descale;
logic signed [DATA_WIDTH-1:0] y_n;           // truncated back to 16 bits


// --- Pipeline Control & IIR Hazard Prevention ---
// We cannot ingest a new sample if the pipeline is currently processing one, 
// because we need the updated y_n to feed back into y_n_1 for the next sample.
logic input_read_handshake, output_write_handshake;
assign input_read_handshake = read_valid_out & read_ready_in;
assign output_write_handshake = y_valid_out & y_ready_in;

typedef enum logic[1:0] {
    IDLE,
    MUL,
    ADD,
    OUT
} Filter_state_t;
Filter_state_t filter_state, filter_next_state;

always_comb begin
    filter_next_state = filter_state;
    read_valid_out = 1'b0;
    y_valid_out = 1'b0;
    case(filter_state)
        IDLE: begin
            read_valid_out = 1'b1;
            if(input_read_handshake) begin
                filter_next_state = ADD;
            end
        end
        ADD: begin
            filter_next_state = SUB;
        end
        SUB: begin
            filter_next_state = OUT;
        end
        OUT: begin
            y_valid_out = 1'b1;
            if(output_write_handshake) filter_next_state = IDLE;
        end
    endcase
end

always_ff @(posedge sys_clk, negedge sys_clk_rst_n) begin : PIPELINED_DIFF_EQUATION
    if(~sys_clk_rst_n) begin
        filter_state <= IDLE;
        mul_x0 <= '0;
        mul_x1 <= '0;
        mul_x2 <= '0;
        mul_y1 <= '0;
        mul_y2 <= '0;
        add_x  <= '0;
        add_y  <= '0;
        y_out_scale <= '0;
        channel_sel <= 1'b0;
        left_x_n_1 <= '0;
        left_x_n_2 <= '0;
        left_y_n_1 <= '0;
        left_y_n_2 <= '0;
        right_x_n_1 <= '0;
        right_x_n_2 <= '0;
        right_y_n_1 <= '0;
        right_y_n_2 <= '0;
    end
    else begin
        filter_state <= filter_next_state;
        // stage 1 - Input & Multiply
        if(filter_state == IDLE && input_read_handshake) begin
            if(!channel_sel) begin
                left_x_n_2  <= left_x_n_1;
                left_x_n_1  <= x_n;
            end else begin
                right_x_n_2 <= right_x_n_1;
                right_x_n_1 <= x_n;
            end
            mul_x0 <= B0 * x_n;
            mul_x1 <= B1 * x_n_1;
            mul_x2 <= B2 * x_n_2;
            mul_y1 <= A1 * y_n_1;
            mul_y2 <= A2 * y_n_2;
        end

        // stage 2 - ADD
        if(filter_state == ADD) begin
            add_x <= mul_x0 + mul_x1 + mul_x2;
            add_y <= mul_y1 + mul_y2;
        end

        // stage 3 - SUB
        if(filter_state == SUB) begin
            y_out_scale <= add_x - add_y;
        end

        // stage 4 - Output & Update feedback
        if(filter_state == OUT) begin
            if(output_write_handshake) begin
                if(!channel_sel) begin
                    left_y_n_2  <= left_y_n_1;
                    left_y_n_1  <= y_n;
                end else begin
                    right_y_n_2 <= right_y_n_1;
                    right_y_n_1 <= y_n;
                end
                channel_sel <= ~channel_sel;
            end
        end
    end
end

assign y_out_descale = y_out_scale >>> SCALING_FACTOR;

// y_n
always_comb begin : OUTPUT_FORMATTING // Saturation Handling
    y_n = y_out_descale[DATA_WIDTH-1:0];
    if(y_out_descale > MAX_OUT) begin        // 2^15-1 = 32767
        y_n = 'd32767;
    end
    else if(y_out_descale < MIN_OUT) begin  // -2^15 = -32768
        y_n = -'sd32768;
    end
end

assign y_out = y_n;

endmodule