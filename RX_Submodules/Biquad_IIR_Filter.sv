/*
Filter Specifications & Q14 Fixed-Point Coefficients (Direct Form I):
    Sampling Rate (f_s): 48kHz
    Cutoff Frequency (f_c): (3kHz)
    Low-pass Scaling Factor: 2^{14} = 16384 times (Q14)

Difference Equation: y[n] = (B0 * x[n]) + (B1 * x[n-1]) + (B2 * x[n-2]) - (A1 * y[n-1) - (A2 * y[n-2])
                          = (mul_x0 + mul_x1 + mul_x2) - (mul_y1 + mul_y2) = add_x - add_y

*/


`timescale 1nns/1ps

module Biquad_IIR_Filter #(
    parameter int DATA_WIDTH = 16,
    parameter int SCALING_FACTOR = 14,
    parameter signed [15:0] B0 = 16'sd551,
    parameter signed [15:0] B1 = 16'sd1101,
    parameter signed [15:0] B2 = 16'sd551,
    parameter signed [31:0] A1 = 32'sd(-24959); // -1.5234 * 16384 (<<< 14)
    parameter signed [15:0] A2 = 16'sd10777;    //  0.6578 * 16384
)
(
    input sys_clk,
    input sys_clk_rst_n,

    // Async FIFO Interface (ADC)
    input  logic signed [DATA_WIDTH-1:0] read_data_in,
    input  logic read_ready_in,       // From FIFO read_ready_out (!empty)
    output logic read_valid_out,    // To FIFO read_valid_in

    // Output Interface
    output [DATA_WIDTH-1:0] y_out,
    output y_valid_out,
    input  logic y_ready_in           // Downstream readiness for backpressure
);

// current input, previous inputs, and previous outputs
logic signed [DATA_WIDTH-1:0] x_n;
logic signed [DATA_WIDTH-1:0] x_n_1, x_n_2, y_n_1, y_n_2;
assign x_n = read_data_in;

// first stage
logic signed [2*DATA_WIDTH+10:0] mul_x0;
logic signed [2*DATA_WIDTH+10:0] mul_x1;
logic signed [2*DATA_WIDTH+10:0] mul_x2;
logic signed [2*DATA_WIDTH+10:0] mul_y1;
logic signed [2*DATA_WIDTH+10:0] mul_y2;
// second stage
logic signed [2*DATA_WIDTH+10:0] add_x;
logic signed [2*DATA_WIDTH+10:0] add_y;
// final stage
logic signed [2*DATA_WIDTH+10:0] y_out_scale;
logic signed [2*DATA_WIDTH+10-SCALING_FACTOR:0] y_out_descale;
logic signed [DATA_WIDTH-1:0] y_n;           // truncated back to 16 bits

// 3-stage difference equation
logic valid_passthrough_stage_1;
logic valid_passthrough_stage_2;
logic valid_passthrough_stage_3;

// --- Pipeline Control & IIR Hazard Prevention ---
// We cannot ingest a new sample if the pipeline is currently processing one, 
// because we need the updated y_n to feed back into y_n_1 for the next sample.
logic busy;
logic stall;
assign busy = valid_passthrough_stage_1 || valid_passthrough_stage_2 || valid_passthrough_stage_3 || stall; // After read handshaking, backpressure the FIFO read till firing output y_n to AXIS bus
assign stall = y_valid_out && !y_ready_in;      // Downstream backpressure stall

logic input_read_handshake;
assign read_valid_out = !busy;
assign input_read_handshake = read_valid_out && read_ready_in;


always_ff @(posedge sys_clk, negedge sys_clk_rst_n) begin : PIPELINED_DIFF_EQUATION
    if(~sys_clk_rst_n) begin
        x_n_1  <= '0;
        x_n_2  <= '0;
        y_n_1  <= '0;
        y_n_2  <= '0;
        mul_x0 <= '0;
        mul_x1 <= '0;
        mul_x2 <= '0;
        mul_y1 <= '0;
        mul_y2 <= '0;
        add_x  <= '0;
        add_y  <= '0;
        y_out_scale <= '0;
        valid_passthrough_stage_1 <= '0;
        valid_passthrough_stage_2 <= '0;
        valid_passthrough_stage_3 <= '0;
    end
    else begin
        // stage 1
        valid_passthrough_stage_1 <= input_read_handshake;
        if(input_read_handshake) begin
            x_n_1 <= x_n;      // Shift history
            x_n_2 <= x_n_1;

            mul_x0 <= B0 * x_n;
            mul_x1 <= B1 * x_n_1;
            mul_x2 <= B2 * x_n_2;
            mul_y1 <= A1 * y_n_1;
            mul_y2 <= A2 * y_n_2;
        end

        // stage 2
        valid_passthrough_stage_2 <= valid_passthrough_stage_1;
        if(valid_passthrough_stage_1) begin
            add_x <= mul_x0 + mul_x1 + mul_x2;
            add_y <= mul_y1 + mul_y2;
        end

        // stage 3
        valid_passthrough_stage_3 <= valid_passthrough_stage_2;
        if(valid_passthrough_stage_2) begin
            y_out_scale <= add_x - add_y;
        end

        // Update y feedback history once stage 3 computation is valid
        if(valid_passthrough_stage_3) begin
            y_n_1 <= y_n;      // y_n is combinationally derived from y_out_scale below
            y_n_2 <= y_n_1;
        end
    end
end

assign y_out_descale = y_out_scale >> SCALING_FACTOR;

// y_n
always_comb begin : OUTPUT_FORMATTING // Saturation Handling
    y_n = y_out_descale[15:0];
    if(y_out_descale > 'sd32767) begin        // 2^15-1 = 32767
        y_n = 'd32767;
    end
    else if(y_out_descale < -'sd32768) begin  // -2^15 = -32768
        y_n = -'sd32768;
    end
end

assign y_valid_out = valid_passthrough_stage_3;
assign y_out = y_n;

endmodule