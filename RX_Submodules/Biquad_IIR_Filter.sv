/*
 * Biquad IIR Filter — Direct Form I, 4-Stage Pipeline
 *
 * Implements a 2nd-order IIR filter with per-channel (L/R) coefficient state.
 * The interleaved L, R samples share one hardware pipeline via channel_sel muxing.
 *
 * Default coefficients: 2nd-order Butterworth LPF, fs=48kHz, fc=3kHz, Q14.
 *
 * Pipeline stages:
 *   IDLE → MUL (multiply)  → ADD (accumulate) → OUT (output & feedback update)
 *
 * y[n] = (B0·x[n] + B1·x[n-1] + B2·x[n-2]) − (A1·y[n-1] + A2·y[n-2])
 */

`timescale 1ns/1ps

module Biquad_IIR_Filter #(
    parameter DATA_WIDTH    = 16,
    parameter SCALING_FACTOR = 14,
    parameter signed [DATA_WIDTH-1:0] B0 = 16'sd551,
    parameter signed [DATA_WIDTH-1:0] B1 = 16'sd1101,
    parameter signed [DATA_WIDTH-1:0] B2 = 16'sd551,
    parameter signed [DATA_WIDTH-1:0] A1 = -16'sd24959,
    parameter signed [DATA_WIDTH-1:0] A2 = 16'sd10777
)(
    input  sys_clk,
    input  rst_n,

    // Upstream (FIFO read) interface
    input  logic signed [DATA_WIDTH-1:0] read_data_in,
    input  logic                         read_ready_in,
    output logic                         read_valid_out,

    // Downstream (FIFO write) interface
    output logic [DATA_WIDTH-1:0] y_out,
    output logic                  y_valid_out,
    input  logic                  y_ready_in
);

    // ---- Arithmetic widths ----
    // 16×16 multiply → 32 bits; 3-term sum → +2 guard bits = 34 bits
    localparam MUL_W = 2 * DATA_WIDTH;
    localparam ACC_W = MUL_W + 2;

    // ---- Per-channel IIR state registers ----
    logic signed [DATA_WIDTH-1:0] left_x1,  left_x2,  left_y1,  left_y2;
    logic signed [DATA_WIDTH-1:0] right_x1, right_x2, right_y1, right_y2;
    logic channel_sel; // 0 = left, 1 = right

    // Muxed history for current channel
    logic signed [DATA_WIDTH-1:0] x_n, x1, x2, y1, y2;
    always_comb begin
        x_n = read_data_in;
        if (!channel_sel) begin
            x1 = left_x1;   x2 = left_x2;
            y1 = left_y1;   y2 = left_y2;
        end else begin
            x1 = right_x1;  x2 = right_x2;
            y1 = right_y1;  y2 = right_y2;
        end
    end

    // Pipeline registers
    logic signed [MUL_W-1:0] mul_x0, mul_x1, mul_x2, mul_y1, mul_y2;
    logic signed [ACC_W-1:0] sum_x, sum_y;
    logic signed [ACC_W-1:0] y_full;
    logic signed [ACC_W-1-SCALING_FACTOR:0] y_descaled;
    logic signed [DATA_WIDTH-1:0] y_sat;

    // ---- Pipeline FSM ----
    typedef enum logic [1:0] { IDLE, MUL, ADD, OUT } state_t;
    state_t state, next_state;

    logic rd_handshake, wr_handshake;
    assign rd_handshake = read_valid_out & read_ready_in;
    assign wr_handshake = y_valid_out    & y_ready_in;

    always_comb begin
        next_state     = state;
        read_valid_out = 1'b0;
        y_valid_out    = 1'b0;
        case (state)
            IDLE: begin
                read_valid_out = 1'b1;
                if (rd_handshake) next_state = MUL;
            end
            MUL:  next_state = ADD;
            ADD:  next_state = OUT;
            OUT: begin
                y_valid_out = 1'b1;
                if (wr_handshake) next_state = IDLE;
            end
        endcase
    end

    // ---- Datapath ----
    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (~rst_n) begin
            state       <= IDLE;
            channel_sel <= 1'b0;
            mul_x0 <= '0; mul_x1 <= '0; mul_x2 <= '0;
            mul_y1 <= '0; mul_y2 <= '0;
            sum_x  <= '0; sum_y  <= '0;
            y_full <= '0;
            left_x1  <= '0; left_x2  <= '0; left_y1  <= '0; left_y2  <= '0;
            right_x1 <= '0; right_x2 <= '0; right_y1 <= '0; right_y2 <= '0;
        end else begin
            state <= next_state;

            // Stage 1 — Multiply & shift history
            if (state == IDLE && rd_handshake) begin
                mul_x0 <= B0 * x_n;
                mul_x1 <= B1 * x1;
                mul_x2 <= B2 * x2;
                mul_y1 <= A1 * y1;
                mul_y2 <= A2 * y2;
                // Shift x-history for current channel
                if (!channel_sel) begin
                    left_x2  <= left_x1;
                    left_x1  <= x_n;
                end else begin
                    right_x2 <= right_x1;
                    right_x1 <= x_n;
                end
            end

            // Stage 2 — Accumulate
            if (state == MUL) begin
                sum_x <= mul_x0 + mul_x1 + mul_x2;
                sum_y <= mul_y1 + mul_y2;
            end

            // Stage 3 — Subtract
            if (state == ADD)
                y_full <= sum_x - sum_y;

            // Stage 4 — Output & update y-history
            if (state == OUT && wr_handshake) begin
                if (!channel_sel) begin
                    left_y2  <= left_y1;
                    left_y1  <= y_sat;
                end else begin
                    right_y2 <= right_y1;
                    right_y1 <= y_sat;
                end
                channel_sel <= ~channel_sel;
            end
        end
    end

    // ---- Descale & saturate ----
    assign y_descaled = y_full >>> SCALING_FACTOR;

    localparam signed [DATA_WIDTH-1:0] SAT_MAX =  $signed((2**(DATA_WIDTH-1)) - 1);
    localparam signed [DATA_WIDTH-1:0] SAT_MIN = -$signed(2**(DATA_WIDTH-1));

    always_comb begin
        y_sat = y_descaled[DATA_WIDTH-1:0];
        if (y_descaled > signed'((ACC_W-SCALING_FACTOR)'(SAT_MAX)))
            y_sat = SAT_MAX;
        else if (y_descaled < signed'((ACC_W-SCALING_FACTOR)'(SAT_MIN)))
            y_sat = SAT_MIN;
    end

    assign y_out = y_sat;

endmodule