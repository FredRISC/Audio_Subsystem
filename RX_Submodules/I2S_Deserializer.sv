`timescale 1ns/1ps

// I2S Deserializer — captures serial I2S audio data and outputs
// parallel left/right channel words. One-bit valid pulses on
// mutually exclusive clock cycles.
module I2S_Deserializer #(
    parameter DATA_WIDTH = 16
)(
    input  i2s_bclk,
    input  rst_n,
    input  i2s_ws,       // Word select: 0 = left, 1 = right
    input  i2s_sdata,    // Serial data input

    output logic [DATA_WIDTH-1:0] left_data,
    output logic                  left_valid,
    output logic [DATA_WIDTH-1:0] right_data,
    output logic                  right_valid
);

    typedef enum logic { IDLE, ACTIVE } state_t;
    state_t state, next_state;

    logic [$clog2(DATA_WIDTH)-1:0] bit_cnt;
    logic latched_ws;
    logic [DATA_WIDTH-1:0] shift_reg;
    logic word_done;

    always_comb begin
        next_state = state;
        case (state)
            IDLE:   if (i2s_ws != latched_ws) next_state = ACTIVE;
            ACTIVE: if (bit_cnt == DATA_WIDTH - 1) next_state = IDLE;
        endcase
    end

    always_ff @(posedge i2s_bclk or negedge rst_n) begin
        if (~rst_n) begin
            state      <= IDLE;
            bit_cnt    <= '0;
            latched_ws <= 1'b1;
            word_done  <= 1'b0;
            shift_reg  <= '0;
            left_valid <= 1'b0;
            right_valid <= 1'b0;
        end else begin
            state      <= next_state;
            latched_ws <= i2s_ws;
            word_done  <= 1'b0;
            left_valid <= 1'b0;
            right_valid <= 1'b0;

            if (state == ACTIVE) begin
                shift_reg[DATA_WIDTH-1-bit_cnt] <= i2s_sdata;
                bit_cnt <= bit_cnt + 1'd1;
                if (bit_cnt == DATA_WIDTH - 1)
                    word_done <= 1'b1;
            end

            if (word_done) begin
                if (latched_ws) begin
                    right_data  <= shift_reg;
                    right_valid <= 1'b1;
                end else begin
                    left_data  <= shift_reg;
                    left_valid <= 1'b1;
                end
            end
        end
    end

endmodule
