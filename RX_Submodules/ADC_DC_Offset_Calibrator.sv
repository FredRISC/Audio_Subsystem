`timescale 1ns/1ps

// ADC DC Offset Calibration — Moore FSM that averages NUMBER_OF_SAMPLE
// ground-shorted readings, then subtracts the result from live data.
module ADC_DC_Offset_Calibrator #(
    parameter NUMBER_OF_SAMPLE = 16,
    parameter DATA_WIDTH       = 16
)(
    input  clk,
    input  rst_n,
    input  start_cal,
    input  signed [DATA_WIDTH-1:0] adc_data_in,
    input  adc_data_valid_in,
    output logic  adc_mux_gnd,
    output logic  cal_done,
    output logic signed [DATA_WIDTH-1:0] calibrated_data_out,
    output logic  calibrated_data_valid_out
);

    localparam CNT_W = $clog2(NUMBER_OF_SAMPLE);
    localparam ACC_W = DATA_WIDTH + CNT_W;  // accumulator width

    typedef enum logic [1:0] { IDLE, ACCUMULATE, DIVIDE, APPLY } state_t;
    state_t state, next_state;

    logic [CNT_W-1:0]         sample_cnt;
    logic signed [ACC_W-1:0]  accumulator;
    logic signed [DATA_WIDTH-1:0] offset_reg;

    // Next-state & Moore outputs
    always_comb begin
        next_state  = state;
        adc_mux_gnd = 1'b0;
        cal_done    = 1'b0;
        case (state)
            IDLE:       if (start_cal) next_state = ACCUMULATE;
            ACCUMULATE: begin
                adc_mux_gnd = 1'b1;
                if (sample_cnt == CNT_W'(NUMBER_OF_SAMPLE - 1) && adc_data_valid_in)
                    next_state = DIVIDE;
            end
            DIVIDE:     next_state = APPLY;
            APPLY:      cal_done = 1'b1;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            state                   <= IDLE;
            sample_cnt              <= '0;
            accumulator             <= '0;
            offset_reg              <= '0;
            calibrated_data_valid_out <= 1'b0;
        end else begin
            state                   <= next_state;
            calibrated_data_valid_out <= 1'b0;

            case (state)
                IDLE: accumulator <= '0;

                ACCUMULATE:
                    if (adc_data_valid_in) begin
                        sample_cnt  <= sample_cnt + 1'd1;
                        accumulator <= accumulator + adc_data_in;
                    end

                DIVIDE:
                    offset_reg <= DATA_WIDTH'(accumulator >>> CNT_W);

                APPLY:
                    if (adc_data_valid_in) begin
                        calibrated_data_out       <= adc_data_in - offset_reg;
                        calibrated_data_valid_out <= 1'b1;
                    end
            endcase
        end
    end

endmodule