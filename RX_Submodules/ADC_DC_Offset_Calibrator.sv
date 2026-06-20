// ADC DC Offset Calibration FSM
module ADC_DC_Offset_Calibrator #(
    parameter NUMBER_OF_SAMPLE = 16
) (
    input clk,
    input rst_n,
    input start_cal,
    input signed [15:0] adc_data_in,
    input adc_data_valid_in,
    output logic adc_mux_gnd,
    output logic cal_done,
    output signed [15:0] calibrated_data_out,
    output calibrated_data_valid_out
);

typedef enum logic[1:0] {
    IDLE,           // Wait for a start_cal signal.
    ACCUMULATE,     // Tell the ADC to ground its inputs (assert adc_mux_gnd = 1). Then, capture and add together exactly 16 samples from the ADC (adc_data_in).
    DIVIDE,         // Divide the accumulated sum by 16 to find the average DC offset. (In digital logic, dividing by 16 is just a bit-shift!).
    APPLY           // Store this average in a register (offset_reg), de-assert adc_mux_gnd, and assert a cal_done flag. Return to IDLE.
} FSM_t;
FSM_t FSM_inst;

FSM_t state, next_state;
logic [3:0] sample_counter;
always_comb begin
    next_state  = state;
    adc_mux_gnd = 1'b0;
    cal_done    = 1'b0;             

    case(state)
        IDLE: begin
            if(start_cal) begin
                next_state = ACCUMULATE;
            end
        end

        ACCUMULATE: begin
            adc_mux_gnd = 1'b1; // Moore FSM, so output depends only on current state (glitch free)
            if(sample_counter == 15 && adc_data_valid_in) begin // Suppose ADC output adc_data_valid_in only when adc_mux_gnd is asserted on its clock edge
                next_state = DIVIDE;
            end
        end

        DIVIDE: begin
            // only stay for only cycle, since dividing the accumulator by 16 simply means right shifting 4 bits, which can be finished efficiently in a cycle
            next_state = APPLY;
        end

        APPLY: begin
            cal_done = 1'b1;   
        end
    endcase
end

logic signed [15:0] offset_reg;  // Register that holds the calculated average DC offset value
logic signed [19:0] accumulator; 
assign offset_reg = accumulator[15:0];

always_ff @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
        state <= IDLE;
        sample_counter <= '0;
        accumulator <= '0;
        offset_reg <= '0;
        calibrated_data_valid_out <= 1'b0;
    end
    else begin
        state <= next_state;
        calibrated_data_valid_out <= 1'b0;

        case(state)
        
            IDLE: begin
                accumulator <= '0;
            end

            ACCUMULATE: begin
                if(adc_data_valid_in) begin
                    sample_counter <= sample_counter + 'd1;   // saturate and go to zero again after hitting the 16th sample
                    accumulator <= accumulator + adc_data_in;
                end 
            end
            
            DIVIDE: begin
                accumulator <= (accumulator >>> $clog2(NUMBER_OF_SAMPLE)); // divided by 16 
            end
            
            APPLY: begin // cal_done = 1, offset_reg is ready
                if(adc_data_valid_in) begin
                    calibrated_data_out <= adc_data_in - offset_reg;
                    calibrated_data_valid_out <= 1'b1;
                end
            end

        endcase
    end
end

endmodule