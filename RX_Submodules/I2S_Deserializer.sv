/* I2S_DESERIALIZER
Even if we are transmitting 16-bit audio, the system might allocate a 32-bit slot for each channel. 
In that case, WS stays low for 32 clock cycles (16 bits of real data + 16 bits of padded zeros) before toggling.
This is because chip noweadays (like ARM Cortex-M, DSP, DMA, register) often have 32-bit or wider data buses, and it's more efficient to align audio samples to these wider buses.
and that the design will be compatible with a wider range of audio formats (like 24-bit or 32-bit audio) without needing to change the deserializer logic. 
*/
module I2S_Deserializer #(
    parameter SYSTEM_BUS_WIDTH = 32,
    parameter WORD_LENGTH = 16
)( 
    // ADC to DSP interface - I2S receiver
    input i2s_bclk,                         // I2S Bit Clock
    input rst_n,
    input i2s_ws,                           // word select, left channel when ws=0, right channel when ws=1
    input i2s_sdata,                        // serial data input

    output logic [SYSTEM_BUS_WIDTH-1:0] left_channel_data,  // even though WORD_LENGTH is 16, we pad zeros to fill 32-bit slot for better compatibility with wider audio formats and efficient bus utilization
    output logic left_valid,
    output logic [SYSTEM_BUS_WIDTH-1:0] right_channel_data, 
    output logic right_valid
);
localparam ZERO_PADDING = SYSTEM_BUS_WIDTH - WORD_LENGTH;

typedef enum logic {
    IDLE,
    ACTIVE
} FSM_t;
FSM_t FSM_inst;

FSM_t state, next_state;
logic [$clog2(WORD_LENGTH)-1:0] bit_counter;
logic latched_ws;
always_comb begin : NEXT_STATE
    next_state = state;
    case(state)
        IDLE: begin
            if(i2s_ws != latched_ws) next_state = ACTIVE;
        end

        ACTIVE: begin
            if(bit_counter == 'd15) begin
                next_state = IDLE;
            end
        end
    endcase
end


logic [WORD_LENGTH-1:0] channel_word_length_out;
logic channel_ready_flag;
logic [SYSTEM_BUS_WIDTH-1:0] channel_system_bus_width_out;
assign channel_system_bus_width_out = {(ZERO_PADDING){1'b0}, channel_word_length_out};
always_ff @(posedge i2s_bclk or negedge rst_n) begin
    if(~rst_n) begin
        state <= IDLE;
        bit_counter <= '0;
        latched_ws <= 1'b1;
        channel_ready_flag <= 1'b0;
        channel_word_length_out <= '0;
        left_valid <= 1'b0;
        right_valid <= 1'b0;
    end
    else begin
        state <= next_state;
        latched_ws <= i2s_ws;
        channel_ready_flag <= 1'b0;
        left_valid <= 1'b0;
        right_valid <= 1'b0;

        if(state == ACTIVE) begin
            channel_word_length_out[WORD_LENGTH-1-bit_counter] <= i2s_sdata;
            bit_counter <= bit_counter + 'd1; // saturate and go to zero after receiving 16 samples
            if(bit_counter == 'd15) begin
                channel_ready_flag <= 1'b1;           
            end
        end

        if(channel_ready_flag) begin
            if(latched_ws) begin
                right_channel_data <= channel_system_bus_width_out;
                right_valid <= 1'b1;
            end
            else begin
                left_channel_data <= channel_system_bus_width_out;
                left_valid <= 1'b1;
            end
        end
    end
end

endmodule
