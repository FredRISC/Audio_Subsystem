`timescale 1ns/1ps

/*
The System Architecture

Imagine this subsystem sitting between 4 High-Speed ADCs and the main SoC DDR Memory Interconnect.

    Clocking & Reset (The Foundation)

        RDC (Reset Domain Crossing): The external asynchronous pad_rst_n arrives and is safely synchronized into two separate reset trees: sys_rst_n and adc_rst_n.

        Clock Divider: A high-speed 500 MHz PLL clock is divided down. We need a Divide-by-5 circuit with a 50% duty cycle to generate the 100 MHz sys_clk for the AXI bus.

    Data Ingestion & CDC (The Analog Boundary)

        4x I2S Receivers: Four independent interfaces capture serialized audio from the ADCs.

        4x Async FIFOs: The deserialized data is pushed into four independent Asynchronous FIFOs to cross safely from the ADC clock domains into the sys_clk domain.

    Arbitration (The Traffic Cop)

        Round-Robin Arbiter: The 4 FIFOs will assert a not_empty request flag. The arbiter uses a Round-Robin scheme to grant access to one channel at a time, preventing any single high-speed ADC from starving the others.

    The AXI4 DMA Engine (The SoC Interface)

        The DMA engine takes the granted audio sample and formats it into an AXI4 Memory-Mapped transaction.

        It manages the AW (Address Write) channel to specify the DDR memory destination, the W (Write Data) channel to burst the audio payload, and the B (Write Response) channel to confirm the data was stored safely.
*/




// This is to ensure the priorities of activation, each submodule need to ensure its reset is synced to its clock domain 
module reset_sequencer (
    input  logic pll_clk,        
    input  logic rst_n,          // global asynchronous reset
    output logic rst_power_n,    // 1st priority deassertion (maybe not used)
    output logic rst_bus_n,      // 2nd priority deassertion
    output logic rst_core_n,     // 3rd priority deassertion
    output logic rst_adc_n       // maybe not used if we simulate adc
);
    
    // synchronizing deassertion of rst_n to clk
    (* async_reg = "true" *) logic rst_n_sync1, rst_n_sync2;
    
    always_ff @(posedge pll_clk or negedge rst_n) begin : SYNC_PLL_CLK_DEASSERTION
        if (~rst_n) begin
            rst_n_sync1 <= 1'b0;
            rst_n_sync2 <= 1'b0;
        end else begin
            rst_n_sync1 <= 1'b1;
            rst_n_sync2 <= rst_n_sync1; // all reset deassertions are synced to pll_clk domain
        end
    end

    // Although all reset deassertions are synced to pll_clk domain, we prioritize their deassertions (activations) for safety
    // IMPORTANT: Every below reset's deassertion need to be synced to its corresponding clock domain (e.g. rst_adc_n's deassertion need to be synced to adc_clk)
    logic [3:0] delay_cnt;
    always_ff @(posedge pll_clk or negedge rst_n_sync2) begin : HIERARCHICAL_DEASSERTIONS
        if (~rst_n_sync2) begin
            delay_cnt   <= 4'd0;
            rst_power_n <= 1'b0;
            rst_bus_n   <= 1'b0;
            rst_core_n  <= 1'b0;
            rst_adc_n   <= 1'b0;
        end 
        else begin
            if (delay_cnt < 4'd15) begin
                delay_cnt <= delay_cnt + 4'd1;
            end
            // phase 1: Power module reset deasserts immediately one cycle after the rst_n_sync2 deasserts
            if (delay_cnt == '0) begin
                rst_power_n <= 1'b1;
            end

            // phase 2: 4 cycles later，deassert Bus reset
            if (delay_cnt >= 'd4) begin
                rst_bus_n <= 1'b1;
            end

            // phase 3: 2 more cycles later, deassert Core reset
            if (delay_cnt >= 'd6) begin
                rst_core_n <= 1'b1;
            end

            // phase 4: 6 more cycles later, deassert ADC reset
            if (delay_cnt >= 'd12) begin
                rst_adc_n <= 1'b1; // this is currently sync to pll_clk domain
                // adc need to do asynchronous reset synchronous deassertion again to sync the rst to adc_clk (let adc_clk control the deassetion)
            end
        end
    end

endmodule


module freq_divider_5 (
    input pll_clk,
    input rst_pll_n, // we can connect this to rst_power_n here
    output sys_clk   // divided-by-5 clk
);

logic [2:0] counter;
logic pos_pulse, neg_pulse;
assign sys_clk = pos_pulse | neg_pulse; // OR solution

always_ff @(posedge pll_clk or negedge rst_pll_n) begin
    if(~rst_pll_n) begin
        counter <= 'd0;
        pos_pulse <= 1'b0;
    end
    else begin
        pos_pulse <= 1'b0;
        counter <= counter + 'd1;
        if(counter < 'd2) begin // raise the pulse for two whole cycles
            pos_pulse <= 1'b1;
        end
        if(counter == 'd4) begin
            counter <= 'd0;
        end
        /* Alternative AND solution
            if(counter == 0 || counter == 3) begin
                pos_pulse <= ~pos_pulse;
            end
        */
        /* Alternative XOR solution
            if(counter == 0) begin
                pos_pulse <= ~pos_pulse;
            end
        */
    end
end

// OR solution
always_ff @(negedge pll_clk or negedge rst_pll_n) begin
    if(~rst_pll_n) begin
        neg_pulse <= 1'b0;
    end
    else begin
        neg_pulse <= pos_pulse; // Delay pos_pulse by half a cycle to extend high time to 2.5 cycles (50% duty cycle)
    end
end

/* Alternative AND solution
always_ff @(negedge pll_clk or negedge rst_pll_n) begin
    if(~rst_n) begin
        neg_pulse <= 1'b0;
    end
    else begin
        neg_pulse <= ~neg_pulse;
    end
end
assign sys_clk = pos_pulse & neg_pulse;
*/

/* Alternative XOR solution
always_ff @(negedge pll_clk or negedge rst_pll_n) begin
    if(~rst_n) begin
        neg_pulse <= 1'b0;
    end
    else if(counter == 2) begin
        neg_pulse <= ~neg_pulse;
    end
end
assign sys_clk = pos_pulse ^ neg_pulse;
*/


endmodule


/* I2S_DESERIALIZER
Even if we are transmitting 16-bit audio, the system might allocate a 32-bit slot for each channel. 
In that case, WS stays low for 32 clock cycles (16 bits of real data + 16 bits of padded zeros) before toggling.
This is because chip noweadays (like ARM Cortex-M, DSP, DMA, register) often have 32-bit or wider data buses, and it's more efficient to align audio samples to these wider buses.
and that the design will be compatible with a wider range of audio formats (like 24-bit or 32-bit audio) without needing to change the deserializer logic. 
*/
module I2S_DESERIALIZER #(
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


// ADC DC Offset Calibration FSM - Task: Design a Moore FSM that controls a simple DC offset calibration sequence for an ADC
module ADC_DC_OFFSET_CALIBRATION #(
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


// Round-Robin Arbiter - selecting the FIFO whose ID is closest to the round_robin_ptr and has at least four words to read (Four_Available_out >= 4) 
module fifo_arbiter (
    input sys_clk,
    input rst_dma_n,
    input logic [3:0] read_burst_ready_in,
    input fifo_grant_valid,  // DMA tell fifo_arbiter to start choosing a fifo
    output fifo_grant_ready, // arbiter has chosen a fifo
    output [1:0] fifo_grant_id // totally 4 fifo candidates
);

logic [1:0] round_robin_ptr;

// Combinational Priority Logic (Finds the first ready FIFO starting from the pointer)
logic [1:0] next_grant_id;
logic any_grant;
always_comb begin
    any_grant = 1'b1;
    if      (read_burst_ready_in[round_robin_ptr])         next_grant_id = round_robin_ptr;
    else if (read_burst_ready_in[2'(round_robin_ptr+1)])   next_grant_id = 2'(round_robin_ptr+1);
    else if (read_burst_ready_in[2'(round_robin_ptr+2)])   next_grant_id = 2'(round_robin_ptr+2);
    else if (read_burst_ready_in[2'(round_robin_ptr+3)])   next_grant_id = 2'(round_robin_ptr+3);
    else begin
        any_grant = 1'b0;
        next_grant_id = round_robin_ptr;
    end
end

always_ff @(posedge sys_clk or negedge rst_dma_n) begin // rst_dma_n has already been synced to sys_clk
    if(~rst_dma_n) begin
        round_robin_ptr  <= 'd0;
        fifo_grant_ready <= 1'b0;
    end
    else if(fifo_grant_valid) begin
        if(~fifo_grant_ready && any_grant) begin
            fifo_grant_ready <= 1'b1;
            fifo_grant_id <= next_grant_id;
            round_robin_ptr <= next_grant_id + 2'd1; // Advance pointer for fairness
        end
        else begin
            fifo_grant_ready <= 1'b0;
        end
    end
end

endmodule


module AXI4_DMA_MASTER (
    input sys_clk,
    input rst_core_n,
    input logic [3:0] read_burst_ready_in, // from each fifos to arbiter
    input logic [3:0] fifo_read_ready_in,  // read_ready
    output logic [3:0] fifo_read_valid_out, // read_valid
    input logic [31:0] fifo_read_data_in [3:0],

    // AXI4 Master interface
    // AW channel
    input AWREADY,
    output logic AWVALID,
    output logic [3:0]  AWID,    
    output logic [31:0] AWADDR,
    output logic [7:0]  AWLEN,  // Assume a 4-beat burst (AWLEN = 3).
    output logic [31:0] AWSIZE, // beat size = 32 bits

    // W channel
    input WREADY,
    output logic WVALID,
    output logic [31:0] WDATA,
    output logic WLAST,

    // B channel
    input [3:0] BID,
    input BVALID,
    output logic BREADY,
    input [1:0] BRESP, // with 2'b10 (SLVERR) or 2'b11 (DECERR), DMA will trigger interrupt so CPU deal with it

    // Error handling
    output logic ERR_ID_mismatch,
    output logic [1:0] ERR_type,
    input ERR_release
);

logic rst_n_sync1, rst_n_sync2;
always_ff @(posedge sys_clk or negedge rst_core_n) begin
    if(~rst_core_n) begin // async asssetion controlled global reset
        rst_n_sync1 <= 1'b0;
        rst_n_sync2 <= 1'b0;
    end
    else begin // 2ff-synchronizer syncing the deassertion to sys_clk
        rst_n_sync1 <= 1'b1;
        rst_n_sync2 <= rst_n_sync1;
    end
end


// FIFO Arbiter
logic fifo_grant_valid, fifo_grant_ready;
logic [1:0] fifo_grant_id;
fifo_arbiter arbiter(
    .sys_clk(sys_clk),
    .rst_dma_n(rst_n_sync2),
    .read_burst_ready_in(read_burst_ready_in),
    .fifo_grant_valid(fifo_grant_valid),
    .fifo_grant_ready(fifo_grant_ready),
    .fifo_grant_id(fifo_grant_id)
);

logic [1:0] axi_counter;

logic AW_handshake, W_handshake, B_handshake;
assign AW_handshake = AWREADY & AWVALID;
assign W_handshake = WREADY & WVALID;
assign B_handshake = BREADY & BVALID;

typedef enum logic[2:0] {
    DMA_IDLE,
    DMA_AW,
    DMA_W,
    DMA_B,
    DMA_ERR
} DMA_FSM_t;
DMA_FSM_t state, next_state;

always_comb begin : NEXT_STATE
    next_state = state;
    fifo_grant_valid = 1'b0;
    fifo_read_valid_out = '{default: '0};
    case(state)
        DMA_IDLE: begin
            fifo_grant_valid = 1'b1; // start arbitrating
            if(fifo_grant_ready) begin
                next_state = DMA_AW;
            end
        end
        DMA_AW: begin
            if(AW_handshake) begin
                next_state = DMA_W;
            end
        end
        DMA_W: begin
            // Directly map WREADY handshake to FIFO pop (FWFT behavior); asserting valid is only for advancing fifo read pointer
            // We assert valid at this moment because W_handshake means the consumer has consumed the WDATA
            fifo_read_valid_out[fifo_grant_id] = W_handshake && fifo_read_ready_in[fifo_grant_id];
            if(W_handshake) begin
                if(axi_counter == 2'd3) begin
                    next_state = DMA_B;
                end
            end
        end
        DMA_B: begin
            if(B_handshake) begin
                if(BRESP != 2'b00 || (BID != AWID)) begin
                    next_state = DMA_ERR;
                end 
                else begin
                    next_state = DMA_IDLE;
                end
            end
        end
        DMA_ERR: begin
            if(ERR_release) begin
                next_state = DMA_IDLE;
            end
        end
    endcase
end

logic [29:0] waddr_ptr [3:0]; // RAM write address of each calibrated ADC data

always_ff @(posedge sys_clk or negedge rst_n_sync2) begin : STATE
    if(~rst_n_sync2) begin
        state <= DMA_IDLE;
        axi_counter <= '0;
        AWVALID <= 1'b0;
        AWADDR <= '0;
        AWID <= 4'b0;
        AWSIZE <= '0;
        AWLEN <= '0;
        WVALID  <= 1'b0;
        WLAST <= 1'b0;
        ERR_ID_mismatch <= 1'b0;
        ERR_type <= 2'b00;
        BREADY <= 1'b0;
        waddr_ptr <= '{default: '0};
    end
    else begin
        state <= next_state;
        AWLEN <= 8'h03; // fixed burst length (4 beats)
        AWSIZE <= 3'b010; // fixed beat size (4 bytes)

        case(state)
            DMA_IDLE: begin
                AWVALID <= 1'b0;
                WVALID  <= 1'b0;
                BREADY  <= 1'b0;
                axi_counter <= '0;
                WLAST <= 1'b0;
                if(fifo_grant_ready) begin
                    AWVALID <= 1'b1;
                    AWADDR  <= {fifo_grant_id, waddr_ptr[fifo_grant_id]};
                    AWID    <= {2'b00, fifo_grant_id};
                end
            end
            DMA_AW: begin
                if(AW_handshake) begin
                    AWVALID <= 1'b0;
                    WVALID <= 1'b1;
                end
            end
            DMA_W: begin
                if(W_handshake) begin
                    axi_counter <= axi_counter + 2'd1;
                    if(axi_counter == 2'd2) begin
                        WLAST <= 1'b1;
                    end
                    if(axi_counter == 2'd3) begin
                        WVALID <= 1'b0;
                        WLAST  <= 1'b0;
                        BREADY <= 1'b1;
                        waddr_ptr[fifo_grant_id] <= waddr_ptr[fifo_grant_id] + 30'd16; // Increment by 16 bytes (four 32-bit data from fifo) for next burst
                    end
                end
            end
            DMA_B: begin
                if(B_handshake) begin
                    BREADY <= 1'b0;
                    ERR_type <= BRESP;                  // CPU will check if ERR_type is non-zero and if ERR_ID_mismatch is raised
                    if(BID != AWID) ERR_ID_mismatch <= 1'b1;
                end
            end
            DMA_ERR: begin
                if(ERR_release) begin
                    ERR_ID_mismatch <= 1'b0;
                end
            end
        endcase
    end
end

assign WDATA = fifo_read_data_in[fifo_grant_id]; // FWFT FIFO data is already ready, otherwise we need fifo_read_handshake

endmodule