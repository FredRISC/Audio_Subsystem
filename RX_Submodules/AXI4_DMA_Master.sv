
// Round-Robin Arbiter - selecting the FIFO whose ID is closest to the round_robin_ptr and has at least four words to read (Four_Available_out >= 4) 
module fifo_arbiter #(
    parameter NUM_OF_ADCs = 4
) (
    input sys_clk,
    input rst_dma_n,
    input logic [NUM_OF_ADCs-1:0] read_burst_ready_in,
    input fifo_grant_valid,  // DMA tell fifo_arbiter to start choosing a fifo
    output fifo_grant_ready, // arbiter has chosen a fifo
    output [$clog2(NUM_OF_ADCs)-1:0] fifo_grant_id // totally 4 fifo candidates
);

logic [$clog2(NUM_OF_ADCs)-1:0] round_robin_ptr;

// Combinational Priority Logic (Finds the first ready FIFO starting from the pointer)
logic [$clog2(NUM_OF_ADCs)-1:0] next_grant_id;
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


module AXI4_DMA_Master #(
    parameter AXI_BURST_LEN = 8'h03,  // 4-beat burst
    parameter AXI_BURST_SIZE = 3'b010,
    parameter DMA_DATA_WIDTH = 32,
    parameter DMA_ADDR_WIDTH = 32,
    parameter NUM_OF_ADCs = 4
)(
    input sys_clk,
    input rst_core_n,
    input logic [NUM_OF_ADCs-1:0] read_burst_ready_in, // from each fifos to arbiter
    input logic [NUM_OF_ADCs-1:0] fifo_read_ready_in,  // read_ready
    output logic [NUM_OF_ADCs-1:0] fifo_read_valid_out, // read_valid
    input logic [DMA_DATA_WIDTH-1:0] fifo_read_data_in [NUM_OF_ADCs-1:0],

    // AXI4 Master interface
    // AW channel
    input AWREADY,
    output logic AWVALID,
    output logic [3:0]  AWID,    
    output logic [ADDR_WIDTH-1:0] AWADDR,
    output logic [7:0]  AWLEN,  // Assume a 4-beat burst (AWLEN = 3).
    output logic [2:0] AWSIZE, // beat size = 32 bits

    // W channel
    input WREADY,
    output logic WVALID,
    output logic [DMA_DATA_WIDTH-1:0] WDATA,
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
logic [$clog2(NUM_OF_ADCs)-1:0] fifo_grant_id;
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

logic [29:0] waddr_ptr [NUM_OF_ADCs-1:0]; // RAM write address of each calibrated ADC data

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
        AWLEN <= AXI_BURST_LEN; // fixed burst length (4 beats)
        AWSIZE <= AXI_BURST_SIZE; // fixed beat size (4 bytes)

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
                        waddr_ptr[fifo_grant_id] <= waddr_ptr[fifo_grant_id] + (DMA_ADDR_WIDTH-$clog2(NUM_OF_ADCs))'d16; // Increment by 16 bytes (four 32-bit data from fifo) for next burst
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






