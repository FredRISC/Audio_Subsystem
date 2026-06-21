`timescale 1ns/1ps

// Round-Robin Arbiter — selects the Sync FIFO whose burst-ready flag
// is asserted, starting from a rotating pointer for fairness.
module Fifo_Arbiter #(
    parameter NUM_CHANNELS = 4
)(
    input  sys_clk,
    input  rst_n,
    input  logic [NUM_CHANNELS-1:0]              burst_ready_in,
    input  logic                                  grant_request,
    output logic                                  grant_valid,
    output logic [$clog2(NUM_CHANNELS)-1:0]       grant_id
);

    localparam ID_W = $clog2(NUM_CHANNELS);
    logic [ID_W-1:0] rr_ptr;
    logic [ID_W-1:0] next_id;
    logic             any_ready;

    always_comb begin
        any_ready = 1'b0;
        next_id   = rr_ptr;
        for (int j = 0; j < NUM_CHANNELS; j++) begin
            logic [ID_W-1:0] idx = ID_W'(rr_ptr + ID_W'(j));
            if (burst_ready_in[idx]) begin
                any_ready = 1'b1;
                next_id   = idx;
                break;
            end
        end
    end

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (~rst_n) begin
            rr_ptr      <= '0;
            grant_valid <= 1'b0;
        end else if (grant_request) begin
            if (~grant_valid && any_ready) begin
                grant_valid <= 1'b1;
                grant_id    <= next_id;
                rr_ptr      <= next_id + ID_W'(1);
            end else begin
                grant_valid <= 1'b0;
            end
        end
    end

endmodule


// AXI4 DMA Write Master — reads from arbitrated Sync FIFOs and issues
// fixed-length AXI4 burst writes to memory.
module AXI4_DMA_Master #(
    parameter AXI_BURST_LEN  = 8'h03,
    parameter AXI_BURST_SIZE = 3'b010,
    parameter DMA_DATA_WIDTH = 32,
    parameter DMA_ADDR_WIDTH = 32,
    parameter NUM_CHANNELS   = 4
)(
    input  sys_clk,
    input  rst_n,

    // Sync FIFO interface (from DSP_Channel_Processors)
    output logic [NUM_CHANNELS-1:0]                fifo_rd_valid,
    input  logic [NUM_CHANNELS-1:0]                burst_ready_in,
    input  logic [NUM_CHANNELS-1:0]                fifo_rd_ready,
    input  logic [DMA_DATA_WIDTH-1:0]              fifo_rd_data [NUM_CHANNELS-1:0],

    // AXI4 Write Address channel
    input  logic                       AWREADY,
    output logic                       AWVALID,
    output logic [3:0]                 AWID,
    output logic [DMA_ADDR_WIDTH-1:0]  AWADDR,
    output logic [7:0]                 AWLEN,
    output logic [2:0]                 AWSIZE,

    // AXI4 Write Data channel
    input  logic                       WREADY,
    output logic                       WVALID,
    output logic [DMA_DATA_WIDTH-1:0]  WDATA,
    output logic                       WLAST,

    // AXI4 Write Response channel
    input  logic [3:0]                 BID,
    input  logic                       BVALID,
    output logic                       BREADY,
    input  logic [1:0]                 BRESP,

    // Error reporting
    output logic                       ERR_ID_mismatch,
    output logic [1:0]                 ERR_type,
    input  logic                       ERR_release
);

    localparam ID_W   = $clog2(NUM_CHANNELS);
    localparam BEATS  = AXI_BURST_LEN + 1;                 // Number of data beats
    localparam ADDR_INC = BEATS * (DMA_DATA_WIDTH / 8);     // Bytes per burst

    // ---- Arbiter ----
    logic             arb_request, arb_valid;
    logic [ID_W-1:0]  arb_id;

    Fifo_Arbiter #(.NUM_CHANNELS(NUM_CHANNELS)) u_arbiter (
        .sys_clk      (sys_clk),
        .rst_n        (rst_n),
        .burst_ready_in(burst_ready_in),
        .grant_request(arb_request),
        .grant_valid  (arb_valid),
        .grant_id     (arb_id)
    );

    // ---- DMA FSM ----
    typedef enum logic [2:0] { S_IDLE, S_AW, S_W, S_B, S_ERR } dma_state_t;
    dma_state_t state, nxt;

    logic [1:0] beat_cnt;
    logic AW_hs, W_hs, B_hs;
    assign AW_hs = AWREADY & AWVALID;
    assign W_hs  = WREADY  & WVALID;
    assign B_hs  = BREADY  & BVALID;

    localparam WADDR_W = DMA_ADDR_WIDTH - ID_W;
    logic [WADDR_W-1:0] waddr_ptr [NUM_CHANNELS-1:0];

    // Next-state logic
    always_comb begin
        nxt            = state;
        arb_request    = 1'b0;
        fifo_rd_valid  = '{default: '0};
        case (state)
            S_IDLE: begin
                arb_request = 1'b1;
                if (arb_valid) nxt = S_AW;
            end
            S_AW: if (AW_hs)  nxt = S_W;
            S_W: begin
                fifo_rd_valid[arb_id] = W_hs && fifo_rd_ready[arb_id];
                if (W_hs && beat_cnt == 2'd3) nxt = S_B;
            end
            S_B: if (B_hs) nxt = (BRESP != 2'b00 || BID != AWID) ? S_ERR : S_IDLE;
            S_ERR: if (ERR_release) nxt = S_IDLE;
        endcase
    end

    // Registered outputs
    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (~rst_n) begin
            state           <= S_IDLE;
            beat_cnt        <= '0;
            AWVALID         <= 1'b0;
            AWADDR          <= '0;
            AWID            <= '0;
            AWLEN           <= '0;
            AWSIZE          <= '0;
            WVALID          <= 1'b0;
            WLAST           <= 1'b0;
            BREADY          <= 1'b0;
            ERR_ID_mismatch <= 1'b0;
            ERR_type        <= 2'b00;
            waddr_ptr       <= '{default: '0};
        end else begin
            state  <= nxt;
            AWLEN  <= AXI_BURST_LEN;
            AWSIZE <= AXI_BURST_SIZE;

            case (state)
                S_IDLE: begin
                    AWVALID  <= 1'b0;
                    WVALID   <= 1'b0;
                    BREADY   <= 1'b0;
                    beat_cnt <= '0;
                    WLAST    <= 1'b0;
                    if (arb_valid) begin
                        AWVALID <= 1'b1;
                        AWADDR  <= {arb_id, waddr_ptr[arb_id]};
                        AWID    <= {{(4-ID_W){1'b0}}, arb_id};
                    end
                end
                S_AW: if (AW_hs) begin
                    AWVALID <= 1'b0;
                    WVALID  <= 1'b1;
                end
                S_W: if (W_hs) begin
                    beat_cnt <= beat_cnt + 2'd1;
                    WLAST    <= (beat_cnt == 2'd2);
                    if (beat_cnt == 2'd3) begin
                        WVALID  <= 1'b0;
                        WLAST   <= 1'b0;
                        BREADY  <= 1'b1;
                        waddr_ptr[arb_id] <= waddr_ptr[arb_id] + WADDR_W'(ADDR_INC);
                    end
                end
                S_B: if (B_hs) begin
                    BREADY   <= 1'b0;
                    ERR_type <= BRESP;
                    if (BID != AWID) ERR_ID_mismatch <= 1'b1;
                end
                S_ERR: if (ERR_release) ERR_ID_mismatch <= 1'b0;
            endcase
        end
    end

    // FWFT data passthrough
    assign WDATA = fifo_rd_data[arb_id];

endmodule
