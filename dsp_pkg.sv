package dsp_pkg;

localparam IS2_WORD_LENGTH       = 16;
localparam CALIBRATED_SAMPLES    = 16;
localparam ASYNC_FIFO_DEPTH      = 32;
localparam SYNC_FIFO_DEPTH       = 32;
localparam NUM_OF_ADCs           = 4;
localparam AXI_BURST_LENGTH      = 8'h03;
localparam AXI_BURST_SIZE        = 3'b010;
localparam DMA_ADDR_WIDTH        = 32;
localparam DMA_DATA_WIDTH      = 32;

/*
typedef struct packed {
    
} struct_name;
*/

endpackage

