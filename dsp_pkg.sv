package dsp_pkg;

    // ---- Audio Data Path ----
    localparam DATA_WIDTH        = 16;     // I2S PCM word length (bits)
    localparam NUM_CHANNELS      = 4;      // Number of ADC channels

    // ---- Calibration ----
    localparam CAL_SAMPLES       = 16;     // DC offset calibration: number of samples to average

    // ---- FIFO Depths ----
    localparam ASYNC_FIFO_DEPTH  = 32;     // CDC FIFO between I2S and sys_clk domains
    localparam SYNC_FIFO_DEPTH   = 32;     // Post-DSP buffering FIFO for DMA bursts

    // ---- AXI4 / DMA ----
    localparam DMA_DATA_WIDTH    = 32;     // AXI write-data bus width
    localparam DMA_ADDR_WIDTH    = 32;     // AXI address bus width
    localparam AXI_BURST_LEN     = 8'h03;  // 4-beat burst (AWLEN encoding)
    localparam AXI_BURST_SIZE    = 3'b010;  // 4 bytes per beat

    // ---- Biquad IIR Filter (Q14 Fixed-Point) ----
    // 2nd-order Butterworth LPF: fs = 48 kHz, fc = 3 kHz
    // Coefficients scaled by 2^14 = 16384
    localparam BIQUAD_SCALE      = 14;
    localparam signed [15:0] BIQUAD_B0 = 16'sd551;     //  0.0336 × 16384
    localparam signed [15:0] BIQUAD_B1 = 16'sd1101;    //  0.0672 × 16384
    localparam signed [15:0] BIQUAD_B2 = 16'sd551;     //  0.0336 × 16384
    localparam signed [15:0] BIQUAD_A1 = -16'sd24959;  // -1.5234 × 16384
    localparam signed [15:0] BIQUAD_A2 = 16'sd10777;   //  0.6578 × 16384

endpackage
