module dft_clean_module (
    // Functional Interfaces
    input  wire        clk,     // Clock Muxing has already done by RTL engineer on top level  e.g. assign clk = scan_mode ? scan_clk : pll_clk;
    input  wire        rst_n,       
    input  wire        mem_req,     
    input  wire        mem_we,      
    input  wire [2:0]  addr,        
    input  wire        data_in,
    output wire        data_out,

    // DFT Interfaces
    input  wire        scan_mode,   // 1: Shift/Test mode, 0: Normal mode
    input  wire        scan_rst_n   // External test reset from ATE
); 

    // =========================================================================
    // 1. Reset Synchronizer (Managed by RTL Engineer with async_reg)
    // =========================================================================
    (* async_reg = "true" *) reg rst_buff;
    (* async_reg = "true" *) reg rst_out_n;
    wire rst_mux_n;

    assign rst_mux_n = scan_mode ? scan_rst_n : rst_n;

    always_ff @(posedge clk or negedge rst_mux_n) begin
        if (~rst_mux_n) begin
            rst_buff  <= 1'b0;
            rst_out_n <= 1'b0;
        end else begin
            rst_buff  <= 1'b1;
            rst_out_n <= rst_buff;
        end
    end

    // =========================================================================
    // 2. Addressable Memory Cell & Isolation (Managed by RTL Engineer)
    // =========================================================================
    wire mem_q; 
    
    MOCK_ADDRESSABLE_RAM u_ram (
        .CLK  (clk),
        .REQ  (mem_req),
        .WE   (mem_we),
        .ADDR (addr),
        .D    (data_in),
        .Q    (mem_q)
    );

    // isolate RAM's output x in scan_mode so it won't corrupt the tested logic (RAM has built-in MBIST to test itself   )
    wire dft_mem_q;
    assign dft_mem_q = scan_mode ? 1'b0 : mem_q;

    // =========================================================================
    // 3. Main Logic Register (EDA tool will auto-replace this with Scan-FF)
    // =========================================================================
    reg q_reg;
    
    always_ff @(posedge clk or negedge rst_out_n) begin
        if (~rst_out_n) 
            q_reg <= 1'b0;
        else            
            q_reg <= dft_mem_q;
    end

    assign data_out = q_reg;

endmodule
