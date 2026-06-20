// This is to ensure the priorities of activation, each submodule need to ensure its reset is synced to its clock domain 
module Reset_Sequencer (
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