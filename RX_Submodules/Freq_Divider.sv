module Freq_Divider #(
    parameter N = 5
) (
    input pll_clk,
    input rst_pll_n, // we can connect this to rst_power_n here
    output sys_clk   // divided-by-N clk
);

logic [2:0] counter;
logic pos_pulse, neg_pulse;

always_ff @(posedge pll_clk or negedge rst_pll_n) begin
    if(~rst_pll_n) begin
        counter <= 'd0;
        pos_pulse <= 1'b0;
    end
    else begin
        pos_pulse <= 1'b0;
        counter <= counter + 'd1;
        if(counter < 3'(N/2)) begin // raise the pulse for floor(N/2) whole cycles
            pos_pulse <= 1'b1;
        end
        if(counter == 3'(N-1)) begin
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
        neg_pulse <= pos_pulse; // Delay pos_pulse by half a cycle to extend high time to N/2 cycles (50% duty cycle)
    end
end
assign sys_clk = pos_pulse | neg_pulse; // OR solution


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
