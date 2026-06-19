/* Waveform
ClkA (快)   : __|~|______________________ (脈衝進來)
Toggle_out  : ____|~~~~~~~~~~~~~~~~~~~~~~ (變成長期電位，絕對不會漏)

ClkB (慢)   : ____|~~~~|____|~~~~|____|~~
FF1 (同步1) : _________|~~~~|____|____|__ (可能亞穩態，被隔離)
FF2 (同步2) : ______________|~~~~|____|__ (安全的電位變化)
FF3 (延遲一拍): _________________|~~~~|__ (舊的電位狀態)

XOR (FF2^FF3): ______________|~|_________ (只有一週期不同，成功還原脈衝！)


/// Another Illustration
[ Clk A 域 ]                    [ Clk B 域 ]
PulseIn ---> [Toggle FF] ----> [FF 1] ---> [FF 2] ---> [FF 3]

                                              |           |
                                              +-->(XOR)---+---> PulseOut
*/

`timescale 1ps/1ps

module pulse_synchronizer(
    input clkA,
    input clkB,
    input rst_n,
    input clkA_pulse_in,
    output clkB_pulse_out
);


// toggle FF
logic toggle_out;
always_ff @(posedge clkA, negedge rst_n) begin
    if(!rst_n) begin
        toggle_out <= 1'b0;
    end
    else if(clkA_pulse_in) begin
        toggle_out <= ~toggle_out;
    end
end

// FF1: first FF of two-ff synchronizer
(* async_reg = "true" *) logic ff1_out, ff2_out;
always_ff @(posedge clkB, negedge rst_n) begin
    if(!rst_n) begin
        ff1_out <= 1'b0;
    end
    else begin
        ff1_out <= toggle_out;
        ff2_out <= ff1_out;
    end
end


// FF3: FF for generating one-cycle pulse after toggling
logic ff3_out;
always_ff @(posedge clkB, negedge rst_n) begin
    if(!rst_n) begin
        ff3_out <= 1'b0;
    end
    else begin
        ff3_out <= ff2_out;
    end
end

logic xor_out;
assign xor_out = ff3_out ^ ff2_out;
assign clkB_pulse_out = xor_out;

endmodule