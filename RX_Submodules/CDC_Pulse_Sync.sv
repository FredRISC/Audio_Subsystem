module CDC_Pulse_Sync(
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