`timescale 1ns/1ps

module Reset_Synchronizer(
    input  clk,
    input  rst_n_in,
    output rst_n_out
);

logic rst_n_sync1, rst_n_sync2;
always_ff @(posedge clk or negedge rst_n_in) begin
    if(~rst_n_in) begin
        rst_n_sync1 <= 1'b0;
        rst_n_sync2 <= 1'b0;
    end
    else begin
        rst_n_sync1 <= 1'b1;
        rst_n_sync2 <= rst_n_sync1;
    end
end

assign rst_n_out = rst_n_sync2;

endmodule