`timescale 1ps/1ps

/*

Design Scope & System Assumptions Input (ADC Domain): 
    Driven by a slow, asynchronous clock clk_adc (12.288MHz).
    We assume the ADC macro encapsulates the Delta-Sigma modulator and CIC filter, providing clean 48kHz 16-bit PCM
    At fixed intervals, the ADC module asserts adc_valid and sends in 16-bit audio data.
Clock Domain Crossing & Buffering (CDC Bridge): 
    Uses an Asynchronous FIFO. Write side: Driven by clk_adc, writes when adc_valid is high.
Read side: 
    Driven by the fast system main clock clk_sys (50 MHz).
DSP Core (Filter Domain): 
    Runs on clk_sys. When it detects the FIFO is not empty, it initiates a read and performs Direct Form I IIR low-pass filtering.
Output (Destination): 
    The filtered data, along with a valid signal, is packed into an industry-standard AXI-Stream (TDATA/TVALID/TREADY) interface, ready to be sent to an on-chip DMA or audio processor.


                   [ FAST CLOCK DOMAIN ]                   │     [ SLOW CLOCK DOMAIN ]
                                                           │
┌──────────────┐      ┌─────────────┐                      │      ┌───────────┐      ┌────────────┐      ┌────────────┐
│ Analog Input │ PDM  │ INTEGRATORS │ 16-bit (Bit Grown)   │48kHz │   COMBS   │16-bit│ Async FIFO │      │  DSP Core  │
│  @ 6MHz Clk  │─────>│ (Accumulate)│─────────────────────>┼─────>│ (Subtract)│─────>│ (Storage)  │─────>│   CPU end  │─────> AXIS DMA
└──────────────┘      └─────────────┘                      │      └───────────┘ PCM  └────────────┘      └────────────┘
                             │                             │            │                  │                    │
                      [Driven by 6MHz]                     │     [Driven by 48kHz]  [Driven by 48kHz]    [Driven by 50MHz]
                                                           │
                                                   DOWNSAMPLER / CDC
                                                (Sample 1 out of 125)
*/

import dsp_pkg::*;

module async_fifo #(
) 
(
    input write_clk,
    input read_clk,
    input write_rst_n,
    input read_rst_n,

    output write_ready_out,
    output read_ready_out,

    // Write Interface
    input  logic write_valid_in,
    input  logic [DATA_WIDTH-1:0] write_data_in,

    // Read Interface
    input  logic read_valid_in,
    output logic [DATA_WIDTH-1:0] read_data_out
);

    localparam ADDR_WIDTH = $clog2(FIFO_DEPTH);

    // Internal FIFO MEMORY
    logic [DATA_WIDTH-1:0] FIFO [FIFO_DEPTH-1:0];

    // Pointers with an extra MSB bit for full/empty conditions
    logic [ADDR_WIDTH:0] wptr_bin, wptr_gray;
    logic [ADDR_WIDTH:0] rptr_bin, rptr_gray;
    assign wptr_gray = (wptr_bin) ^ (wptr_bin >> 1);
    assign rptr_gray = (rptr_bin) ^ (rptr_bin >> 1);

    // Synchronized pointers
    logic [ADDR_WIDTH:0] wptr_gray_sync1, wptr_gray_sync2;
    logic [ADDR_WIDTH:0] rptr_gray_sync1, rptr_gray_sync2;

    logic write_handshake, read_handshake;

    assign write_handshake = write_valid_in && write_ready_out; // write_ready_out is asserted when FIFO is deemed full
    assign read_handshake  = read_valid_in && read_ready_out;   //

    // ---------------------------------------------------------
    // WRITE DOMAIN
    // ---------------------------------------------------------
    always_ff @(posedge write_clk or negedge write_rst_n) begin : FIFO_WRITE_PTR
        if (!write_rst_n) begin
            wptr_bin  <= '0;
        end 
        else if (write_handshake) begin
            wptr_bin  <= wptr_bin + 1'b1;
            FIFO[wptr_bin[ADDR_WIDTH-1:0]] <= write_data_in;
        end
    end


    // Synchronize Read Pointer to Write Domain
    always_ff @(posedge write_clk or negedge write_rst_n) begin : READ_PTR_SYNC
        if (!write_rst_n) begin
            rptr_gray_sync1 <= '0;
            rptr_gray_sync2 <= '0;
        end 
        else begin
            rptr_gray_sync1 <= rptr_gray;
            rptr_gray_sync2 <= rptr_gray_sync1;
        end
    end

    // Full Flag Logic (MSB & 2nd MSB inverted, others matching, because in binary only MSB is different)
    // Pessimistic Full, a read ptr advance takes two cycle to reflect on write side
    logic FIFO_FULL;
    assign FIFO_FULL = (wptr_gray == {~rptr_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1], rptr_gray_sync2[ADDR_WIDTH-2:0]});
    assign write_ready_out = !FIFO_FULL;

    // ---------------------------------------------------------
    // READ DOMAIN
    // ---------------------------------------------------------
    always_ff @(posedge read_clk or negedge read_rst_n) begin : FIFO_READ_PTR
        if (!read_rst_n) begin
            rptr_bin  <= '0;
        end 
        else if (read_handshake) begin
            rptr_bin  <= rptr_bin + 1'b1;
        end
    end

    // Read Data Output (First-Word Fall-Through / FWFT)
    // Data is combinationally available as long as the FIFO is not empty.
    // BRAM cannot be inferred with this approach, but for small depth like 100 it synthesizes 
    // efficiently into Distributed RAM (LUTs/Registers).
    assign read_data_out = FIFO[rptr_bin[ADDR_WIDTH-1:0]];

    // Synchronize Write Pointer to Read Domain
    always_ff @(posedge read_clk or negedge read_rst_n) begin : WRITE_PTR_SYNC
        if (!read_rst_n) begin
            wptr_gray_sync1 <= '0;
            wptr_gray_sync2 <= '0;
        end else begin
            wptr_gray_sync1 <= wptr_gray;
            wptr_gray_sync2 <= wptr_gray_sync1;
        end
    end

    // Empty Flag Logic (Pointers match exactly)
    logic FIFO_EMPTY;
    assign FIFO_EMPTY = (rptr_gray == wptr_gray_sync2);
    assign read_ready_out = ~FIFO_EMPTY;
endmodule


module DSP_Subsystem #(
) (
    input sys_clk,
    input adc_cic_clk,
    input rst_n,
    // AXI-Stream Interface (Host/PC/AXI DMA)
    output logic [DATA_WIDTH-1:0] m_axis_tdata_out,
    output logic m_axis_tlast,
    output logic m_axis_tvalid,
    input  logic m_axis_tready,

    // ADC / I2S Parallel Input Interface
    input  logic [DATA_WIDTH-1:0] adc_data_in,
    input  logic adc_valid_in,
    output logic adc_ready_out

);
    
// Generate synchronous reset_n
// Asynchronous assertion - Synchronous deassertion
logic sys_clk_deassert_rst_n;
logic sys_clk_rst_n;
always_ff @(posedge sys_clk, negedge rst_n) begin
    if(~rst_n) begin
        sys_clk_deassert_rst_n   <= 1'b0;
        sys_clk_rst_n            <= 1'b0;  // reset the circuit
    end
    else begin
        sys_clk_deassert_rst_n   <= 1'b1;                       // On sys_clk posedge, if rst_n gets released simiultaneouslys, this creates a metastable state 
        sys_clk_rst_n            <= sys_clk_deassert_rst_n;     // output reset synchronized with sys_clk. 
    end
end

logic adc_cic_clk_rst_n;
logic adc_cic_deassert_rst_n;
always_ff @(posedge adc_cic_clk, negedge rst_n) begin
    if(~rst_n) begin
        adc_cic_deassert_rst_n <= 1'b0;
        adc_cic_clk_rst_n      <= 1'b0;     // reset the circuit
    end
    else begin
        adc_cic_deassert_rst_n <= 1'b1;
        adc_cic_clk_rst_n      <= adc_cic_deassert_rst_n;        
    end
end


logic [DATA_WIDTH-1:0] afifo_read_data;
logic afifo_read_ready;
logic afifo_read_valid;

assign adc_ready_out = afifo_write_ready;
logic afifo_write_ready; // Driven by async_fifo

logic [DATA_WIDTH-1:0] y_out_1;
logic y_valid_out_1;
logic [DATA_WIDTH-1:0] y_out_2;
logic y_valid_out_2;

logic bq2_ready; // Backpressure from Biquad 2 to Biquad 1
logic axis_ready; // Backpressure from AXIS formatter to Biquad 2


Biquad_IIR_Filter Biquad_IIR_Filter_1(
    .sys_clk(sys_clk),
    .rst_n(sys_clk_rst_n),
    .read_data_in(afifo_read_data),
    .read_ready_in(afifo_read_ready),
    .read_valid_out(afifo_read_valid),
    .y_out(y_out_1),
    .y_valid_out(y_valid_out_1),
    .y_ready_in(bq2_ready)
);

Biquad_IIR_Filter Biquad_IIR_Filter_2(
    .sys_clk(sys_clk),
    .rst_n(sys_clk_rst_n),
    .read_data_in(y_out_1),
    .read_ready_in(y_valid_out_1),
    .read_valid_out(bq2_ready),
    .y_out(y_out_2),
    .y_valid_out(y_valid_out_2),
    .y_ready_in(axis_ready)
);

async_fifo async_fifo_inst(
    .write_clk(adc_cic_clk),
    .read_clk(sys_clk),
    .write_rst_n(adc_cic_clk_rst_n),
    .read_rst_n(sys_clk_rst_n),
    .write_ready_out(afifo_write_ready),
    .read_ready_out(afifo_read_ready),        
    .write_valid_in(adc_valid_in),
    .write_data_in(adc_data_in),
    .read_valid_in(afifo_read_valid),
    .read_data_out(afifo_read_data)
);


// AXIS Interface
logic [$clog2(BURST_LENGTH)-1:0] m_axis_burst_counter;
logic m_axis_handshake;
assign m_axis_handshake = m_axis_tready && m_axis_tvalid;
assign axis_stall = m_axis_tvalid && !m_axis_tready;

always_ff @(posedge sys_clk, negedge rst_n) begin : M_AXIS_INTERFACE
    if(~sys_clk_rst_n) begin
        m_axis_burst_counter <= '0;
        m_axis_tvalid <= 1'b0;
        m_axis_tdata_out <= '0;
        m_axis_tlast <= 1'b0;
    end
    else begin
        // Clear TVALID upon successful handshake
        if(m_axis_handshake) begin
            m_axis_tvalid <= 1'b0;
            
            // Advance the burst counter on handshake
            if (m_axis_burst_counter == BURST_LENGTH - 1) begin
                m_axis_burst_counter <= '0;
            end 
            else begin
                m_axis_burst_counter <= m_axis_burst_counter + 1'b1;
            end
        end

        // If new valid data exits the pipeline and we aren't stalled
        if (y_valid_out_2 && !axis_stall) begin
            m_axis_tvalid    <= 1'b1;
            m_axis_tdata_out <= y_out_2;
        end

        m_axis_tlast <= (m_axis_burst_counter == BURST_LENGTH - 1);
    end
end

endmodule