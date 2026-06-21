# Audio SoC — Multi-Channel ADC RX Subsystem with Inline DSP

A synthesizable SystemVerilog Audio SoC subsystem that captures, calibrates, filters, and DMA-transfers multi-channel I2S audio data to external memory over AXI4.

## Architecture

```
                         ┌────────────────────────────────────────────────────────────────┐
                         │                      Audio_SoC_top                             │
                         │                                                                │
  pad_rst_n ──▶ Reset_Sequencer ──▶ Freq_Divider (÷5) ──▶ sys_clk (100 MHz)             │
  pll_clk       │                   │                                                     │
  (500 MHz)     └──▶ Reset_Synchronizer ×2  (→ rst_sys_n, → rst_bclk_n)                  │
                                                                                          │
  i2s_bclk ─────────────────────────────────────────────────────────────────┐              │
  i2s_ws[3:0] ──┐                                                          │              │
  i2s_sdata[3:0]│       ×4 generate (per ADC channel)                      │              │
                ▼       ┌───────────────────────────────────────────────┐   │              │
           I2S_Deserializer ──▶ ADC_DC_Offset_Calibrator (L)            │   │              │
                          ──▶ ADC_DC_Offset_Calibrator (R) ──┐ MUX     │   │              │
                                                              ▼         │   │              │
                         DSP_Channel_Processor                          │   │              │
                         ┌─────────────────────────────────────────┐    │   │              │
                         │ Async_FIFO ──▶ Biquad ──▶ Biquad ──▶ Sync_FIFO │              │
                         │   (CDC)        IIR #0     IIR #1    (burst) │    │              │
                         └─────────────────────────────────────────┘    │   │              │
                                                              │         │   │              │
                └───────────────────────────────────────────────────────┘   │              │
                                          │                                                │
                                          ▼                                                │
                         AXI4_DMA_Master                                                   │
                         ┌──────────────────────────────────┐                              │
                         │ Fifo_Arbiter (Round-Robin)       │                              │
                         │ AXI4 Write Engine (AW → W → B)  │──▶ DDR Memory                │
                         └──────────────────────────────────┘                              │
                         └────────────────────────────────────────────────────────────────┘
```

## Modules

| Module | File | Description |
|---|---|---|
| `Audio_SoC_top` | [`Audio_SoC_top.sv`](Audio_SoC_top.sv) | Top-level integration: clock/reset, 4× channel pipeline, DMA engine |
| `Reset_Sequencer` | [`RX_Submodules/Reset_Sequencer.sv`](RX_Submodules/Reset_Sequencer.sv) | Phased reset deassertion (power → bus → core → ADC) |
| `Reset_Synchronizer` | [`RX_Submodules/Reset_Synchronizer.sv`](RX_Submodules/Reset_Synchronizer.sv) | 2-FF async assert / sync deassert reset synchronizer |
| `Freq_Divider` | [`RX_Submodules/Freq_Divider.sv`](RX_Submodules/Freq_Divider.sv) | Odd-integer clock divider with 50% duty cycle |
| `I2S_Deserializer` | [`RX_Submodules/I2S_Deserializer.sv`](RX_Submodules/I2S_Deserializer.sv) | Serial-to-parallel I2S receiver (L/R channels) |
| `ADC_DC_Offset_Calibrator` | [`RX_Submodules/ADC_DC_Offset_Calibrator.sv`](RX_Submodules/ADC_DC_Offset_Calibrator.sv) | Moore FSM: averages ground-shorted samples, subtracts DC offset |
| `DSP_Channel_Processor` | [`RX_Submodules/DSP_Channel_Processor.sv`](RX_Submodules/DSP_Channel_Processor.sv) | Per-channel wrapper: Async FIFO → 2× cascaded biquad → Sync FIFO |
| `Biquad_IIR_Filter` | [`RX_Submodules/Biquad_IIR_Filter.sv`](RX_Submodules/Biquad_IIR_Filter.sv) | Direct Form I IIR filter, 4-stage pipeline, dual-channel state |
| `Async_FIFO` | [`RX_Submodules/Async_FIFO.sv`](RX_Submodules/Async_FIFO.sv) | Gray-code pointer CDC FIFO (FWFT), separate read/write resets |
| `Sync_FIFO` | [`RX_Submodules/Sync_FIFO.sv`](RX_Submodules/Sync_FIFO.sv) | Synchronous FWFT FIFO with burst-ready signaling for DMA |
| `Fifo_Arbiter` | [`RX_Submodules/AXI4_DMA_Master.sv`](RX_Submodules/AXI4_DMA_Master.sv) | Round-robin channel arbiter |
| `AXI4_DMA_Master` | [`RX_Submodules/AXI4_DMA_Master.sv`](RX_Submodules/AXI4_DMA_Master.sv) | AXI4 burst write engine with error handling |
| `CDC_Pulse_Sync` | [`RX_Submodules/CDC_Pulse_Sync.sv`](RX_Submodules/CDC_Pulse_Sync.sv) | Toggle-based single-pulse CDC synchronizer (utility) |

## Key Design Decisions

- **Inline DSP over DMA round-trip**: Biquad filters run inline between the CDC FIFO and the DMA output FIFO. At 48 kHz audio sample rates and 100 MHz `sys_clk`, each biquad has ~2000 clock cycles of headroom per sample. This avoids a memory round-trip and is the standard approach for fixed-function audio DSP (Cirrus Logic, TI, ADI codec architectures).

- **Dual-channel biquad state**: Left and right channels share one biquad pipeline by alternating `channel_sel` on each sample. Per-channel IIR history registers (`left_x1`, `right_y1`, etc.) prevent cross-channel contamination without duplicating hardware.

- **Clock Domain Crossing**: The `Async_FIFO` uses Gray-code pointer synchronization with separate read/write domain resets for clean CDC. The `Reset_Synchronizer` implements async-assert / sync-deassert for each domain.

- **Phased reset deassertion**: `Reset_Sequencer` deasserts power, bus, core, and ADC resets in a staggered sequence to prevent initialization race conditions.

## Default Filter Configuration

2nd-order Butterworth low-pass filter in Q14 fixed-point:

| Parameter | Value | Description |
|---|---|---|
| Sample Rate | 48 kHz | Standard audio rate |
| Cutoff Frequency | 3 kHz | -3 dB point |
| B0 | 551 | 0.0336 × 2¹⁴ |
| B1 | 1101 | 0.0672 × 2¹⁴ |
| B2 | 551 | 0.0336 × 2¹⁴ |
| A1 | -24959 | -1.5234 × 2¹⁴ |
| A2 | 10777 | 0.6578 × 2¹⁴ |

All coefficients fit in signed 16-bit. Override via module parameters for different filter responses.

## Project Structure

```
Audio_Subsystem/
├── Audio_SoC_top.sv            # Top-level integration
├── dsp_pkg.sv                  # Global parameters & constants
├── dft_clean_system.sv         # DFT scan-mode support module
├── RX_Submodules/
│   ├── Reset_Sequencer.sv
│   ├── Reset_Synchronizer.sv
│   ├── Freq_Divider.sv
│   ├── I2S_Deserializer.sv
│   ├── ADC_DC_Offset_Calibrator.sv
│   ├── Async_FIFO.sv
│   ├── Sync_FIFO.sv
│   ├── Biquad_IIR_Filter.sv
│   ├── DSP_Channel_Processor.sv
│   ├── AXI4_DMA_Master.sv      # Includes Fifo_Arbiter
│   └── CDC_Pulse_Sync.sv
└── tb/
    └── dsp_tb.cpp              # Verilator C++ testbench (WIP)
```

## Verification

Testbench infrastructure uses [Verilator](https://www.veripool.org/verilator/) for cycle-accurate simulation. See the `tb/` directory.

## License

This project is for educational and portfolio demonstration purposes.
