// =============================================================================
// Module      : sync_2ff
// Project     : Asynchronous FIFO with CDC Verification
// File        : sync_2ff.sv
// Description : Parameterized 2-Stage Flip-Flop Synchronizer for safe
//               Clock Domain Crossing (CDC) of multi-bit Gray-coded signals.
//
//               ┌----------------------------------------------------------┐
//               │              Destination Clock Domain                    │
//               │                                                          │
//               │  d_in --► [FF Stage 1] --► [FF Stage 2] --► d_out      │
//               │               (sync_s1)        (sync_s2)                │
//               │                                                          │
//               └----------------------------------------------------------┘
//
// Key Properties:
//   - Only 1 bit changes per clock cycle at input (Gray-code guarantee).
//     This ensures that even if FF Stage 1 metastabilizes, only one bit is
//     at risk; FF Stage 2 allows metastability to resolve before sampling.
//   - All FFs in this synchronizer MUST be placed in the SAME clock region
//     (physically adjacent) by the place-and-route tool to minimize
//     inter-stage skew. Use synthesis attributes to enforce this.
//   - Width (WIDTH) is parameterized to handle multi-bit Gray-coded pointers
//     (e.g., 5-bit pointer for a depth-16 FIFO).
//
// Parameters  :
//   WIDTH       - Number of bits to synchronize          (default: 5)
//   RESET_VAL   - Value driven on d_out during reset     (default: 0)
//
// Interface   :
//   clk         - Destination domain clock (rising-edge triggered)
//   rst_n       - Active-low, asynchronous reset (destination domain)
//   d_in        - Input signal from source clock domain
//   d_out       - Synchronized output, safe in destination clock domain
//
// Synthesis Attributes:
//   - "ASYNC_REG" (Xilinx/Vivado): Prevents logic optimization across FF
//     stages and places FFs in the same slice.
//   - "PRESERVE"  (Quartus/Intel): Prevents register merging optimization.
//   - "async_reg" (Synopsys DC / ICC2): Annotates FFs as async registers
//     for STA exception (set_false_path or set_max_delay -datapath_only).
//
// IMPORTANT: In your SDC/XDC constraint file, you MUST add:
//   set_false_path -to [get_cells -hierarchical -filter {ASYNC_REG == TRUE}]
//   OR
//   set_max_delay -datapath_only -from <src_clk> -to <dst_ff_stage1>
//   This tells the STA tool not to enforce setup timing on the first FF,
//   as metastability is expected and handled by the 2-stage structure.
//
// Metastability Analysis:
//   MTBF = e^(Tresolution / τ) / (f_src × f_dst × C)
//   Where:
//     Tresolution = time for metastability to resolve (1 clock period - setup)
//     τ           = technology-dependent metastability constant (~50-100ps typical)
//     C           = data transition rate at the synchronizer input
//   For most designs (< 500 MHz), 2 stages provide MTBF >> 100 years.
//   For > 500 MHz or safety-critical designs, use WIDTH=3 stages.
//
// Coding Standard : IEEE 1800-2012 (SystemVerilog)
// Author          : ASIC Design Team
// Date            : 2026-10-04
// Version         : 1.0
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sync_2ff #(
    parameter int unsigned                WIDTH     = 5,  // Bit-width of the signal to synchronize
    parameter logic        [WIDTH-1:0]    RESET_VAL = '0  // Value of d_out during/after reset
) (
    input  wire              clk,    // Destination domain clock (rising-edge)
    input  wire              rst_n,  // Active-low asynchronous reset (destination domain)
    input  wire [WIDTH-1:0]  d_in,   // Unsynchronized input from source clock domain
    output wire [WIDTH-1:0]  d_out   // Synchronized output, safe in destination domain
);

    // =========================================================================
    // Synchronizer Flip-Flop Declarations
    //
    // SYNTHESIS ATTRIBUTES - CRITICAL:
    //   These attributes prevent the synthesis tool from:
    //     1. Merging or optimizing away the intermediate flop (sync_s1).
    //     2. Inserting combinational logic between the two FF stages.
    //     3. Placing the FFs far apart (causing excessive inter-stage skew).
    //
    //   Xilinx Vivado  : (* ASYNC_REG = "TRUE" *) - must precede the declaration
    //   Intel Quartus  : (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
    //   Synopsys DC    : Use 'set_dont_touch' and 'set_multicycle_path' in SDC
    // =========================================================================

    (* ASYNC_REG = "TRUE" *) logic [WIDTH-1:0] sync_s1; // Stage 1: metastability capture FF
    (* ASYNC_REG = "TRUE" *) logic [WIDTH-1:0] sync_s2; // Stage 2: resolved output FF

    // =========================================================================
    // Two-Stage Synchronizer Register Chain
    //
    // Stage 1 (sync_s1):
    //   - Directly samples d_in from the source clock domain.
    //   - May go metastable if d_in transitions near the destination clock edge.
    //   - Given one full destination clock cycle to resolve metastability.
    //
    // Stage 2 (sync_s2):
    //   - Samples sync_s1 after it has had time to resolve.
    //   - Output is considered metastability-safe.
    //   - Connected to d_out and consumed by destination-domain logic.
    //
    // Reset Behavior:
    //   - Asynchronous reset: both stages immediately driven to RESET_VAL.
    //   - This is safe because Gray-coded pointer = 0 is a valid reset state.
    //   - The reset here must be driven by the DESTINATION domain reset signal.
    //   - NEVER drive this module's rst_n from the source clock domain.
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin : sync_chain
        if (!rst_n) begin
            // Asynchronous active-low reset: clear both pipeline stages.
            // Pointer value 0 is safe and corresponds to the reset state
            // of both wptr and rptr in the parent async_fifo_top module.
            sync_s1 <= RESET_VAL;
            sync_s2 <= RESET_VAL;
        end else begin
            // Normal operation: shift d_in through the two-stage pipeline.
            // Both assignments execute in the same clock cycle (parallel FF update).
            sync_s1 <= d_in;    // Stage 1: capture (may be metastable)
            sync_s2 <= sync_s1; // Stage 2: resolve  (safe to use downstream)
        end
    end : sync_chain

    // =========================================================================
    // Output Assignment
    // d_out is driven by the final synchronized stage (sync_s2).
    // This signal is now safe for use in destination-domain combinational
    // and sequential logic (full/empty flag generation, etc.).
    // =========================================================================
    assign d_out = sync_s2;

    // =========================================================================
    // Parameter Validation (Elaboration-time checks)
    // =========================================================================
    initial begin : param_check
        if (WIDTH < 1 || WIDTH > 64) begin
            $fatal(1, "[sync_2ff] FATAL: WIDTH=%0d is out of valid range [1,64].", WIDTH);
        end
    end

    // =========================================================================
    // Simulation-Only Metastability Warning
    // In simulation, X-propagation can be used to model metastability.
    // This block adds a runtime check that d_in only changes by 1 bit per
    // clock cycle - validating the Gray-code assumption at the synchronizer.
    // =========================================================================
    // synthesis translate_off
    logic [WIDTH-1:0] d_in_prev;

    always_ff @(posedge clk or negedge rst_n) begin : gray_check
        if (!rst_n) begin
            d_in_prev <= RESET_VAL;
        end else begin
            d_in_prev <= d_in;
            // Verify only 0 or 1 bit changes per clock edge (Gray-code property)
            if ($countones(d_in ^ d_in_prev) > 1) begin
                $warning("[sync_2ff] WARNING at time %0t: d_in changed by %0d bits in one cycle! \
Expected Gray-code (max 1-bit change). d_in_prev=%0b, d_in=%0b",
                         $time, $countones(d_in ^ d_in_prev), d_in_prev, d_in);
            end
        end
    end : gray_check
    // synthesis translate_on

endmodule : sync_2ff

`default_nettype wire
// =============================================================================
// End of sync_2ff.sv
// =============================================================================
