// =============================================================================
// Module      : async_fifo
// Project     : Asynchronous FIFO with CDC Verification
// File        : async_fifo.sv
// Description : Top-level parameterized Asynchronous FIFO.
//               Safely transfers data between two fully independent,
//               asynchronous clock domains using the Clifford Cummings
//               (SNUG 2002) Gray-code pointer synchronization methodology.
//
// Architecture:
//   ┌-----------------------------------------------------------------┐
//   │                     async_fifo (this module)                    │
//   │                                                                 │
//   │  WRITE DOMAIN           DUAL-DOMAIN          READ DOMAIN       │
//   │  --------------         --------------        -------------     │
//   │  wptr_bin (reg)         dp_ram                rptr_bin (reg)   │
//   │  wptr_gray (reg)  ---►  sync_2ff (W→R) ---►  wptr_gray_sync   │
//   │  wfull (reg)            sync_2ff (R→W) ◄---  rptr_gray (reg)  │
//   │  rptr_gray_sync  ◄---                         rempty (reg)     │
//   └-----------------------------------------------------------------┘
//
// Sub-modules instantiated:
//   1. dp_ram      : Parameterized simple dual-port SRAM
//   2. sync_2ff    : 2-flop synchronizer: wptr_gray → rclk domain
//   3. sync_2ff    : 2-flop synchronizer: rptr_gray → wclk domain
//
// Key CDC Design Rules (enforced in this module):
//   [R1] Only Gray-coded pointers cross clock domains. Binary pointers
//        NEVER cross clock domain boundaries.
//   [R2] wfull is computed and registered exclusively in the wclk domain.
//   [R3] rempty is computed and registered exclusively in the rclk domain.
//   [R4] Full uses Gray-code MSB comparison (no binary arithmetic needed).
//   [R5] Empty uses direct Gray-code equality (no binary arithmetic needed).
//   [R6] Flags are conservative: wfull may assert early (safe),
//        rempty may deassert late (safe). Neither causes data corruption.
//   [R7] Resets are domain-local; wrst_n resets write domain only,
//        rrst_n resets read domain only.
//
// Full Condition Mathematics (Gray-code comparison):
//   When wptr is exactly FIFO_DEPTH locations ahead of rptr, the two
//   MSBs of their Gray codes are inverted while all lower bits match:
//     wfull_next = (wptr_gray_next[PTR_W-1] != rptr_gray_sync[PTR_W-1]) &&
//                  (wptr_gray_next[PTR_W-2] != rptr_gray_sync[PTR_W-2]) &&
//                  (wptr_gray_next[PTR_W-3:0] == rptr_gray_sync[PTR_W-3:0])
//
// Empty Condition Mathematics (direct Gray equality):
//   FIFO is empty when write pointer equals read pointer in Gray space:
//     rempty_next = (rptr_gray_next == wptr_gray_sync)
//
// Parameters  :
//   DATA_WIDTH  - Width of each data word in bits      (default: 8)
//   FIFO_DEPTH  - Number of words in the FIFO          (default: 16)
//                 MUST be a power of 2 (≥ 4)
//
// Derived Internals:
//   ADDR_WIDTH  = $clog2(FIFO_DEPTH)                   (default: 4)
//   PTR_WIDTH   = ADDR_WIDTH + 1                       (default: 5)
//                 Extra MSB disambiguates full vs. empty when addresses match.
//
// Coding Standard : IEEE 1800-2012 (SystemVerilog)
// Author          : ASIC Design Team
// Date            : 2026-10-04
// Version         : 1.0
// Reference       : Cummings, C.E. - "Simulation and Synthesis Techniques for
//                   Asynchronous FIFO Design", SNUG San Jose 2002.
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module async_fifo #(
    parameter int unsigned DATA_WIDTH = 8,              // Data word width (bits)
    parameter int unsigned FIFO_DEPTH = 16              // FIFO capacity (words); must be power-of-2
) (
    // =========================================================================
    // Write Clock Domain
    // =========================================================================
    input  wire                  wclk,    // Write clock  (rising-edge triggered)
    input  wire                  wrst_n,  // Write domain active-low async reset
    input  wire                  winc,    // Write enable (write request)
    input  wire [DATA_WIDTH-1:0] wdata,   // Write data bus
    output logic                 wfull,   // FIFO full flag  (write domain, registered)

    // =========================================================================
    // Read Clock Domain
    // =========================================================================
    input  wire                  rclk,    // Read clock   (rising-edge triggered)
    input  wire                  rrst_n,  // Read domain  active-low async reset
    input  wire                  rinc,    // Read enable  (read request)
    output logic [DATA_WIDTH-1:0] rdata,  // Read data bus
    output logic                 rempty   // FIFO empty flag (read domain, registered)
);

    // =========================================================================
    // Derived Parameters
    // =========================================================================
    localparam int unsigned ADDR_WIDTH = $clog2(FIFO_DEPTH); // Memory address bits
    localparam int unsigned PTR_WIDTH  = ADDR_WIDTH + 1;     // Pointer width (extra MSB)

    // =========================================================================
    // Internal Signal Declarations
    // =========================================================================

    // --- Write Domain Signals ------------------------------------------------
    logic [PTR_WIDTH-1:0]  wptr_bin;       // Binary write pointer (registered)
    logic [PTR_WIDTH-1:0]  wptr_bin_next;  // Next binary write pointer (combinational)
    logic [PTR_WIDTH-1:0]  wptr_gray;      // Gray-coded write pointer (registered, sent to sync)
    logic [PTR_WIDTH-1:0]  wptr_gray_next; // Next Gray write pointer  (combinational)
    logic                  wfull_next;     // Combinational next-cycle full flag
    logic [ADDR_WIDTH-1:0] waddr;          // Write address to dp_ram (lower PTR bits)

    // --- Read Domain Signals -------------------------------------------------
    logic [PTR_WIDTH-1:0]  rptr_bin;       // Binary read pointer (registered)
    logic [PTR_WIDTH-1:0]  rptr_bin_next;  // Next binary read pointer (combinational)
    logic [PTR_WIDTH-1:0]  rptr_gray;      // Gray-coded read pointer (registered, sent to sync)
    logic [PTR_WIDTH-1:0]  rptr_gray_next; // Next Gray read pointer  (combinational)
    logic                  rempty_next;    // Combinational next-cycle empty flag
    logic [ADDR_WIDTH-1:0] raddr;          // Read address to dp_ram (lower PTR bits)

    // --- CDC Synchronized Pointers -------------------------------------------
    // wptr_gray synchronized into the read clock domain (via sync_w2r)
    logic [PTR_WIDTH-1:0]  wptr_gray_sync; // Available in rclk domain
    // rptr_gray synchronized into the write clock domain (via sync_r2w)
    logic [PTR_WIDTH-1:0]  rptr_gray_sync; // Available in wclk domain

    // =========================================================================
    // ADDRESS EXTRACTION
    // Lower ADDR_WIDTH bits of the binary pointer are the physical SRAM address.
    // The MSB (wrap bit) is used only for full/empty logic - never sent to RAM.
    // =========================================================================
    assign waddr = wptr_bin[ADDR_WIDTH-1:0];
    assign raddr = rptr_bin[ADDR_WIDTH-1:0];

    // =========================================================================
    // BINARY-TO-GRAY CONVERSION
    //
    // Formula: gray[n] = bin[n] XOR bin[n+1]  ;  gray[MSB] = bin[MSB]
    // Compact: gray = (bin >> 1) ^ bin
    //
    // This is applied to the NEXT pointer value so that the registered
    // Gray pointer is always one step ahead - reducing flag latency by 1 cycle.
    // =========================================================================
    assign wptr_gray_next = (wptr_bin_next >> 1) ^ wptr_bin_next;
    assign rptr_gray_next = (rptr_bin_next >> 1) ^ rptr_bin_next;

    // =========================================================================
    // NEXT POINTER CALCULATION (Combinational)
    //
    // Pointers increment by 1 only on a valid transaction:
    //   - Write: winc asserted AND FIFO is not full
    //   - Read : rinc asserted AND FIFO is not empty
    //
    // The single-bit increment is zero-padded to PTR_WIDTH to avoid width
    // mismatch warnings under strict IEEE 1800-2012 elaboration.
    // Natural unsigned overflow from all-1s → 0 implements wrap-around
    // correctly and is by design (no masking required).
    // =========================================================================
    assign wptr_bin_next = wptr_bin + {{(PTR_WIDTH-1){1'b0}}, (winc & ~wfull)};
    assign rptr_bin_next = rptr_bin + {{(PTR_WIDTH-1){1'b0}}, (rinc & ~rempty)};

    // =========================================================================
    // WRITE POINTER & wfull REGISTER (Write Clock Domain)
    //
    // Both the binary pointer (wptr_bin) and its Gray-coded form (wptr_gray)
    // are registered on the rising edge of wclk.
    //
    // Reset Behavior:
    //   - wptr_bin  → 0 (pointer at first memory location)
    //   - wptr_gray → 0 (Gray(0) = 0, consistent)
    //   - wfull     → 0 (FIFO is not full after reset)
    // =========================================================================
    always_ff @(posedge wclk or negedge wrst_n) begin : write_ptr_reg
        if (!wrst_n) begin
            wptr_bin  <= {PTR_WIDTH{1'b0}};
            wptr_gray <= {PTR_WIDTH{1'b0}};
            wfull     <= 1'b0;
        end else begin
            wptr_bin  <= wptr_bin_next;
            wptr_gray <= wptr_gray_next;  // Registered Gray ptr → drives sync_r2w input
            wfull     <= wfull_next;
        end
    end : write_ptr_reg

    // =========================================================================
    // READ POINTER & rempty REGISTER (Read Clock Domain)
    //
    // Both the binary pointer (rptr_bin) and its Gray-coded form (rptr_gray)
    // are registered on the rising edge of rclk.
    //
    // Reset Behavior:
    //   - rptr_bin  → 0 (pointer at first memory location)
    //   - rptr_gray → 0 (Gray(0) = 0, consistent)
    //   - rempty    → 1 (FIFO is empty after reset - correct initial state)
    // =========================================================================
    always_ff @(posedge rclk or negedge rrst_n) begin : read_ptr_reg
        if (!rrst_n) begin
            rptr_bin  <= {PTR_WIDTH{1'b0}};
            rptr_gray <= {PTR_WIDTH{1'b0}};
            rempty    <= 1'b1;            // Reset to EMPTY (conservative, correct)
        end else begin
            rptr_bin  <= rptr_bin_next;
            rptr_gray <= rptr_gray_next;  // Registered Gray ptr → drives sync_w2r input
            rempty    <= rempty_next;
        end
    end : read_ptr_reg

    // =========================================================================
    // EMPTY FLAG LOGIC (Read Clock Domain)
    //
    // The FIFO is empty when the next read pointer (in Gray code) equals the
    // write pointer synchronized into the read clock domain.
    //
    // Key insight: We compare rptr_gray_NEXT (not current) against the
    // synchronized wptr. This means rempty asserts in the same cycle the
    // last word is consumed - no extra read-latency for empty detection.
    //
    // Safety: If wptr_gray_sync is stale (synchronizer latency), rempty
    // may stay high for up to 2 extra rclk cycles after a write. This is
    // SAFE: the consumer simply waits - no data is lost or corrupted.
    //
    //   rempty_next = (rptr_gray_next == wptr_gray_sync)
    // =========================================================================
    assign rempty_next = (rptr_gray_next == wptr_gray_sync);

    // =========================================================================
    // FULL FLAG LOGIC (Write Clock Domain)
    //
    // The FIFO is full when the write pointer has lapped the read pointer by
    // exactly FIFO_DEPTH locations. This condition is detected in Gray space
    // using the two-MSB inversion property of Gray codes:
    //
    //   When wptr_bin = rptr_bin + FIFO_DEPTH (in binary):
    //     - Gray(wptr)[MSB]   ≠ Gray(rptr)[MSB]       ← top bit inverted
    //     - Gray(wptr)[MSB-1] ≠ Gray(rptr)[MSB-1]     ← second bit inverted
    //     - Gray(wptr)[MSB-2:0] == Gray(rptr)[MSB-2:0] ← all lower bits equal
    //
    // Mathematical derivation:
    //   For a depth-16 FIFO (PTR_WIDTH=5), when wptr=16 and rptr=0:
    //     Binary wptr: 10000, Gray wptr: 11000
    //     Binary rptr: 00000, Gray rptr: 00000
    //     Bits [4,3] differ; bits [2:0] are equal → FULL condition met.
    //
    // We compare wptr_gray_NEXT against the synchronized rptr to detect
    // full one cycle early, preventing overflow.
    //
    // Safety: If rptr_gray_sync is stale (synchronizer latency), wfull may
    // assert up to 2 extra wclk cycles before the FIFO is truly full.
    // This is SAFE: the producer simply throttles - no data overflow occurs.
    //
    //   wfull_next = (wptr_gray_next[PTR_WIDTH-1]   != rptr_gray_sync[PTR_WIDTH-1]) &&
    //               (wptr_gray_next[PTR_WIDTH-2]   != rptr_gray_sync[PTR_WIDTH-2]) &&
    //               (wptr_gray_next[PTR_WIDTH-3:0] == rptr_gray_sync[PTR_WIDTH-3:0])
    // =========================================================================
    assign wfull_next =
        (wptr_gray_next[PTR_WIDTH-1]   != rptr_gray_sync[PTR_WIDTH-1]) &&
        (wptr_gray_next[PTR_WIDTH-2]   != rptr_gray_sync[PTR_WIDTH-2]) &&
        (wptr_gray_next[PTR_WIDTH-3:0] == rptr_gray_sync[PTR_WIDTH-3:0]);

    // =========================================================================
    // SUB-MODULE INSTANTIATION 1: DUAL-PORT RAM
    //
    // - Write port: synchronous on wclk; enabled only on valid writes.
    //   A "valid write" is: winc asserted AND FIFO not full.
    //   The enable (wen) guards against spurious writes when wfull=1.
    //
    // - Read port: ASYNC (combinational) - rdata is driven by mem[raddr]
    //   purely combinationally. This means rdata is valid as long as raddr
    //   is valid (i.e., the read pointer is stable and FIFO is not empty).
    //   The consumer is responsible for registering rdata in its own domain.
    //
    // - wrst_n/rrst_n are passed through but unused by the ASYNC read mode.
    //   The RAM array is not reset; only write control is guarded by wfull.
    // =========================================================================
    dp_ram #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH),
        .READ_MODE  ("ASYNC")
    ) u_dp_ram (
        // Write port
        .wclk   (wclk),
        .wrst_n (wrst_n),
        .wen    (winc & ~wfull),       // Guard: write only when FIFO has space
        .waddr  (waddr),
        .wdata  (wdata),
        // Read port
        .rclk   (rclk),
        .rrst_n (rrst_n),
        .ren    (1'b1),                // Always enable async read; pointer controls access
        .raddr  (raddr),
        .rdata  (rdata)
    );

    // =========================================================================
    // SUB-MODULE INSTANTIATION 2: WRITE-TO-READ SYNCHRONIZER (sync_w2r)
    //
    // Transfers the Gray-coded write pointer from the wclk domain to the
    // rclk domain. The output (wptr_gray_sync) is used by the empty flag
    // logic in the read domain.
    //
    // Clock  : rclk     (destination domain)
    // Reset  : rrst_n   (destination domain - CRITICAL: NEVER use wrst_n here)
    // Input  : wptr_gray (registered in wclk domain)
    // Output : wptr_gray_sync (safe in rclk domain)
    // =========================================================================
    sync_2ff #(
        .WIDTH     (PTR_WIDTH),
        .RESET_VAL ({PTR_WIDTH{1'b0}})
    ) u_sync_w2r (
        .clk   (rclk),             // Destination clock = rclk
        .rst_n (rrst_n),           // Destination reset = rrst_n
        .d_in  (wptr_gray),        // Source: registered Gray write pointer
        .d_out (wptr_gray_sync)    // Output: wptr visible in read domain
    );

    // =========================================================================
    // SUB-MODULE INSTANTIATION 3: READ-TO-WRITE SYNCHRONIZER (sync_r2w)
    //
    // Transfers the Gray-coded read pointer from the rclk domain to the
    // wclk domain. The output (rptr_gray_sync) is used by the full flag
    // logic in the write domain.
    //
    // Clock  : wclk     (destination domain)
    // Reset  : wrst_n   (destination domain - CRITICAL: NEVER use rrst_n here)
    // Input  : rptr_gray (registered in rclk domain)
    // Output : rptr_gray_sync (safe in wclk domain)
    // =========================================================================
    sync_2ff #(
        .WIDTH     (PTR_WIDTH),
        .RESET_VAL ({PTR_WIDTH{1'b0}})
    ) u_sync_r2w (
        .clk   (wclk),             // Destination clock = wclk
        .rst_n (wrst_n),           // Destination reset = wrst_n
        .d_in  (rptr_gray),        // Source: registered Gray read pointer
        .d_out (rptr_gray_sync)    // Output: rptr visible in write domain
    );

    // =========================================================================
    // PARAMETER & CONFIGURATION VALIDATION
    // All checks are elaboration-time ($fatal) or simulation-time ($error).
    // These will not synthesize into hardware.
    // =========================================================================
    initial begin : param_validation
        // [V1] FIFO_DEPTH must be a power of 2
        if ((FIFO_DEPTH & (FIFO_DEPTH - 1)) !== 0) begin
            $fatal(1, "[async_fifo] FATAL: FIFO_DEPTH=%0d is not a power of 2. \
Gray-code pointer arithmetic requires power-of-2 depth.", FIFO_DEPTH);
        end

        // [V2] FIFO_DEPTH must be at least 4 (PTR_WIDTH >= 3)
        // This ensures the full-condition Gray-code comparison (which checks
        // bits [PTR_WIDTH-1], [PTR_WIDTH-2], and [PTR_WIDTH-3:0]) is valid.
        // A depth of 2 (PTR_WIDTH=2) has no lower bits, breaking the comparison.
        if (FIFO_DEPTH < 4) begin
            $fatal(1, "[async_fifo] FATAL: FIFO_DEPTH=%0d must be >= 4. \
Minimum supported depth is 4 (ADDR_WIDTH=2, PTR_WIDTH=3).", FIFO_DEPTH);
        end

        // [V3] DATA_WIDTH sanity
        if (DATA_WIDTH < 1 || DATA_WIDTH > 256) begin
            $fatal(1, "[async_fifo] FATAL: DATA_WIDTH=%0d is out of range [1,256].", DATA_WIDTH);
        end

        // [V4] Report configuration at simulation start
        $display("[async_fifo] INFO: Configuration - DATA_WIDTH=%0d, FIFO_DEPTH=%0d, \
ADDR_WIDTH=%0d, PTR_WIDTH=%0d", DATA_WIDTH, FIFO_DEPTH, ADDR_WIDTH, PTR_WIDTH);
    end : param_validation

    // =========================================================================
    // SIMULATION-ONLY RUNTIME CHECKS
    // These guards catch protocol violations during simulation.
    // Wrapped in translate_off/on so they are removed during synthesis.
    // =========================================================================
    // synthesis translate_off

    // [C1] Write-while-full violation check
    always_ff @(posedge wclk) begin : check_write_overflow
        if (wrst_n && wfull && winc) begin
            $error("[async_fifo] PROTOCOL VIOLATION at %0t: winc asserted while wfull=1. \
Write will be dropped (pointer does NOT advance).", $time);
        end
    end : check_write_overflow

    // [C2] Read-while-empty violation check
    always_ff @(posedge rclk) begin : check_read_underflow
        if (rrst_n && rempty && rinc) begin
            $error("[async_fifo] PROTOCOL VIOLATION at %0t: rinc asserted while rempty=1. \
Read will be dropped (pointer does NOT advance).", $time);
        end
    end : check_read_underflow

    // [C3] X/Z propagation check on wdata during valid write
    always_ff @(posedge wclk) begin : check_wdata_x
        if (wrst_n && winc && !wfull && $isunknown(wdata)) begin
            $error("[async_fifo] DATA INTEGRITY WARNING at %0t: X/Z detected on wdata \
during a valid write (winc=1, wfull=0). Written value is unknown.", $time);
        end
    end : check_wdata_x

    // synthesis translate_on

endmodule : async_fifo

`default_nettype wire
// =============================================================================
// End of async_fifo.sv
// =============================================================================
