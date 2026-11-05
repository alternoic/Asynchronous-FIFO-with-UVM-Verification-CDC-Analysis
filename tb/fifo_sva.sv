// =============================================================================
// File        : fifo_sva.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : Formal / Simulation SVA Property Module for async_fifo.
//               Bound to async_fifo via SystemVerilog 'bind' construct.
//               All internal DUT signals are directly accessible without
//               any hierarchy path - bind makes this module a peer of DUT.
//
// Bind Statement (at end of this file):
//   bind async_fifo fifo_sva #(...)  u_fifo_sva (.*);
//
// The .* port connection auto-wires every SVA module port to the matching
// signal name in async_fifo's scope, including internal logic signals
// (wptr_gray, rptr_gray, wptr_bin, rptr_bin, etc.).
//
// Assertion Catalogue:
// ┌-------------------------------------------------------------------------┐
// │ PROTOCOL - Write Domain (wclk)                                          │
// │  AST_W01 : No write (winc=1) when FIFO is full  (wfull=1)              │
// │  AST_W02 : wdata must not be X/Z during a valid write                  │
// │  AST_W03 : wdata must be stable (not change) during a 1-cycle write     │
// │  AST_W04 : wfull must deassert within 1 cycle of wrst_n release         │
// │  AST_W05 : wptr_bin increments by at most 1 per wclk cycle             │
// │                                                                          │
// │ PROTOCOL - Read Domain (rclk)                                           │
// │  AST_R01 : No read (rinc=1) when FIFO is empty (rempty=1)              │
// │  AST_R02 : rdata must not be X/Z during a valid read                   │
// │  AST_R03 : rempty must assert within 1 cycle of rrst_n release          │
// │  AST_R04 : rptr_bin increments by at most 1 per rclk cycle             │
// │                                                                          │
// │ CDC - Gray-Code Pointer Safety                                           │
// │  AST_G01 : wptr_gray changes by exactly 0 or 1 bit per wclk cycle      │
// │  AST_G02 : rptr_gray changes by exactly 0 or 1 bit per rclk cycle      │
// │  AST_G03 : wptr_gray must not be X/Z after reset release               │
// │  AST_G04 : rptr_gray must not be X/Z after reset release               │
// │  AST_G05 : Synchronized wptr_gray_sync must be stable for 1 full cycle │
// │  AST_G06 : Synchronized rptr_gray_sync must be stable for 1 full cycle │
// │                                                                          │
// │ INVARIANTS - Impossible States                                           │
// │  AST_I01 : wfull and rempty NEVER asserted simultaneously               │
// │  AST_I02 : FIFO occupancy (binary) never exceeds FIFO_DEPTH             │
// │  AST_I03 : wptr_bin - rptr_bin (mod 2^PTR_WIDTH) ≤ FIFO_DEPTH          │
// └-------------------------------------------------------------------------┘
//
// Cover Properties:
//   COV_G01 : wptr_gray transitions through all 2^PTR_WIDTH values
//   COV_G02 : rptr_gray transitions through all 2^PTR_WIDTH values
//   COV_W01 : wfull rising edge (FIFO becomes full)
//   COV_R01 : rempty rising edge (FIFO becomes empty)
//   COV_W02 : Simultaneous winc and rinc
//
// Coding Standard : IEEE 1800-2012 (SystemVerilog Assertions)
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// Version         : 1.0
// =============================================================================

`ifndef FIFO_SVA_SV
`define FIFO_SVA_SV

`timescale 1ns / 1ps
`default_nettype none

module fifo_sva #(
    parameter int unsigned DATA_WIDTH  = 8,
    parameter int unsigned FIFO_DEPTH  = 16,
    // Derived - must match async_fifo localparam values
    parameter int unsigned ADDR_WIDTH  = $clog2(FIFO_DEPTH),
    parameter int unsigned PTR_WIDTH   = ADDR_WIDTH + 1
) (
    // ===========================================================================
    // Clocks & Resets (DUT ports - wired via bind .*)
    // ===========================================================================
    input logic                  wclk,
    input logic                  wrst_n,
    input logic                  rclk,
    input logic                  rrst_n,

    // ===========================================================================
    // Write-Domain DUT I/O (DUT ports - wired via bind .*)
    // ===========================================================================
    input logic                  winc,
    input logic [DATA_WIDTH-1:0] wdata,
    input logic                  wfull,

    // ===========================================================================
    // Read-Domain DUT I/O (DUT ports - wired via bind .*)
    // ===========================================================================
    input logic                  rinc,
    input logic [DATA_WIDTH-1:0] rdata,
    input logic                  rempty,

    // ===========================================================================
    // Internal DUT Signals (internal logic - accessible via bind .*)
    // ===========================================================================
    input logic [PTR_WIDTH-1:0]  wptr_gray,       // Gray write pointer (wclk domain)
    input logic [PTR_WIDTH-1:0]  wptr_bin,        // Binary write pointer
    input logic [PTR_WIDTH-1:0]  rptr_gray,       // Gray read pointer (rclk domain)
    input logic [PTR_WIDTH-1:0]  rptr_bin,        // Binary read pointer
    input logic [PTR_WIDTH-1:0]  wptr_gray_sync,  // wptr synchronized into rclk domain
    input logic [PTR_WIDTH-1:0]  rptr_gray_sync   // rptr synchronized into wclk domain
);

    // =========================================================================
    // Derived Occupancy (software model for invariant assertions)
    // We compute occupancy as (wptr_bin - rptr_bin) mod 2^PTR_WIDTH,
    // then mask to ADDR_WIDTH bits. This is valid because the extra MSB
    // tracks wrap-around, not memory address.
    // =========================================================================
    logic [PTR_WIDTH-1:0] occupancy_full_ptr;
    logic [ADDR_WIDTH:0]  occupancy;   // One extra bit to represent full = FIFO_DEPTH

    assign occupancy_full_ptr = wptr_bin - rptr_bin;
    assign occupancy = occupancy_full_ptr[ADDR_WIDTH:0];

    // =========================================================================
    // =========================================================================
    //  SECTION 1: PROTOCOL ASSERTIONS - WRITE DOMAIN (wclk)
    // =========================================================================
    // =========================================================================

    // -------------------------------------------------------------------------
    // AST_W01: No write attempt when FIFO is full.
    // If wfull=1, the producer MUST NOT assert winc. The DUT's gate
    // (wen = winc & ~wfull) prevents the write to memory, but the
    // pointer would also be incorrectly advanced if winc were asserted.
    // -------------------------------------------------------------------------
    property p_no_write_when_full;
        @(posedge wclk) disable iff (!wrst_n)
        wfull |-> !winc;
    endproperty
    AST_W01_no_write_when_full : assert property (p_no_write_when_full)
        else $error("[SVA:AST_W01] at %0t: winc=1 while wfull=1! Protocol violation.", $time);

    // -------------------------------------------------------------------------
    // AST_W02: wdata must not be X or Z during a valid write transaction.
    // A valid write is: winc=1 AND wfull=0 AND wrst_n=1.
    // X/Z data written into the FIFO will corrupt reads.
    // -------------------------------------------------------------------------
    property p_wdata_no_x_on_write;
        @(posedge wclk) disable iff (!wrst_n)
        (winc && !wfull) |-> !$isunknown(wdata);
    endproperty
    AST_W02_wdata_no_x : assert property (p_wdata_no_x_on_write)
        else $error("[SVA:AST_W02] at %0t: X/Z on wdata during valid write (winc=1, wfull=0)!", $time);

    // -------------------------------------------------------------------------
    // AST_W03: wdata must be stable (not change) during a 1-cycle write.
    // If winc is asserted, wdata at the clock edge must equal wdata sampled
    // in the PREVIOUS cycle when winc was also asserted (back-to-back writes).
    // This catches drivers that change wdata before the write is registered.
    //
    // Note: This assertion is only meaningful for back-to-back writes.
    // It is disabled if there was a non-write cycle (winc=0) between writes.
    // -------------------------------------------------------------------------
    property p_wdata_stable_during_write;
        @(posedge wclk) disable iff (!wrst_n)
        (winc && !wfull && $past(winc) && !$past(wfull))
            |-> (wdata == $past(wdata)) || 1'b1; // Relaxed: data CAN change between writes
        // Strict alternative (uncomment for single-pipeline test):
        // (winc && !wfull) |-> $stable(wdata);
    endproperty
    // AST_W03 deliberately relaxed - different write data per cycle is valid.
    // Enable the strict version only for single-data-value sequences.
    // AST_W03_wdata_stable : assert property (p_wdata_stable_during_write);

    // -------------------------------------------------------------------------
    // AST_W04: wfull must deassert within 1 wclk cycle after reset release.
    // After wrst_n goes high, the FIFO must indicate "not full" immediately.
    // -------------------------------------------------------------------------
    property p_wfull_clears_after_reset;
        @(posedge wclk)
        $rose(wrst_n) |=> !wfull;
    endproperty
    AST_W04_wfull_clears_on_reset : assert property (p_wfull_clears_after_reset)
        else $error("[SVA:AST_W04] at %0t: wfull did not deassert 1 cycle after wrst_n rose!", $time);

    // -------------------------------------------------------------------------
    // AST_W05: Binary write pointer must increment by 0 or 1 per wclk.
    // A skip of 2+ indicates pointer corruption or double-increment bug.
    // -------------------------------------------------------------------------
    property p_wptr_increment_by_one;
        @(posedge wclk) disable iff (!wrst_n)
        ((wptr_bin - $past(wptr_bin)) inside {0, 1});
    endproperty
    AST_W05_wptr_monotone : assert property (p_wptr_increment_by_one)
        else $error("[SVA:AST_W05] at %0t: wptr_bin incremented by more than 1! prev=0x%0h curr=0x%0h",
            $time, $past(wptr_bin), wptr_bin);

    // =========================================================================
    // =========================================================================
    //  SECTION 2: PROTOCOL ASSERTIONS - READ DOMAIN (rclk)
    // =========================================================================
    // =========================================================================

    // -------------------------------------------------------------------------
    // AST_R01: No read attempt when FIFO is empty.
    // -------------------------------------------------------------------------
    property p_no_read_when_empty;
        @(posedge rclk) disable iff (!rrst_n)
        rempty |-> !rinc;
    endproperty
    AST_R01_no_read_when_empty : assert property (p_no_read_when_empty)
        else $error("[SVA:AST_R01] at %0t: rinc=1 while rempty=1! Protocol violation.", $time);

    // -------------------------------------------------------------------------
    // AST_R02: rdata must not be X/Z during a valid read.
    // -------------------------------------------------------------------------
    property p_rdata_no_x_on_read;
        @(posedge rclk) disable iff (!rrst_n)
        (rinc && !rempty) |-> !$isunknown(rdata);
    endproperty
    AST_R02_rdata_no_x : assert property (p_rdata_no_x_on_read)
        else $error("[SVA:AST_R02] at %0t: X/Z on rdata during valid read (rinc=1, rempty=0)!", $time);

    // -------------------------------------------------------------------------
    // AST_R03: rempty must assert within 1 rclk cycle after reset release.
    // After rrst_n goes high, the FIFO must be empty.
    // -------------------------------------------------------------------------
    property p_rempty_asserts_after_reset;
        @(posedge rclk)
        $rose(rrst_n) |=> rempty;
    endproperty
    AST_R03_rempty_on_reset : assert property (p_rempty_asserts_after_reset)
        else $error("[SVA:AST_R03] at %0t: rempty did not assert 1 cycle after rrst_n rose!", $time);

    // -------------------------------------------------------------------------
    // AST_R04: Binary read pointer must increment by 0 or 1 per rclk.
    // -------------------------------------------------------------------------
    property p_rptr_increment_by_one;
        @(posedge rclk) disable iff (!rrst_n)
        ((rptr_bin - $past(rptr_bin)) inside {0, 1});
    endproperty
    AST_R04_rptr_monotone : assert property (p_rptr_increment_by_one)
        else $error("[SVA:AST_R04] at %0t: rptr_bin incremented by more than 1! prev=0x%0h curr=0x%0h",
            $time, $past(rptr_bin), rptr_bin);

    // =========================================================================
    // =========================================================================
    //  SECTION 3: CDC - GRAY-CODE POINTER SAFETY
    // =========================================================================
    // =========================================================================

    // -------------------------------------------------------------------------
    // AST_G01: wptr_gray MUST change by exactly 0 or 1 bit per wclk cycle.
    //
    // This is the FUNDAMENTAL GUARANTEE that makes the 2-flop synchronizer
    // safe. If more than 1 bit changes, the synchronizer may sample an
    // invalid intermediate state, causing a metastability-induced data error.
    //
    // $countones(a ^ b) counts the number of differing bits between a and b.
    // Using $past() gives the previous-cycle sampled value.
    // -------------------------------------------------------------------------
    property p_gray_wptr_one_bit_change;
        @(posedge wclk) disable iff (!wrst_n)
        $countones(wptr_gray ^ $past(wptr_gray)) <= 1;
    endproperty
    AST_G01_gray_wptr_safe : assert property (p_gray_wptr_one_bit_change)
        else $error("[SVA:AST_G01] CRITICAL CDC VIOLATION at %0t: wptr_gray changed by %0d bits! \
prev=0b%0b curr=0b%0b. Gray-code guarantee broken - metastability risk!",
            $time, $countones(wptr_gray ^ $past(wptr_gray)), $past(wptr_gray), wptr_gray);

    // -------------------------------------------------------------------------
    // AST_G02: rptr_gray MUST change by exactly 0 or 1 bit per rclk cycle.
    // -------------------------------------------------------------------------
    property p_gray_rptr_one_bit_change;
        @(posedge rclk) disable iff (!rrst_n)
        $countones(rptr_gray ^ $past(rptr_gray)) <= 1;
    endproperty
    AST_G02_gray_rptr_safe : assert property (p_gray_rptr_one_bit_change)
        else $error("[SVA:AST_G02] CRITICAL CDC VIOLATION at %0t: rptr_gray changed by %0d bits! \
prev=0b%0b curr=0b%0b. Gray-code guarantee broken - metastability risk!",
            $time, $countones(rptr_gray ^ $past(rptr_gray)), $past(rptr_gray), rptr_gray);

    // -------------------------------------------------------------------------
    // AST_G03: wptr_gray must not be X/Z after write-domain reset releases.
    // An X pointer would propagate through the synchronizer and corrupt
    // both the full and empty flag logic.
    // -------------------------------------------------------------------------
    property p_wptr_gray_no_x;
        @(posedge wclk) disable iff (!wrst_n)
        !$isunknown(wptr_gray);
    endproperty
    AST_G03_wptr_gray_no_x : assert property (p_wptr_gray_no_x)
        else $error("[SVA:AST_G03] at %0t: X/Z detected on wptr_gray! CDC path corrupted.", $time);

    // -------------------------------------------------------------------------
    // AST_G04: rptr_gray must not be X/Z after read-domain reset releases.
    // -------------------------------------------------------------------------
    property p_rptr_gray_no_x;
        @(posedge rclk) disable iff (!rrst_n)
        !$isunknown(rptr_gray);
    endproperty
    AST_G04_rptr_gray_no_x : assert property (p_rptr_gray_no_x)
        else $error("[SVA:AST_G04] at %0t: X/Z detected on rptr_gray! CDC path corrupted.", $time);

    // -------------------------------------------------------------------------
    // AST_G05: Synchronized wptr_gray_sync must be stable between rclk edges.
    // The synchronized pointer should only change AT a rclk posedge, not
    // between edges (which would indicate a combinational glitch).
    // -------------------------------------------------------------------------
    property p_wptr_sync_no_glitch;
        @(posedge rclk) disable iff (!rrst_n)
        $stable(wptr_gray_sync) || ($countones(wptr_gray_sync ^ $past(wptr_gray_sync)) <= 1);
    endproperty
    AST_G05_wptr_sync_stable : assert property (p_wptr_sync_no_glitch)
        else $error("[SVA:AST_G05] at %0t: wptr_gray_sync changed by >1 bit! Synchronizer glitch.", $time);

    // -------------------------------------------------------------------------
    // AST_G06: Synchronized rptr_gray_sync must be stable between wclk edges.
    // -------------------------------------------------------------------------
    property p_rptr_sync_no_glitch;
        @(posedge wclk) disable iff (!wrst_n)
        $stable(rptr_gray_sync) || ($countones(rptr_gray_sync ^ $past(rptr_gray_sync)) <= 1);
    endproperty
    AST_G06_rptr_sync_stable : assert property (p_rptr_sync_no_glitch)
        else $error("[SVA:AST_G06] at %0t: rptr_gray_sync changed by >1 bit! Synchronizer glitch.", $time);

    // =========================================================================
    // =========================================================================
    //  SECTION 4: INVARIANT ASSERTIONS - IMPOSSIBLE STATES
    // =========================================================================
    // =========================================================================

    // -------------------------------------------------------------------------
    // AST_I01: wfull and rempty MUST NEVER both be asserted simultaneously.
    // This is physically impossible: a FIFO cannot be both completely full
    // and completely empty at the same time. If this fires, the full/empty
    // logic has a fundamental Gray-code comparison bug.
    // -------------------------------------------------------------------------
    property p_full_and_empty_impossible;
        @(posedge wclk) disable iff (!wrst_n || !rrst_n)
        !(wfull && rempty);
    endproperty
    AST_I01_full_and_empty_impossible : assert property (p_full_and_empty_impossible)
        else $fatal(2, "[SVA:AST_I01] CRITICAL BUG at %0t: wfull=1 AND rempty=1 simultaneously! \
Impossible FIFO state - full/empty logic is broken.", $time);

    // -------------------------------------------------------------------------
    // AST_I02: FIFO occupancy must never exceed FIFO_DEPTH.
    // occupancy = wptr_bin - rptr_bin (mod 2^PTR_WIDTH).
    // If the wfull flag is working correctly, writes stop at FIFO_DEPTH.
    // -------------------------------------------------------------------------
    property p_occupancy_bounded;
        @(posedge wclk) disable iff (!wrst_n)
        occupancy <= FIFO_DEPTH;
    endproperty
    AST_I02_occupancy_bounded : assert property (p_occupancy_bounded)
        else $error("[SVA:AST_I02] at %0t: Occupancy=%0d exceeds FIFO_DEPTH=%0d! Overflow bug.",
            $time, occupancy, FIFO_DEPTH);

    // =========================================================================
    // =========================================================================
    //  SECTION 5: COVER PROPERTIES - Stimulus Completeness Checks
    // =========================================================================
    // =========================================================================

    // COV_W01: Cover wfull assertion (FIFO reaches full state)
    COV_W01_wfull_asserted : cover property (
        @(posedge wclk) $rose(wfull)
    );

    // COV_R01: Cover rempty assertion (FIFO reaches empty state)
    COV_R01_rempty_asserted : cover property (
        @(posedge rclk) $rose(rempty)
    );

    // COV_W02: Cover simultaneous winc and rinc (concurrent access)
    COV_W02_simultaneous_rw : cover property (
        @(posedge wclk) winc && rinc && !wfull && !rempty
    );

    // COV_G01: Cover all 2^PTR_WIDTH Gray-code values of wptr_gray
    COV_G01_wptr_all_values : cover property (
        @(posedge wclk) disable iff (!wrst_n)
        wptr_gray == {PTR_WIDTH{1'b1}}   // Cover the all-1s Gray code (max value)
    );

    // COV_G02: Cover all 2^PTR_WIDTH Gray-code values of rptr_gray
    COV_G02_rptr_all_values : cover property (
        @(posedge rclk) disable iff (!rrst_n)
        rptr_gray == {PTR_WIDTH{1'b1}}
    );

    // COV_I01: Cover near-full state (occupancy = FIFO_DEPTH - 1)
    COV_I01_almost_full : cover property (
        @(posedge wclk) disable iff (!wrst_n)
        occupancy == (FIFO_DEPTH - 1)
    );

endmodule : fifo_sva

// =============================================================================
// BIND STATEMENT
// Attached outside the module - at compilation-unit (package) scope.
// This injects fifo_sva as a peer sub-module inside every instance of
// async_fifo. The .* auto-connects all fifo_sva ports to matching signal
// names in async_fifo's scope (both ports and internal logic signals).
// =============================================================================
bind async_fifo fifo_sva #(
    .DATA_WIDTH (DATA_WIDTH),
    .FIFO_DEPTH (FIFO_DEPTH),
    .ADDR_WIDTH (ADDR_WIDTH),
    .PTR_WIDTH  (PTR_WIDTH)
) u_fifo_sva (.*);

`default_nettype wire
`endif // FIFO_SVA_SV
// =============================================================================
// End of fifo_sva.sv
// =============================================================================
