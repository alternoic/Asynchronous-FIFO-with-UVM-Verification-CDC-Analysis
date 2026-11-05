// =============================================================================
// File        : fifo_if.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : SystemVerilog Interface for async_fifo DUT.
//               Bundles all DUT signals into a single virtual interface handle
//               that can be shared across all UVM components (driver, monitor,
//               scoreboard, coverage collector).
//
//               Contains FOUR clocking blocks for strict timing-correct access:
//
//   ┌----------------------------------------------------------------┐
//   │ Clocking Block   │ Clock  │ Mode    │ Used By                  │
//   ├------------------┼--------┼---------┼--------------------------┤
//   │ wdrv_cb          │ wclk   │ Active  │ fifo_write_driver        │
//   │ rdrv_cb          │ rclk   │ Active  │ fifo_read_driver         │
//   │ wmon_cb          │ wclk   │ Passive │ fifo_write_monitor       │
//   │ rmon_cb          │ rclk   │ Passive │ fifo_read_monitor        │
//   └----------------------------------------------------------------┘
//
//               Contains FIVE modports for safe component-level port binding:
//
//   ┌-----------------┬--------------------------------------------┐
//   │ Modport          │ Used By                                    │
//   ├-----------------┼--------------------------------------------┤
//   │ MP_WRITE_DRV     │ Write-domain UVM driver                   │
//   │ MP_READ_DRV      │ Read-domain UVM driver                    │
//   │ MP_WRITE_MON     │ Write-domain UVM monitor (passive)        │
//   │ MP_READ_MON      │ Read-domain UVM monitor (passive)         │
//   │ MP_DUT           │ DUT port connection in tb_top             │
//   └-----------------┴--------------------------------------------┘
//
// Timing Convention:
//   - Output skew : #1  (driven 1ns after the active clock edge)
//     Ensures outputs are sampled in the NEXT clock cycle by the DUT.
//   - Input  skew : #1step (sampled just before the clock edge in simulation)
//     Provides hold-time-safe sampling of DUT outputs.
//
// Inline Assertions:
//   - Protocol-level assertions are embedded in the interface using the
//     clocking blocks as the timing reference. This makes assertions
//     independent of the UVM component hierarchy and always active.
//
// Parameters:
//   DATA_WIDTH  - Must match async_fifo DUT parameter (default: 8)
//
// Usage in tb_top:
//   async_fifo_if #(.DATA_WIDTH(8)) fifo_if (.wclk(wclk), .rclk(rclk));
//   uvm_config_db #(virtual async_fifo_if)::set(null, "*", "vif", fifo_if);
//
// Coding Standard : IEEE 1800-2012 (SystemVerilog)
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// Version         : 1.0
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

interface async_fifo_if #(
    parameter int unsigned DATA_WIDTH = 8      // Must match DUT DATA_WIDTH
) (
    input wire wclk,   // Write clock (from tb_top clock generator)
    input wire rclk    // Read  clock (from tb_top clock generator)
);

    // =========================================================================
    // Signal Declarations
    // All signals are declared as 'logic' (tri-state safe for SV drivers).
    // Clocking blocks enforce directional discipline.
    // =========================================================================

    // --- Write Clock Domain Signals ------------------------------------------
    logic                  wrst_n;  // Active-low async write-domain reset
    logic                  winc;    // Write increment / write enable
    logic [DATA_WIDTH-1:0] wdata;   // Write data payload
    logic                  wfull;   // FIFO full flag (output from DUT)

    // --- Read Clock Domain Signals -------------------------------------------
    logic                  rrst_n;  // Active-low async read-domain reset
    logic                  rinc;    // Read increment / read enable
    logic [DATA_WIDTH-1:0] rdata;   // Read data output (from DUT)
    logic                  rempty;  // FIFO empty flag (output from DUT)

    // =========================================================================
    // CLOCKING BLOCK 1: wdrv_cb - Write Driver Clocking Block
    //
    // Used by: fifo_write_driver (active agent)
    // Clock  : wclk (write clock domain)
    // Outputs: wrst_n, winc, wdata → driven 1 time-unit AFTER wclk posedge
    // Inputs : wfull               → sampled #1step BEFORE wclk posedge
    //
    // The #1step input skew samples DUT outputs just before the active edge,
    // equivalent to "sample at previous posedge + hold" - preventing
    // race conditions between drive and sample in zero-time simulation.
    // =========================================================================
    clocking wdrv_cb @(posedge wclk);
        default input  #1step;   // Sample inputs 1 simulation step before posedge
        default output #1;       // Drive outputs 1ns  after  posedge

        output wrst_n;           // Drive write reset
        output winc;             // Drive write enable
        output wdata;            // Drive write data

        input  wfull;            // Sample write-full flag from DUT
    endclocking : wdrv_cb

    // =========================================================================
    // CLOCKING BLOCK 2: rdrv_cb - Read Driver Clocking Block
    //
    // Used by: fifo_read_driver (active agent)
    // Clock  : rclk (read clock domain)
    // Outputs: rrst_n, rinc → driven 1 time-unit AFTER rclk posedge
    // Inputs : rdata, rempty → sampled #1step BEFORE rclk posedge
    // =========================================================================
    clocking rdrv_cb @(posedge rclk);
        default input  #1step;
        default output #1;

        output rrst_n;           // Drive read reset
        output rinc;             // Drive read enable

        input  rdata;            // Sample read data from DUT
        input  rempty;           // Sample read-empty flag from DUT
    endclocking : rdrv_cb

    // =========================================================================
    // CLOCKING BLOCK 3: wmon_cb - Write Monitor Clocking Block
    //
    // Used by: fifo_write_monitor (passive - only inputs, no driving)
    // Clock  : wclk (write clock domain)
    // All signals sampled #1step before posedge (non-intrusive observation).
    //
    // The monitor observes ALL write-domain signals to build write transactions
    // and push them to the scoreboard via analysis ports.
    // =========================================================================
    clocking wmon_cb @(posedge wclk);
        default input #1step;    // Passive: only sample, never drive

        input wrst_n;            // Observe reset state
        input winc;              // Observe write enable
        input wdata;             // Observe data being written
        input wfull;             // Observe full flag from DUT
    endclocking : wmon_cb

    // =========================================================================
    // CLOCKING BLOCK 4: rmon_cb - Read Monitor Clocking Block
    //
    // Used by: fifo_read_monitor (passive - only inputs, no driving)
    // Clock  : rclk (read clock domain)
    // All signals sampled #1step before posedge (non-intrusive observation).
    //
    // The monitor observes ALL read-domain signals to capture read transactions
    // and compare against the scoreboard's expected output queue.
    // =========================================================================
    clocking rmon_cb @(posedge rclk);
        default input #1step;

        input rrst_n;            // Observe reset state
        input rinc;              // Observe read enable
        input rdata;             // Observe data read from DUT
        input rempty;            // Observe empty flag from DUT
    endclocking : rmon_cb

    // =========================================================================
    // MODPORTS
    // Modports restrict signal visibility and direction per component type.
    // They enforce interface discipline at elaboration time, preventing
    // accidental driving of input-only or monitoring-only signals.
    // =========================================================================

    // --- Modport: Write Driver -----------------------------------------------
    // Provides access to the write-domain DRIVER clocking block.
    // The write driver uses this modport to synchronize all drives to wclk.
    modport MP_WRITE_DRV (
        clocking wdrv_cb,        // Full access to write driver clocking block
        input    wclk            // Direct clock access for wait statements
    );

    // --- Modport: Read Driver ------------------------------------------------
    // Provides access to the read-domain DRIVER clocking block.
    modport MP_READ_DRV (
        clocking rdrv_cb,        // Full access to read driver clocking block
        input    rclk            // Direct clock access for wait statements
    );

    // --- Modport: Write Monitor ----------------------------------------------
    // Read-only view of write-domain signals via the monitor clocking block.
    modport MP_WRITE_MON (
        clocking wmon_cb,        // Passive write monitor clocking block
        input    wclk            // Clock for event-based wait
    );

    // --- Modport: Read Monitor -----------------------------------------------
    // Read-only view of read-domain signals via the monitor clocking block.
    modport MP_READ_MON (
        clocking rmon_cb,        // Passive read monitor clocking block
        input    rclk            // Clock for event-based wait
    );

    // --- Modport: DUT Connection (used in tb_top) ----------------------------
    // Exposes raw wire-level signal directions for direct DUT port binding.
    // This modport is used in the tb_top module where the DUT is instantiated.
    modport MP_DUT (
        // Write domain - inputs to DUT
        output wrst_n,
        output winc,
        output wdata,
        // Write domain - outputs from DUT
        input  wfull,

        // Read domain - inputs to DUT
        output rrst_n,
        output rinc,
        // Read domain - outputs from DUT
        input  rdata,
        input  rempty
    );

    // =========================================================================
    // INLINE PROTOCOL ASSERTIONS
    // These assertions are always-active during simulation (no need to
    // instantiate them from a bind module - they live in the interface itself).
    // All assertions reference clocking blocks for glitch-free sampling.
    //
    // Disable condition:
    //   wrst_n / rrst_n = 0 → assertions disabled during reset
    //   Using ($fell / $rose) for clean reset handling.
    // =========================================================================

    // --- Write Domain Assertions ---------------------------------------------

    // [AST-IF-W01] No write when FIFO is full
    // If wfull is asserted, winc must NOT be asserted in the same cycle.
    property p_no_write_when_full;
        @(wdrv_cb) disable iff (!wrst_n)
        wdrv_cb.wfull |-> !wdrv_cb.winc;
    endproperty
    AST_IF_no_write_when_full : assert property (p_no_write_when_full)
        else $error("[fifo_if][AST-IF-W01] at %0t: winc asserted while wfull=1!", $time);

    // [AST-IF-W02] wfull must deassert after reset releases
    property p_wfull_clear_on_reset;
        @(posedge wclk)
        $rose(wrst_n) |=> !wfull;
    endproperty
    AST_IF_wfull_clear_on_reset : assert property (p_wfull_clear_on_reset)
        else $error("[fifo_if][AST-IF-W02] at %0t: wfull did not clear within 1 cycle of wrst_n release!", $time);

    // [AST-IF-W03] wdata must not be X/Z during a valid write
    property p_wdata_no_x_on_write;
        @(wdrv_cb) disable iff (!wrst_n)
        (wdrv_cb.winc && !wdrv_cb.wfull) |-> !$isunknown(wdrv_cb.wdata);
    endproperty
    AST_IF_wdata_no_x_on_write : assert property (p_wdata_no_x_on_write)
        else $error("[fifo_if][AST-IF-W03] at %0t: X/Z on wdata during valid write!", $time);

    // --- Read Domain Assertions ----------------------------------------------

    // [AST-IF-R01] No read when FIFO is empty
    property p_no_read_when_empty;
        @(rdrv_cb) disable iff (!rrst_n)
        rdrv_cb.rempty |-> !rdrv_cb.rinc;
    endproperty
    AST_IF_no_read_when_empty : assert property (p_no_read_when_empty)
        else $error("[fifo_if][AST-IF-R01] at %0t: rinc asserted while rempty=1!", $time);

    // [AST-IF-R02] rempty must assert after reset releases
    property p_rempty_set_on_reset;
        @(posedge rclk)
        $rose(rrst_n) |=> rempty;
    endproperty
    AST_IF_rempty_set_on_reset : assert property (p_rempty_set_on_reset)
        else $error("[fifo_if][AST-IF-R02] at %0t: rempty did not assert within 1 cycle of rrst_n release!", $time);

    // [AST-IF-R03] rdata must not be X/Z during a valid read
    property p_rdata_no_x_on_read;
        @(rdrv_cb) disable iff (!rrst_n)
        (rdrv_cb.rinc && !rdrv_cb.rempty) |-> !$isunknown(rdrv_cb.rdata);
    endproperty
    AST_IF_rdata_no_x_on_read : assert property (p_rdata_no_x_on_read)
        else $error("[fifo_if][AST-IF-R03] at %0t: X/Z on rdata during valid read!", $time);

    // =========================================================================
    // COVER PROPERTIES (Functional Coverage via Interface)
    // These ensure that key scenarios are exercised during simulation.
    // =========================================================================

    // [COV-IF-01] Cover: write to an almost-full FIFO (last slot)
    // (Uses raw signal - no access to occupancy counter here; covered in SB)

    // [COV-IF-02] Cover: wfull assertion event
    COV_IF_wfull_asserted : cover property (
        @(posedge wclk) $rose(wfull)
    );

    // [COV-IF-03] Cover: rempty assertion event
    COV_IF_rempty_asserted : cover property (
        @(posedge rclk) $rose(rempty)
    );

    // [COV-IF-04] Cover: wfull deassertion (read freed a slot)
    COV_IF_wfull_deasserted : cover property (
        @(posedge wclk) $fell(wfull)
    );

    // [COV-IF-05] Cover: rempty deassertion (write produced data)
    COV_IF_rempty_deasserted : cover property (
        @(posedge rclk) $fell(rempty)
    );

    // [COV-IF-06] Cover: simultaneous write and read activity
    COV_IF_simultaneous_rw : cover property (
        @(posedge wclk) winc && rinc && !wfull && !rempty
    );

endinterface : async_fifo_if

`default_nettype wire
// =============================================================================
// End of fifo_if.sv
// =============================================================================
