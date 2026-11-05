// =============================================================================
// File        : tb_top.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : Absolute Top-Level Simulation Module.
//               Instantiates all verification and RTL infrastructure:
//               clock generators, reset logic, DUT, interface, and UVM.
//
// Clock Generation:
//   wclk : 100.0 MHz  (period = 10 ns, half = 5 ns)
//   rclk :  33.3 MHz  (period = 30 ns, half = 15 ns)  3:1 wclk:rclk ratio
//   Phase: rclk intentionally offset by 7.3 ns to guarantee asynchronous
//          (non-locked) phase relationship between the two domains.
//
// Reset Strategy:
//   tb_top provides power-on reset (wrst_n=0, rrst_n=0) for the first
//   20 wclk cycles. The UVM sequences take over reset control thereafter.
//
// UVM Integration:
//   - VIF is set in uvm_config_db for all 4 consumer keys before run_test().
//   - run_test() is called with no hardcoded test name - use +UVM_TESTNAME.
//   - Simulation timeout watchdog: 5 ms (prevents infinite stalls).
//
// Compilation Order (for manual compilation):
//   1. rtl/dp_ram.sv
//   2. rtl/sync_2ff.sv
//   3. rtl/async_fifo.sv
//   4. tb/fifo_if.sv
//   5. tb/fifo_sva.sv          (contains bind statement)
//   6. tb/fifo_seq_item.sv
//   7. tb/fifo_wr_driver.sv
//   8. tb/fifo_wr_monitor.sv
//   9. tb/fifo_rd_driver.sv
//   10. tb/fifo_rd_monitor.sv
//   11. tb/fifo_agent.sv
//   12. tb/fifo_scoreboard.sv
//   13. tb/fifo_coverage.sv
//   14. tb/fifo_env.sv
//   15. tb/fifo_base_seq.sv
//   16. tb/fifo_sanity_seq.sv
//   17. tb/fifo_stress_seq.sv
//   18. tb/fifo_base_test.sv
//   19. tb/tb_top.sv
//
// Simulation Commands (Questa/ModelSim):
//   vlog -sv -mfcu +incdir+rtl +incdir+tb \
//        rtl/*.sv tb/fifo_if.sv tb/fifo_sva.sv tb/*.sv -L uvm_lib
//   vsim -L uvm_lib tb_top +UVM_TESTNAME=fifo_stress_test \
//        +UVM_VERBOSITY=UVM_MEDIUM -sv_seed random
//
// Coding Standard : IEEE 1800-2012 (SystemVerilog)
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// Version         : 1.0
// =============================================================================

`timescale 1ns / 1ps

// UVM infrastructure
`include "uvm_macros.svh"
import uvm_pkg::*;

// Interface and RTL includes
//`include "fifo_if.sv"
//`include "fifo_sva.sv"

// UVM testbench includes (compilation order enforced)
`include "fifo_seq_item.sv"
`include "fifo_wr_driver.sv"
`include "fifo_wr_monitor.sv"
`include "fifo_rd_driver.sv"
`include "fifo_rd_monitor.sv"
`include "fifo_agent.sv"
`include "fifo_scoreboard.sv"
`include "fifo_coverage.sv"
`include "fifo_env.sv"
`include "fifo_base_seq.sv"
`include "fifo_sanity_seq.sv"
`include "fifo_stress_seq.sv"
`include "fifo_base_test.sv"

// =============================================================================
// MODULE: tb_top
// =============================================================================
module tb_top;

    // =========================================================================
    // DUT Configuration Parameters
    // =========================================================================
    localparam int unsigned DATA_WIDTH = 8;
    localparam int unsigned FIFO_DEPTH = 16;

    // =========================================================================
    // Clock Generation
    // =========================================================================
    // Write clock: 100.0 MHz
    localparam real WCLK_HALF_PERIOD_NS = 5.0;    // 5ns half-period → 10ns period
    // Read clock:  33.3 MHz
    localparam real RCLK_HALF_PERIOD_NS = 15.0;   // 15ns half-period → 30ns period
    // Phase offset for rclk (ensures non-synchronous relationship)
    localparam real RCLK_PHASE_OFFSET_NS = 7.3;   // Irrational fraction of period

    logic wclk;
    logic rclk;

    // Write clock: 100 MHz, starts immediately
    initial wclk = 1'b0;
    always #(WCLK_HALF_PERIOD_NS) wclk = ~wclk;

    // Read clock: 33.3 MHz, starts with phase offset for asynchronous CDC
    initial begin
        rclk = 1'b0;
        #(RCLK_PHASE_OFFSET_NS);   // Phase offset: makes clocks truly async
    end
    always #(RCLK_HALF_PERIOD_NS) rclk = ~rclk;

    // =========================================================================
    // Interface Instantiation
    // Clocks are injected here - all signals flow through the interface
    // to DUT and UVM components.
    // =========================================================================
    async_fifo_if #(
        .DATA_WIDTH (DATA_WIDTH)
    ) fifo_if (
        .wclk (wclk),
        .rclk (rclk)
    );

    // =========================================================================
    // DUT Instantiation
    // Signals are connected from the interface (no separate signal declarations
    // needed in tb_top - the interface is the single source of truth).
    // =========================================================================
    async_fifo #(
        .DATA_WIDTH (DATA_WIDTH),
        .FIFO_DEPTH (FIFO_DEPTH)
    ) dut (
        // Write domain
        .wclk   (fifo_if.wclk),
        .wrst_n (fifo_if.wrst_n),
        .winc   (fifo_if.winc),
        .wdata  (fifo_if.wdata),
        .wfull  (fifo_if.wfull),
        // Read domain
        .rclk   (fifo_if.rclk),
        .rrst_n (fifo_if.rrst_n),
        .rinc   (fifo_if.rinc),
        .rdata  (fifo_if.rdata),
        .rempty (fifo_if.rempty)
    );
    // NOTE: fifo_sva is automatically bound to async_fifo via the 'bind'
    // statement at the bottom of fifo_sva.sv - no explicit instantiation needed.

    // =========================================================================
    // Power-On Reset
    // Assert both domain resets for 20 wclk cycles at simulation start.
    // The UVM sequences (do_reset task in fifo_base_seq) will re-apply
    // resets during test execution as needed.
    // =========================================================================
    initial begin : power_on_reset
        // Drive all write-domain inputs to safe idle state
        fifo_if.wrst_n = 1'b0;
        fifo_if.winc   = 1'b0;
        fifo_if.wdata  = {DATA_WIDTH{1'b0}};

        // Drive all read-domain inputs to safe idle state
        fifo_if.rrst_n = 1'b0;
        fifo_if.rinc   = 1'b0;

        // Hold reset for 20 write-clock cycles
        repeat(20) @(posedge wclk);

        // Deassert write-domain reset
        @(posedge wclk);
        fifo_if.wrst_n = 1'b1;

        // Deassert read-domain reset (3 rclk cycles after write domain)
        repeat(3) @(posedge rclk);
        fifo_if.rrst_n = 1'b1;

        $display("[tb_top] %0t: Power-on reset complete. wrst_n=1, rrst_n=1.", $time);
    end : power_on_reset

    // =========================================================================
    // UVM Configuration Database - Virtual Interface Registration
    //
    // All four VIF keys are set with the SAME interface handle but consumed
    // by different components (each uses a different clocking block):
    //   WDRV_VIF → fifo_wr_driver  (uses wdrv_cb)
    //   WMON_VIF → fifo_wr_monitor (uses wmon_cb)
    //   RDRV_VIF → fifo_rd_driver  (uses rdrv_cb)
    //   RMON_VIF → fifo_rd_monitor (uses rmon_cb)
    //
    // The wildcard path ("*") ensures all UVM components at any hierarchy
    // depth can retrieve the interface without path-specific set calls.
    // =========================================================================
    initial begin : uvm_config_setup
        // ALL OF THIS HAPPENS AT TIME 0

        // Register virtual interface for write-domain driver
        uvm_config_db #(virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)))::set(
            null, "*", "WDRV_VIF", fifo_if);

        // Register virtual interface for write-domain monitor
        uvm_config_db #(virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)))::set(
            null, "*", "WMON_VIF", fifo_if);

        // Register virtual interface for read-domain driver
        uvm_config_db #(virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)))::set(
            null, "*", "RDRV_VIF", fifo_if);

        // Register virtual interface for read-domain monitor
        uvm_config_db #(virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)))::set(
            null, "*", "RMON_VIF", fifo_if);

        $display("[tb_top] %0t: All virtual interfaces registered in uvm_config_db.", $time);

        // =====================================================================
        // Start the UVM test at TIME 0!
        // =====================================================================
        run_test();
    end : uvm_config_setup

    // =========================================================================
    // Simulation Timeout Watchdog
    // If the test does not complete within TIMEOUT_NS, force $finish.
    // Prevents infinite stalls from deadlocked sequences.
    // =========================================================================
    localparam real TIMEOUT_NS = 5_000_000.0;  // 5 ms simulation timeout

    initial begin : watchdog
        #(TIMEOUT_NS);
        $display("[tb_top] WATCHDOG TIMEOUT at %0t ns! Simulation forced to end.", $time);
        $display("[tb_top] Check for deadlocked sequences (stalled on wfull or rempty).");
        $finish(2);
    end : watchdog

    // =========================================================================
    // Waveform Dump (VCD / FSDB)
    // Uncomment the appropriate block for your simulator:
    // =========================================================================

    // --- VCD (Icarus / ModelSim free) ----------------------------------------
    initial begin : waveform_vcd
        $dumpfile("fifo_sim.vcd");
        $dumpvars(0, tb_top);   // 0 = dump all levels from tb_top downward
    end : waveform_vcd

    // --- FSDB (Verdi / Debussy) - uncomment if using Synopsys tools ----------
    // initial begin : waveform_fsdb
    //     $fsdbDumpfile("fifo_sim.fsdb");
    //     $fsdbDumpvars(0, tb_top);
    //     $fsdbDumpSVA(0, tb_top);   // Also dump SVA assertion activity
    // end : waveform_fsdb

    // =========================================================================
    // Simulation Start Banner
    // =========================================================================
    initial begin : banner
        $display("");
        $display("==============================================================");
        $display("  Async FIFO UVM Testbench - Simulation Start");
        $display("  wclk = %.1f MHz (period = %.1f ns)",
                 1000.0 / (2.0 * WCLK_HALF_PERIOD_NS),
                 2.0 * WCLK_HALF_PERIOD_NS);
        $display("  rclk = %.1f MHz (period = %.1f ns)",
                 1000.0 / (2.0 * RCLK_HALF_PERIOD_NS),
                 2.0 * RCLK_HALF_PERIOD_NS);
        $display("  rclk phase offset = %.1f ns", RCLK_PHASE_OFFSET_NS);
        $display("  DUT: DATA_WIDTH=%0d, FIFO_DEPTH=%0d", DATA_WIDTH, FIFO_DEPTH);
        $display("  Timeout: %.0f ns", TIMEOUT_NS);
        $display("==============================================================");
        $display("");
    end : banner

endmodule : tb_top
// =============================================================================
// End of tb_top.sv
// =============================================================================
