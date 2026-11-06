// =============================================================================
// File        : testbench.sv  (EDA Playground - "Testbench" tab)
// Project     : Asynchronous FIFO with CDC Verification
//
// Purpose     : Single compilation entry-point for the entire UVM testbench.
//               In EDA Playground, paste this file into the RIGHT (Testbench)
//               tab. All other TB source files must be uploaded via the "Files"
//               panel so `include can locate them by filename.
//
// UVM Package:
//   EDA Playground provides the UVM library as a pre-compiled package.
//   Check the "UVM 1.2" checkbox in the Libraries panel - do NOT manually
//   `include uvm_pkg.sv as the compiled library is already in scope.
//   The `include "uvm_macros.svh" is still needed for macros.
//
// Compilation Order (strict - dependency-driven):
//   Tier 0 : UVM infrastructure      (library, not included here)
//   Tier 1 : Interface definition     (fifo_if.sv)
//   Tier 2 : SVA + bind              (fifo_sva.sv - uses async_fifo internals)
//   Tier 3 : Sequence item           (fifo_seq_item.sv)
//   Tier 4 : Drivers & Monitors      (fifo_wr_driver, fifo_wr_monitor,
//                                     fifo_rd_driver, fifo_rd_monitor)
//   Tier 5 : Agents                  (fifo_agent.sv)
//   Tier 6 : Analysis subscribers    (fifo_scoreboard.sv, fifo_coverage.sv)
//   Tier 7 : Environment             (fifo_env.sv - includes virtual seqr)
//   Tier 8 : Sequences               (fifo_base_seq.sv, fifo_sanity_seq.sv,
//                                     fifo_stress_seq.sv)
//   Tier 9 : Tests                   (fifo_base_test.sv)
//   Tier 10: Simulation Top          (tb_top.sv - calls run_test())
//
// EDA Playground Settings (IMPORTANT - apply BEFORE running):
//   Simulator   : Aldec Riviera-PRO  (2023.04 or later recommended)
//     OR          Cadence Xcelium 23.03 (also supported)
//   Libraries   : ☑ UVM 1.2
//   Top Module  : tb_top
//   Compile Opt : -sv2012 +define+UVM_NO_DEPRECATED
//   Run Options : +UVM_TESTNAME=fifo_stress_test
//                 +UVM_VERBOSITY=UVM_MEDIUM
//                 -access +r       (for internal signal access via bind)
//   Timescale   : 1ns/1ps  (set in tb_top.sv `timescale directive)
//
// For SVA bind to work (fifo_sva.sv), the simulator must be granted
// read access to DUT internal signals. In Riviera-PRO this is the default.
// In VCS, add: -debug_access+all or -debug_pp to compile options.
// =============================================================================

`timescale 1ns / 1ps

// =============================================================================
// TIER 0: UVM Macros
// The UVM package itself is pre-compiled - only macros need including.
// =============================================================================
`include "uvm_macros.svh"

// Import UVM package into the compilation unit namespace
import uvm_pkg::*;

// =============================================================================
// TIER 3: UVM Sequence Item (Transaction Object)
// fifo_seq_item defines the fifo_txn_type_e enum used by all components.
// Must be compiled before any driver, monitor, or sequence.
// =============================================================================
`include "fifo_seq_item.sv"

// =============================================================================
// TIER 4: UVM Drivers and Monitors
// Order within this tier does not matter - no cross-dependencies.
// =============================================================================

// Write-domain driver: drives winc, wdata, wrst_n via wdrv_cb
`include "fifo_wr_driver.sv"
// Write-domain monitor: observes wmon_cb, broadcasts via ap
`include "fifo_wr_monitor.sv"

// Read-domain driver: drives rinc, rrst_n via rdrv_cb
`include "fifo_rd_driver.sv"

// Read-domain monitor: observes rmon_cb, broadcasts via ap_read and ap_flag
`include "fifo_rd_monitor.sv"

// =============================================================================
// TIER 1: SystemVerilog Interface
// Must be compiled before any module or component that references it.
// Contains: 4 clocking blocks, 5 modports, 6 inline assertions.
// =============================================================================
`include "fifo_if.sv"

// =============================================================================
// TIER 2: SVA Bind Module
// fifo_sva.sv defines the SVA module AND includes the bind statement.
// The bind statement is at compilation-unit scope (outside any module),
// which is valid per IEEE 1800-2012 Section 23.11.
// NOTE: async_fifo must already be compiled (done in design.sv) for the
// bind to resolve all internal signal names via .* auto-connection.
// =============================================================================
`include "fifo_sva.sv"







// =============================================================================
// TIER 5: UVM Agents
// fifo_agent.sv contains both fifo_wr_agent and fifo_rd_agent.
// Depends on: Tier 3 (seq item), Tier 4 (drivers and monitors).
// =============================================================================
`include "fifo_agent.sv"

// =============================================================================
// TIER 6: UVM Analysis Subscribers
// Scoreboard and coverage both use `uvm_analysis_imp_decl(_wr/_rd).
// NOTE: These macros create new types - both files must be compiled in the
// same compilation unit. The macro guard `FIFO_APS_DECLARED in coverage.sv
// prevents duplicate macro expansion if scoreboard is compiled first.
// =============================================================================

// Queue-based self-checking scoreboard (CDC-safe, no cycle comparison)
`include "fifo_scoreboard.sv"

// 6-covergroup functional coverage collector
`include "fifo_coverage.sv"

// =============================================================================
// TIER 7: UVM Environment
// Contains fifo_virtual_sequencer and fifo_env.
// Depends on: Tiers 3-6.
// =============================================================================
`include "fifo_env.sv"

// =============================================================================
// TIER 8: UVM Sequences
// Base sequence must precede derived sequences.
// =============================================================================

// Abstract base virtual sequence with utility task library
`include "fifo_base_seq.sv"

// Sanity/smoke test: 4 known-data writes → CDC settle → ordered readback
`include "fifo_sanity_seq.sv"

// Stress test: fill/overflow/drain/underflow/wrap-around/concurrent CDC
`include "fifo_stress_seq.sv"

// =============================================================================
// TIER 9: UVM Test Classes
// fifo_base_test.sv contains three classes:
//   fifo_base_test   (abstract)
//   fifo_sanity_test (+UVM_TESTNAME=fifo_sanity_test)
//   fifo_stress_test (+UVM_TESTNAME=fifo_stress_test)
// =============================================================================
`include "fifo_base_test.sv"

// =============================================================================
// TIER 10: Simulation Top Module
// tb_top instantiates: async_fifo_if, async_fifo DUT.
// Generates wclk (100 MHz) and rclk (33.3 MHz, +7.3 ns phase offset).
// Sets all 4 config_db VIF keys and calls run_test().
// Contains 5 ms watchdog and VCD/FSDB dump setup.
// =============================================================================
`include "tb_top.sv"

// =============================================================================
// End of testbench.sv
// =============================================================================
