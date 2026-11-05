// =============================================================================
// File        : design.sv  (EDA Playground - "Design" tab)
// Project     : Asynchronous FIFO with CDC Verification
//
// Purpose     : Single compilation entry-point for all RTL source files.
//               In EDA Playground, paste this file into the LEFT (Design) tab.
//               All other RTL source files must be uploaded individually via
//               the EDA Playground "Files" panel so `include can locate them.
//
// Compilation Order (dependency-driven):
//   1. dp_ram.sv      - Parameterized dual-port sync RAM (no dependencies)
//   2. sync_2ff.sv    - 2-stage CDC synchronizer   (no dependencies)
//   3. async_fifo.sv  - Top-level FIFO wrapper     (depends on 1 and 2)
//
// EDA Playground Instructions:
//   - Simulator   : Aldec Riviera-PRO 2023.04 (or later)
//   - UVM Version : UVM 1.2  (check "UVM" checkbox in Libraries panel)
//   - Compile Opt : -sv2012    (ensures IEEE 1800-2012 compliance)
//   - Top Module  : tb_top    (set in "Testbench" tab → Top Module field)
//   - Run Options : +UVM_TESTNAME=fifo_stress_test +UVM_VERBOSITY=UVM_MEDIUM
//
// NOTE: Do NOT set a top module on the Design side - the RTL is instantiated
//       by tb_top.sv on the testbench side via DUT instantiation.
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

// -----------------------------------------------------------------------------
// BLOCK 1: Parameterized Dual-Port RAM
// Provides independent read and write ports with separate clock enables.
// Supports both synchronous-read and asynchronous-read configurations.
// No dependencies on other RTL files.
// -----------------------------------------------------------------------------
`include "dp_ram.sv"

// -----------------------------------------------------------------------------
// BLOCK 2: 2-Stage Flip-Flop CDC Synchronizer
// Metastability-hardened with ASYNC_REG synthesis attribute pragma.
// Parameterized by WIDTH (default: pointer width = log2(DEPTH)+1).
// No dependencies on other RTL files.
// -----------------------------------------------------------------------------
`include "sync_2ff.sv"

// -----------------------------------------------------------------------------
// BLOCK 3: Asynchronous FIFO Top-Level Wrapper
// Implements:
//   - Binary-to-Gray pointer conversion (wclk and rclk domains)
//   - Full flag  : wptr_gray vs. rptr_gray_sync  (wclk domain)
//   - Empty flag : rptr_gray vs. wptr_gray_sync  (rclk domain)
//   - Instantiates dp_ram and two sync_2ff instances
// Depends on: dp_ram.sv, sync_2ff.sv (compiled above)
// -----------------------------------------------------------------------------
`include "async_fifo.sv"

`default_nettype wire
// =============================================================================
// End of design.sv
// =============================================================================
