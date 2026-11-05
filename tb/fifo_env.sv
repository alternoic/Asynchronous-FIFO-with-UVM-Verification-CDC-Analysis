// =============================================================================
// File        : fifo_env.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : UVM Environment - top-level container that instantiates,
//               configures, and wires all verification components together.
//
// Component Hierarchy:
//   fifo_env
//   |--- fifo_wr_agent   (wr_agent)    - write clock domain
//   |   |--- uvm_sequencer             - wr_agent.sequencer
//   |   |--- fifo_wr_driver
//   |   |--- fifo_wr_monitor
//   |--- fifo_rd_agent   (rd_agent)    - read clock domain
//   |   |--- uvm_sequencer             - rd_agent.sequencer
//   |   |--- fifo_rd_driver
//   |   |--- fifo_rd_monitor
//   |--- fifo_scoreboard (scoreboard)
//   |--- fifo_coverage   (coverage)
//   |--- fifo_virtual_sequencer (vseqr)
//
// TLM Analysis Port Connectivity Map:
//
//   wr_agent.ap       --(fan-out)--> scoreboard.wr_export   (data push)
//                                --> coverage.wr_export      (write coverage)
//
//   rd_agent.ap_read  ----------> scoreboard.rd_export   (data pop + compare)
//
//   rd_agent.ap_flag  ----------> coverage.rd_export     (flag + read coverage)
//
// Virtual Sequencer:
//   fifo_virtual_sequencer holds references to both agent sequencers,
//   enabling virtual sequences to orchestrate both clock domains concurrently.
//   vseqr.wr_seqr → wr_agent.sequencer
//   vseqr.rd_seqr → rd_agent.sequencer
//
// config_db Keys Set by This Env (consumed by agents and drivers):
//   "WDRV_VIF" → virtual async_fifo_if  (write driver)
//   "WMON_VIF" → virtual async_fifo_if  (write monitor)
//   "RDRV_VIF" → virtual async_fifo_if  (read  driver)
//   "RMON_VIF" → virtual async_fifo_if  (read  monitor)
//   (These keys are set by tb_top, not by env - the env only retrieves them.)
//
// Coding Standard : IEEE 1800-2012 | UVM 1.2
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// Version         : 1.0
// =============================================================================

`ifndef FIFO_ENV_SV
`define FIFO_ENV_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

// Include all sub-components (order matters for compilation dependency)
`include "fifo_seq_item.sv"
`include "fifo_agent.sv"       // includes drivers and monitors transitively
`include "fifo_scoreboard.sv"
`include "fifo_coverage.sv"

// =============================================================================
// CLASS: fifo_virtual_sequencer
// A lightweight sequencer that holds handles to both the write-domain and
// read-domain sequencers. Virtual sequences use `uvm_declare_p_sequencer
// to cast p_sequencer to this type and access both sub-sequencers.
// =============================================================================
class fifo_virtual_sequencer #(
    parameter int unsigned DATA_WIDTH = 8
) extends uvm_sequencer;

    `uvm_component_param_utils(fifo_virtual_sequencer #(DATA_WIDTH))

    // Handles to domain-specific sequencers (set in env.connect_phase)
    uvm_sequencer #(fifo_seq_item #(DATA_WIDTH)) wr_seqr;
    uvm_sequencer #(fifo_seq_item #(DATA_WIDTH)) rd_seqr;

    function new(string name = "fifo_virtual_sequencer", uvm_component parent = null);
        super.new(name, parent);
    endfunction : new

endclass : fifo_virtual_sequencer

// =============================================================================
// CLASS: fifo_env
// Top-level UVM environment.
// =============================================================================
class fifo_env #(
    parameter int unsigned DATA_WIDTH  = 8,
    parameter int unsigned FIFO_DEPTH  = 16
) extends uvm_env;

    `uvm_component_param_utils(fifo_env #(DATA_WIDTH, FIFO_DEPTH))

    // =========================================================================
    // Component Handles
    // =========================================================================
    fifo_wr_agent    #(.DATA_WIDTH(DATA_WIDTH))                      wr_agent;
    fifo_rd_agent    #(.DATA_WIDTH(DATA_WIDTH))                      rd_agent;
    fifo_scoreboard  #(.DATA_WIDTH(DATA_WIDTH), .FIFO_DEPTH(FIFO_DEPTH)) scoreboard;
    fifo_coverage    #(.DATA_WIDTH(DATA_WIDTH), .FIFO_DEPTH(FIFO_DEPTH)) coverage;
    fifo_virtual_sequencer #(.DATA_WIDTH(DATA_WIDTH))                vseqr;

    // =========================================================================
    // Constructor
    // =========================================================================
    function new(string name = "fifo_env", uvm_component parent = null);
        super.new(name, parent);
    endfunction : new

    // =========================================================================
    // build_phase
    // Creates all sub-components using the UVM factory.
    // Agent is_active mode can be overridden via config_db if needed.
    // =========================================================================
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);

        // Create both agents in ACTIVE mode (drivers + sequencers + monitors)
        wr_agent   = fifo_wr_agent   #(.DATA_WIDTH(DATA_WIDTH))::type_id::create(
                         "wr_agent", this);
        rd_agent   = fifo_rd_agent   #(.DATA_WIDTH(DATA_WIDTH))::type_id::create(
                         "rd_agent", this);

        // Create scoreboard and coverage
        scoreboard = fifo_scoreboard #(.DATA_WIDTH(DATA_WIDTH),
                                       .FIFO_DEPTH(FIFO_DEPTH))::type_id::create(
                         "scoreboard", this);
        coverage   = fifo_coverage   #(.DATA_WIDTH(DATA_WIDTH),
                                       .FIFO_DEPTH(FIFO_DEPTH))::type_id::create(
                         "coverage", this);

        // Create virtual sequencer (holds pointers to domain sequencers)
        vseqr      = fifo_virtual_sequencer #(.DATA_WIDTH(DATA_WIDTH))::type_id::create(
                         "vseqr", this);

        `uvm_info("FIFO_ENV",
            $sformatf("Environment built: DATA_WIDTH=%0d FIFO_DEPTH=%0d",
                DATA_WIDTH, FIFO_DEPTH), UVM_MEDIUM)
    endfunction : build_phase

    // =========================================================================
    // connect_phase
    // Wires TLM analysis ports, analysis imps, and virtual sequencer handles.
    //
    // Fan-out wiring (UVM analysis port → multiple subscribers):
    //   UVM analysis ports natively support one-to-many connections.
    //   Each call to port.connect(imp) adds a new subscriber to the port's
    //   internal list. When port.write(item) is called, all connected
    //   subscribers receive the item in the order they were connected.
    //
    // Connection diagram:
    //   wr_agent.ap  --> scoreboard.wr_export  [push to reference queue]
    //   wr_agent.ap  --> coverage.wr_export    [write-domain coverage]
    //   rd_agent.ap_read --> scoreboard.rd_export [pop + compare]
    //   rd_agent.ap_flag --> coverage.rd_export   [flag + read coverage]
    // =========================================================================
    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);

        // --- Scoreboard connections ------------------------------------------
        // Write monitor analysis port → scoreboard write import
        wr_agent.ap.connect(scoreboard.wr_export);

        // Read monitor valid-read port → scoreboard read import
        rd_agent.ap_read.connect(scoreboard.rd_export);

        // --- Coverage connections --------------------------------------------
        // Write monitor analysis port → coverage write import (fan-out from wr_agent.ap)
        wr_agent.ap.connect(coverage.wr_export);

        // Read monitor all-cycle flag port → coverage read import
        // Note: ap_flag fires every rclk cycle (superset of valid reads),
        //       which gives coverage the full flag-transition picture.
        rd_agent.ap_flag.connect(coverage.rd_export);

        // --- Virtual sequencer connections -----------------------------------
        // Give the virtual sequencer handles to both domain sequencers
        // so virtual sequences can drive both domains concurrently.
        vseqr.wr_seqr = wr_agent.sequencer;
        vseqr.rd_seqr = rd_agent.sequencer;

        `uvm_info("FIFO_ENV", "TLM connections established successfully.", UVM_MEDIUM)
    endfunction : connect_phase

    // =========================================================================
    // start_of_simulation_phase: Print component topology
    // =========================================================================
    function void start_of_simulation_phase(uvm_phase phase);
        `uvm_info("FIFO_ENV", "\n=== Environment Topology ===", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  fifo_env", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  |--- fifo_wr_agent  [ACTIVE]", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  |   |--- uvm_sequencer (wr_seqr)", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  |   |--- fifo_wr_driver", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  |   |--- fifo_wr_monitor  --> ap", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  |--- fifo_rd_agent  [ACTIVE]", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  |   |--- uvm_sequencer (rd_seqr)", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  |   |--- fifo_rd_driver", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  |   |--- fifo_rd_monitor  --> ap_read, ap_flag", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  |--- fifo_scoreboard  <-- wr_export, rd_export", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  |--- fifo_coverage    <-- wr_export, rd_export", UVM_MEDIUM)
        `uvm_info("FIFO_ENV", "  |--- fifo_virtual_sequencer", UVM_MEDIUM)
    endfunction : start_of_simulation_phase

endclass : fifo_env

`endif // FIFO_ENV_SV
// =============================================================================
// End of fifo_env.sv
// =============================================================================
