// =============================================================================
// File        : fifo_wr_monitor.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : UVM Write-Domain Monitor.
//               Passively observes the write-side interface signals using the
//               'wmon_cb' clocking block (all inputs, #1step sampled).
//               Constructs a fifo_seq_item for every VALID write transaction
//               (winc=1 AND wfull=0 AND wrst_n=1) and broadcasts it via the
//               UVM analysis port to connected subscribers:
//                 - fifo_scoreboard  : pushes data_in into reference queue
//                 - fifo_coverage    : samples write-domain covergroups
//
// Analysis Port: ap (uvm_analysis_port #(fifo_seq_item))
//   Transaction contents:
//     item.wr_en     = winc   sampled at write clock edge
//     item.data_in   = wdata  sampled at write clock edge
//     item.got_wfull = wfull  sampled at write clock edge
//     item.txn_type  = FIFO_WRITE_ONLY
//
// Config DB Key : "WMON_VIF" → virtual async_fifo_if
//
// Coding Standard : IEEE 1800-2012 | UVM 1.2
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// =============================================================================

`ifndef FIFO_WR_MONITOR_SV
`define FIFO_WR_MONITOR_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

class fifo_wr_monitor #(
    parameter int unsigned DATA_WIDTH = 8
) extends uvm_monitor;

    `uvm_component_param_utils(fifo_wr_monitor #(DATA_WIDTH))

    // =========================================================================
    // Analysis Port: broadcasts valid write transactions to all subscribers
    // =========================================================================
    uvm_analysis_port #(fifo_seq_item #(DATA_WIDTH)) ap;

    // =========================================================================
    // Virtual Interface Handle
    // =========================================================================
    virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)) vif;

    // =========================================================================
    // Statistics Counters (accessible via uvm_report or direct query)
    // =========================================================================
    int unsigned num_writes;        // Count of valid writes observed
    int unsigned num_full_events;   // Count of wfull assertion events seen
    int unsigned num_resets;        // Count of wrst_n assertion events seen

    // =========================================================================
    // Constructor
    // =========================================================================
    function new(string name = "fifo_wr_monitor", uvm_component parent = null);
        super.new(name, parent);
        num_writes      = 0;
        num_full_events = 0;
        num_resets      = 0;
    endfunction : new

    // =========================================================================
    // build_phase
    // =========================================================================
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);

        // Create analysis port
        ap = new("ap", this);

        // Retrieve virtual interface
        if (!uvm_config_db #(virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)))::get(
                this, "", "WMON_VIF", vif)) begin
            `uvm_fatal("FIFO_WR_MON", "Virtual interface 'WMON_VIF' not found in config_db.")
        end
    endfunction : build_phase

    // =========================================================================
    // run_phase: Passive observation loop - never drives any signal
    // =========================================================================
    task run_phase(uvm_phase phase);
        fifo_seq_item #(DATA_WIDTH) item;

        `uvm_info("FIFO_WR_MON", "Write monitor started - observing wmon_cb", UVM_MEDIUM)

        // Wait for reset to deassert before beginning observation
        wait_for_reset();

        forever begin
            // Synchronize to the write clock via wmon_cb (#1step sampling)
            @(vif.wmon_cb);

            // ------------------------------------------------------------------
            // Observe reset assertion (wrst_n going low mid-simulation)
            // ------------------------------------------------------------------
            if (!vif.wmon_cb.wrst_n) begin
                `uvm_info("FIFO_WR_MON", "Detected wrst_n assertion - pausing observation", UVM_MEDIUM)
                num_resets++;
                wait_for_reset();  // Block until reset is released
                continue;
            end

            // ------------------------------------------------------------------
            // Observe wfull assertion event
            // ------------------------------------------------------------------
            if (vif.wmon_cb.wfull) begin
                num_full_events++;
                `uvm_info("FIFO_WR_MON",
                    $sformatf("wfull asserted (event #%0d)", num_full_events), UVM_HIGH)
            end

            // ------------------------------------------------------------------
            // Capture VALID write transaction: winc=1 AND wfull=0 AND wrst_n=1
            // ------------------------------------------------------------------
            if (vif.wmon_cb.winc && !vif.wmon_cb.wfull && vif.wmon_cb.wrst_n) begin
                item = fifo_seq_item #(DATA_WIDTH)::type_id::create("wr_item");
                collect_write_txn(item);
                num_writes++;

                `uvm_info("FIFO_WR_MON",
                    $sformatf("[WRITE #%0d] data=0x%0h wfull=%0b",
                        num_writes, item.data_in, item.got_wfull), UVM_MEDIUM)

                // Broadcast to scoreboard and coverage
                ap.write(item);
            end
        end
    endtask : run_phase

    // =========================================================================
    // TASK: wait_for_reset
    // Blocks until wrst_n is deasserted (high). Also handles the case where
    // the monitor starts before any reset is applied.
    // =========================================================================
    task wait_for_reset();
        // If already out of reset, return immediately
        if (vif.wmon_cb.wrst_n) return;
        // Wait for reset to deassert
        @(posedge vif.wclk iff vif.wrst_n);
        // Add 1 extra cycle for synchronizer settling
        @(vif.wmon_cb);
        `uvm_info("FIFO_WR_MON", "Write domain reset released - resuming observation", UVM_MEDIUM)
    endtask : wait_for_reset

    // =========================================================================
    // TASK: collect_write_txn
    // Populates a fifo_seq_item from the current wmon_cb snapshot.
    // All fields are read from the clocking block - no direct signal access.
    // =========================================================================
    task collect_write_txn(fifo_seq_item #(DATA_WIDTH) item);
        item.wr_en     = vif.wmon_cb.winc;   // Write enable observed
        item.data_in   = vif.wmon_cb.wdata;  // Write data payload
        item.got_wfull = vif.wmon_cb.wfull;  // Full flag at time of write
        item.txn_type  = FIFO_WRITE_ONLY;    // Classification
        // rd_en, data_out, got_rempty are left at default (not observable here)
    endtask : collect_write_txn

    // =========================================================================
    // report_phase: Print observation statistics at end of test
    // =========================================================================
    function void report_phase(uvm_phase phase);
        `uvm_info("FIFO_WR_MON",
            $sformatf("\n=== Write Monitor Statistics ===\n  Valid writes observed : %0d\n  wfull events seen     : %0d\n  wrst_n assertions     : %0d",
                num_writes, num_full_events, num_resets), UVM_NONE)
    endfunction : report_phase

endclass : fifo_wr_monitor

`endif // FIFO_WR_MONITOR_SV
// =============================================================================
// End of fifo_wr_monitor.sv
// =============================================================================
