// =============================================================================
// File        : fifo_rd_monitor.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : UVM Read-Domain Monitor.
//               Passively observes all read-side interface signals using the
//               'rmon_cb' clocking block (#1step sampled, non-intrusive).
//               Publishes two types of events via analysis ports:
//
//   ap_read  (primary) : Valid read transactions (rinc=1 AND rempty=0)
//               → Sent to scoreboard to compare data_out vs. reference queue.
//               → Sent to coverage collector for read-side covergroups.
//
//   ap_flag  (secondary) : Flag-only observations (any rclk edge)
//               → Sent to coverage collector for rempty transitions.
//               → Can be used by a flag checker component.
//
// Analysis Ports:
//   ap_read  : uvm_analysis_port #(fifo_seq_item) - valid reads
//   ap_flag  : uvm_analysis_port #(fifo_seq_item) - all flag samples
//
// Transaction contents on ap_read:
//   item.rd_en      = rinc    (observed)
//   item.data_out   = rdata   (read payload captured)
//   item.got_rempty = rempty  (empty flag at read time)
//   item.txn_type   = FIFO_READ_ONLY
//
// Config DB Key : "RMON_VIF" → virtual async_fifo_if
//
// Coding Standard : IEEE 1800-2012 | UVM 1.2
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// =============================================================================

`ifndef FIFO_RD_MONITOR_SV
`define FIFO_RD_MONITOR_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

class fifo_rd_monitor #(
    parameter int unsigned DATA_WIDTH = 8
) extends uvm_monitor;

    `uvm_component_param_utils(fifo_rd_monitor #(DATA_WIDTH))

    // =========================================================================
    // Analysis Ports
    // =========================================================================
    // Primary port: valid read transactions → scoreboard comparison
    uvm_analysis_port #(fifo_seq_item #(DATA_WIDTH)) ap_read;

    // Secondary port: all-cycle flag samples → coverage / flag checker
    uvm_analysis_port #(fifo_seq_item #(DATA_WIDTH)) ap_flag;

    // =========================================================================
    // Virtual Interface Handle
    // =========================================================================
    virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)) vif;

    // =========================================================================
    // Statistics Counters
    // =========================================================================
    int unsigned num_reads;          // Total valid reads observed
    int unsigned num_empty_events;   // rempty assertion events
    int unsigned num_resets;         // rrst_n assertion events
    // Consecutive rempty cycles (for stall detection reporting)
    int unsigned rempty_stall_cycles;

    // =========================================================================
    // Constructor
    // =========================================================================
    function new(string name = "fifo_rd_monitor", uvm_component parent = null);
        super.new(name, parent);
        num_reads          = 0;
        num_empty_events   = 0;
        num_resets         = 0;
        rempty_stall_cycles = 0;
    endfunction : new

    // =========================================================================
    // build_phase
    // =========================================================================
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);

        ap_read = new("ap_read", this);
        ap_flag = new("ap_flag", this);

        if (!uvm_config_db #(virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)))::get(
                this, "", "RMON_VIF", vif)) begin
            `uvm_fatal("FIFO_RD_MON", "Virtual interface 'RMON_VIF' not found in config_db.")
        end
    endfunction : build_phase

    // =========================================================================
    // run_phase: Passive observation loop - never drives any signal
    // =========================================================================
    task run_phase(uvm_phase phase);
        fifo_seq_item #(DATA_WIDTH) read_item;
        fifo_seq_item #(DATA_WIDTH) flag_item;

        `uvm_info("FIFO_RD_MON", "Read monitor started - observing rmon_cb", UVM_MEDIUM)

        // Wait for read-domain reset to deassert
        wait_for_reset();

        forever begin
            // Synchronize to rclk via rmon_cb
            @(vif.rmon_cb);

            // ------------------------------------------------------------------
            // Detect reset assertion mid-simulation
            // ------------------------------------------------------------------
            if (!vif.rmon_cb.rrst_n) begin
                `uvm_info("FIFO_RD_MON", "Detected rrst_n assertion - pausing observation", UVM_MEDIUM)
                num_resets++;
                rempty_stall_cycles = 0;
                wait_for_reset();
                continue;
            end

            // ------------------------------------------------------------------
            // Track rempty stall cycles (for diagnostics)
            // ------------------------------------------------------------------
            if (vif.rmon_cb.rempty) begin
                rempty_stall_cycles++;
                if (rempty_stall_cycles == 1) begin
                    num_empty_events++;
                    `uvm_info("FIFO_RD_MON",
                        $sformatf("rempty asserted (event #%0d)", num_empty_events), UVM_HIGH)
                end
            end else begin
                rempty_stall_cycles = 0;
            end

            // ------------------------------------------------------------------
            // Publish flag sample to ap_flag on every clock edge
            // (Coverage collector uses this for rempty transition tracking)
            // ------------------------------------------------------------------
            flag_item = fifo_seq_item #(DATA_WIDTH)::type_id::create("flag_item");
            flag_item.rd_en      = vif.rmon_cb.rinc;
            flag_item.got_rempty = vif.rmon_cb.rempty;
            flag_item.txn_type   = FIFO_IDLE;
            ap_flag.write(flag_item);

            // ------------------------------------------------------------------
            // Capture VALID read transaction: rinc=1 AND rempty=0 AND rrst_n=1
            // ------------------------------------------------------------------
            if (vif.rmon_cb.rinc && !vif.rmon_cb.rempty && vif.rmon_cb.rrst_n) begin
                read_item = fifo_seq_item #(DATA_WIDTH)::type_id::create("rd_item");
                collect_read_txn(read_item);
                num_reads++;

                `uvm_info("FIFO_RD_MON",
                    $sformatf("[READ #%0d] data=0x%0h rempty=%0b",
                        num_reads, read_item.data_out, read_item.got_rempty), UVM_MEDIUM)

                // Broadcast to scoreboard for data comparison
                ap_read.write(read_item);
            end
        end
    endtask : run_phase

    // =========================================================================
    // TASK: wait_for_reset
    // Blocks until rrst_n is deasserted (high).
    // =========================================================================
    task wait_for_reset();
        if (vif.rmon_cb.rrst_n) return;
        @(posedge vif.rclk iff vif.rrst_n);
        @(vif.rmon_cb);  // 1 settling cycle
        `uvm_info("FIFO_RD_MON", "Read domain reset released - resuming observation", UVM_MEDIUM)
    endtask : wait_for_reset

    // =========================================================================
    // TASK: collect_read_txn
    // Populates a fifo_seq_item from the current rmon_cb snapshot.
    // =========================================================================
    task collect_read_txn(fifo_seq_item #(DATA_WIDTH) item);
        item.rd_en      = vif.rmon_cb.rinc;    // Read enable observed
        item.data_out   = vif.rmon_cb.rdata;   // Data captured from FIFO output
        item.got_rempty = vif.rmon_cb.rempty;  // Empty flag at time of read
        item.txn_type   = FIFO_READ_ONLY;
    endtask : collect_read_txn

    // =========================================================================
    // report_phase: Print read statistics at end of test
    // =========================================================================
    function void report_phase(uvm_phase phase);
        `uvm_info("FIFO_RD_MON",
            $sformatf("\n=== Read Monitor Statistics ===\n  Valid reads observed  : %0d\n  rempty events seen    : %0d\n  rrst_n assertions     : %0d",
                num_reads, num_empty_events, num_resets), UVM_NONE)
    endfunction : report_phase

endclass : fifo_rd_monitor

`endif // FIFO_RD_MONITOR_SV
// =============================================================================
// End of fifo_rd_monitor.sv
// =============================================================================
