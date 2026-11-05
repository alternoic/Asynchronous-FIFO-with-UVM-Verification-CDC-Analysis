// =============================================================================
// File        : fifo_rd_driver.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : UVM Read-Domain Driver.
//               Drives read-side stimulus (rinc, rrst_n) to the async_fifo
//               DUT through the 'rdrv_cb' clocking block of async_fifo_if.
//               Operates exclusively in the rclk domain.
//
// Behavior:
//   1. Retrieves fifo_seq_item from the read sequencer.
//   2. If rrst_req=1 : pulses rrst_n low for RESET_CYCLES rclk cycles.
//   3. Waits rd_idle rclk cycles (inter-transaction gap).
//   4. If rd_en=1   : stalls (backpressure) until rempty=0, then drives
//                     rinc=1 for one rclk cycle and captures rdata into
//                     the sequence item's data_out field.
//   5. Deasserts rinc=0 and calls item_done().
//
// Backpressure:
//   When rempty=1, the driver stalls until data is available. A $warning
//   is issued after BACKPRESSURE_WARN_CYCLES cycles to detect deadlocks.
//
// Response Capture:
//   After a valid read, the driver samples rdata from rdrv_cb and stores
//   it in req.data_out. This is used by sequences that check the read
//   result before signaling item_done - enabling in-sequence checking.
//
// Config DB Key : "RDRV_VIF" → virtual async_fifo_if
//
// Coding Standard : IEEE 1800-2012 | UVM 1.2
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// =============================================================================

`ifndef FIFO_RD_DRIVER_SV
`define FIFO_RD_DRIVER_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

class fifo_rd_driver #(
    parameter int unsigned DATA_WIDTH = 8
) extends uvm_driver #(fifo_seq_item #(DATA_WIDTH));

    `uvm_component_param_utils(fifo_rd_driver #(DATA_WIDTH))

    // =========================================================================
    // Local Parameters
    // =========================================================================
    localparam int unsigned RESET_CYCLES             = 4;
    localparam int unsigned BACKPRESSURE_WARN_CYCLES = 20;

    // =========================================================================
    // Virtual Interface Handle
    // =========================================================================
    virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)) vif;

    // =========================================================================
    // Constructor
    // =========================================================================
    function new(string name = "fifo_rd_driver", uvm_component parent = null);
        super.new(name, parent);
    endfunction : new

    // =========================================================================
    // build_phase
    // =========================================================================
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db #(virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)))::get(
                this, "", "RDRV_VIF", vif)) begin
            `uvm_fatal("FIFO_RD_DRV", "Virtual interface 'RDRV_VIF' not found in config_db. \
Ensure tb_top sets: uvm_config_db::set(null, \"*\", \"RDRV_VIF\", fifo_if)")
        end
    endfunction : build_phase

    // =========================================================================
    // run_phase
    // =========================================================================
    task run_phase(uvm_phase phase);
        // Initialize read-domain outputs to safe idle
        init_signals();

        // Apply initial reset
        apply_reset();

        forever begin
            seq_item_port.get_next_item(req);

            `uvm_info("FIFO_RD_DRV",
                $sformatf("Driving: %s", req.convert2string()), UVM_HIGH)

            drive_transaction(req);

            seq_item_port.item_done();
        end
    endtask : run_phase

    // =========================================================================
    // TASK: init_signals
    // =========================================================================
    task init_signals();
        vif.rdrv_cb.rrst_n <= 1'b0;   // Assert reset
        vif.rdrv_cb.rinc   <= 1'b0;   // No read
        @(vif.rdrv_cb);
    endtask : init_signals

    // =========================================================================
    // TASK: apply_reset
    // Asserts rrst_n (active-low) for RESET_CYCLES rclk cycles, then
    // deasserts. Can be called mid-simulation from rrst_req transactions.
    // =========================================================================
    task apply_reset();
        `uvm_info("FIFO_RD_DRV", "Applying read-domain reset (rrst_n=0)", UVM_MEDIUM)
        vif.rdrv_cb.rinc   <= 1'b0;
        vif.rdrv_cb.rrst_n <= 1'b0;
        repeat(RESET_CYCLES) @(vif.rdrv_cb);
        vif.rdrv_cb.rrst_n <= 1'b1;
        @(vif.rdrv_cb);
        `uvm_info("FIFO_RD_DRV", "Read-domain reset released (rrst_n=1)", UVM_MEDIUM)
    endtask : apply_reset

    // =========================================================================
    // TASK: drive_transaction
    // Drives a single fifo_seq_item onto the read-domain interface.
    //
    // Protocol:
    //   Step 1: If rrst_req=1, apply a reset pulse first.
    //   Step 2: Wait rd_idle idle rclk cycles.
    //   Step 3: If rd_en=0, drive a no-op (rinc=0) for 1 cycle and return.
    //   Step 4: If rd_en=1:
    //             - Stall until rempty=0 (backpressure).
    //             - Drive rinc=1 for 1 rclk cycle.
    //             - Sample rdata into item.data_out.
    //             - Deassert rinc=0.
    //
    // NOTE on data capture:
    //   rdata is sampled in the cycle AFTER rinc=1 (since it is registered
    //   by the consumer flip-flop or read from async RAM into a stable state).
    //   We sample rdrv_cb.rdata at the posedge AFTER rinc was driven to get
    //   the valid read value. This matches the waveform behavior of the DUT.
    // =========================================================================
    task drive_transaction(fifo_seq_item #(DATA_WIDTH) item);
        // --- Step 1: Optional reset pulse -----------------------------------
        if (item.rrst_req) begin
            apply_reset();
        end

        // --- Step 2: Inter-transaction idle gap ----------------------------
        if (item.rd_idle > 0) begin
            vif.rdrv_cb.rinc <= 1'b0;
            repeat(item.rd_idle) @(vif.rdrv_cb);
        end

        // --- Step 3: No-op transaction -------------------------------------
        if (!item.rd_en) begin
            vif.rdrv_cb.rinc <= 1'b0;
            @(vif.rdrv_cb);
            item.got_rempty = vif.rdrv_cb.rempty;
            return;
        end

        // --- Step 4: Read with backpressure handling ----------------------
        // Wait until FIFO has data (rempty=0)
        begin
            int unsigned stall_count = 0;
            while (vif.rdrv_cb.rempty) begin
                vif.rdrv_cb.rinc <= 1'b0;
                @(vif.rdrv_cb);
                stall_count++;
                if (stall_count == BACKPRESSURE_WARN_CYCLES) begin
                    `uvm_warning("FIFO_RD_DRV",
                        $sformatf("Read stalled on rempty for %0d cycles. \
Possible deadlock or test misconfiguration.", stall_count))
                end
            end
        end

        // Drive rinc for exactly 1 rclk cycle
        item.got_rempty    = vif.rdrv_cb.rempty;  // Capture rempty state at read
        vif.rdrv_cb.rinc  <= 1'b1;
        @(vif.rdrv_cb);

        // Capture rdata in the cycle during which rinc is driven high.
        // For ASYNC read mode: rdata is combinationally driven from mem[raddr],
        // so it is valid as soon as the pointer advances (i.e., same cycle).
        item.data_out = vif.rdrv_cb.rdata;

        // Deassert rinc
        vif.rdrv_cb.rinc <= 1'b0;

        `uvm_info("FIFO_RD_DRV",
            $sformatf("Read data=0x%0h | rempty=%0b", item.data_out, item.got_rempty),
            UVM_HIGH)
    endtask : drive_transaction

endclass : fifo_rd_driver

`endif // FIFO_RD_DRIVER_SV
// =============================================================================
// End of fifo_rd_driver.sv
// =============================================================================
