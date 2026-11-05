// =============================================================================
// File        : fifo_wr_driver.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : UVM Write-Domain Driver.
//               Drives write-side stimulus (winc, wdata, wrst_n) to the
//               async_fifo DUT through the 'wdrv_cb' clocking block of the
//               async_fifo_if interface. Operates exclusively in wclk domain.
//
// Behavior:
//   1. Retrieves fifo_seq_item from the write sequencer.
//   2. If wrst_req=1 : pulses wrst_n low for RESET_CYCLES wclk cycles.
//   3. Waits wr_idle wclk cycles (inter-transaction gap).
//   4. If wr_en=1   : stalls (backpressure) until wfull=0, then drives
//                     winc=1, wdata=data_in for one wclk cycle.
//   5. Deasserts winc=0 and calls item_done().
//
// Backpressure:
//   When the DUT asserts wfull, the driver stalls in a wait loop - it does
//   NOT drop the transaction. This models a well-behaved write producer.
//   A $warning is issued after BACKPRESSURE_WARN_CYCLES stall cycles.
//
// Config DB Key : "WDRV_VIF" → virtual async_fifo_if
//
// Coding Standard : IEEE 1800-2012 | UVM 1.2
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// =============================================================================

`ifndef FIFO_WR_DRIVER_SV
`define FIFO_WR_DRIVER_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

class fifo_wr_driver #(
    parameter int unsigned DATA_WIDTH = 8
) extends uvm_driver #(fifo_seq_item #(DATA_WIDTH));

    `uvm_component_param_utils(fifo_wr_driver #(DATA_WIDTH))

    // =========================================================================
    // Local Parameters
    // =========================================================================
    localparam int unsigned RESET_CYCLES          = 4;   // wrst_n pulse width (wclk cycles)
    localparam int unsigned BACKPRESSURE_WARN_CYCLES = 20; // Warn if stalled this long

    // =========================================================================
    // Virtual Interface Handle
    // =========================================================================
    virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)) vif;

    // =========================================================================
    // Constructor
    // =========================================================================
    function new(string name = "fifo_wr_driver", uvm_component parent = null);
        super.new(name, parent);
    endfunction : new

    // =========================================================================
    // build_phase: Retrieve virtual interface from config_db
    // =========================================================================
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db #(virtual async_fifo_if #(.DATA_WIDTH(DATA_WIDTH)))::get(
                this, "", "WDRV_VIF", vif)) begin
            `uvm_fatal("FIFO_WR_DRV", "Virtual interface 'WDRV_VIF' not found in config_db. \
Ensure tb_top sets: uvm_config_db::set(null, \"*\", \"WDRV_VIF\", fifo_if)")
        end
    endfunction : build_phase

    // =========================================================================
    // run_phase: Initialize DUT inputs then enter the get/drive loop
    // =========================================================================
    task run_phase(uvm_phase phase);
        // Initialize all write-domain outputs to safe idle state
        init_signals();

        // Apply an initial reset before accepting sequences
        apply_reset();

        // Main driver loop
        forever begin
            // Get next item from sequencer (blocking)
            seq_item_port.get_next_item(req);

            `uvm_info("FIFO_WR_DRV",
                $sformatf("Driving: %s", req.convert2string()), UVM_HIGH)

            // Drive the transaction
            drive_transaction(req);

            // Signal to sequencer that item has been consumed
            seq_item_port.item_done();
        end
    endtask : run_phase

    // =========================================================================
    // TASK: init_signals
    // Places all write-domain DUT inputs into a known, safe idle state.
    // Called once at the start of run_phase before any reset.
    // =========================================================================
    task init_signals();
        vif.wdrv_cb.wrst_n <= 1'b0;   // Assert reset immediately
        vif.wdrv_cb.winc   <= 1'b0;   // No write
        vif.wdrv_cb.wdata  <= {DATA_WIDTH{1'b0}};
        @(vif.wdrv_cb);                // Hold for one clock edge
    endtask : init_signals

    // =========================================================================
    // TASK: apply_reset
    // Asserts wrst_n (active-low) for RESET_CYCLES wclk cycles, then
    // deasserts. Safe to call mid-simulation (e.g., from wrst_req).
    // =========================================================================
    task apply_reset();
        `uvm_info("FIFO_WR_DRV", "Applying write-domain reset (wrst_n=0)", UVM_MEDIUM)
        vif.wdrv_cb.winc   <= 1'b0;
        vif.wdrv_cb.wdata  <= {DATA_WIDTH{1'b0}};
        vif.wdrv_cb.wrst_n <= 1'b0;
        repeat(RESET_CYCLES) @(vif.wdrv_cb);
        vif.wdrv_cb.wrst_n <= 1'b1;
        @(vif.wdrv_cb);               // 1 idle cycle after reset release
        `uvm_info("FIFO_WR_DRV", "Write-domain reset released (wrst_n=1)", UVM_MEDIUM)
    endtask : apply_reset

    // =========================================================================
    // TASK: drive_transaction
    // Drives a single fifo_seq_item onto the write-domain interface.
    //
    // Protocol:
    //   Step 1: If wrst_req=1, apply a reset pulse first.
    //   Step 2: Wait wr_idle idle wclk cycles.
    //   Step 3: If wr_en=0, drive a no-op (winc=0) for 1 cycle and return.
    //   Step 4: If wr_en=1, stall until wfull=0 (backpressure), then:
    //             - Drive winc=1, wdata=data_in for 1 clock cycle.
    //             - Deassert winc=0.
    // =========================================================================
    task drive_transaction(fifo_seq_item #(DATA_WIDTH) item);
        // --- Step 1: Optional reset pulse -----------------------------------
        if (item.wrst_req) begin
            apply_reset();
        end

        // --- Step 2: Inter-transaction idle gap ----------------------------
        if (item.wr_idle > 0) begin
            vif.wdrv_cb.winc  <= 1'b0;
            vif.wdrv_cb.wdata <= {DATA_WIDTH{1'b0}};
            repeat(item.wr_idle) @(vif.wdrv_cb);
        end

        // --- Step 3: No-op transaction -------------------------------------
        if (!item.wr_en) begin
            vif.wdrv_cb.winc  <= 1'b0;
            vif.wdrv_cb.wdata <= {DATA_WIDTH{1'b0}};
            @(vif.wdrv_cb);
            return;
        end

        // --- Step 4: Write with backpressure handling ----------------------
        // Wait until FIFO has space (wfull=0)
        begin
            int unsigned stall_count = 0;
            while (vif.wdrv_cb.wfull) begin
                vif.wdrv_cb.winc  <= 1'b0;
                vif.wdrv_cb.wdata <= {DATA_WIDTH{1'b0}};
                @(vif.wdrv_cb);
                stall_count++;
                if (stall_count == BACKPRESSURE_WARN_CYCLES) begin
                    `uvm_warning("FIFO_WR_DRV",
                        $sformatf("Write stalled on wfull for %0d cycles. \
Possible deadlock or test misconfiguration.", stall_count))
                end
            end
        end

        // Drive the write for exactly 1 wclk cycle
        vif.wdrv_cb.winc  <= 1'b1;
        vif.wdrv_cb.wdata <= item.data_in;
        @(vif.wdrv_cb);

        // Deassert immediately after the cycle
        vif.wdrv_cb.winc  <= 1'b0;
        vif.wdrv_cb.wdata <= {DATA_WIDTH{1'b0}};

        `uvm_info("FIFO_WR_DRV",
            $sformatf("Wrote data=0x%0h | wfull=%0b", item.data_in, vif.wdrv_cb.wfull),
            UVM_HIGH)
    endtask : drive_transaction

endclass : fifo_wr_driver

`endif // FIFO_WR_DRIVER_SV
// =============================================================================
// End of fifo_wr_driver.sv
// =============================================================================
