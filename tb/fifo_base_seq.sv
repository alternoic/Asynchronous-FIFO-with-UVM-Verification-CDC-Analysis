// =============================================================================
// File        : fifo_base_seq.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : Base Virtual Sequence for the Async FIFO UVM Testbench.
//               All test sequences extend this class.
//
// Design:
//   This is a VIRTUAL sequence - it targets the fifo_virtual_sequencer and
//   uses p_sequencer to access both wr_seqr and rd_seqr concurrently.
//   The `uvm_declare_p_sequencer macro performs a safe downcast of the
//   inherited m_sequencer handle to fifo_virtual_sequencer at runtime.
//
// Usage Pattern:
//   Derived sequences extend fifo_base_seq and call its utility tasks:
//     do_reset()              - pulse both domain resets simultaneously
//     do_write(data)          - drive one write item to wr_seqr
//     do_read()               - drive one read item  to rd_seqr
//     do_write_burst(n, data) - drive n writes back-to-back to wr_seqr
//     do_read_burst(n)        - drive n reads  back-to-back to rd_seqr
//     wait_wr_cycles(n)       - insert n idle write-clock beats
//     wait_rd_cycles(n)       - insert n idle read-clock  beats
//
// All tasks use start_item() / finish_item() with explicit sequencer targeting
// to correctly send items to the right domain sequencer regardless of which
// sequencer this virtual sequence was started on.
//
// Coding Standard : IEEE 1800-2012 | UVM 1.2
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// Version         : 1.0
// =============================================================================

`ifndef FIFO_BASE_SEQ_SV
`define FIFO_BASE_SEQ_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

`include "fifo_seq_item.sv"
`include "fifo_env.sv"         // For fifo_virtual_sequencer type

class fifo_base_seq #(
    parameter int unsigned DATA_WIDTH = 8
) extends uvm_sequence #(fifo_seq_item #(DATA_WIDTH));

    `uvm_object_param_utils(fifo_base_seq #(DATA_WIDTH))

    // Downcast m_sequencer to fifo_virtual_sequencer for domain access
    `uvm_declare_p_sequencer(fifo_virtual_sequencer #(DATA_WIDTH))

    // =========================================================================
    // Configurable Parameters (can be set by derived classes or tests)
    // =========================================================================
    int unsigned reset_duration_wr = 4;  // wrst_n pulse width (wclk cycles)
    int unsigned reset_duration_rd = 4;  // rrst_n pulse width (rclk cycles)
    int unsigned cdc_settle_cycles = 6;  // Extra cycles after reset for CDC settling

    // =========================================================================
    // Constructor
    // =========================================================================
    function new(string name = "fifo_base_seq");
        super.new(name);
    endfunction : new

    // =========================================================================
    // body(): Default body is empty - derived sequences override this.
    // =========================================================================
    virtual task body();
        `uvm_info(get_type_name(), "fifo_base_seq::body() - override in derived class", UVM_MEDIUM)
    endtask : body

    // =========================================================================
    // UTILITY TASK: do_reset
    // Concurrently asserts both domain resets (wrst_n and rrst_n).
    // Uses fork/join so both reset items are sent simultaneously to their
    // respective domain sequencers - modelling actual reset behaviour where
    // both domains are reset together at simulation start.
    //
    // After both reset items are dispatched, waits cdc_settle_cycles extra
    // idle beats on the write side to let the 2-flop synchronizers flush.
    // =========================================================================
    task do_reset();
        fifo_seq_item #(DATA_WIDTH) wr_rst_item;
        fifo_seq_item #(DATA_WIDTH) rd_rst_item;

        `uvm_info(get_type_name(), "Asserting both domain resets...", UVM_MEDIUM)

        fork
            // --- Write-domain reset -----------------------------------------
            begin
                wr_rst_item = fifo_seq_item #(DATA_WIDTH)::type_id::create("wr_rst");
                start_item(wr_rst_item, -1, p_sequencer.wr_seqr);
                if (!wr_rst_item.randomize() with {
                    wrst_req == 1'b1;
                    rrst_req == 1'b0;
                    wr_en    == 1'b0;
                    rd_en    == 1'b0;
                    wr_idle  == 0;
                    rd_idle  == 0;
                }) `uvm_fatal(get_type_name(), "Randomization of wr_rst_item failed!")
                finish_item(wr_rst_item);
            end

            // --- Read-domain reset ------------------------------------------
            begin
                rd_rst_item = fifo_seq_item #(DATA_WIDTH)::type_id::create("rd_rst");
                start_item(rd_rst_item, -1, p_sequencer.rd_seqr);
                if (!rd_rst_item.randomize() with {
                    rrst_req == 1'b1;
                    wrst_req == 1'b0;
                    wr_en    == 1'b0;
                    rd_en    == 1'b0;
                    wr_idle  == 0;
                    rd_idle  == 0;
                }) `uvm_fatal(get_type_name(), "Randomization of rd_rst_item failed!")
                finish_item(rd_rst_item);
            end
        join

        // Wait extra idle cycles on write side for synchronizer pipeline flush
        wait_wr_cycles(cdc_settle_cycles);

        `uvm_info(get_type_name(), "Both domain resets released and CDC settled.", UVM_MEDIUM)
    endtask : do_reset

    // =========================================================================
    // UTILITY TASK: do_write
    // Drives a single write transaction with a specified data value.
    // Uses randomize-with to set wr_en=1, data_in=data, and constrain
    // idle to [0:1] (near-back-to-back). Targets wr_seqr.
    // =========================================================================
    task do_write(input logic [DATA_WIDTH-1:0] data);
        fifo_seq_item #(DATA_WIDTH) item;
        item = fifo_seq_item #(DATA_WIDTH)::type_id::create("wr_item");

        start_item(item, -1, p_sequencer.wr_seqr);
        if (!item.randomize() with {
            wr_en    == 1'b1;
            rd_en    == 1'b0;
            data_in  == data;
            wr_idle  inside {[0:1]};
            rd_idle  == 0;
            wrst_req == 1'b0;
            rrst_req == 1'b0;
        }) `uvm_fatal(get_type_name(),
            $sformatf("Randomization failed for do_write(data=0x%0h)", data))

        finish_item(item);

        `uvm_info(get_type_name(),
            $sformatf("do_write: data=0x%0h dispatched to wr_seqr", data), UVM_HIGH)
    endtask : do_write

    // =========================================================================
    // UTILITY TASK: do_read
    // Drives a single read transaction to rd_seqr.
    // The driver will stall if FIFO is empty (rempty backpressure).
    // =========================================================================
    task do_read();
        fifo_seq_item #(DATA_WIDTH) item;
        item = fifo_seq_item #(DATA_WIDTH)::type_id::create("rd_item");

        start_item(item, -1, p_sequencer.rd_seqr);
        if (!item.randomize() with {
            wr_en    == 1'b0;
            rd_en    == 1'b1;
            rd_idle  inside {[0:1]};
            wr_idle  == 0;
            wrst_req == 1'b0;
            rrst_req == 1'b0;
        }) `uvm_fatal(get_type_name(), "Randomization failed for do_read()")

        finish_item(item);

        `uvm_info(get_type_name(), "do_read: dispatched to rd_seqr", UVM_HIGH)
    endtask : do_read

    // =========================================================================
    // UTILITY TASK: do_write_burst
    // Writes N words back-to-back (no inter-write gaps).
    // If data_arr is provided (non-null), uses those values.
    // Otherwise generates incrementing data pattern: base, base+1, base+2...
    // =========================================================================
    task do_write_burst(
        input int unsigned             n,
        input logic [DATA_WIDTH-1:0]   base_data = '0,
        input bit                       use_random = 1'b0
    );
        fifo_seq_item #(DATA_WIDTH) item;
        logic [DATA_WIDTH-1:0] payload;

        `uvm_info(get_type_name(),
            $sformatf("do_write_burst: sending %0d words...", n), UVM_MEDIUM)

        for (int i = 0; i < n; i++) begin
            item = fifo_seq_item #(DATA_WIDTH)::type_id::create(
                       $sformatf("wr_burst_%0d", i));

            // Determine payload
            if (use_random) begin
                if (!item.randomize() with {
                    wr_en    == 1'b1;
                    rd_en    == 1'b0;
                    wr_idle  == 0;
                    wrst_req == 1'b0;
                    rrst_req == 1'b0;
                }) `uvm_fatal(get_type_name(), "Burst randomize failed!")
            end else begin
                payload = base_data + logic'(i);
                start_item(item, -1, p_sequencer.wr_seqr);
                if (!item.randomize() with {
                    wr_en    == 1'b1;
                    rd_en    == 1'b0;
                    data_in  == payload;
                    wr_idle  == 0;
                    wrst_req == 1'b0;
                    rrst_req == 1'b0;
                }) `uvm_fatal(get_type_name(),
                    $sformatf("Burst randomize failed at i=%0d", i))
                finish_item(item);
                continue;
            end

            start_item(item, -1, p_sequencer.wr_seqr);
            finish_item(item);
        end

        `uvm_info(get_type_name(),
            $sformatf("do_write_burst: %0d words sent.", n), UVM_MEDIUM)
    endtask : do_write_burst

    // =========================================================================
    // UTILITY TASK: do_read_burst
    // Reads N words back-to-back (no inter-read gaps).
    // The driver handles backpressure on rempty automatically.
    // =========================================================================
    task do_read_burst(input int unsigned n);
        fifo_seq_item #(DATA_WIDTH) item;

        `uvm_info(get_type_name(),
            $sformatf("do_read_burst: reading %0d words...", n), UVM_MEDIUM)

        for (int i = 0; i < n; i++) begin
            item = fifo_seq_item #(DATA_WIDTH)::type_id::create(
                       $sformatf("rd_burst_%0d", i));
            start_item(item, -1, p_sequencer.rd_seqr);
            if (!item.randomize() with {
                wr_en    == 1'b0;
                rd_en    == 1'b1;
                rd_idle  == 0;
                wrst_req == 1'b0;
                rrst_req == 1'b0;
            }) `uvm_fatal(get_type_name(),
                $sformatf("Read burst randomize failed at i=%0d", i))
            finish_item(item);
        end

        `uvm_info(get_type_name(),
            $sformatf("do_read_burst: %0d reads dispatched.", n), UVM_MEDIUM)
    endtask : do_read_burst

    // =========================================================================
    // UTILITY TASK: wait_wr_cycles
    // Inserts N idle cycles on the write clock domain.
    // Sends no-op items (wr_en=0) to pad the write timeline.
    // =========================================================================
    task wait_wr_cycles(input int unsigned n);
        fifo_seq_item #(DATA_WIDTH) item;
        for (int i = 0; i < n; i++) begin
            item = fifo_seq_item #(DATA_WIDTH)::type_id::create("wr_nop");
            start_item(item, -1, p_sequencer.wr_seqr);
            if (!item.randomize() with {
                wr_en    == 1'b0;
                rd_en    == 1'b0;
                wr_idle  == 1;
                wrst_req == 1'b0;
                rrst_req == 1'b0;
            }) `uvm_fatal(get_type_name(), "NOP randomize failed!")
            finish_item(item);
        end
    endtask : wait_wr_cycles

    // =========================================================================
    // UTILITY TASK: wait_rd_cycles
    // Inserts N idle cycles on the read clock domain.
    // =========================================================================
    task wait_rd_cycles(input int unsigned n);
        fifo_seq_item #(DATA_WIDTH) item;
        for (int i = 0; i < n; i++) begin
            item = fifo_seq_item #(DATA_WIDTH)::type_id::create("rd_nop");
            start_item(item, -1, p_sequencer.rd_seqr);
            if (!item.randomize() with {
                wr_en    == 1'b0;
                rd_en    == 1'b0;
                rd_idle  == 1;
                wrst_req == 1'b0;
                rrst_req == 1'b0;
            }) `uvm_fatal(get_type_name(), "NOP randomize failed!")
            finish_item(item);
        end
    endtask : wait_rd_cycles

endclass : fifo_base_seq

`endif // FIFO_BASE_SEQ_SV
// =============================================================================
// End of fifo_base_seq.sv
// =============================================================================
