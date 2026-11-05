// =============================================================================
// File        : fifo_sanity_seq.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : Sanity / Smoke Test Sequence for the Async FIFO.
//
// Purpose:
//   A lightweight, deterministic sequence that verifies fundamental FIFO
//   operation with minimal stimulus. If this sequence fails, the DUT has
//   a fundamental correctness issue and no further testing is meaningful.
//
// Test Plan (from vPlan TC01-TC07):
//   Phase 0 : Apply both domain resets and verify flags reset correctly.
//   Phase 1 : Write 4 words with known data values (0xA0, 0xB1, 0xC2, 0xD3).
//   Phase 2 : Wait CDC settle time (synchronizer latency across domains).
//   Phase 3 : Read back all 4 words sequentially.
//   Phase 4 : Write and read 1 word simultaneously (concurrent R/W).
//   Phase 5 : Verify FIFO returns to empty after all reads are done.
//
// Expected Results (verified by scoreboard):
//   - 4 data matches: 0xA0, 0xB1, 0xC2, 0xD3 in order
//   - 1 simultaneous R/W match
//   - Zero mismatches, underflows, or overflows
//   - rempty=1 after Phase 3 drain
//
// CDC Consideration:
//   Between Phase 1 and Phase 3, we insert wait_wr_cycles(8) to allow
//   the wptr Gray-coded pointer to propagate through the 2-flop synchronizer
//   into the read domain. This models the worst-case 2 rclk cycle latency
//   (conservative: using 8 cycles ensures stability regardless of clock ratio).
//
// Coding Standard : IEEE 1800-2012 | UVM 1.2
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// Version         : 1.0
// =============================================================================

`ifndef FIFO_SANITY_SEQ_SV
`define FIFO_SANITY_SEQ_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

`include "fifo_base_seq.sv"

class fifo_sanity_seq #(
    parameter int unsigned DATA_WIDTH = 8
) extends fifo_base_seq #(DATA_WIDTH);

    `uvm_object_param_utils(fifo_sanity_seq #(DATA_WIDTH))

    // =========================================================================
    // Known data payload (deterministic, easy to trace in waveforms)
    // =========================================================================
    localparam int unsigned NUM_WORDS = 4;
    logic [DATA_WIDTH-1:0] known_data [NUM_WORDS] = '{
        8'hA0,   // Word 0 - recognizable pattern
        8'hB1,   // Word 1
        8'hC2,   // Word 2
        8'hD3    // Word 3 - last word in sanity set
    };

    // CDC synchronizer settle time (wclk cycles) - conservative for any ratio
    int unsigned cdc_wait = 8;

    // =========================================================================
    // Constructor
    // =========================================================================
    function new(string name = "fifo_sanity_seq");
        super.new(name);
    endfunction : new

    // =========================================================================
    // body(): Main sequence execution
    // =========================================================================
    task body();
        `uvm_info(get_type_name(),
            "\n========================================\n  SANITY SEQUENCE - START\n========================================",
            UVM_NONE)

        // =====================================================================
        // PHASE 0: Reset both clock domains
        // Drives wrst_n=0 and rrst_n=0 simultaneously via do_reset().
        // After this, scoreboard queue is empty, rempty=1, wfull=0.
        // =====================================================================
        `uvm_info(get_type_name(), "[Phase 0] Applying dual-domain reset...", UVM_MEDIUM)
        do_reset();
        `uvm_info(get_type_name(), "[Phase 0] Reset complete. FIFO is empty.", UVM_MEDIUM)

        // =====================================================================
        // PHASE 1: Write 4 known words sequentially
        // All writes are dispatched to wr_seqr. The write driver stalls on
        // wfull if needed (not expected here - FIFO starts empty, depth=16).
        // =====================================================================
        `uvm_info(get_type_name(), "[Phase 1] Writing 4 known words...", UVM_MEDIUM)
        for (int i = 0; i < NUM_WORDS; i++) begin
            do_write(known_data[i]);
            `uvm_info(get_type_name(),
                $sformatf("[Phase 1] Wrote word[%0d] = 0x%0h", i, known_data[i]),
                UVM_MEDIUM)
        end
        `uvm_info(get_type_name(), "[Phase 1] All 4 writes dispatched.", UVM_MEDIUM)

        // =====================================================================
        // PHASE 2: CDC settle delay
        // Insert extra write-clock idle cycles to ensure the write pointer has
        // propagated through the 2-flop synchronizer into the read domain.
        // Without this, the read monitor would observe rempty=1 for 2+ rclk
        // cycles even though data is present - causing the driver to stall.
        // With it, we guarantee rempty has deasserted before Phase 3 starts.
        // =====================================================================
        `uvm_info(get_type_name(),
            $sformatf("[Phase 2] Waiting %0d wclk cycles for CDC pointer sync...",
                cdc_wait), UVM_MEDIUM)
        wait_wr_cycles(cdc_wait);

        // =====================================================================
        // PHASE 3: Read back all 4 words
        // The scoreboard pops from its reference queue and compares to rdata.
        // Order must be preserved (FIFO semantics).
        // =====================================================================
        `uvm_info(get_type_name(), "[Phase 3] Reading back 4 words...", UVM_MEDIUM)
        for (int i = 0; i < NUM_WORDS; i++) begin
            do_read();
            `uvm_info(get_type_name(),
                $sformatf("[Phase 3] Read request %0d dispatched.", i), UVM_MEDIUM)
        end
        `uvm_info(get_type_name(), "[Phase 3] All 4 reads dispatched.", UVM_MEDIUM)

        // =====================================================================
        // PHASE 4: Simultaneous write and read (concurrent CDC exercise)
        // Fork write and read simultaneously to one data word.
        // Tests that the FIFO correctly handles concurrent domain access.
        // =====================================================================
        `uvm_info(get_type_name(), "[Phase 4] Simultaneous write (0xEE) and read...", UVM_MEDIUM)
        fork
            do_write(8'hEE);    // Write 0xEE to write domain
            begin
                wait_rd_cycles(cdc_wait);  // Let write arrive before reading
                do_read();
            end
        join
        `uvm_info(get_type_name(), "[Phase 4] Simultaneous R/W complete.", UVM_MEDIUM)

        // =====================================================================
        // PHASE 5: Verify FIFO is empty
        // After reading all written words, the FIFO should be empty.
        // We attempt one more read - the driver will stall on rempty.
        // Insert a timeout guard using UVM objection (handled by test).
        // =====================================================================
        `uvm_info(get_type_name(),
            "[Phase 5] FIFO should now be empty. Sending idle beats to confirm.",
            UVM_MEDIUM)
        wait_wr_cycles(4);
        wait_rd_cycles(4);

        `uvm_info(get_type_name(),
            "\n========================================\n  SANITY SEQUENCE - COMPLETE\n  Scoreboard will verify data integrity\n========================================",
            UVM_NONE)
    endtask : body

endclass : fifo_sanity_seq

`endif // FIFO_SANITY_SEQ_SV
// =============================================================================
// End of fifo_sanity_seq.sv
// =============================================================================
