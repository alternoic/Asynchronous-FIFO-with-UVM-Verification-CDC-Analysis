// =============================================================================
// File        : fifo_scoreboard.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : UVM Self-Checking Scoreboard for the Async FIFO.
//
// Architecture:
//   ┌--------------------------------------------------------------┐
//   |                  fifo_scoreboard                             |
//   |                                                              |
//   |  wr_export <--- fifo_wr_monitor.ap  (valid writes)          |
//   |  rd_export <--- fifo_rd_monitor.ap_read (valid reads)       |
//   |                                                              |
//   |  ┌------------------------------------------------------┐   |
//   |  |  Reference Model: logic [DW-1:0] ref_queue[$]        |   |
//   |  |                                                        |   |
//   |  |  write_wr() --> push_back(data_in)                   |   |
//   |  |  write_rd() --> pop_front() vs data_out              |   |
//   |  └-------------------------------------------------------|   |
//   └---------------------------------------------------------------|
//
// CDC Latency Handling:
//   This scoreboard uses a QUEUE-BASED reference model, NOT cycle-by-cycle
//   comparison. This is essential for async FIFO verification because:
//
//   1. Write and read transactions arrive from INDEPENDENT clock domains
//      with no fixed phase relationship.
//   2. The 2-flop synchronizer introduces 2-cycle latency in EACH direction,
//      meaning write data may not be readable for several rclk cycles after
//      being written.
//   3. A cycle-accurate model would produce false failures during the
//      synchronizer latency window.
//
//   The queue model only cares about ORDERING and DATA INTEGRITY:
//     "Was the Nth word read back the same as the Nth word written?"
//   It does not care WHEN the read happened relative to the write.
//
// Reference Queue Rules:
//   PUSH  : When write monitor broadcasts a valid write (winc=1, wfull=0).
//   POP   : When read monitor  broadcasts a valid read  (rinc=1, rempty=0).
//   MATCH : If pop value == item.data_out → PASS.
//   ERROR : If pop value != item.data_out → FAIL (data corruption).
//   ERROR : If pop from empty queue       → FAIL (read underflow, DUT bug).
//   ERROR : If push beyond FIFO_DEPTH     → FAIL (write overflow, DUT bug).
//   WARNING: Queue not empty at test end  → WARN (unread data).
//
// Reset Handling:
//   When a reset is detected (via a dedicated analysis imp), the reference
//   queue is flushed. This correctly models the hardware behavior where
//   both pointers reset to 0, logically emptying the FIFO.
//
// Analysis Imp Macros:
//   UVM requires unique suffix macros when a class has more than one
//   analysis import of the same transaction type:
//     `uvm_analysis_imp_decl(_wr)  → write_wr() callback
//     `uvm_analysis_imp_decl(_rd)  → write_rd() callback
//
// Coding Standard : IEEE 1800-2012 | UVM 1.2
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// Version         : 1.0
// =============================================================================

`ifndef FIFO_SCOREBOARD_SV
`define FIFO_SCOREBOARD_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

// Must be declared outside the class, at package/file scope.
// Creates two specialised analysis-implementation classes:
//   uvm_analysis_imp_wr #(T, IMP) - calls write_wr()
//   uvm_analysis_imp_rd #(T, IMP) - calls write_rd()
`uvm_analysis_imp_decl(_wr)
`uvm_analysis_imp_decl(_rd)

class fifo_scoreboard #(
    parameter int unsigned DATA_WIDTH  = 8,
    parameter int unsigned FIFO_DEPTH  = 16   // Used for overflow detection
) extends uvm_scoreboard;

    `uvm_component_param_utils(fifo_scoreboard #(DATA_WIDTH, FIFO_DEPTH))

    // =========================================================================
    // Analysis Imports
    // wr_export : connected to fifo_wr_monitor.ap  (valid write items)
    // rd_export : connected to fifo_rd_monitor.ap_read (valid read items)
    // =========================================================================
    uvm_analysis_imp_wr #(fifo_seq_item #(DATA_WIDTH),
                          fifo_scoreboard #(DATA_WIDTH, FIFO_DEPTH)) wr_export;

    uvm_analysis_imp_rd #(fifo_seq_item #(DATA_WIDTH),
                          fifo_scoreboard #(DATA_WIDTH, FIFO_DEPTH)) rd_export;

    // =========================================================================
    // Reference Model: Unbounded Queue of Write Data
    //
    // Stores data_in values in write order. The DUT is expected to deliver
    // them in the same FIFO order. The queue is bounded by FIFO_DEPTH for
    // overflow detection but is otherwise software-managed (no hardware timing).
    //
    // Note: We store only the DATA (not the full seq_item) to keep the
    // reference model minimal and free of timing assumptions.
    // =========================================================================
    logic [DATA_WIDTH-1:0] ref_queue[$];

    // =========================================================================
    // Scoreboard Statistics - accessible via get_* tasks or report_phase
    // =========================================================================
    int unsigned num_writes_rcvd;        // Valid writes received from wr monitor
    int unsigned num_reads_rcvd;         // Valid reads  received from rd monitor
    int unsigned num_matches;            // data_out == expected  (PASS events)
    int unsigned num_mismatches;         // data_out != expected  (FAIL events)
    int unsigned num_rd_underflows;      // Read from empty queue (DUT bug: read from empty FIFO)
    int unsigned num_wr_overflows;       // Write beyond FIFO_DEPTH (DUT bug: write to full FIFO)
    int unsigned num_resets;             // Reset events observed
    int unsigned num_queue_flushed;      // Words flushed from queue on reset
    int unsigned max_queue_depth_seen;   // High-water mark of ref_queue depth

    // =========================================================================
    // Constructor
    // =========================================================================
    function new(string name = "fifo_scoreboard", uvm_component parent = null);
        super.new(name, parent);
        // Initialize all counters
        num_writes_rcvd     = 0;
        num_reads_rcvd      = 0;
        num_matches         = 0;
        num_mismatches      = 0;
        num_rd_underflows   = 0;
        num_wr_overflows    = 0;
        num_resets          = 0;
        num_queue_flushed   = 0;
        max_queue_depth_seen = 0;
    endfunction : new

    // =========================================================================
    // build_phase: Create analysis import ports
    // =========================================================================
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        wr_export = new("wr_export", this);
        rd_export = new("rd_export", this);
    endfunction : build_phase

    // =========================================================================
    // FUNCTION: write_wr - called by wr_export when write monitor broadcasts
    //
    // This is the PUSH side of the reference model.
    //
    // CDC Note:
    //   The write monitor broadcasts as soon as winc=1 & wfull=0 is observed
    //   on wclk. The data has NOT yet passed through the synchronizer and is
    //   NOT yet readable by the read domain. We simply store it and wait.
    //   There is NO timeout here - the read could arrive arbitrarily later.
    //
    // Overflow Guard:
    //   If the ref_queue exceeds FIFO_DEPTH, it indicates the DUT failed to
    //   assert wfull correctly (the write agent should have been stalled).
    //   This is logged as a DUT bug, not a test error.
    // =========================================================================
    function void write_wr(fifo_seq_item #(DATA_WIDTH) item);
        // Update stats
        num_writes_rcvd++;

        // Overflow check: ref_queue should never exceed FIFO_DEPTH
        if (ref_queue.size() >= FIFO_DEPTH) begin
            num_wr_overflows++;
            `uvm_error("FIFO_SB",
                $sformatf("[OVERFLOW] Write received but ref_queue already at FIFO_DEPTH=%0d! \
data_in=0x%0h. DUT failed to assert wfull correctly. \
Total overflows: %0d",
                FIFO_DEPTH, item.data_in, num_wr_overflows))
            return;  // Do not push - would corrupt reference model
        end

        // Push data into the reference queue (back = newest)
        ref_queue.push_back(item.data_in);

        // Track high-water mark
        if (ref_queue.size() > max_queue_depth_seen)
            max_queue_depth_seen = ref_queue.size();

        `uvm_info("FIFO_SB",
            $sformatf("[PUSH] data=0x%0h | queue_depth=%0d | write#=%0d",
                item.data_in, ref_queue.size(), num_writes_rcvd), UVM_HIGH)
    endfunction : write_wr

    // =========================================================================
    // FUNCTION: write_rd - called by rd_export when read monitor broadcasts
    //
    // This is the POP+COMPARE side of the reference model.
    //
    // CDC Note:
    //   The read monitor broadcasts as soon as rinc=1 & rempty=0 is observed
    //   on rclk. The DUT's empty flag ensures no read happens until the write
    //   pointer has been synchronized (2 rclk cycles minimum). By the time
    //   the read monitor fires, the push to our ref_queue has ALREADY happened
    //   (the write occurred earlier in wclk domain). Therefore:
    //
    //   INVARIANT: If DUT is correct, ref_queue.size() > 0 whenever
    //              the read monitor fires (because rempty=0 guarantees data
    //              is present, which only happens after the write synchronized).
    //
    //   We pop the oldest entry (FIFO order = pop_front) and compare to
    //   item.data_out received from the read monitor.
    //
    // Underflow Guard:
    //   If ref_queue is empty when a read arrives, it means either:
    //     (a) The DUT failed to assert rempty when the FIFO was empty, or
    //     (b) The scoreboard lost track of writes (connection error).
    //   Either case is a critical DUT or TB bug.
    // =========================================================================
    function void write_rd(fifo_seq_item #(DATA_WIDTH) item);
        logic [DATA_WIDTH-1:0] expected_data;

        // Update stats
        num_reads_rcvd++;

        // Underflow check: ref_queue must not be empty on a valid read
        if (ref_queue.size() == 0) begin
            num_rd_underflows++;
            `uvm_error("FIFO_SB",
                $sformatf("[UNDERFLOW] Read received (data_out=0x%0h) but ref_queue is EMPTY! \
DUT asserted rinc while FIFO was logically empty. \
Total underflows: %0d",
                item.data_out, num_rd_underflows))
            return;
        end

        // Pop the oldest (front) entry from the reference queue (FIFO order)
        expected_data = ref_queue.pop_front();

        // =====================================================================
        // DATA COMPARISON
        // Use !== (4-state inequality) to catch X/Z propagation bugs.
        // A match requires bit-exact equality (no X tolerance).
        // =====================================================================
        if (item.data_out !== expected_data) begin
            num_mismatches++;
            `uvm_error("FIFO_SB",
                $sformatf("[MISMATCH #%0d] Read#=%0d | Expected=0x%0h | Got=0x%0h | \
Error=%0b | queue_remaining=%0d",
                num_mismatches,
                num_reads_rcvd,
                expected_data,
                item.data_out,
                item.data_out ^ expected_data,
                ref_queue.size()))
        end else begin
            num_matches++;
            `uvm_info("FIFO_SB",
                $sformatf("[MATCH ] Read#=%0d | data=0x%0h | queue_remaining=%0d",
                    num_reads_rcvd, item.data_out, ref_queue.size()), UVM_HIGH)
        end
    endfunction : write_rd

    // =========================================================================
    // FUNCTION: flush_on_reset
    // Called externally (or via a reset analysis port) when wrst_n or rrst_n
    // is asserted. Flushes the reference queue to model the hardware behavior
    // where both pointers reset to 0, logically emptying the FIFO.
    //
    // IMPORTANT: When ONLY rrst_n is asserted (write domain still active),
    //   the hardware FIFO data is not truly erased - but the read pointer
    //   resets to 0, making previously-written data inaccessible. The full
    //   queue flush is the conservative safe choice for the scoreboard.
    //
    //   If partial reset (only one domain) is tested, call this with
    //   flush_count = ref_queue.size() (full flush) or manage separately.
    // =========================================================================
    function void flush_on_reset(string domain = "BOTH");
        int unsigned flushed = ref_queue.size();
        if (flushed > 0) begin
            `uvm_warning("FIFO_SB",
                $sformatf("[RESET FLUSH] Domain=%s | Flushing %0d entries from ref_queue",
                    domain, flushed))
        end
        ref_queue.delete();
        num_resets++;
        num_queue_flushed += flushed;
        `uvm_info("FIFO_SB",
            $sformatf("[RESET] Queue flushed. Domain=%s | Words discarded=%0d",
                domain, flushed), UVM_MEDIUM)
    endfunction : flush_on_reset

    // =========================================================================
    // check_phase: Final scoreboard consistency checks after simulation ends
    //
    // Checks performed:
    //   1. ref_queue must be empty (all written data was read back)
    //   2. num_matches must equal num_reads_rcvd (no uncompared reads)
    //   3. num_mismatches must be zero (no data corruption)
    //   4. num_rd_underflows must be zero (no protocol violations)
    //   5. num_wr_overflows must be zero (no protocol violations)
    // =========================================================================
    function void check_phase(uvm_phase phase);
        super.check_phase(phase);

        // [CHECK-1] Unread data remaining in the queue
        if (ref_queue.size() > 0) begin
            `uvm_warning("FIFO_SB",
                $sformatf("[UNREAD DATA] %0d word(s) written but never read back. \
This may be intentional for fill-only tests. Dump: %p",
                ref_queue.size(), ref_queue))
        end

        // [CHECK-2] Mismatch count
        if (num_mismatches > 0) begin
            `uvm_error("FIFO_SB",
                $sformatf("[FAIL] %0d data mismatch(es) detected!", num_mismatches))
        end else begin
            `uvm_info("FIFO_SB", "[PASS] Zero data mismatches.", UVM_NONE)
        end

        // [CHECK-3] Underflow count
        if (num_rd_underflows > 0) begin
            `uvm_error("FIFO_SB",
                $sformatf("[FAIL] %0d read underflow(s) - DUT read from empty FIFO!",
                    num_rd_underflows))
        end

        // [CHECK-4] Overflow count
        if (num_wr_overflows > 0) begin
            `uvm_error("FIFO_SB",
                $sformatf("[FAIL] %0d write overflow(s) - DUT wrote to full FIFO!",
                    num_wr_overflows))
        end

        // [CHECK-5] Match count should equal reads received (minus underflows)
        begin
            int unsigned expected_matches = num_reads_rcvd - num_rd_underflows;
            if (num_matches != expected_matches) begin
                `uvm_error("FIFO_SB",
                    $sformatf("[FAIL] Match count mismatch! Expected %0d matches, got %0d.",
                        expected_matches, num_matches))
            end
        end
    endfunction : check_phase

    // =========================================================================
    // report_phase: Print final statistics table
    // =========================================================================
    function void report_phase(uvm_phase phase);
        string sep = "---------------------------------------------";
        `uvm_info("FIFO_SB", $sformatf(
            "\n%s\n  FIFO SCOREBOARD - FINAL REPORT\n%s\n  Writes received        : %0d\n  Reads  received        : %0d\n  Data matches (PASS)    : %0d\n  Data mismatches (FAIL) : %0d\n  Read  underflows (FAIL): %0d\n  Write overflows (FAIL) : %0d\n  Resets handled         : %0d\n  Words flushed on reset : %0d\n  Max queue depth seen   : %0d / %0d\n  Residual queue depth   : %0d\n%s",
            sep, sep,
            num_writes_rcvd, num_reads_rcvd,
            num_matches, num_mismatches,
            num_rd_underflows, num_wr_overflows,
            num_resets, num_queue_flushed,
            max_queue_depth_seen, FIFO_DEPTH,
            ref_queue.size(),
            sep
        ), UVM_NONE)
    endfunction : report_phase

endclass : fifo_scoreboard

`endif // FIFO_SCOREBOARD_SV
// =============================================================================
// End of fifo_scoreboard.sv
// =============================================================================
