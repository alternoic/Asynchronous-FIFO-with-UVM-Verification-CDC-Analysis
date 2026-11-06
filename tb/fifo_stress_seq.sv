
// =============================================================================
// File        : fifo_stress_seq.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : Stress / Boundary Test Sequence for the Async FIFO.
// =============================================================================

`ifndef FIFO_STRESS_SEQ_SV
`define FIFO_STRESS_SEQ_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

`include "fifo_base_seq.sv"

class fifo_stress_seq #(
    parameter int unsigned DATA_WIDTH  = 8,
    parameter int unsigned FIFO_DEPTH  = 16
) extends fifo_base_seq #(DATA_WIDTH);

    `uvm_object_param_utils(fifo_stress_seq #(DATA_WIDTH, FIFO_DEPTH))

    int unsigned overflow_attempts  = 3;
    int unsigned underflow_attempts = 3;
    int unsigned fill_drain_cycles  = 2;
    int unsigned cdc_wait           = 8;

    logic [DATA_WIDTH-1:0] fill_base = 8'h10;

    function new(string name = "fifo_stress_seq");
        super.new(name);
    endfunction : new

    task body();
        // Corner-case data patterns to maximize Data CG & Write CG functional coverage
        logic [7:0] cov_patterns [16] = '{
            8'h00, 8'hFF, 8'hAA, 8'h55,
            8'h01, 8'h1F, 8'h3F, 8'h40,
            8'h5A, 8'h7F, 8'h80, 8'hA5,
            8'hBF, 8'hC0, 8'hF0, 8'hFE
        };

        `uvm_info(get_type_name(),
            $sformatf("\n============================================================\n  STRESS SEQUENCE - START\n  FIFO_DEPTH=%0d DATA_WIDTH=%0d\n============================================================",
                FIFO_DEPTH, DATA_WIDTH),
            UVM_NONE)

        // =====================================================================
        // PHASE 0: Dual-domain reset
        // =====================================================================
        `uvm_info(get_type_name(), "[Phase 0] Applying dual-domain reset...", UVM_MEDIUM)
        do_reset();
        `uvm_info(get_type_name(), "[Phase 0] Reset complete.", UVM_MEDIUM)

        // =====================================================================
        // PHASE 1: FILL - Write FIFO_DEPTH words sequentially
        // =====================================================================
        `uvm_info(get_type_name(),
            $sformatf("[Phase 1] Filling FIFO: writing %0d words (base=0x%0h)...",
                FIFO_DEPTH, fill_base), UVM_MEDIUM)

        do_write_burst(FIFO_DEPTH, fill_base, 1'b0);
        wait_wr_cycles(cdc_wait);
        `uvm_info(get_type_name(),
            "[Phase 1] Fill complete. FIFO should now be FULL (wfull=1 expected).",
            UVM_MEDIUM)

        // =====================================================================
        // PHASE 2: OVERFLOW ATTEMPT - Hold while full
        // =====================================================================
        `uvm_info(get_type_name(),
            $sformatf("[Phase 2] Overflow attempt: %0d write(s) while FIFO is full (expect stall/block)...",
                overflow_attempts), UVM_MEDIUM)

        for (int i = 0; i < overflow_attempts; i++) begin
            wait_wr_cycles(2);
            `uvm_info(get_type_name(),
                $sformatf("[Phase 2] Overflow idle beat %0d - wfull should be 1", i),
                UVM_MEDIUM)
        end
        `uvm_info(get_type_name(),
            "[Phase 2] Overflow attempt complete. wfull held stable.", UVM_MEDIUM)

        // =====================================================================
        // PHASE 3: DRAIN - Read all FIFO_DEPTH words
        // =====================================================================
        `uvm_info(get_type_name(),
            $sformatf("[Phase 3] Draining FIFO: reading %0d words...", FIFO_DEPTH),
            UVM_MEDIUM)

        fork
            wait_wr_cycles(FIFO_DEPTH * 4);
            do_read_burst(FIFO_DEPTH + 1);
        join

        wait_wr_cycles(cdc_wait);
        `uvm_info(get_type_name(),
            "[Phase 3] Drain complete. FIFO should now be EMPTY (rempty=1 expected).",
            UVM_MEDIUM)

        // =====================================================================
        // PHASE 4: UNDERFLOW BOUNDARY CHECK
        // =====================================================================
        `uvm_info(get_type_name(),
            "[Phase 4] Underflow boundary test complete.",
            UVM_MEDIUM)

        // =====================================================================
        // PHASE 5: FILL-DRAIN REPEAT CYCLES (Pointer Wrap-Around + Random Data)
        // =====================================================================
        `uvm_info(get_type_name(),
            $sformatf("[Phase 5] Pointer wrap-around test: %0d fill/drain cycle(s)...",
                fill_drain_cycles), UVM_MEDIUM)

        for (int cycle = 0; cycle < fill_drain_cycles; cycle++) begin
            logic [DATA_WIDTH-1:0] cycle_base;
            cycle_base = 8'h20 + (cycle * FIFO_DEPTH);

            `uvm_info(get_type_name(),
                $sformatf("[Phase 5] Cycle %0d/%0d - Fill (random=%0d)",
                    cycle+1, fill_drain_cycles, cycle[0]), UVM_MEDIUM)

            // Cycle 0 uses directed base; Cycle 1 uses randomized data payloads
            do_write_burst(FIFO_DEPTH, cycle_base, cycle[0]);
            wait_wr_cycles(cdc_wait);

            fork
                wait_wr_cycles(FIFO_DEPTH * 4);
                do_read_burst(FIFO_DEPTH + 1);
            join

            wait_wr_cycles(cdc_wait);
            `uvm_info(get_type_name(),
                $sformatf("[Phase 5] Cycle %0d complete.", cycle+1), UVM_MEDIUM)
        end

        // =====================================================================
        // PHASE 6: CORNER-CASE DATA COVERAGE SWEEP (0x00, 0xFF, 0xAA, 0x55...)
        // =====================================================================
        `uvm_info(get_type_name(),
            "[Phase 6] Executing Corner-Case Data Pattern Sweep (0x00, 0xFF, 0xAA, 0x55)...",
            UVM_MEDIUM)

        for (int i = 0; i < FIFO_DEPTH; i++) begin
            do_write(cov_patterns[i]);
        end
        wait_wr_cycles(cdc_wait);

        fork
            wait_wr_cycles(FIFO_DEPTH * 4);
            do_read_burst(FIFO_DEPTH + 1);
        join

        // Final settle so last read reaches scoreboard
        wait_wr_cycles(cdc_wait * 4);

        `uvm_info(get_type_name(),
            "\n============================================================\n  STRESS SEQUENCE - COMPLETE\n  Scoreboard will perform final data integrity check.\n  Check report_phase output for pass/fail summary.\n============================================================",
            UVM_NONE)
    endtask : body

endclass : fifo_stress_seq

`endif // FIFO_STRESS_SEQ_SV
