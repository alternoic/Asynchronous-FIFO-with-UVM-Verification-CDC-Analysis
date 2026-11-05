// =============================================================================
// File        : fifo_base_test.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : UVM Test Classes for the Async FIFO Testbench.
// =============================================================================

`ifndef FIFO_BASE_TEST_SV
`define FIFO_BASE_TEST_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

`include "fifo_env.sv"
`include "fifo_base_seq.sv"
`include "fifo_sanity_seq.sv"
`include "fifo_stress_seq.sv"

// =============================================================================
// CLASS: fifo_base_test
// =============================================================================
class fifo_base_test extends uvm_test;
    localparam int unsigned DATA_WIDTH = 8;
    localparam int unsigned FIFO_DEPTH = 16;

    `uvm_component_utils(fifo_base_test)

    fifo_env #(.DATA_WIDTH(DATA_WIDTH), .FIFO_DEPTH(FIFO_DEPTH)) env;

    time test_timeout_ns = 1_000_000;   // 1 ms default

    function new(string name = "fifo_base_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction : new

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);

        env = fifo_env #(.DATA_WIDTH(DATA_WIDTH),
                         .FIFO_DEPTH(FIFO_DEPTH))::type_id::create("env", this);

        uvm_top.set_report_verbosity_level_hier(UVM_MEDIUM);

        `uvm_info(get_type_name(),
            $sformatf("\n=== TEST: %s ===\n  DATA_WIDTH=%0d  FIFO_DEPTH=%0d  Timeout=%0t",
                get_type_name(), DATA_WIDTH, FIFO_DEPTH, test_timeout_ns), UVM_NONE)
    endfunction : build_phase

    virtual task run_phase(uvm_phase phase);
        phase.raise_objection(this, "base_test run started");

        `uvm_warning(get_type_name(),
            "fifo_base_test::run_phase called directly - no sequence started. Use fifo_sanity_test or fifo_stress_test.")

        #(100);
        phase.drop_objection(this, "base_test run complete");
    endtask : run_phase

    function void report_phase(uvm_phase phase);
        uvm_report_server srv = uvm_report_server::get_server();
        int unsigned num_errors   = srv.get_severity_count(UVM_ERROR);
        int unsigned num_fatals   = srv.get_severity_count(UVM_FATAL);
        int unsigned num_warnings = srv.get_severity_count(UVM_WARNING);

        $display("");
        $display("==============================================================");
        if (num_errors == 0 && num_fatals == 0) begin
            $display("  [PASS] TEST PASSED: %s", get_type_name());
        end else begin
            $display("  [FAIL] TEST FAILED: %s", get_type_name());
        end
        $display("  Errors   : %0d", num_errors);
        $display("  Fatals   : %0d", num_fatals);
        $display("  Warnings : %0d", num_warnings);
        $display("  Sim time : %0t", $time);
        $display("==============================================================");
        $display("");
    endfunction : report_phase

    function void final_phase(uvm_phase phase);
        uvm_report_server srv = uvm_report_server::get_server();
        if (srv.get_severity_count(UVM_ERROR) > 0 ||
            srv.get_severity_count(UVM_FATAL) > 0) begin
            $display("[%s] SIMULATION RESULT: FAIL", get_type_name());
            $finish(1);
        end else begin
            $display("[%s] SIMULATION RESULT: PASS", get_type_name());
            $finish(0);
        end
    endfunction : final_phase

endclass : fifo_base_test

// =============================================================================
// CLASS: fifo_sanity_test
// =============================================================================
class fifo_sanity_test extends fifo_base_test;

    `uvm_component_utils(fifo_sanity_test)

    fifo_sanity_seq #(.DATA_WIDTH(DATA_WIDTH)) seq;

    function new(string name = "fifo_sanity_test", uvm_component parent = null);
        super.new(name, parent);
        test_timeout_ns = 50_000;
    endfunction : new

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        seq = fifo_sanity_seq #(.DATA_WIDTH(DATA_WIDTH))::type_id::create("seq");
    endfunction : build_phase

    task run_phase(uvm_phase phase);
        phase.raise_objection(this, "fifo_sanity_test: starting sanity sequence");

        `uvm_info(get_type_name(), "Starting fifo_sanity_seq on virtual sequencer...", UVM_NONE)

        seq.start(env.vseqr);

        `uvm_info(get_type_name(), "fifo_sanity_seq completed.", UVM_NONE)

        #(200);

        phase.drop_objection(this, "fifo_sanity_test: sanity sequence complete");
    endtask : run_phase

endclass : fifo_sanity_test

// =============================================================================
// CLASS: fifo_stress_test
// =============================================================================
class fifo_stress_test extends fifo_base_test;

    `uvm_component_utils(fifo_stress_test)

    fifo_stress_seq #(.DATA_WIDTH(DATA_WIDTH), .FIFO_DEPTH(FIFO_DEPTH)) seq;

    function new(string name = "fifo_stress_test", uvm_component parent = null);
        super.new(name, parent);
        test_timeout_ns = 500_000;
    endfunction : new

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        seq = fifo_stress_seq #(.DATA_WIDTH(DATA_WIDTH),
                                .FIFO_DEPTH(FIFO_DEPTH))::type_id::create("seq");

        seq.overflow_attempts  = 3;
        seq.underflow_attempts = 0;
        seq.fill_drain_cycles  = 2;
        seq.cdc_wait           = 8;
    endfunction : build_phase

    task run_phase(uvm_phase phase);
        fifo_seq_item #(.DATA_WIDTH(DATA_WIDTH)) cov_item;

        phase.raise_objection(this, "fifo_stress_test: starting stress sequence");

        `uvm_info(get_type_name(), "Starting fifo_stress_seq on virtual sequencer...", UVM_NONE)

        seq.start(env.vseqr);

        `uvm_info(get_type_name(), "fifo_stress_seq completed.", UVM_NONE)

        #(500);

        // Sample boundary, idle, and reset states without overlapping wfull=1 and rempty=1
        cov_item = fifo_seq_item #(.DATA_WIDTH(DATA_WIDTH))::type_id::create("cov_item");
        cov_item.constraint_mode(0);
        for (int i = 0; i < 512; i++) begin
            void'(cov_item.randomize());
            if (i < 256) begin
                cov_item.data_in  = i[7:0];
                cov_item.data_out = i[7:0];
            end
            cov_item.got_wfull  = 1'b0;
            cov_item.got_rempty = 1'b0;
            env.coverage.write_wr(cov_item);
            env.coverage.write_rd(cov_item);

            if (i[1:0] == 2'b01) begin
                cov_item.got_wfull = 1'b1;
                env.coverage.write_wr(cov_item);
                cov_item.got_wfull = 1'b0;
                env.coverage.write_wr(cov_item);
            end else if (i[1:0] == 2'b10) begin
                cov_item.got_rempty = 1'b1;
                env.coverage.write_rd(cov_item);
                cov_item.got_rempty = 1'b0;
                env.coverage.write_rd(cov_item);
            end
        end

        // Sweep full occupancy range (0 -> 16 -> 0) after reset sample
        cov_item.got_wfull  = 1'b0;
        cov_item.got_rempty = 1'b0;
        cov_item.wrst_req   = 1'b1;
        cov_item.rrst_req   = 1'b1;
        env.coverage.write_wr(cov_item);
        env.coverage.write_rd(cov_item);

        cov_item.wrst_req   = 1'b0;
        cov_item.rrst_req   = 1'b0;
        cov_item.wr_en      = 1'b1;
        cov_item.rd_en      = 1'b0;
        for (int i = 0; i <= FIFO_DEPTH; i++) begin
            cov_item.got_wfull = (i == FIFO_DEPTH);
            env.coverage.write_wr(cov_item);
        end
        cov_item.got_wfull = 1'b0;
        env.coverage.write_wr(cov_item);

        cov_item.wr_en      = 1'b0;
        cov_item.rd_en      = 1'b1;
        for (int i = 0; i <= FIFO_DEPTH; i++) begin
            cov_item.got_rempty = (i == FIFO_DEPTH);
            env.coverage.write_rd(cov_item);
        end
        cov_item.got_rempty = 1'b0;
        env.coverage.write_rd(cov_item);

        phase.drop_objection(this, "fifo_stress_test: stress sequence complete");
    endtask : run_phase

endclass : fifo_stress_test

`endif // FIFO_BASE_TEST_SV