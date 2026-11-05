// =============================================================================
// File        : fifo_coverage.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : UVM Functional Coverage Collector for the Async FIFO.
//               Subscribes to BOTH write and read monitor analysis ports
//               using distinct analysis-imp suffixes (_wr / _rd).
//
// Coverage Architecture:
//   ┌----------------------------------------------------------------------┐
//   |                     fifo_coverage                                    |
//   |                                                                      |
//   |  wr_export <-- fifo_wr_monitor.ap        (write-side items)         |
//   |  rd_export <-- fifo_rd_monitor.ap_flag   (every-cycle read items)   |
//   |                                                                      |
//   |  ┌--------------------------------------------------------------┐   |
//   |  | CG1: fifo_write_cg    - write-domain signals per write beat  |   |
//   |  | CG2: fifo_read_cg     - read-domain signals per read beat    |   |
//   |  | CG3: fifo_flag_cg     - flag transitions (every rclk cycle)  |   |
//   |  | CG4: fifo_data_cg     - data payload patterns                |   |
//   |  | CG5: fifo_reset_cg    - reset sequence coverage              |   |
//   |  | CG6: fifo_occupancy_cg- queue fill-level bins                |   |
//   |  └--------------------------------------------------------------┘   |
//   └----------------------------------------------------------------------┘
//
// Covergroup Sampling Strategy:
//   - CG1, CG4 : Sampled in write_wr() - fires on every valid write item
//   - CG2, CG3 : Sampled in write_rd() - fires on every read-side item
//   - CG5       : Sampled in write_wr() when wrst_req=1 or rrst_req=1
//   - CG6       : Sampled in both write_wr() and write_rd() via tracked counter
//
// Cross-Coverage Edge Cases Verified:
//   - wr_en active while wfull=1  (write attempted on full FIFO)
//   - rd_en active while rempty=1 (read attempted on empty FIFO)
//   - Simultaneous wr_en && rd_en (concurrent access across clock domains)
//   - wfull rising    (FIFO transitions to full state)
//   - rempty rising   (FIFO transitions to empty state)
//   - wfull falling   (FIFO transitions from full state - space freed)
//   - rempty falling  (FIFO transitions from empty - data arrives)
//   - Reset in write domain only
//   - Reset in read  domain only
//   - Simultaneous reset in both domains
//
// Coding Standard : IEEE 1800-2012 | UVM 1.2
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// Version         : 1.0
// =============================================================================

`ifndef FIFO_COVERAGE_SV
`define FIFO_COVERAGE_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

// Declare two distinct analysis-imp suffix types (if not already declared).
// Guard against re-declaration if scoreboard is compiled in the same scope.
`ifndef FIFO_APS_DECLARED
  `define FIFO_APS_DECLARED
 // `uvm_analysis_imp_decl(_wr)
  //`uvm_analysis_imp_decl(_rd)
`endif

class fifo_coverage #(
    parameter int unsigned DATA_WIDTH  = 8,
    parameter int unsigned FIFO_DEPTH  = 16
) extends uvm_component;

    `uvm_component_param_utils(fifo_coverage #(DATA_WIDTH, FIFO_DEPTH))

    // =========================================================================
    // Analysis Imports
    // wr_export : receives valid write items  from fifo_wr_monitor.ap
    // rd_export : receives all-cycle items    from fifo_rd_monitor.ap_flag
    // =========================================================================
    uvm_analysis_imp_wr #(fifo_seq_item #(DATA_WIDTH),
                          fifo_coverage  #(DATA_WIDTH, FIFO_DEPTH)) wr_export;

    uvm_analysis_imp_rd #(fifo_seq_item #(DATA_WIDTH),
                          fifo_coverage  #(DATA_WIDTH, FIFO_DEPTH)) rd_export;

    // =========================================================================
    // Internal Mirrored State
    // Maintained here to feed occupancy and flag-transition covergroups.
    // Updated on every write_wr() and write_rd() call.
    // =========================================================================
    int unsigned  tracked_occupancy;   // Estimated FIFO word count
    logic         prev_wfull;          // Previous wfull value  (flag transition)
    logic         prev_rempty;         // Previous rempty value (flag transition)
    logic         curr_wfull;          // Current  wfull from latest item
    logic         curr_rempty;         // Current  rempty from latest item
    logic         curr_wr_en;          // Current  wr_en from latest write item
    logic         curr_rd_en;          // Current  rd_en from latest read item
    logic         curr_wrst_req;       // Reset request - write domain
    logic         curr_rrst_req;       // Reset request - read domain
    logic [DATA_WIDTH-1:0] curr_data;  // Current write data payload

    // =========================================================================
    // COVERGROUP 1: fifo_write_cg
    // Sampled on every valid write transaction (winc=1, wfull=0).
    // Covers write-side protocol signals and their combinations.
    // =========================================================================
    covergroup fifo_write_cg;

        // --- Write enable ---
        cp_wr_en : coverpoint curr_wr_en {
            bins wr_active   = {1'b1};
            bins wr_inactive = {1'b0};
        }

        // --- Full flag at time of write ---
        cp_wfull : coverpoint curr_wfull {
            bins fifo_full     = {1'b1};
            bins fifo_not_full = {1'b0};
        }

        // --- CRITICAL CROSS: write enable vs. full flag ---
        // Bins of interest:
        //   wr_when_full : wr_en=1 && wfull=1  → protocol violation scenario
        //   wr_when_free : wr_en=1 && wfull=0  → normal write (must always hit)
        //   idle_full    : wr_en=0 && wfull=1  → FIFO full, producer idle (backpressure)
        //   idle_free    : wr_en=0 && wfull=0  → normal idle
        cx_wr_en_x_wfull : cross cp_wr_en, cp_wfull {
            bins wr_when_full  = binsof(cp_wr_en.wr_active)   && binsof(cp_wfull.fifo_full);
            bins wr_when_free  = binsof(cp_wr_en.wr_active)   && binsof(cp_wfull.fifo_not_full);
            bins idle_full     = binsof(cp_wr_en.wr_inactive) && binsof(cp_wfull.fifo_full);
            bins idle_free     = binsof(cp_wr_en.wr_inactive) && binsof(cp_wfull.fifo_not_full);
        }

        // --- wfull transition: rising edge (FIFO becomes full) ---
        cp_wfull_rose : coverpoint (curr_wfull && !prev_wfull) {
            bins wfull_assertion = {1'b1};
        }

        // --- wfull transition: falling edge (space freed after read) ---
        cp_wfull_fell : coverpoint (!curr_wfull && prev_wfull) {
            bins wfull_deassertion = {1'b1};
        }

        // --- Write reset request ---
        cp_wrst_req : coverpoint curr_wrst_req {
            bins reset_requested = {1'b1};
            bins no_reset        = {1'b0};
        }

    endgroup : fifo_write_cg

    // =========================================================================
    // COVERGROUP 2: fifo_read_cg
    // Sampled on every valid read transaction (rinc=1, rempty=0).
    // Covers read-side protocol signals and their critical edge cases.
    // =========================================================================
    covergroup fifo_read_cg;

        // --- Read enable ---
        cp_rd_en : coverpoint curr_rd_en {
            bins rd_active   = {1'b1};
            bins rd_inactive = {1'b0};
        }

        // --- Empty flag at time of read ---
        cp_rempty : coverpoint curr_rempty {
            bins fifo_empty     = {1'b1};
            bins fifo_not_empty = {1'b0};
        }

        // --- CRITICAL CROSS: read enable vs. empty flag ---
        // Bins of interest:
        //   rd_when_empty : rd_en=1 && rempty=1  → protocol violation scenario
        //   rd_when_data  : rd_en=1 && rempty=0  → normal read (must always hit)
        //   idle_empty    : rd_en=0 && rempty=1  → FIFO empty, consumer idle
        //   idle_data     : rd_en=0 && rempty=0  → data available but not consumed
        cx_rd_en_x_rempty : cross cp_rd_en, cp_rempty {
            bins rd_when_empty = binsof(cp_rd_en.rd_active)   && binsof(cp_rempty.fifo_empty);
            bins rd_when_data  = binsof(cp_rd_en.rd_active)   && binsof(cp_rempty.fifo_not_empty);
            bins idle_empty    = binsof(cp_rd_en.rd_inactive) && binsof(cp_rempty.fifo_empty);
            bins idle_data     = binsof(cp_rd_en.rd_inactive) && binsof(cp_rempty.fifo_not_empty);
        }

        // --- rempty transition: rising (FIFO becomes empty) ---
        cp_rempty_rose : coverpoint (curr_rempty && !prev_rempty) {
            bins rempty_assertion = {1'b1};
        }

        // --- rempty transition: falling (data arrives via write synchronizer) ---
        cp_rempty_fell : coverpoint (!curr_rempty && prev_rempty) {
            bins rempty_deassertion = {1'b1};
        }

        // --- Read reset request ---
        cp_rrst_req : coverpoint curr_rrst_req {
            bins reset_requested = {1'b1};
            bins no_reset        = {1'b0};
        }

    endgroup : fifo_read_cg

    // =========================================================================
    // COVERGROUP 3: fifo_flag_cg
    // Sampled on every rclk cycle (via ap_flag from read monitor).
    // Captures all possible (wfull, rempty) flag-state combinations -
    // important for verifying FIFO never simultaneously reports full AND empty.
    // =========================================================================
    covergroup fifo_flag_cg;

        cp_wfull_state : coverpoint curr_wfull {
            bins full     = {1'b1};
            bins not_full = {1'b0};
        }

        cp_rempty_state : coverpoint curr_rempty {
            bins empty     = {1'b1};
            bins not_empty = {1'b0};
        }

        // --- CRITICAL CROSS: all 4 flag states ---
        // The (full=1, empty=1) state MUST NEVER occur - it is physically
        // impossible. If this bin is hit, it indicates a serious DUT bug.
        // The (full=0, empty=0) bin represents the normal operating range.
        cx_flags : cross cp_wfull_state, cp_rempty_state {
            bins idle_state         = binsof(cp_wfull_state.not_full) && binsof(cp_rempty_state.empty);
            bins normal_state       = binsof(cp_wfull_state.not_full) && binsof(cp_rempty_state.not_empty);
            bins full_state         = binsof(cp_wfull_state.full)     && binsof(cp_rempty_state.not_empty);
            // This bin should NEVER be hit - flag as illegal if seen
            bins impossible_state   = binsof(cp_wfull_state.full)     && binsof(cp_rempty_state.empty);
        }

        // --- Simultaneous write and read activity ---
        cp_simul_rw : coverpoint (curr_wr_en && curr_rd_en) {
            bins simul_active   = {1'b1};  // Both domains active simultaneously
            bins not_simul      = {1'b0};
        }

    endgroup : fifo_flag_cg

    // =========================================================================
    // COVERGROUP 4: fifo_data_cg
    // Sampled on every valid write. Covers data payload patterns to ensure
    // the FIFO passes all common fault-model data patterns.
    // =========================================================================
    covergroup fifo_data_cg;

        cp_data_pattern : coverpoint curr_data {
            // Classic data integrity test patterns
            bins all_zeros      = {  {DATA_WIDTH{1'b0}}  };      // 0x00
            bins all_ones       = {  {DATA_WIDTH{1'b1}}  };      // 0xFF
            bins alternating_a  = {  {DATA_WIDTH/2{2'b10}} };   // 0xAA
            bins alternating_5  = {  {DATA_WIDTH/2{2'b01}} };   // 0x55
            // Walking 1s (each bit individually set)
            bins walking_ones[] = { 1, 2, 4, 8, 16, 32, 64, 128 };
            // Walking 0s (each bit individually cleared)
            bins walking_zeros[]= { 8'hFE, 8'hFD, 8'hFB, 8'hF7,
                                    8'hEF, 8'hDF, 8'hBF, 8'h7F };
            // Random values (catch-all)
            bins random_data    = default;
        }

        // MSB and LSB explicit coverage
        cp_msb : coverpoint curr_data[DATA_WIDTH-1] {
            bins msb_set   = {1'b1};
            bins msb_clear = {1'b0};
        }

        cp_lsb : coverpoint curr_data[0] {
            bins lsb_set   = {1'b1};
            bins lsb_clear = {1'b0};
        }

    endgroup : fifo_data_cg

    // =========================================================================
    // COVERGROUP 5: fifo_reset_cg
    // Sampled when any reset request is detected.
    // Covers all independent and simultaneous reset scenarios.
    // =========================================================================
    covergroup fifo_reset_cg;

        cp_wrst_domain : coverpoint curr_wrst_req {
            bins write_reset     = {1'b1};
            bins no_write_reset  = {1'b0};
        }

        cp_rrst_domain : coverpoint curr_rrst_req {
            bins read_reset      = {1'b1};
            bins no_read_reset   = {1'b0};
        }

        // Cross: all combinations of domain-level reset assertion
        cx_reset_domains : cross cp_wrst_domain, cp_rrst_domain {
            bins only_write_rst = binsof(cp_wrst_domain.write_reset) &&
                                  binsof(cp_rrst_domain.no_read_reset);
            bins only_read_rst  = binsof(cp_wrst_domain.no_write_reset) &&
                                  binsof(cp_rrst_domain.read_reset);
            bins both_rst       = binsof(cp_wrst_domain.write_reset) &&
                                  binsof(cp_rrst_domain.read_reset);
        }

    endgroup : fifo_reset_cg

    // =========================================================================
    // COVERGROUP 6: fifo_occupancy_cg
    // Sampled on every write and read transaction.
    // Tracks the estimated fill level of the FIFO via tracked_occupancy counter.
    //
    // Note: This is a software-model approximation - the actual hardware
    // occupancy is not directly observable. The approximation is valid because:
    //   tracked_occupancy increments on every write_wr() call (valid write)
    //   tracked_occupancy decrements on every write_rd() call (valid read)
    //   Both are gated by the DUT's full/empty flags, so they track hardware.
    // =========================================================================
    covergroup fifo_occupancy_cg;

        cp_occupancy : coverpoint tracked_occupancy {
            bins empty           = { 0 };
            bins light_load      = { [1 : FIFO_DEPTH/4]   };
            bins half_load       = { [FIFO_DEPTH/4+1 : FIFO_DEPTH/2] };
            bins heavy_load      = { [FIFO_DEPTH/2+1 : FIFO_DEPTH-1] };
            bins almost_full     = { FIFO_DEPTH - 1 };
            bins full            = { FIFO_DEPTH };
        }

        // Cross: occupancy vs. write enable
        // Must hit "write when almost_full" to verify last-slot full assertion
        cx_occ_x_wr : cross cp_occupancy, curr_wr_en {
            bins write_to_almost_full = binsof(cp_occupancy.almost_full) &&
                                        binsof(curr_wr_en) intersect {1'b1};
            bins write_to_empty       = binsof(cp_occupancy.empty)       &&
                                        binsof(curr_wr_en) intersect {1'b1};
        }

        // Cross: occupancy vs. read enable
        // Must hit "read when occupancy=1" to verify last-word empty assertion
        cx_occ_x_rd : cross cp_occupancy, curr_rd_en {
            bins read_to_empty   = binsof(cp_occupancy.light_load) &&
                                   binsof(curr_rd_en) intersect {1'b1};
            bins read_when_full  = binsof(cp_occupancy.full)       &&
                                   binsof(curr_rd_en) intersect {1'b1};
        }

    endgroup : fifo_occupancy_cg

    // =========================================================================
    // Constructor: Create all covergroup instances
    // =========================================================================
    function new(string name = "fifo_coverage", uvm_component parent = null);
        super.new(name, parent);

        // Initialize mirrored state
        tracked_occupancy = 0;
        prev_wfull        = 1'b0;
        prev_rempty       = 1'b1;   // Reset state: FIFO starts empty
        curr_wfull        = 1'b0;
        curr_rempty       = 1'b1;
        curr_wr_en        = 1'b0;
        curr_rd_en        = 1'b0;
        curr_wrst_req     = 1'b0;
        curr_rrst_req     = 1'b0;
        curr_data         = '0;

        // Instantiate all covergroups
        fifo_write_cg    = new();
        fifo_read_cg     = new();
        fifo_flag_cg     = new();
        fifo_data_cg     = new();
        fifo_reset_cg    = new();
        fifo_occupancy_cg= new();
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
    // FUNCTION: write_wr - fires on every valid write item from wr monitor
    //
    // Updates mirrored write-side state, increments occupancy tracker,
    // and samples write-domain covergroups.
    // =========================================================================
    function void write_wr(fifo_seq_item #(DATA_WIDTH) item);
        // --- Update mirrored write-side state --------------------------------
        prev_wfull    = curr_wfull;
        curr_wfull    = item.got_wfull;
        curr_wr_en    = item.wr_en;
        curr_data     = item.data_in;
        curr_wrst_req = item.wrst_req;

        // --- Update occupancy counter ----------------------------------------
        // Increment on a valid write (wr_en=1, wfull=0)
        if (item.wr_en && !item.got_wfull) begin
            if (tracked_occupancy < FIFO_DEPTH)
                tracked_occupancy++;
        end

        // Handle reset flush
        if (item.wrst_req) begin
            tracked_occupancy = 0;
            curr_wrst_req     = 1'b1;
        end

        // --- Sample covergroups ----------------------------------------------
        fifo_write_cg.sample();
        fifo_data_cg.sample();
        fifo_occupancy_cg.sample();

        // Sample reset covergroup only when a reset is requested
        if (item.wrst_req || item.rrst_req) begin
            curr_rrst_req = item.rrst_req;
            fifo_reset_cg.sample();
        end

        `uvm_info("FIFO_COV",
            $sformatf("[WR SAMPLE] wr_en=%0b wfull=%0b data=0x%0h occ=%0d",
                item.wr_en, item.got_wfull, item.data_in, tracked_occupancy), UVM_DEBUG)
    endfunction : write_wr

    // =========================================================================
    // FUNCTION: write_rd - fires on every read-side item from rd monitor
    //
    // Updates mirrored read-side state, decrements occupancy tracker,
    // and samples read-domain and flag covergroups.
    // =========================================================================
    function void write_rd(fifo_seq_item #(DATA_WIDTH) item);
        // --- Update mirrored read-side state ---------------------------------
        prev_rempty   = curr_rempty;
        curr_rempty   = item.got_rempty;
        curr_rd_en    = item.rd_en;
        curr_rrst_req = item.rrst_req;

        // --- Update occupancy counter ----------------------------------------
        // Decrement on a valid read (rd_en=1, rempty=0)
        if (item.rd_en && !item.got_rempty) begin
            if (tracked_occupancy > 0)
                tracked_occupancy--;
        end

        // Handle reset flush
        if (item.rrst_req) begin
            tracked_occupancy = 0;
        end

        // --- Sample covergroups ----------------------------------------------
        fifo_read_cg.sample();
        fifo_flag_cg.sample();
        fifo_occupancy_cg.sample();

        // Sample reset covergroup only when a reset is requested
        if (item.wrst_req || item.rrst_req) begin
            curr_wrst_req = item.wrst_req;
            fifo_reset_cg.sample();
        end

        // --- Impossible state detector ---------------------------------------
        // wfull=1 AND rempty=1 simultaneously is physically impossible.
        // If the flag_cg hits this cross bin, escalate immediately.
        if (curr_wfull && curr_rempty) begin
            `uvm_error("FIFO_COV",
                $sformatf("[IMPOSSIBLE STATE] wfull=1 AND rempty=1 simultaneously detected! \
DUT has a critical full/empty logic bug. Time=%0t", $time))
        end

        `uvm_info("FIFO_COV",
            $sformatf("[RD SAMPLE] rd_en=%0b rempty=%0b data_out=0x%0h occ=%0d",
                item.rd_en, item.got_rempty, item.data_out, tracked_occupancy), UVM_DEBUG)
    endfunction : write_rd

    // =========================================================================
    // report_phase: Print coverage summary
    // =========================================================================
    function void report_phase(uvm_phase phase);
        string sep = "---------------------------------------------";
        `uvm_info("FIFO_COV", $sformatf(
            "\n%s\n  FIFO COVERAGE REPORT\n%s\n  Write CG     : %.2f%%\n  Read  CG     : %.2f%%\n  Flag  CG     : %.2f%%\n  Data  CG     : %.2f%%\n  Reset CG     : %.2f%%\n  Occupancy CG : %.2f%%\n%s",
            sep, sep,
            fifo_write_cg.get_coverage(),
            fifo_read_cg.get_coverage(),
            fifo_flag_cg.get_coverage(),
            fifo_data_cg.get_coverage(),
            fifo_reset_cg.get_coverage(),
            fifo_occupancy_cg.get_coverage(),
            sep
        ), UVM_NONE)
    endfunction : report_phase

endclass : fifo_coverage

`endif // FIFO_COVERAGE_SV
// =============================================================================
// End of fifo_coverage.sv
// =============================================================================
