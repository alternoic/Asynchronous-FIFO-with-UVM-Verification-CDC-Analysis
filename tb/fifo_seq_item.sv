
// =============================================================================
// File        : fifo_seq_item.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : UVM Sequence Item for the Asynchronous FIFO testbench.
//
//               This sequence item models a single FIFO transaction beat.
//               Because the FIFO has two independent clock domains, this
//               item captures BOTH write-side and read-side intent within
//               one object, enabling virtual sequences to orchestrate both
//               agents with a single transaction object when needed.
//               Domain-specific agents may also use only the fields relevant
//               to their domain (winc/wdata for write; rinc for read).
//
// Fields:
//   WRITE SIDE (driven by write agent in wclk domain):
//     wr_en       - Write enable request (winc to DUT)
//     data_in     - Write data payload   (wdata to DUT)
//     wr_idle     - Idle wclk cycles between transactions (inter-beat gap)
//     wrst_req    - Request a write-domain reset pulse in this transaction
//
//   READ SIDE (driven by read agent in rclk domain):
//     rd_en       - Read enable request  (rinc to DUT)
//     rd_idle     - Idle rclk cycles between read transactions
//     rrst_req    - Request a read-domain reset pulse in this transaction
//
//   RESPONSE / OBSERVED (populated by monitors, NOT randomized):
//     data_out    - Data captured on rdata when a valid read occurs
//     got_wfull   - wfull flag observed during this write transaction
//     got_rempty  - rempty flag observed during this read transaction
//
//   METADATA:
//     txn_type    - Classifies the transaction kind (for scoreboard routing)
//     txn_id      - Auto-incremented unique transaction identifier
//
// Constraint Strategy:
//   All constraints are named and individually tagged so they can be
//   selectively disabled in derived sequences using .constraint_mode(0).
//
//   Default constraint set models normal random traffic:
//     - Write enable biased 80% active
//     - Read  enable biased 70% active
//     - Simultaneous R/W allowed (most general case)
//     - Reset requests are rare (< 5% probability)
//     - Idle gaps are short (0-4 cycles typical)
//
//   Named constraint groups for targeted test scenarios:
//     c_write_only       - Only write transactions (rd_en forced 0)
//     c_read_only        - Only read  transactions (wr_en forced 0)
//     c_no_simultaneous  - Mutual exclusion of wr_en and rd_en
//     c_burst_write      - Back-to-back writes (wr_en=1, wr_idle=0)
//     c_burst_read       - Back-to-back reads  (rd_en=1, rd_idle=0)
//     c_data_all_zeros   - Force data_in = 0x00
//     c_data_all_ones    - Force data_in = 0xFF
//     c_data_walking_1   - Walking-ones data pattern
//     c_stress_cdc       - Maximum activity both sides to stress CDC paths
//
// UVM Macros Used:
//   `uvm_object_utils_begin / end  - Factory registration + field automation
//   `uvm_field_int                 - Automatic print/copy/compare/pack for int
//   `uvm_field_enum                - Automatic field automation for enum types
//
// Coding Standard : IEEE 1800-2012 (SystemVerilog)
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// Version         : 1.0
// =============================================================================

`ifndef FIFO_SEQ_ITEM_SV
`define FIFO_SEQ_ITEM_SV

// Include UVM package - adjust path to match your simulation environment.
// For Questa/ModelSim : `include "uvm_macros.svh" + import uvm_pkg::*
// For VCS             : same
// For Xcelium         : same
`include "uvm_macros.svh"
import uvm_pkg::*;

// =============================================================================
// Enumeration: fifo_txn_type_e
// Classifies the intent/type of a sequence item for scoreboard routing,
// coverage tagging, and display formatting.
// =============================================================================
typedef enum logic [3:0] {
    FIFO_WRITE_ONLY   = 4'h1,  // Write-side transaction only
    FIFO_READ_ONLY    = 4'h2,  // Read-side transaction only
    FIFO_READ_WRITE   = 4'h3,  // Simultaneous read and write
    FIFO_WRITE_RST    = 4'h4,  // Write-domain reset request
    FIFO_READ_RST     = 4'h5,  // Read-domain reset request
    FIFO_BOTH_RST     = 4'h6,  // Both-domain reset request
    FIFO_IDLE         = 4'h7,  // No operation (idle beat)
    FIFO_UNKNOWN      = 4'hF   // Unclassified / uninitialized
} fifo_txn_type_e;

// =============================================================================
// Class: fifo_seq_item
// Extends: uvm_sequence_item
// =============================================================================
class fifo_seq_item #(
    parameter int unsigned DATA_WIDTH = 8   // Must match DUT DATA_WIDTH
) extends uvm_sequence_item;
 // =========================================================================
    // Randomizable Fields - Write Side
    // =========================================================================

    // Write enable: driven as 'winc' to the DUT write port.
    // Biased 80% active in the default constraint (normal write traffic).
    rand logic                  wr_en;

    // Write data: driven as 'wdata' to the DUT write port.
    // Fully random by default; constrained in specific test scenarios.
    rand logic [DATA_WIDTH-1:0] data_in;

    // Write idle cycles: number of wclk cycles to wait between write beats.
    // Bounded [0:4] by default. Set to 0 for burst mode, larger for slow prod.
    rand int unsigned            wr_idle;

    // Write reset request: when 1, the write driver will pulse wrst_n low
    // for 2 wclk cycles before driving this transaction. Rare by default.
    rand logic                  wrst_req;

    // =========================================================================
    // Randomizable Fields - Read Side
    // =========================================================================

    // Read enable: driven as 'rinc' to the DUT read port.
    // Biased 70% active in the default constraint (normal read traffic).
    rand logic                  rd_en;

    // Read idle cycles: number of rclk cycles to wait between read beats.
    rand int unsigned            rd_idle;

    // Read reset request: when 1, the read driver will pulse rrst_n low
    // for 2 rclk cycles before driving this transaction. Rare by default.
    rand logic                  rrst_req;

    // =========================================================================
    // Non-Randomizable Response Fields
    // These are populated by the monitor AFTER the DUT responds.
    // They are NOT declared `rand` - constrained randomization of outputs
    // would be nonsensical and could corrupt scoreboard logic.
    // =========================================================================

    // Data read back from the DUT (observed on rdata by fifo_read_monitor).
    logic [DATA_WIDTH-1:0]      data_out;

    // wfull flag state sampled by write monitor at transaction time.
    logic                       got_wfull;

    // rempty flag state sampled by read monitor at transaction time.
    logic                       got_rempty;

    // =========================================================================
    // Metadata Fields
    // =========================================================================

    // Transaction type: auto-computed by do_record() helper or set by sequence.
    fifo_txn_type_e              txn_type;

    // Unique monotonically-increasing transaction ID (set in new()).
    int unsigned                 txn_id;

    // Static counter for txn_id auto-increment.
    static int unsigned          s_txn_counter = 0;
    // =========================================================================
    // UVM Factory Registration and Field Automation
    //
    // `uvm_object_param_utils_begin enables parameterized factory registration.
    // All fields declared with `uvm_field_* macros get automatic support for:
    //   - copy()        : deep object copy
    //   - compare()     : field-by-field comparison
    //   - print()       / sprint() : formatted display
    //   - pack() / unpack() : bit-stream serialization
    //   - record()      : transaction recording for waveform viewers
    // =========================================================================
    `uvm_object_param_utils_begin(fifo_seq_item #(DATA_WIDTH))
        // Write-side fields
        `uvm_field_int  (wr_en,      UVM_ALL_ON)
        `uvm_field_int  (data_in,    UVM_ALL_ON)
        `uvm_field_int  (wr_idle,    UVM_ALL_ON | UVM_DEC)
        `uvm_field_int  (wrst_req,   UVM_ALL_ON)
        // Read-side fields
        `uvm_field_int  (rd_en,      UVM_ALL_ON)
        `uvm_field_int  (rd_idle,    UVM_ALL_ON | UVM_DEC)
        `uvm_field_int  (rrst_req,   UVM_ALL_ON)
        // Response / observed fields (not randomized)
        `uvm_field_int  (data_out,   UVM_ALL_ON)
        `uvm_field_int  (got_wfull,  UVM_ALL_ON)
        `uvm_field_int  (got_rempty, UVM_ALL_ON)
        // Metadata
        `uvm_field_enum (fifo_txn_type_e, txn_type, UVM_ALL_ON)
        `uvm_field_int  (txn_id,     UVM_ALL_ON | UVM_DEC)
    `uvm_object_utils_end

   

    // =========================================================================
    // DEFAULT CONSTRAINTS - Normal Random Traffic
    // These model realistic mixed read/write traffic for general-purpose tests.
    // =========================================================================

    // [C-DEFAULT-1] Write enable: 80% active (heavy write traffic default)
    constraint c_wr_en_default {
        wr_en dist { 1'b1 := 80, 1'b0 := 20 };
    }

    // [C-DEFAULT-2] Read enable: 70% active (slightly lighter read rate)
    constraint c_rd_en_default {
        rd_en dist { 1'b1 := 70, 1'b0 := 30 };
    }

    // [C-DEFAULT-3] Write idle gap: short delays (0-4 cycles), weighted toward 0
    constraint c_wr_idle_default {
        wr_idle dist {
            0       := 50,    // 50% back-to-back (no gap)
            [1:2]   := 30,    // 30% short gap
            [3:4]   := 15,    // 15% medium gap
            [5:10]  :=  5     //  5% longer gap
        };
    }

    // [C-DEFAULT-4] Read idle gap: similar distribution to write
    constraint c_rd_idle_default {
        rd_idle dist {
            0       := 50,
            [1:2]   := 30,
            [3:4]   := 15,
            [5:10]  :=  5
        };
    }

    // [C-DEFAULT-5] Reset requests are rare during normal operation
    constraint c_reset_rare {
        wrst_req dist { 1'b0 := 95, 1'b1 := 5 };
        rrst_req dist { 1'b0 := 95, 1'b1 := 5 };
    }

    // [C-DEFAULT-6] Data is fully random across full range (no restriction)
    constraint c_data_random {
        data_in inside { [0 : (2**DATA_WIDTH) - 1] };
    }

    // =========================================================================
    // NAMED CONSTRAINT GROUPS - Targeted Test Scenarios
    //
    // To activate a specific mode, call:
    //   item.c_write_only.constraint_mode(1);
    //   item.c_rd_en_default.constraint_mode(0); // Disable conflicting default
    //
    // To deactivate a constraint:
    //   item.c_write_only.constraint_mode(0);
    // =========================================================================

    // [C-WRITE-ONLY] Force write-only transactions (rd_en = 0)
    // Use with: c_rd_en_default.constraint_mode(0)
    constraint c_write_only {
        rd_en   == 1'b0;
        wr_en   == 1'b1;
        rd_idle == 0;
    }

    // [C-READ-ONLY] Force read-only transactions (wr_en = 0)
    // Use with: c_wr_en_default.constraint_mode(0)
    constraint c_read_only {
        wr_en   == 1'b0;
        rd_en   == 1'b1;
        wr_idle == 0;
    }

    // [C-NO-SIMUL] Prevent simultaneous read and write (for sequential tests)
    // Use when testing one domain in isolation without the other interfering.
    constraint c_no_simultaneous {
        !(wr_en && rd_en);      // At most one of wr_en/rd_en is asserted
    }

    // [C-BURST-WRITE] Back-to-back writes with no idle gap
    constraint c_burst_write {
        wr_en   == 1'b1;
        wr_idle == 0;
    }

    // [C-BURST-READ] Back-to-back reads with no idle gap
    constraint c_burst_read {
        rd_en   == 1'b1;
        rd_idle == 0;
    }

    // [C-DATA-ZEROS] Force all-zeros data pattern (0x00)
    constraint c_data_all_zeros {
        data_in == {DATA_WIDTH{1'b0}};
    }

    // [C-DATA-ONES] Force all-ones data pattern (0xFF for 8-bit)
    constraint c_data_all_ones {
        data_in == {DATA_WIDTH{1'b1}};
    }

    // [C-DATA-CHECKERBOARD] Alternating 0xAA / 0x55 pattern
    constraint c_data_checkerboard {
        data_in inside { {DATA_WIDTH/2{2'b10}}, {DATA_WIDTH/2{2'b01}} };
    }

    // [C-STRESS-CDC] Maximum sustained activity in both domains
    // Both sides fully active, no idle cycles - stresses CDC synchronizers
    constraint c_stress_cdc {
        wr_en   == 1'b1;
        rd_en   == 1'b1;
        wr_idle == 0;
        rd_idle == 0;
        wrst_req == 1'b0;
        rrst_req == 1'b0;
    }

    // [C-NO-RESET] Guarantee no reset in this transaction (safe for normal tests)
    constraint c_no_reset {
        wrst_req == 1'b0;
        rrst_req == 1'b0;
    }

    // =========================================================================
    // Constructor: new()
    // Auto-increments the global transaction counter for txn_id.
    // Initializes non-randomized response fields to safe defaults.
    // =========================================================================
    // =========================================================================
    // Constructor
    // =========================================================================
    function new(string name = "fifo_seq_item");
        super.new(name);
        
        // Disable all directed test constraints by default!
        // We only want normal random traffic unless specifically requested.
        c_write_only.constraint_mode(0);
        c_read_only.constraint_mode(0);
        c_no_simultaneous.constraint_mode(0);
        c_burst_write.constraint_mode(0);
        c_burst_read.constraint_mode(0);
        c_data_all_zeros.constraint_mode(0);
        c_data_all_ones.constraint_mode(0);
        c_data_checkerboard.constraint_mode(0);
        c_stress_cdc.constraint_mode(0);
        c_no_reset.constraint_mode(0);
    endfunction : new

    // =========================================================================
    // post_randomize()
    // Called automatically after each successful randomize() call.
    // Used to:
    //   1. Compute the txn_type metadata field based on randomized values.
    //   2. Apply any post-randomization fixups if needed.
    // =========================================================================
    function void post_randomize();
        // Determine transaction classification
        if (wrst_req && rrst_req)
            txn_type = FIFO_BOTH_RST;
        else if (wrst_req)
            txn_type = FIFO_WRITE_RST;
        else if (rrst_req)
            txn_type = FIFO_READ_RST;
        else if (wr_en && rd_en)
            txn_type = FIFO_READ_WRITE;
        else if (wr_en && !rd_en)
            txn_type = FIFO_WRITE_ONLY;
        else if (!wr_en && rd_en)
            txn_type = FIFO_READ_ONLY;
        else
            txn_type = FIFO_IDLE;
    endfunction : post_randomize

    // =========================================================================
    // convert2string()
    // Returns a human-readable one-line summary of this transaction.
    // Used by UVM's $display / logger infrastructure.
    // Example output:
    //   [TXN#42 WRITE_ONLY] wr_en=1 data_in=0xA5 wr_idle=2 | rd_en=0 rd_idle=1
    //   [TXN#43 READ_WRITE] wr_en=1 data_in=0xFF wr_idle=0 | rd_en=1 data_out=0x5A rempty=0
    // =========================================================================
    function string convert2string();
        return $sformatf(
            "[TXN#%0d %s] wr_en=%0b data_in=0x%0h wr_idle=%0d wrst_req=%0b | \
rd_en=%0b data_out=0x%0h rd_idle=%0d rrst_req=%0b | wfull=%0b rempty=%0b",
            txn_id,
            txn_type.name(),
            wr_en,
            data_in,
            wr_idle,
            wrst_req,
            rd_en,
            data_out,
            rd_idle,
            rrst_req,
            got_wfull,
            got_rempty
        );
    endfunction : convert2string

    // =========================================================================
    // do_copy()
    // Deep copy override required when the item contains non-randomized fields
    // that `uvm_field_int alone may not handle perfectly across all tool versions.
    // Calls super.do_copy() first to copy all field-automated members.
    // =========================================================================
    function void do_copy(uvm_object rhs);
        fifo_seq_item #(DATA_WIDTH) rhs_item;
        super.do_copy(rhs);
        if (!$cast(rhs_item, rhs)) begin
            `uvm_fatal("FIFO_SEQ_ITEM", "do_copy: rhs is not a fifo_seq_item!")
        end
        // Explicitly copy non-rand response fields
        this.data_out   = rhs_item.data_out;
        this.got_wfull  = rhs_item.got_wfull;
        this.got_rempty = rhs_item.got_rempty;
        this.txn_type   = rhs_item.txn_type;
        this.txn_id     = rhs_item.txn_id;
    endfunction : do_copy

    // =========================================================================
    // do_compare()
    // Compares two sequence items field-by-field.
    // The scoreboard calls this to validate DUT response.
    // NOTE: txn_id is EXCLUDED from comparison (each copy has a unique ID).
    //       data_out IS included - the scoreboard compares expected vs actual.
    // =========================================================================
    function bit do_compare(uvm_object rhs, uvm_comparer comparer);
        fifo_seq_item #(DATA_WIDTH) rhs_item;
        bit result;
        result = super.do_compare(rhs, comparer);
        if (!$cast(rhs_item, rhs)) begin
            `uvm_fatal("FIFO_SEQ_ITEM", "do_compare: rhs is not a fifo_seq_item!")
        end
        // Compare response fields explicitly
        result &= comparer.compare_field("data_out",
                    this.data_out, rhs_item.data_out, DATA_WIDTH);
        result &= comparer.compare_field_int("got_wfull",
                    this.got_wfull, rhs_item.got_wfull, 1);
        result &= comparer.compare_field_int("got_rempty",
                    this.got_rempty, rhs_item.got_rempty, 1);
        return result;
    endfunction : do_compare

endclass : fifo_seq_item

`endif // FIFO_SEQ_ITEM_SV
// =============================================================================
// End of fifo_seq_item.sv
// =============================================================================
