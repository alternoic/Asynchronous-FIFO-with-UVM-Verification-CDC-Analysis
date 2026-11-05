// =============================================================================
// File        : fifo_agent.sv
// Project     : Asynchronous FIFO with CDC Verification
// Description : UVM Agent classes for the Asynchronous FIFO testbench.
//               Contains TWO separate agent classes in one file:
//
//   fifo_wr_agent  - Write clock domain agent (wclk)
//   fifo_rd_agent  - Read  clock domain agent (rclk)
//
// Rationale for separate agents (vs. unified agent):
//   The async FIFO has two COMPLETELY INDEPENDENT clock domains with separate
//   VIF clocking blocks, separate reset networks, and separate driver/monitor
//   classes. A unified agent would require artificial multiplexing of the two
//   domains and would obscure the CDC architecture. Two agents cleanly model
//   the two independent producers/consumers in the system.
//
// Each agent follows the standard UVM agent pattern:
//   - Contains: uvm_sequencer, domain driver, domain monitor
//   - is_active: UVM_ACTIVE  → builds sequencer + driver + monitor
//                UVM_PASSIVE → builds monitor ONLY (no driving)
//   - Exposes analysis ports from the monitor for env-level connectivity
//
// Instantiation in fifo_env:
//   fifo_wr_agent #(.DATA_WIDTH(8)) wr_agent;
//   fifo_rd_agent #(.DATA_WIDTH(8)) rd_agent;
//
// config_db keys consumed by this agent and its sub-components:
//   "WDRV_VIF" → vif for fifo_wr_driver  (write agent)
//   "WMON_VIF" → vif for fifo_wr_monitor (write agent)
//   "RDRV_VIF" → vif for fifo_rd_driver  (read  agent)
//   "RMON_VIF" → vif for fifo_rd_monitor (read  agent)
//
// Coding Standard : IEEE 1800-2012 | UVM 1.2
// Author          : ASIC Verification Team
// Date            : 2026-10-04
// =============================================================================

`ifndef FIFO_AGENT_SV
`define FIFO_AGENT_SV

`include "uvm_macros.svh"
import uvm_pkg::*;

// Include sub-component files (ordering matters for compilation)
`include "fifo_seq_item.sv"
`include "fifo_wr_driver.sv"
`include "fifo_wr_monitor.sv"
`include "fifo_rd_driver.sv"
`include "fifo_rd_monitor.sv"

// =============================================================================
// CLASS: fifo_wr_agent
// Write-domain UVM Agent. Drives and monitors the wclk-domain signals.
// =============================================================================
class fifo_wr_agent #(
    parameter int unsigned DATA_WIDTH = 8
) extends uvm_agent;

    `uvm_component_param_utils(fifo_wr_agent #(DATA_WIDTH))

    // =========================================================================
    // Sub-component handles
    // =========================================================================
    // Sequencer: arbitrates between multiple write sequences
    uvm_sequencer #(fifo_seq_item #(DATA_WIDTH)) sequencer;

    // Driver: drives write-domain signals to the DUT (active mode only)
    fifo_wr_driver  #(.DATA_WIDTH(DATA_WIDTH)) driver;

    // Monitor: observes write-domain signals (always instantiated)
    fifo_wr_monitor #(.DATA_WIDTH(DATA_WIDTH)) monitor;

    // =========================================================================
    // Forwarded Analysis Port
    // Wired directly to monitor.ap so the env can connect to the agent
    // without knowing the internal monitor hierarchy.
    // =========================================================================
    uvm_analysis_port #(fifo_seq_item #(DATA_WIDTH)) ap;

    // =========================================================================
    // Constructor
    // =========================================================================
    function new(string name = "fifo_wr_agent", uvm_component parent = null);
        super.new(name, parent);
    endfunction : new

    // =========================================================================
    // build_phase
    // Creates sub-components based on is_active setting.
    //   UVM_ACTIVE  : sequencer + driver + monitor (full active agent)
    //   UVM_PASSIVE : monitor only (observation-only, no stimulus)
    // =========================================================================
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);

        // Monitor is ALWAYS created (passive or active mode)
        monitor = fifo_wr_monitor #(.DATA_WIDTH(DATA_WIDTH))::type_id::create(
                      "monitor", this);

        if (get_is_active() == UVM_ACTIVE) begin
            // Create sequencer and driver only in active mode
            sequencer = uvm_sequencer #(fifo_seq_item #(DATA_WIDTH))::type_id::create(
                            "sequencer", this);
            driver = fifo_wr_driver #(.DATA_WIDTH(DATA_WIDTH))::type_id::create(
                         "driver", this);

            `uvm_info("FIFO_WR_AGENT", "Built in ACTIVE mode (sequencer + driver + monitor)", UVM_MEDIUM)
        end else begin
            `uvm_info("FIFO_WR_AGENT", "Built in PASSIVE mode (monitor only)", UVM_MEDIUM)
        end

        // Create the forwarded analysis port
        ap = new("ap", this);
    endfunction : build_phase

    // =========================================================================
    // connect_phase
    // 1. Connect driver's seq_item_port to sequencer's seq_item_export.
    // 2. Forward the monitor's analysis port to the agent's ap.
    // =========================================================================
    function void connect_phase(uvm_phase phase);
        // Forward monitor's analysis port to agent's ap
        // The env connects to wr_agent.ap → scoreboard/coverage
        monitor.ap.connect(ap);

        if (get_is_active() == UVM_ACTIVE) begin
            // Standard TLM-1 driver ↔ sequencer connection
            driver.seq_item_port.connect(sequencer.seq_item_export);
        end
    endfunction : connect_phase

    // =========================================================================
    // start_of_simulation_phase: Print agent configuration summary
    // =========================================================================
    function void start_of_simulation_phase(uvm_phase phase);
        `uvm_info("FIFO_WR_AGENT",
            $sformatf("Write Agent Configuration:\n  DATA_WIDTH = %0d\n  is_active  = %s",
                DATA_WIDTH, get_is_active().name()), UVM_MEDIUM)
    endfunction : start_of_simulation_phase

endclass : fifo_wr_agent

// =============================================================================
// CLASS: fifo_rd_agent
// Read-domain UVM Agent. Drives and monitors the rclk-domain signals.
// =============================================================================
class fifo_rd_agent #(
    parameter int unsigned DATA_WIDTH = 8
) extends uvm_agent;

    `uvm_component_param_utils(fifo_rd_agent #(DATA_WIDTH))

    // =========================================================================
    // Sub-component handles
    // =========================================================================
    // Sequencer: arbitrates between multiple read sequences
    uvm_sequencer #(fifo_seq_item #(DATA_WIDTH)) sequencer;

    // Driver: drives read-domain signals (active mode only)
    fifo_rd_driver  #(.DATA_WIDTH(DATA_WIDTH)) driver;

    // Monitor: observes read-domain signals (always instantiated)
    fifo_rd_monitor #(.DATA_WIDTH(DATA_WIDTH)) monitor;

    // =========================================================================
    // Forwarded Analysis Ports
    // ap_read → scoreboard (valid reads for data comparison)
    // ap_flag → coverage   (flag transition sampling)
    // =========================================================================
    uvm_analysis_port #(fifo_seq_item #(DATA_WIDTH)) ap_read;
    uvm_analysis_port #(fifo_seq_item #(DATA_WIDTH)) ap_flag;

    // =========================================================================
    // Constructor
    // =========================================================================
    function new(string name = "fifo_rd_agent", uvm_component parent = null);
        super.new(name, parent);
    endfunction : new

    // =========================================================================
    // build_phase
    // =========================================================================
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);

        // Monitor always created
        monitor = fifo_rd_monitor #(.DATA_WIDTH(DATA_WIDTH))::type_id::create(
                      "monitor", this);

        if (get_is_active() == UVM_ACTIVE) begin
            sequencer = uvm_sequencer #(fifo_seq_item #(DATA_WIDTH))::type_id::create(
                            "sequencer", this);
            driver = fifo_rd_driver #(.DATA_WIDTH(DATA_WIDTH))::type_id::create(
                         "driver", this);

            `uvm_info("FIFO_RD_AGENT", "Built in ACTIVE mode (sequencer + driver + monitor)", UVM_MEDIUM)
        end else begin
            `uvm_info("FIFO_RD_AGENT", "Built in PASSIVE mode (monitor only)", UVM_MEDIUM)
        end

        // Create forwarded analysis ports
        ap_read = new("ap_read", this);
        ap_flag = new("ap_flag", this);
    endfunction : build_phase

    // =========================================================================
    // connect_phase
    // =========================================================================
    function void connect_phase(uvm_phase phase);
        // Forward monitor analysis ports to agent-level ports
        monitor.ap_read.connect(ap_read);
        monitor.ap_flag.connect(ap_flag);

        if (get_is_active() == UVM_ACTIVE) begin
            driver.seq_item_port.connect(sequencer.seq_item_export);
        end
    endfunction : connect_phase

    // =========================================================================
    // start_of_simulation_phase
    // =========================================================================
    function void start_of_simulation_phase(uvm_phase phase);
        `uvm_info("FIFO_RD_AGENT",
            $sformatf("Read Agent Configuration:\n  DATA_WIDTH = %0d\n  is_active  = %s",
                DATA_WIDTH, get_is_active().name()), UVM_MEDIUM)
    endfunction : start_of_simulation_phase

endclass : fifo_rd_agent

`endif // FIFO_AGENT_SV
// =============================================================================
// End of fifo_agent.sv
// =============================================================================
