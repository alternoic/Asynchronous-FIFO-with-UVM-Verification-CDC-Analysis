// =============================================================================
// Module      : dp_ram
// Project     : Asynchronous FIFO with CDC Verification
// File        : dp_ram.sv
// Description : Parameterized Simple Dual-Port RAM (SDP-RAM).
//               - Write port : Synchronous, clocked on `wclk`.
//               - Read  port : Configurable via READ_MODE parameter.
//                              "ASYNC" → Combinational/asynchronous read output.
//                              "SYNC"  → Registered output, clocked on `rclk`.
//               - Intended use: Internal memory array for async_fifo_top.
//               - Active-low, asynchronous reset on read-side output register
//                 (only applies when READ_MODE = "SYNC").
//
// Parameters  :
//   DATA_WIDTH  - Width of each memory word          (default: 8)
//   ADDR_WIDTH  - Address bus width; depth = 2^ADDR_WIDTH (default: 4 → 16 words)
//   READ_MODE   - "ASYNC" or "SYNC" read port        (default: "ASYNC")
//
// Interface   :
//   Write Port : wclk, wrst_n (unused for ASYNC READ_MODE), wen, waddr, wdata
//   Read  Port : rclk, rrst_n (only used for SYNC READ_MODE), ren, raddr, rdata
//
// Notes       :
//   - No write-to-read forwarding (no bypass mux). The FIFO control logic
//     guarantees the read pointer never points to a location being written
//     simultaneously (empty flag prevents reads to unwritten locations).
//   - FIFO_DEPTH must be a power of 2 (enforced by the parent async_fifo_top).
//   - The memory array itself is NOT reset on power-up; only the registered
//     read output (SYNC mode) is cleared on rrst_n.
//   - For FPGA targets, this style infers Block RAM (BRAM) correctly.
//   - For ASIC targets, replace the logic array with a foundry SRAM macro
//     and preserve this wrapper interface.
//
// Coding Standard : IEEE 1800-2012 (SystemVerilog)
// Author          : ASIC Design Team
// Date            : 2026-10-04
// Version         : 1.0
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module dp_ram #(
    parameter  int unsigned DATA_WIDTH = 8,               // Word width in bits
    parameter  int unsigned ADDR_WIDTH = 4,               // Address width; depth = 2**ADDR_WIDTH
    parameter  string       READ_MODE  = "ASYNC"          // "ASYNC" or "SYNC" read port
) (
    // -------------------------------------------------------------------------
    // Write Port (Write Clock Domain)
    // -------------------------------------------------------------------------
    input  wire                  wclk,   // Write clock (rising-edge triggered)
    input  wire                  wrst_n, // Active-low async reset (write domain)
                                         // NOTE: Unused for pure ASYNC read mode;
                                         //       kept in port list for consistency.
    input  wire                  wen,    // Write enable (active high)
    input  wire [ADDR_WIDTH-1:0] waddr,  // Write address
    input  wire [DATA_WIDTH-1:0] wdata,  // Write data

    // -------------------------------------------------------------------------
    // Read Port (Read Clock Domain)
    // -------------------------------------------------------------------------
    input  wire                  rclk,   // Read clock (used only in SYNC mode)
    input  wire                  rrst_n, // Active-low async reset (read domain)
                                         // NOTE: Clears rdata register in SYNC mode.
    input  wire                  ren,    // Read enable  (active high)
                                         // ASYNC mode: acts as output enable (optional)
                                         // SYNC  mode: gates the read register update
    input  wire [ADDR_WIDTH-1:0] raddr,  // Read address
    output wire [DATA_WIDTH-1:0] rdata   // Read data output
);

    // =========================================================================
    // Local Parameters & Derived Constants
    // =========================================================================
    localparam int unsigned DEPTH = 2 ** ADDR_WIDTH; // Memory depth (number of words)

    // =========================================================================
    // Memory Array Declaration
    // =========================================================================
    // Synthesis tools infer SDP Block RAM from this pattern when:
    //   - Write is synchronous (posedge wclk)
    //   - Read  is synchronous (posedge rclk) OR asynchronous (combinational)
    //   - No read-during-write collision handling is required
    // =========================================================================
    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1]; // Memory array: DEPTH × DATA_WIDTH bits

    // =========================================================================
    // Write Port Logic
    // Synchronous write on rising edge of wclk.
    // Write occurs only when wen is asserted.
    // No reset of memory contents (undefined on power-up, as is standard).
    // =========================================================================
    always_ff @(posedge wclk) begin : write_port
        if (wen) begin
            mem[waddr] <= wdata;
        end
    end

    // =========================================================================
    // Read Port Logic
    // Controlled by READ_MODE parameter at elaboration time.
    // =========================================================================
    generate
        // ---------------------------------------------------------------------
        // ASYNC Read Mode (Combinational / Asynchronous Output)
        // ---------------------------------------------------------------------
        // - rdata is driven combinationally from the memory array.
        // - rclk and rrst_n are unused in this mode (tied off by parent).
        // - ren acts as an output enable: when deasserted, rdata outputs 'X
        //   (simulation) / high-impedance NOT guaranteed on ASIC; parent logic
        //   should ensure ren is always asserted during valid read windows).
        // - Preferred for async FIFO because the consumer registers the data
        //   in its own clock domain.
        // ---------------------------------------------------------------------
        if (READ_MODE == "ASYNC") begin : gen_async_read

            assign rdata = mem[raddr]; // Pure combinational read; ren unused
                                       // Parent logic (rptr_empty) ensures
                                       // raddr is valid before rinc is asserted.

            // Suppress "unused port" warnings for rclk, rrst_n, ren in ASYNC mode
            // (No logic referencing them; synthesis will optimize away.)

        // ---------------------------------------------------------------------
        // SYNC Read Mode (Registered Output, clocked on rclk)
        // ---------------------------------------------------------------------
        // - rdata is a registered output, updated on rising edge of rclk.
        // - ren gates the update (read enable must be asserted to latch new data).
        // - rrst_n asynchronously clears the output register to all zeros.
        // - Use this mode when the FIFO consumer requires registered data output
        //   (adds 1 rclk cycle of read latency).
        // ---------------------------------------------------------------------
        end else begin : gen_sync_read

            logic [DATA_WIDTH-1:0] rdata_reg;

            always_ff @(posedge rclk or negedge rrst_n) begin : read_port
                if (!rrst_n) begin
                    rdata_reg <= {DATA_WIDTH{1'b0}}; // Clear output on reset
                end else if (ren) begin
                    rdata_reg <= mem[raddr];          // Latch data on read enable
                end
            end

            assign rdata = rdata_reg;

        end
    endgenerate

    // =========================================================================
    // Parameter Validation (Elaboration-time checks)
    // =========================================================================
    // Enforce power-of-2 depth: required for Gray-code pointer wrap-around.
    // Enforce minimum and maximum dimensions.
    // =========================================================================
    initial begin : param_check
        if ((DEPTH & (DEPTH - 1)) !== 0) begin
            $fatal(1, "[dp_ram] FATAL: ADDR_WIDTH=%0d results in DEPTH=%0d, which is NOT a power of 2. \
Gray-code FIFO requires power-of-2 depth.", ADDR_WIDTH, DEPTH);
        end
        if (DATA_WIDTH < 1 || DATA_WIDTH > 256) begin
            $fatal(1, "[dp_ram] FATAL: DATA_WIDTH=%0d is out of valid range [1,256].", DATA_WIDTH);
        end
        if (ADDR_WIDTH < 1 || ADDR_WIDTH > 12) begin
            $fatal(1, "[dp_ram] FATAL: ADDR_WIDTH=%0d is out of valid range [1,12].", ADDR_WIDTH);
        end
        if (READ_MODE != "ASYNC" && READ_MODE != "SYNC") begin
            $fatal(1, "[dp_ram] FATAL: READ_MODE must be 'ASYNC' or 'SYNC'. Got: %s", READ_MODE);
        end
    end

endmodule : dp_ram

`default_nettype wire
// =============================================================================
// End of dp_ram.sv
// =============================================================================
