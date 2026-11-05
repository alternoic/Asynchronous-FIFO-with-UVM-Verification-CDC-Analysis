# Asynchronous FIFO with UVM Verification & CDC Analysis

<div align="center">

![SystemVerilog](https://img.shields.io/badge/Language-SystemVerilog_IEEE_1800--2012-blue?style=for-the-badge&logo=v)
![UVM](https://img.shields.io/badge/Methodology-UVM_1.2-green?style=for-the-badge)
![SVA](https://img.shields.io/badge/Formal-SVA_Assertions-orange?style=for-the-badge)
![CDC](https://img.shields.io/badge/CDC-Gray_Code_Synchronization-red?style=for-the-badge)
![EDA Playground](https://img.shields.io/badge/EDA_Playground-Available-purple?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)

*A production-grade, fully parameterized Asynchronous FIFO design verified with a complete UVM 1.2 testbench, formal SVA properties, and CDC-specific coverage — built from first principles.*

**[▶ Run on EDA Playground](#)** ← *(paste your EDA Playground link here)*

</div>

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Architecture](#2-architecture)
3. [RTL Design — CDC Strategy](#3-rtl-design--cdc-strategy)
4. [UVM Testbench Architecture](#4-uvm-testbench-architecture)
5. [CDC Latency Handling in the Scoreboard](#5-cdc-latency-handling-in-the-scoreboard)
6. [SVA Assertion Catalogue](#6-sva-assertion-catalogue)
7. [Functional Coverage Model](#7-functional-coverage-model)
8. [Repository Structure](#8-repository-structure)
9. [EDA Playground Setup](#9-eda-playground-setup)
10. [Local Simulation (Questa / ModelSim)](#10-local-simulation-questa--modelsim)
11. [Key Design Decisions](#11-key-design-decisions)
12. [References](#12-references)

---

## 1. Project Overview

This project implements a **parameterized Asynchronous FIFO** (default: 8-bit width, 16-word depth) using the **Clifford Cummings methodology** (SNUG 2002) and verifies it with a **complete UVM 1.2 testbench** including:

- ✅ Dual-clock-domain RTL with Gray-coded pointer synchronization
- ✅ 13 SystemVerilog Assertions (SVAs) bound to the DUT via `bind`
- ✅ UVM environment: 2 independent agents, scoreboard, coverage, virtual sequencer
- ✅ CDC-safe scoreboard using a queue-based reference model (no cycle-by-cycle comparison)
- ✅ 6 functional covergroups including cross-coverage for full/empty boundary conditions
- ✅ Simulation-ready on EDA Playground (Aldec Riviera-PRO + UVM 1.2)

| Parameter | Default | Range |
|---|---|---|
| `DATA_WIDTH` | 8 bits | 1–64 bits |
| `FIFO_DEPTH` | 16 words | 4–256 (must be power of 2) |
| `ADDR_WIDTH` | 4 bits | `$clog2(FIFO_DEPTH)` |
| `PTR_WIDTH` | 5 bits | `ADDR_WIDTH + 1` (extra MSB for full/empty distinguish) |

---

## 2. Architecture

### 2.1 System Block Diagram

```
                            ASYNC FIFO TOP — async_fifo.sv
  ┌─────────────────────────────────────────────────────────────────────────┐
  │                                                                         │
  │   WRITE CLOCK DOMAIN (wclk)        READ CLOCK DOMAIN (rclk)             │
  │   ════════════════════════         ════════════════════════             │
  │                                                                         │
  │   winc, wdata                              rinc                         │
  │       │                                      │                          │
  │   ┌───▼──────────┐   Binary     ┌────────────▼──────────┐               │
  │   │  wptr_bin    │  ──Gray──►  │    sync_2ff           │                │
  │   │  (wclk reg)  │   encode    │  (2-flop, rclk)       │                │
  │   └───┬──────────┘             │  wptr_gray → rclk     │                │
  │       │ waddr                  └────────────┬──────────┘                │
  │       │                                     │ wptr_gray_sync            │
  │   ┌───▼──────────────────────────────────────▼──────────┐               │
  │   │              dp_ram (Dual-Port SRAM)                 │              │
  │   │   Write port (wclk)        Read port (rclk)         │               │
  │   └───────────────────────────────────────┬────────────┘                │
  │                                           │ rdata, raddr                │
  │   ┌──────────────┐   Gray     ┌───────────▼──────────┐                  │
  │   │  sync_2ff    │◄──encode── │  rptr_bin            │                  │
  │   │  (2-flop,    │            │  (rclk reg)          │                  │
  │   │   wclk)      │            └───────────────────────┘                 │
  │   └──────┬───────┘                                                      │
  │          │ rptr_gray_sync                                               │
  │          ▼                                                              │
  │   ┌──────────────┐                        ┌─────────────────┐           │
  │   │  Full Logic  │──────► wfull           │  Empty Logic    │──► rempty │
  │   │  (wclk dom)  │                        │  (rclk domain)  │           │
  │   └──────────────┘                        └─────────────────┘           │
  │                                                                         │
  └─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 UVM Testbench Hierarchy

```
tb_top
└── fifo_env
    ├── fifo_wr_agent  [UVM_ACTIVE]            (wclk domain)
    │   ├── uvm_sequencer #(fifo_seq_item)
    │   ├── fifo_wr_driver   ──drives──► fifo_if.wdrv_cb
    │   └── fifo_wr_monitor  ──samples─► fifo_if.wmon_cb
    │             │ ap (valid writes)
    │             ├─────────────────────────────────────────┐
    │             │                                         │
    ├── fifo_rd_agent  [UVM_ACTIVE]            (rclk domain)│
    │   ├── uvm_sequencer #(fifo_seq_item)                  │
    │   ├── fifo_rd_driver   ──drives──► fifo_if.rdrv_cb    │
    │   └── fifo_rd_monitor  ──samples─► fifo_if.rmon_cb    │
    │             │ ap_read (valid reads)                    │
    │             │ ap_flag (every rclk cycle)               │
    │             │                                          │
    ├── fifo_scoreboard ◄──── wr_export ◄─────────────────┘ │
    │   (queue-based)  ◄──── rd_export ◄──ap_read           │
    │                                                        │
    ├── fifo_coverage   ◄──── wr_export ◄──────────────────┘
    │   (6 covergroups) ◄──── rd_export ◄──ap_flag
    │
    └── fifo_virtual_sequencer
            ├── wr_seqr → wr_agent.sequencer
            └── rd_seqr → rd_agent.sequencer
```

---

## 3. RTL Design — CDC Strategy

### 3.1 The Fundamental CDC Problem

An Asynchronous FIFO has two clocks with no phase relationship. Naively passing a binary pointer across clock domains causes **multi-bit metastability** — if the pointer transitions from `0111 → 1000`, all 4 bits change simultaneously. The receiving flip-flop may sample any intermediate value (`0000`, `1111`, `1001`...), producing a catastrophically wrong pointer.

### 3.2 Gray-Code Pointer Synchronization

The solution is to convert binary pointers to **Gray code** before crossing the clock domain boundary. Gray code guarantees that **only 1 bit changes per increment**, regardless of the counter value.

```
Binary → Gray (XOR-reduce):  gray[i] = bin[i] ^ bin[i+1]
Gray → Binary (iterative):   bin[MSB] = gray[MSB]; bin[i] = bin[i+1] ^ gray[i]

Example (4-bit counter):
  Binary: 0111 → 1000    (4 bits change — UNSAFE for direct CDC)
  Gray:   0100 → 1100    (1 bit changes  — SAFE for 2-flop synchronizer)
```

### 3.3 Full and Empty Flag Generation

Full and empty flags are generated **locally in their respective clock domains** using the synchronized Gray pointer — no binary conversion is needed at comparison time.

**Empty Flag** (generated in `rclk` domain):
```systemverilog
// FIFO is empty when read pointer equals synchronized write pointer
assign rempty_next = (rptr_gray_next == wptr_gray_sync);
```

**Full Flag** (generated in `wclk` domain, requires modified comparison):
```
Full when: MSB and MSB-1 differ, remaining bits equal
// Formal: wptr_gray == {~rptr_gray_sync[PTR_WIDTH-1:PTR_WIDTH-2],
//                        rptr_gray_sync[PTR_WIDTH-3:0]}
```
This 2-MSB inversion trick (Cummings 2002) directly compares Gray pointers without binary decoding, ensuring the full flag is glitch-free and generated entirely in the `wclk` domain.

### 3.4 RTL Files

| File | Description | Key Logic |
|---|---|---|
| [`rtl/dp_ram.sv`](rtl/dp_ram.sv) | Parameterized dual-port SRAM | Separate read/write clocks and enables |
| [`rtl/sync_2ff.sv`](rtl/sync_2ff.sv) | 2-stage CDC synchronizer | `ASYNC_REG` attribute, active-low async reset |
| [`rtl/async_fifo.sv`](rtl/async_fifo.sv) | Top-level FIFO wrapper | Gray encode, 2-flip synchronizers, flag logic |

---

## 4. UVM Testbench Architecture

### 4.1 Component Roles

| Component | File | Role |
|---|---|---|
| `async_fifo_if` | `tb/fifo_if.sv` | Interface with 4 clocking blocks (`#1step` input, `#1` output) |
| `fifo_seq_item` | `tb/fifo_seq_item.sv` | Transaction object with 11 named constraint groups |
| `fifo_wr_driver` | `tb/fifo_wr_driver.sv` | Drives `winc`, `wdata`, `wrst_n` via `wdrv_cb`; backpressure on `wfull` |
| `fifo_wr_monitor` | `tb/fifo_wr_monitor.sv` | Passively samples `wmon_cb`; broadcasts valid writes via `ap` |
| `fifo_rd_driver` | `tb/fifo_rd_driver.sv` | Drives `rinc`, `rrst_n` via `rdrv_cb`; captures `rdata` into item |
| `fifo_rd_monitor` | `tb/fifo_rd_monitor.sv` | Passively samples `rmon_cb`; broadcasts via `ap_read` and `ap_flag` |
| `fifo_wr_agent` | `tb/fifo_agent.sv` | Write-domain agent; `is_active`-controlled sequencer + driver |
| `fifo_rd_agent` | `tb/fifo_agent.sv` | Read-domain agent; `is_active`-controlled sequencer + driver |
| `fifo_scoreboard` | `tb/fifo_scoreboard.sv` | Queue-based CDC-safe self-checker |
| `fifo_coverage` | `tb/fifo_coverage.sv` | 6 functional covergroups with cross-coverage |
| `fifo_env` | `tb/fifo_env.sv` | Environment; wires TLM analysis ports; creates virtual sequencer |
| `fifo_base_seq` | `tb/fifo_base_seq.sv` | Virtual base sequence with utility tasks (`do_write`, `do_read`, etc.) |
| `fifo_sanity_seq` | `tb/fifo_sanity_seq.sv` | 5-phase smoke test with known deterministic data |
| `fifo_stress_seq` | `tb/fifo_stress_seq.sv` | 6-phase fill/overflow/drain/underflow/wrap/concurrent test |
| `fifo_base_test` | `tb/fifo_base_test.sv` | Base test + `fifo_sanity_test` + `fifo_stress_test` |
| `tb_top` | `tb/tb_top.sv` | Simulation top; clocks, resets, DUT, VIF, `run_test()` |

### 4.2 Sequence Architecture

```
fifo_virtual_sequencer
      │
      ├── wr_seqr  ──────────────────────────────────────┐
      │                                                   │
      └── rd_seqr  ──────────────────────────────────────┤
                                                          │
fifo_stress_seq::body()                                   │
  Phase 0: do_reset()          ──fork──► wr_seqr: wrst_req item
                                         rd_seqr: rrst_req item
  Phase 1: do_write_burst(16)  ─────────► wr_seqr: 16× write items
  Phase 3: fork { write idle | do_read_burst(16) }
           ──────────────────────────────► wr_seqr: NOP items
                                          rd_seqr: 16× read items
```

---

## 5. CDC Latency Handling in the Scoreboard

This is the most critical design decision in the testbench.

### 5.1 Why Cycle-Accurate Comparison Fails

In a synchronous FIFO, a write on cycle N guarantees the data is readable on cycle N+1. This allows a cycle-by-cycle scoreboard.

In an **asynchronous FIFO**, the write pointer must propagate through **two synchronizer flip-flops** in the read clock domain before `rempty` deasserts. This takes a **minimum of 2 rclk cycles** — but the exact latency depends on the clock frequency ratio and phase alignment, which change every run with a random seed.

A cycle-accurate scoreboard would produce **false failures** during the synchronizer latency window.

### 5.2 The Queue-Based Solution

```
Write Monitor broadcasts:  data_in = 0xA5
                                │
                                ▼
                     ref_queue.push_back(0xA5)
                                │
                     [2–N rclk cycles pass — CDC latency]
                                │
Read Monitor broadcasts:   data_out = 0xA5
                                │
                                ▼
                     expected = ref_queue.pop_front()  → 0xA5
                     assert(data_out === expected)      → PASS ✅
```

The scoreboard only verifies two properties:
1. **Ordering**: Data is read back in the same order it was written (FIFO semantics).
2. **Integrity**: The exact bit pattern is preserved (no bit flips or X propagation).

It makes **no timing assumptions** — the pop can happen 2 cycles or 200 cycles after the push, and the result is equally valid.

---

## 6. SVA Assertion Catalogue

The SVA module [`tb/fifo_sva.sv`](tb/fifo_sva.sv) is bound to `async_fifo` via:
```systemverilog
bind async_fifo fifo_sva #(.DATA_WIDTH(DATA_WIDTH), .FIFO_DEPTH(FIFO_DEPTH)) u_fifo_sva (.*);
```

The `.*` auto-connects all ports — including internal signals like `wptr_gray`, `rptr_bin`, `wptr_gray_sync` — without requiring any modification to the DUT.

| ID | Clock | Property | Failure Severity |
|---|---|---|---|
| `AST_W01` | `wclk` | No `winc` while `wfull=1` | `$error` |
| `AST_W02` | `wclk` | No X/Z on `wdata` during valid write | `$error` |
| `AST_W04` | `wclk` | `wfull` clears within 1 cycle of `wrst_n↑` | `$error` |
| `AST_W05` | `wclk` | `wptr_bin` increments by ≤1 per cycle | `$error` |
| `AST_R01` | `rclk` | No `rinc` while `rempty=1` | `$error` |
| `AST_R02` | `rclk` | No X/Z on `rdata` during valid read | `$error` |
| `AST_R03` | `rclk` | `rempty` asserts within 1 cycle of `rrst_n↑` | `$error` |
| `AST_R04` | `rclk` | `rptr_bin` increments by ≤1 per cycle | `$error` |
| **`AST_G01`** | `wclk` | **`wptr_gray` changes ≤1 bit/cycle** | `$error` |
| **`AST_G02`** | `rclk` | **`rptr_gray` changes ≤1 bit/cycle** | `$error` |
| `AST_G03` | `wclk` | No X/Z on `wptr_gray` post-reset | `$error` |
| `AST_G04` | `rclk` | No X/Z on `rptr_gray` post-reset | `$error` |
| **`AST_I01`** | `wclk` | **`wfull ∧ rempty` is physically impossible** | **`$fatal(2)`** |
| `AST_I02` | `wclk` | Occupancy `≤ FIFO_DEPTH` always | `$error` |

> **Assertions G01 and G02** are the most critical: if more than 1 bit of a Gray pointer changes per clock, the 2-flop synchronizer's fundamental metastability guarantee is violated, meaning any read could return corrupted data.

---

## 7. Functional Coverage Model

Six covergroups in [`tb/fifo_coverage.sv`](tb/fifo_coverage.sv):

| Covergroup | Trigger | Key Coverpoints |
|---|---|---|
| `fifo_write_cg` | Valid write (winc=1) | `wr_en`, `wfull`, `wfull_rose`, `cx_wr_en_x_wfull` |
| `fifo_read_cg` | Valid read (rinc=1) | `rd_en`, `rempty`, `rempty_fell`, `cx_rd_en_x_rempty` |
| `fifo_flag_cg` | Every `rclk` edge | All 4 `(wfull, rempty)` state combos; simultaneous R/W |
| `fifo_data_cg` | Valid write | All-zeros, all-ones, walking-1s/0s, checkerboard, random |
| `fifo_reset_cg` | Reset request | Write-only, read-only, simultaneous both-domain reset |
| `fifo_occupancy_cg` | Write + Read | Empty, light (1–4), half (5–8), heavy (9–15), full (16) |

**Critical cross-coverage bins:**

| Cross Bin | Why It Matters |
|---|---|
| `wr_when_full` | Verifies DUT asserts `wfull` before accepting writes |
| `rd_when_empty` | Verifies DUT asserts `rempty` before accepting reads |
| `impossible_state` (`wfull=1 ∧ rempty=1`) | Should **never** be sampled — escalates to `$uvm_error` |
| `write_to_almost_full` | Verifies last-slot full assertion timing (occupancy = `DEPTH-1 → DEPTH`) |
| `both_rst` | Verifies simultaneous dual-domain reset recovery |

---

## 8. Repository Structure

```
Asynchronous-FIFO-with-UVM-Verification-CDC-Analysis/
│
├── rtl/
│   ├── dp_ram.sv                  # Parameterized dual-port SRAM
│   ├── sync_2ff.sv                # 2-stage CDC synchronizer (ASYNC_REG)
│   └── async_fifo.sv              # Top-level async FIFO (Gray-code CDC)
│
├── tb/
│   ├── fifo_if.sv                 # Interface: 4 clocking blocks, 5 modports
│   ├── fifo_seq_item.sv           # UVM sequence item: 11 constraint groups
│   ├── fifo_wr_driver.sv          # Write-domain UVM driver
│   ├── fifo_wr_monitor.sv         # Write-domain UVM monitor (analysis port)
│   ├── fifo_rd_driver.sv          # Read-domain UVM driver (captures rdata)
│   ├── fifo_rd_monitor.sv         # Read-domain UVM monitor (dual ap ports)
│   ├── fifo_agent.sv              # fifo_wr_agent + fifo_rd_agent
│   ├── fifo_scoreboard.sv         # Queue-based CDC-safe self-checker
│   ├── fifo_coverage.sv           # 6 functional covergroups
│   ├── fifo_env.sv                # Environment + virtual sequencer
│   ├── fifo_base_seq.sv           # Base virtual sequence + utility tasks
│   ├── fifo_sanity_seq.sv         # Smoke test: 4 known writes → CDC → reads
│   ├── fifo_stress_seq.sv         # Stress: fill/overflow/drain/wrap/concurrent
│   ├── fifo_sva.sv                # 13 SVA assertions + bind statement
│   ├── fifo_base_test.sv          # UVM tests: base, sanity, stress
│   └── tb_top.sv                  # Simulation top: clocks, DUT, run_test()
│
└── sim/
    ├── design.sv                  # EDA Playground RTL entry-point
    └── testbench.sv               # EDA Playground TB entry-point
```

**Project statistics:**
- **Total files**: 21
- **RTL lines**: 758
- **TB lines**: 4,581
- **Total SV lines**: ~5,339

---

## 9. EDA Playground Setup

### Step-by-step Instructions

**1. Open EDA Playground**
Go to [https://www.edaplayground.com](https://www.edaplayground.com) and log in (free account required).

**2. Create a New Playground**
Click **"New"** to start a fresh project.

**3. Configure Simulator Settings** (left panel)

| Setting | Value |
|---|---|
| **Simulator** | `Aldec Riviera-PRO 2023.04` |
| **UVM / OVM** | ☑ `UVM 1.2` |
| **Compile Options** | `-sv2012 +define+UVM_NO_DEPRECATED` |
| **Run Options** | `+UVM_TESTNAME=fifo_stress_test +UVM_VERBOSITY=UVM_MEDIUM` |
| **Top Module** | `tb_top` |

> **Alternative simulator**: Cadence Xcelium 23.03 also supports UVM 1.2 and SystemVerilog 2012. Synopsys VCS requires `-debug_access+all` for `bind` to access internal signals.

**4. Upload Source Files**
Click **"Files"** → upload ALL files from both `rtl/` and `tb/` directories (19 `.sv` files total) into the same flat directory.

**5. Paste Entry-Point Files**

| Tab | File to paste |
|---|---|
| **Design** (left editor) | Contents of `sim/design.sv` |
| **Testbench** (right editor) | Contents of `sim/testbench.sv` |

**6. Run the Simulation**
Click **"Run"**. Expected output includes:
```
[FIFO_ENV]    Environment built: DATA_WIDTH=8 FIFO_DEPTH=16
[FIFO_SB]     [PUSH] data=0x10 | queue_depth=1 | write#=1
[FIFO_SB]     [MATCH ✓] Read#=1 | data=0x10 | queue_remaining=0
...
[FIFO_SB]     FIFO SCOREBOARD — FINAL REPORT
              Writes received        : 64
              Data matches (PASS)    : 64
              Data mismatches (FAIL) : 0
══════════════════════════════════════════
  ✅  TEST PASSED: fifo_stress_test
══════════════════════════════════════════
```

> **Note on SVA bind**: Riviera-PRO enables internal signal access by default. If assertions fail to bind, add `-access +r` to compile options.

**7. Switch Tests**
Change the run option to `+UVM_TESTNAME=fifo_sanity_test` for the smoke test.

---

## 10. Local Simulation (Questa / ModelSim)

### Run via script (from project directory):
`
vsim -do run.do
`

### Manual step-by-step in ModelSim GUI:
`	cl
cd c:/Users/Shivam/Desktop/async_fifo_uvm
if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work
vlog -suppress 2892 -work work -sv dp_ram.sv
vlog -suppress 2892 -work work -sv sync_2ff.sv
vlog -suppress 2892 -work work -sv async_fifo.sv
vlog -suppress 2892 -work work -sv fifo_if.sv
vlog -suppress 2892 -work work -sv fifo_sva.sv
vlog -suppress 2892 -work work -sv +incdir+. testbench.sv
vlog -suppress 2892 -work work -sv +incdir+. tb_top.sv
vsim -voptargs="+acc" -assertdebug work.tb_top +UVM_TESTNAME=fifo_stress_test
add wave -noupdate -r /tb_top/*
run -all
wave zoom full
`

---

---

## 12. References

| Source | Description |
|---|---|
| **Cummings, C. E.** (2002). *Simulation and Synthesis Techniques for Asynchronous FIFO Design*. SNUG San Jose 2002. | The foundational paper for this implementation. Gray-code pointer method, full/empty flag generation. |
| **IEEE Std 1800-2012** — *IEEE Standard for SystemVerilog* | Language reference for all RTL and TB code. |
| **IEEE Std 1800.2-2017** — *UVM Standard* | UVM methodology and class library reference. |
| **Bening, L. & Foster, H.** (2001). *Principles of Verifiable RTL Design*. Kluwer Academic. | SVA and formal verification methodology. |
| [EDA Playground](https://www.edaplayground.com) | Online simulation platform. |
| [Aldec Riviera-PRO Documentation](https://www.aldec.com/en/products/functional_verification/riviera-pro) | Simulator-specific compile and run options. |

---

<div align="center">

**Built as part of VLSI Design & Verification Portfolio**

*If you found this project useful, please ⭐ star the repository!*

[![GitHub](https://img.shields.io/badge/GitHub-Portfolio-black?style=for-the-badge&logo=github)](https://github.com)

</div>
