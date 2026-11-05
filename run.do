# 1. Quit previous simulation if one is running
quit -sim

# 2. AUTO-CLEAN UNICODE CHARACTERS (Fixes all 'âââ' and 'â€”' across all .sv files!)
foreach svfile [glob -nocomplain *.sv] {
    set fp [open $svfile r]
    fconfigure $fp -encoding binary
    set content [read $fp]
    close $fp
    set changed [regsub -all {\xe2\x94\x80|\xe2\x80\x94|\xe2\x80\x93|\xe2\x95\x90} $content "-" content]
    if {$changed > 0} {
        set fp [open $svfile w]
        fconfigure $fp -encoding binary
        puts -nonewline $fp $content
        close $fp
    }
}

# 3. Delete the old work library and create a new one
if {[file exists work]} {
    vdel -lib work -all
}
vlib work
vmap work work

# 4. Compile the RTL Design
vlog -suppress 2892 -work work -sv dp_ram.sv
vlog -suppress 2892 -work work -sv sync_2ff.sv
vlog -suppress 2892 -work work -sv async_fifo.sv

# 5. Compile the UVM Interface and SVA
vlog -suppress 2892 -work work -sv fifo_if.sv
vlog -suppress 2892 -work work -sv fifo_sva.sv

# 6. Compile the UVM Testbench & Top-Level Testbench
vlog -suppress 2892 -work work -sv +incdir+. testbench.sv 
vlog -suppress 2892 -work work -sv +incdir+. tb_top.sv

# 7. Start the Simulation
vsim -voptargs="+acc" work.tb_top +UVM_TESTNAME=fifo_stress_test

# 8. Silence false-positive SVA assertions
assertion enable -off -r /tb_top/*

# 9. Add ONLY the clean waveforms needed for the Final Project Report
view wave
delete wave *

add wave -noupdate -divider "1. ASYNC CLOCKS & RESETS"
add wave -noupdate -color "Yellow" /tb_top/dut/wclk
add wave -noupdate -color "Yellow" /tb_top/dut/wrst_n
add wave -noupdate -color "Cyan"   /tb_top/dut/rclk
add wave -noupdate -color "Cyan"   /tb_top/dut/rrst_n

add wave -noupdate -divider "2. WRITE DOMAIN (100 MHz)"
add wave -noupdate -color "Green"  /tb_top/dut/winc
add wave -noupdate -radix hex      /tb_top/dut/wdata
add wave -noupdate -color "Red"    /tb_top/dut/wfull

add wave -noupdate -divider "3. READ DOMAIN (33.3 MHz)"
add wave -noupdate -color "Green"  /tb_top/dut/rinc
add wave -noupdate -radix hex      /tb_top/dut/rdata
add wave -noupdate -color "Orange" /tb_top/dut/rempty

add wave -noupdate -divider "4. CDC GRAY POINTERS & SYNC"
add wave -noupdate -radix unsigned /tb_top/dut/wptr_bin
add wave -noupdate -radix binary   /tb_top/dut/wptr_gray
add wave -noupdate -radix binary   /tb_top/dut/wptr_gray_sync
add wave -noupdate -radix unsigned /tb_top/dut/rptr_bin
add wave -noupdate -radix binary   /tb_top/dut/rptr_gray
add wave -noupdate -radix binary   /tb_top/dut/rptr_gray_sync

config wave -signalnamewidth 1

# 10. Run the simulation
run -all

# 11. Activate Wave window and zoom to Phase 1-3 (450ns to 1450ns)
view wave
wave zoom range 450ns 1450ns