# Verilated -*- Makefile -*-
# DESCRIPTION: Verilator output: Make include file with class lists
#
# This file lists generated Verilated files, for including in higher level makefiles.
# See Vdesign_2_top.mk for the caller.

### Switches...
# C11 constructs required?  0/1 (always on now)
VM_C11 = 1
# Timing enabled?  0/1
VM_TIMING = 0
# Coverage output mode?  0/1 (from --coverage)
VM_COVERAGE = 0
# Parallel builds?  0/1 (from --output-split)
VM_PARALLEL_BUILDS = 0
# Tracing output mode?  0/1 (from --trace/--trace-fst)
VM_TRACE = 1
# Tracing output mode in VCD format?  0/1 (from --trace)
VM_TRACE_VCD = 1
# Tracing output mode in FST format?  0/1 (from --trace-fst)
VM_TRACE_FST = 0

### Object file lists...
# Generated module classes, fast-path, compile with highest optimization
VM_CLASSES_FAST += \
	Vdesign_2_top \
	Vdesign_2_top___024root__DepSet_h58151e31__0 \
	Vdesign_2_top___024root__DepSet_h228f4159__0 \
	Vdesign_2_top_design_2_top__DepSet_h8c8c3a73__0 \
	Vdesign_2_top_forte_soc_top__DepSet_h325e14c4__0 \
	Vdesign_2_top_forte_soc_top__DepSet_h0c5646ac__0 \
	Vdesign_2_top_ram__DepSet_h3fc8abf3__0 \
	Vdesign_2_top_dp_ram__DepSet_hba5257fc__0 \
	Vdesign_2_top_dp_ram__DepSet_h844a87a4__0 \

# Generated module classes, non-fast-path, compile with low/medium optimization
VM_CLASSES_SLOW += \
	Vdesign_2_top__ConstPool_0 \
	Vdesign_2_top___024root__Slow \
	Vdesign_2_top___024root__DepSet_h58151e31__0__Slow \
	Vdesign_2_top___024root__DepSet_h228f4159__0__Slow \
	Vdesign_2_top_design_2_top__Slow \
	Vdesign_2_top_design_2_top__DepSet_h76081d1b__0__Slow \
	Vdesign_2_top_forte_soc_top__Slow \
	Vdesign_2_top_forte_soc_top__DepSet_h325e14c4__0__Slow \
	Vdesign_2_top_forte_soc_top__DepSet_h0c5646ac__0__Slow \
	Vdesign_2_top___024unit__Slow \
	Vdesign_2_top___024unit__DepSet_h38b06db2__0__Slow \
	Vdesign_2_top_ram__Slow \
	Vdesign_2_top_ram__DepSet_h06c3ab9b__0__Slow \
	Vdesign_2_top_dp_ram__Slow \
	Vdesign_2_top_dp_ram__DepSet_h844a87a4__0__Slow \

# Generated support classes, fast-path, compile with highest optimization
VM_SUPPORT_FAST += \
	Vdesign_2_top__Dpi \
	Vdesign_2_top__Trace__0 \

# Generated support classes, non-fast-path, compile with low/medium optimization
VM_SUPPORT_SLOW += \
	Vdesign_2_top__Syms \
	Vdesign_2_top__Trace__0__Slow \
	Vdesign_2_top__TraceDecls__0__Slow \

# Global classes, need linked once per executable, fast-path, compile with highest optimization
VM_GLOBAL_FAST += \
	verilated \
	verilated_dpi \
	verilated_vcd_c \
	verilated_threads \

# Global classes, need linked once per executable, non-fast-path, compile with low/medium optimization
VM_GLOBAL_SLOW += \


# Verilated -*- Makefile -*-
