// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Symbol table implementation internals

#include "Vdesign_2_top__pch.h"
#include "Vdesign_2_top.h"
#include "Vdesign_2_top___024root.h"
#include "Vdesign_2_top_design_2_top.h"
#include "Vdesign_2_top_forte_soc_top.h"
#include "Vdesign_2_top___024unit.h"
#include "Vdesign_2_top_ram.h"
#include "Vdesign_2_top_dp_ram.h"

// FUNCTIONS
Vdesign_2_top__Syms::~Vdesign_2_top__Syms()
{
}

Vdesign_2_top__Syms::Vdesign_2_top__Syms(VerilatedContext* contextp, const char* namep, Vdesign_2_top* modelp)
    : VerilatedSyms{contextp}
    // Setup internal state of the Syms class
    , __Vm_modelp{modelp}
    // Setup module instances
    , TOP{this, namep}
    , TOP__design_2_top{this, Verilated::catName(namep, "design_2_top")}
    , TOP__design_2_top__forte_soc_top_i{this, Verilated::catName(namep, "design_2_top.forte_soc_top_i")}
    , TOP__design_2_top__forte_soc_top_i__ram_0{this, Verilated::catName(namep, "design_2_top.forte_soc_top_i.ram_0")}
    , TOP__design_2_top__forte_soc_top_i__ram_0__dp_ram_i{this, Verilated::catName(namep, "design_2_top.forte_soc_top_i.ram_0.dp_ram_i")}
{
        // Check resources
        Verilated::stackCheck(1163);
    // Configure time unit / time precision
    _vm_contextp__->timeunit(-12);
    _vm_contextp__->timeprecision(-12);
    // Setup each module's pointers to their submodules
    TOP.design_2_top = &TOP__design_2_top;
    TOP__design_2_top.forte_soc_top_i = &TOP__design_2_top__forte_soc_top_i;
    TOP__design_2_top__forte_soc_top_i.ram_0 = &TOP__design_2_top__forte_soc_top_i__ram_0;
    TOP__design_2_top__forte_soc_top_i__ram_0.dp_ram_i = &TOP__design_2_top__forte_soc_top_i__ram_0__dp_ram_i;
    // Setup each module's pointer back to symbol table (for public functions)
    TOP.__Vconfigure(true);
    TOP__design_2_top.__Vconfigure(true);
    TOP__design_2_top__forte_soc_top_i.__Vconfigure(true);
    TOP__design_2_top__forte_soc_top_i__ram_0.__Vconfigure(true);
    TOP__design_2_top__forte_soc_top_i__ram_0__dp_ram_i.__Vconfigure(true);
    // Setup export functions
    for (int __Vfinal = 0; __Vfinal < 2; ++__Vfinal) {
    }
}
