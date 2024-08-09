// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See Vdesign_2_top.h for the primary calling header

#include "Vdesign_2_top__pch.h"
#include "Vdesign_2_top__Syms.h"
#include "Vdesign_2_top_ram.h"

void Vdesign_2_top_ram___ctor_var_reset(Vdesign_2_top_ram* vlSelf);

Vdesign_2_top_ram::Vdesign_2_top_ram(Vdesign_2_top__Syms* symsp, const char* v__name)
    : VerilatedModule{v__name}
    , vlSymsp{symsp}
 {
    // Reset structure values
    Vdesign_2_top_ram___ctor_var_reset(this);
}

void Vdesign_2_top_ram::__Vconfigure(bool first) {
    (void)first;  // Prevent unused variable warning
}

Vdesign_2_top_ram::~Vdesign_2_top_ram() {
}
