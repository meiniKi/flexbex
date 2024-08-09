// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design internal header
// See Vdesign_2_top.h for the primary calling header

#ifndef VERILATED_VDESIGN_2_TOP___024UNIT_H_
#define VERILATED_VDESIGN_2_TOP___024UNIT_H_  // guard

#include "verilated.h"


class Vdesign_2_top__Syms;

class alignas(VL_CACHE_LINE_BYTES) Vdesign_2_top___024unit final : public VerilatedModule {
  public:

    // INTERNAL VARIABLES
    Vdesign_2_top__Syms* const vlSymsp;

    // CONSTRUCTORS
    Vdesign_2_top___024unit(Vdesign_2_top__Syms* symsp, const char* v__name);
    ~Vdesign_2_top___024unit();
    VL_UNCOPYABLE(Vdesign_2_top___024unit);

    // INTERNAL METHODS
    void __Vconfigure(bool first);
};


#endif  // guard
