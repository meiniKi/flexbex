// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Symbol table internal header
//
// Internal details; most calling programs do not need this header,
// unless using verilator public meta comments.

#ifndef VERILATED_VDESIGN_2_TOP__SYMS_H_
#define VERILATED_VDESIGN_2_TOP__SYMS_H_  // guard

#include "verilated.h"

// INCLUDE MODEL CLASS

#include "Vdesign_2_top.h"

// INCLUDE MODULE CLASSES
#include "Vdesign_2_top___024root.h"
#include "Vdesign_2_top_design_2_top.h"
#include "Vdesign_2_top_forte_soc_top.h"
#include "Vdesign_2_top___024unit.h"
#include "Vdesign_2_top_ram.h"
#include "Vdesign_2_top_dp_ram.h"

// DPI TYPES for DPI Export callbacks (Internal use)

// SYMS CLASS (contains all model state)
class alignas(VL_CACHE_LINE_BYTES)Vdesign_2_top__Syms final : public VerilatedSyms {
  public:
    // INTERNAL STATE
    Vdesign_2_top* const __Vm_modelp;
    bool __Vm_activity = false;  ///< Used by trace routines to determine change occurred
    uint32_t __Vm_baseCode = 0;  ///< Used by trace routines when tracing multiple models
    VlDeleter __Vm_deleter;
    bool __Vm_didInit = false;

    // MODULE INSTANCE STATE
    Vdesign_2_top___024root        TOP;
    Vdesign_2_top_design_2_top     TOP__design_2_top;
    Vdesign_2_top_forte_soc_top    TOP__design_2_top__forte_soc_top_i;
    Vdesign_2_top_ram              TOP__design_2_top__forte_soc_top_i__ram_0;
    Vdesign_2_top_dp_ram           TOP__design_2_top__forte_soc_top_i__ram_0__dp_ram_i;

    // CONSTRUCTORS
    Vdesign_2_top__Syms(VerilatedContext* contextp, const char* namep, Vdesign_2_top* modelp);
    ~Vdesign_2_top__Syms();

    // METHODS
    const char* name() { return TOP.name(); }
};

#endif  // guard
