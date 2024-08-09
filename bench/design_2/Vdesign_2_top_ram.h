// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design internal header
// See Vdesign_2_top.h for the primary calling header

#ifndef VERILATED_VDESIGN_2_TOP_RAM_H_
#define VERILATED_VDESIGN_2_TOP_RAM_H_  // guard

#include "verilated.h"
class Vdesign_2_top_dp_ram;


class Vdesign_2_top__Syms;

class alignas(VL_CACHE_LINE_BYTES) Vdesign_2_top_ram final : public VerilatedModule {
  public:
    // CELLS
    Vdesign_2_top_dp_ram* dp_ram_i;

    // DESIGN SPECIFIC STATE
    VL_IN8(__PVT__clk,0,0);
    VL_IN8(__PVT__instr_req_i,0,0);
    VL_OUT8(__PVT__instr_rvalid_o,0,0);
    VL_OUT8(__PVT__instr_gnt_o,0,0);
    VL_IN8(__PVT__ibex_data_req_i,0,0);
    VL_IN8(__PVT__ibex_data_we_i,0,0);
    VL_IN8(__PVT__ibex_data_be_i,3,0);
    VL_OUT8(__PVT__ibex_data_rvalid_o,0,0);
    VL_OUT8(__PVT__ibex_data_gnt_o,0,0);
    VL_IN8(__PVT__uart_data_req_i,0,0);
    VL_IN8(__PVT__uart_data_we_i,0,0);
    VL_IN8(__PVT__uart_data_be_i,3,0);
    VL_OUT8(__PVT__uart_data_rvalid_o,0,0);
    VL_OUT8(__PVT__uart_data_gnt_o,0,0);
    CData/*0:0*/ __PVT__data_req_i;
    VL_IN16(__PVT__instr_addr_i,11,0);
    VL_IN16(__PVT__ibex_data_addr_i,11,0);
    VL_IN16(__PVT__uart_data_addr_i,11,0);
    SData/*11:0*/ __PVT__data_addr_i;
    VL_OUT(__PVT__instr_rdata_o,31,0);
    VL_IN(__PVT__ibex_data_wdata_i,31,0);
    VL_OUT(__PVT__ibex_data_rdata_o,31,0);
    VL_IN(__PVT__uart_data_wdata_i,31,0);
    VL_OUT(__PVT__uart_data_rdata_o,31,0);
    IData/*31:0*/ __PVT__data_wdata_i;

    // INTERNAL VARIABLES
    Vdesign_2_top__Syms* const vlSymsp;

    // CONSTRUCTORS
    Vdesign_2_top_ram(Vdesign_2_top__Syms* symsp, const char* v__name);
    ~Vdesign_2_top_ram();
    VL_UNCOPYABLE(Vdesign_2_top_ram);

    // INTERNAL METHODS
    void __Vconfigure(bool first);
};


#endif  // guard
