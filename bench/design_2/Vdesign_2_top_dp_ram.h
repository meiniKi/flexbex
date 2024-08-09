// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design internal header
// See Vdesign_2_top.h for the primary calling header

#ifndef VERILATED_VDESIGN_2_TOP_DP_RAM_H_
#define VERILATED_VDESIGN_2_TOP_DP_RAM_H_  // guard

#include "verilated.h"


class Vdesign_2_top__Syms;

class alignas(VL_CACHE_LINE_BYTES) Vdesign_2_top_dp_ram final : public VerilatedModule {
  public:

    // DESIGN SPECIFIC STATE
    VL_IN8(__PVT__clk,0,0);
    VL_IN8(__PVT__en_a_i,0,0);
    VL_IN8(__PVT__o_be_a_i,3,0);
    VL_IN8(__PVT__we_a_i,0,0);
    VL_IN8(__PVT__en_b_i,0,0);
    VL_IN8(__PVT__o_be_b_i,3,0);
    VL_IN8(__PVT__we_b_i,0,0);
    CData/*3:0*/ __PVT__be_b_i;
    CData/*7:0*/ __VdlyVal__ram_block__v0;
    CData/*7:0*/ __VdlyVal__ram_block__v1;
    CData/*7:0*/ __VdlyVal__ram_block__v2;
    CData/*7:0*/ __VdlyVal__ram_block__v3;
    CData/*0:0*/ __VdlySet__ram_block__v0;
    CData/*0:0*/ __VdlySet__ram_block__v1;
    CData/*0:0*/ __VdlySet__ram_block__v2;
    CData/*0:0*/ __VdlySet__ram_block__v3;
    VL_IN16(__PVT__addr_a_i,11,0);
    VL_IN16(__PVT__addr_b_i,11,0);
    SData/*11:0*/ __VdlyDim0__ram_block__v0;
    SData/*11:0*/ __VdlyDim0__ram_block__v1;
    SData/*11:0*/ __VdlyDim0__ram_block__v2;
    SData/*11:0*/ __VdlyDim0__ram_block__v3;
    VL_IN(__PVT__wdata_a_i,31,0);
    VL_OUT(__PVT__rdata_a_o,31,0);
    VL_IN(__PVT__wdata_b_i,31,0);
    VL_OUT(__PVT__rdata_b_o,31,0);
    VlUnpacked<IData/*31:0*/, 4096> __PVT__ram_block;

    // INTERNAL VARIABLES
    Vdesign_2_top__Syms* const vlSymsp;

    // CONSTRUCTORS
    Vdesign_2_top_dp_ram(Vdesign_2_top__Syms* symsp, const char* v__name);
    ~Vdesign_2_top_dp_ram();
    VL_UNCOPYABLE(Vdesign_2_top_dp_ram);

    // INTERNAL METHODS
    void __Vconfigure(bool first);
    uint32_t readByte(uint32_t byte_addr);
    uint32_t readWord(uint32_t word_addr);
    void writeByte(uint32_t byte_addr, uint32_t val);
    void writeWord(uint32_t addr, uint32_t val);
};


#endif  // guard
