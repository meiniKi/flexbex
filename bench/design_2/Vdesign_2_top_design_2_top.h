// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design internal header
// See Vdesign_2_top.h for the primary calling header

#ifndef VERILATED_VDESIGN_2_TOP_DESIGN_2_TOP_H_
#define VERILATED_VDESIGN_2_TOP_DESIGN_2_TOP_H_  // guard

#include "verilated.h"
class Vdesign_2_top_forte_soc_top;


class Vdesign_2_top__Syms;

class alignas(VL_CACHE_LINE_BYTES) Vdesign_2_top_design_2_top final : public VerilatedModule {
  public:
    // CELLS
    Vdesign_2_top_forte_soc_top* forte_soc_top_i;

    // DESIGN SPECIFIC STATE
    VL_IN8(clk,0,0);
    VL_IN8(reset,0,0);
    VL_IN8(we_i,0,0);
    VL_OUT8(irq_id_o,4,0);
    VL_IN8(irq_id_i,4,0);
    VL_IN8(irq_i,0,0);
    VL_OUT8(irq_ack_o,0,0);
    VL_IN8(debug_req_i,0,0);
    VL_IN8(start,0,0);
    VL_OUT8(cont_2_uart_w_0_complete,0,0);
    VL_IN8(start_ibex,0,0);
    VL_OUT8(uart_recv_error,0,0);
    CData/*3:0*/ __PVT__ibex_core_w_0_data_be_o;
    CData/*0:0*/ __PVT__ibex_core_w_0_data_req_o;
    CData/*0:0*/ __PVT__ibex_core_w_0_data_we_o;
    CData/*0:0*/ __PVT__ibex_core_w_0_instr_req_o;
    CData/*0:0*/ __PVT__ram_w_0_ibex_data_gnt_o;
    CData/*0:0*/ __PVT__ram_w_0_ibex_data_rvalid_o;
    CData/*0:0*/ __PVT__ram_w_0_instr_gnt_o;
    CData/*0:0*/ __PVT__ram_w_0_instr_rvalid_o;
    CData/*0:0*/ __PVT__ram_w_0_uart_data_gnt_o;
    CData/*0:0*/ __PVT__ram_w_0_uart_data_rvalid_o;
    CData/*3:0*/ __PVT__uart_to_mem_w_0_data_be_o;
    CData/*0:0*/ __PVT__uart_to_mem_w_0_data_req_o;
    CData/*0:0*/ __PVT__uart_to_mem_w_0_data_we_o;
    CData/*0:0*/ __PVT__cont_2_uart_i__DOT__transmit;
    CData/*7:0*/ __PVT__cont_2_uart_i__DOT__send;
    CData/*2:0*/ __PVT__cont_2_uart_i__DOT__UART_STATE;
    CData/*0:0*/ __PVT__cont_2_uart_i__DOT__trans_txn_ff2;
    CData/*0:0*/ __PVT__cont_2_uart_i__DOT__trans_txn_ff;
    CData/*3:0*/ __PVT__cont_2_uart_i__DOT__idle_count;
    CData/*1:0*/ __PVT__cont_2_uart_i__DOT__data_count;
    CData/*1:0*/ __PVT__cont_2_uart_i__DOT__read_count;
    CData/*2:0*/ __PVT__cont_2_uart_i__DOT__uart_i__DOT__recv_state;
    CData/*5:0*/ __PVT__cont_2_uart_i__DOT__uart_i__DOT__rx_countdown;
    CData/*3:0*/ __PVT__cont_2_uart_i__DOT__uart_i__DOT__rx_bits_remaining;
    CData/*7:0*/ __PVT__cont_2_uart_i__DOT__uart_i__DOT__rx_data;
    CData/*0:0*/ __PVT__cont_2_uart_i__DOT__uart_i__DOT__tx_out;
    CData/*1:0*/ __PVT__cont_2_uart_i__DOT__uart_i__DOT__tx_state;
    CData/*5:0*/ __PVT__cont_2_uart_i__DOT__uart_i__DOT__tx_countdown;
    CData/*3:0*/ __PVT__cont_2_uart_i__DOT__uart_i__DOT__tx_bits_remaining;
    CData/*7:0*/ __PVT__cont_2_uart_i__DOT__uart_i__DOT__tx_data;
    VL_IN16(address,11,0);
    SData/*11:0*/ __PVT__uart_to_mem_w_0_data_addr_o;
    SData/*10:0*/ __PVT__cont_2_uart_i__DOT__uart_i__DOT__rx_clk_divider;
    SData/*10:0*/ __PVT__cont_2_uart_i__DOT__uart_i__DOT__tx_clk_divider;
    VL_OUT(cont_2_uart_w_0_read_data_o,31,0);
    VL_IN(data,31,0);
    VL_OUT(eFPGA_operand_a_o,31,0);
    VL_OUT(eFPGA_operand_b_o,31,0);
    VL_IN(eFPGA_result_a_i,31,0);
    VL_IN(eFPGA_result_b_i,31,0);
    VL_IN(eFPGA_result_c_i,31,0);
    IData/*31:0*/ __PVT__ibex_core_w_0_data_addr_o;
    IData/*31:0*/ __PVT__ibex_core_w_0_data_wdata_o;
    IData/*31:0*/ __PVT__ibex_core_w_0_instr_addr_o;
    IData/*31:0*/ __PVT__ram_w_0_ibex_data_rdata_o;
    IData/*31:0*/ __PVT__ram_w_0_instr_rdata_o;
    IData/*31:0*/ __PVT__ram_w_0_uart_data_rdata_o;
    IData/*31:0*/ __PVT__uart_to_mem_w_0_data_wdata_o;

    // INTERNAL VARIABLES
    Vdesign_2_top__Syms* const vlSymsp;

    // CONSTRUCTORS
    Vdesign_2_top_design_2_top(Vdesign_2_top__Syms* symsp, const char* v__name);
    ~Vdesign_2_top_design_2_top();
    VL_UNCOPYABLE(Vdesign_2_top_design_2_top);

    // INTERNAL METHODS
    void __Vconfigure(bool first);
};


#endif  // guard
