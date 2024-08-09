// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Model implementation (design independent parts)

#include "Vdesign_2_top__pch.h"
#include "verilated_vcd_c.h"

//============================================================
// Constructors

Vdesign_2_top::Vdesign_2_top(VerilatedContext* _vcontextp__, const char* _vcname__)
    : VerilatedModel{*_vcontextp__}
    , vlSymsp{new Vdesign_2_top__Syms(contextp(), _vcname__, this)}
    , reset{vlSymsp->TOP.reset}
    , clk{vlSymsp->TOP.clk}
    , we_i{vlSymsp->TOP.we_i}
    , irq_id_o{vlSymsp->TOP.irq_id_o}
    , irq_id_i{vlSymsp->TOP.irq_id_i}
    , irq_i{vlSymsp->TOP.irq_i}
    , irq_ack_o{vlSymsp->TOP.irq_ack_o}
    , debug_req_i{vlSymsp->TOP.debug_req_i}
    , start{vlSymsp->TOP.start}
    , cont_2_uart_w_0_complete{vlSymsp->TOP.cont_2_uart_w_0_complete}
    , start_ibex{vlSymsp->TOP.start_ibex}
    , uart_recv_error{vlSymsp->TOP.uart_recv_error}
    , address{vlSymsp->TOP.address}
    , cont_2_uart_w_0_read_data_o{vlSymsp->TOP.cont_2_uart_w_0_read_data_o}
    , data{vlSymsp->TOP.data}
    , eFPGA_operand_a_o{vlSymsp->TOP.eFPGA_operand_a_o}
    , eFPGA_operand_b_o{vlSymsp->TOP.eFPGA_operand_b_o}
    , eFPGA_result_a_i{vlSymsp->TOP.eFPGA_result_a_i}
    , eFPGA_result_b_i{vlSymsp->TOP.eFPGA_result_b_i}
    , eFPGA_result_c_i{vlSymsp->TOP.eFPGA_result_c_i}
    , design_2_top{vlSymsp->TOP.design_2_top}
    , rootp{&(vlSymsp->TOP)}
{
    // Register model with the context
    contextp()->addModel(this);
    contextp()->traceBaseModelCbAdd(
        [this](VerilatedTraceBaseC* tfp, int levels, int options) { traceBaseModel(tfp, levels, options); });
}

Vdesign_2_top::Vdesign_2_top(const char* _vcname__)
    : Vdesign_2_top(Verilated::threadContextp(), _vcname__)
{
}

//============================================================
// Destructor

Vdesign_2_top::~Vdesign_2_top() {
    delete vlSymsp;
}

//============================================================
// Evaluation function

#ifdef VL_DEBUG
void Vdesign_2_top___024root___eval_debug_assertions(Vdesign_2_top___024root* vlSelf);
#endif  // VL_DEBUG
void Vdesign_2_top___024root___eval_static(Vdesign_2_top___024root* vlSelf);
void Vdesign_2_top___024root___eval_initial(Vdesign_2_top___024root* vlSelf);
void Vdesign_2_top___024root___eval_settle(Vdesign_2_top___024root* vlSelf);
void Vdesign_2_top___024root___eval(Vdesign_2_top___024root* vlSelf);

void Vdesign_2_top::eval_step() {
    VL_DEBUG_IF(VL_DBG_MSGF("+++++TOP Evaluate Vdesign_2_top::eval_step\n"); );
#ifdef VL_DEBUG
    // Debug assertions
    Vdesign_2_top___024root___eval_debug_assertions(&(vlSymsp->TOP));
#endif  // VL_DEBUG
    vlSymsp->__Vm_activity = true;
    vlSymsp->__Vm_deleter.deleteAll();
    if (VL_UNLIKELY(!vlSymsp->__Vm_didInit)) {
        vlSymsp->__Vm_didInit = true;
        VL_DEBUG_IF(VL_DBG_MSGF("+ Initial\n"););
        Vdesign_2_top___024root___eval_static(&(vlSymsp->TOP));
        Vdesign_2_top___024root___eval_initial(&(vlSymsp->TOP));
        Vdesign_2_top___024root___eval_settle(&(vlSymsp->TOP));
    }
    VL_DEBUG_IF(VL_DBG_MSGF("+ Eval\n"););
    Vdesign_2_top___024root___eval(&(vlSymsp->TOP));
    // Evaluate cleanup
    Verilated::endOfEval(vlSymsp->__Vm_evalMsgQp);
}

//============================================================
// Events and timing
bool Vdesign_2_top::eventsPending() { return false; }

uint64_t Vdesign_2_top::nextTimeSlot() {
    VL_FATAL_MT(__FILE__, __LINE__, "", "%Error: No delays in the design");
    return 0;
}

//============================================================
// Utilities

const char* Vdesign_2_top::name() const {
    return vlSymsp->name();
}

//============================================================
// Invoke final blocks

void Vdesign_2_top___024root___eval_final(Vdesign_2_top___024root* vlSelf);

VL_ATTR_COLD void Vdesign_2_top::final() {
    Vdesign_2_top___024root___eval_final(&(vlSymsp->TOP));
}

//============================================================
// Implementations of abstract methods from VerilatedModel

const char* Vdesign_2_top::hierName() const { return vlSymsp->name(); }
const char* Vdesign_2_top::modelName() const { return "Vdesign_2_top"; }
unsigned Vdesign_2_top::threads() const { return 1; }
void Vdesign_2_top::prepareClone() const { contextp()->prepareClone(); }
void Vdesign_2_top::atClone() const {
    contextp()->threadPoolpOnClone();
}
std::unique_ptr<VerilatedTraceConfig> Vdesign_2_top::traceConfig() const {
    return std::unique_ptr<VerilatedTraceConfig>{new VerilatedTraceConfig{false, false, false}};
};

//============================================================
// Trace configuration

void Vdesign_2_top___024root__trace_decl_types(VerilatedVcd* tracep);

void Vdesign_2_top___024root__trace_init_top(Vdesign_2_top___024root* vlSelf, VerilatedVcd* tracep);

VL_ATTR_COLD static void trace_init(void* voidSelf, VerilatedVcd* tracep, uint32_t code) {
    // Callback from tracep->open()
    Vdesign_2_top___024root* const __restrict vlSelf VL_ATTR_UNUSED = static_cast<Vdesign_2_top___024root*>(voidSelf);
    Vdesign_2_top__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    if (!vlSymsp->_vm_contextp__->calcUnusedSigs()) {
        VL_FATAL_MT(__FILE__, __LINE__, __FILE__,
            "Turning on wave traces requires Verilated::traceEverOn(true) call before time 0.");
    }
    vlSymsp->__Vm_baseCode = code;
    tracep->pushPrefix(std::string{vlSymsp->name()}, VerilatedTracePrefixType::SCOPE_MODULE);
    Vdesign_2_top___024root__trace_decl_types(tracep);
    Vdesign_2_top___024root__trace_init_top(vlSelf, tracep);
    tracep->popPrefix();
}

VL_ATTR_COLD void Vdesign_2_top___024root__trace_register(Vdesign_2_top___024root* vlSelf, VerilatedVcd* tracep);

VL_ATTR_COLD void Vdesign_2_top::traceBaseModel(VerilatedTraceBaseC* tfp, int levels, int options) {
    (void)levels; (void)options;
    VerilatedVcdC* const stfp = dynamic_cast<VerilatedVcdC*>(tfp);
    if (VL_UNLIKELY(!stfp)) {
        vl_fatal(__FILE__, __LINE__, __FILE__,"'Vdesign_2_top::trace()' called on non-VerilatedVcdC object;"
            " use --trace-fst with VerilatedFst object, and --trace with VerilatedVcd object");
    }
    stfp->spTrace()->addModel(this);
    stfp->spTrace()->addInitCb(&trace_init, &(vlSymsp->TOP));
    Vdesign_2_top___024root__trace_register(&(vlSymsp->TOP), stfp->spTrace());
}
