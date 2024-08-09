



module forte_soc_top #(
parameter ADDR_WIDTH = 12)
   (clk_i,
    debug_req_i,
    fetch_enable_i,
    irq_ack_o,
    irq_i,
    irq_id_i,
    irq_id_o,
    reset,
    rx_i,
    tx_o,
    eFPGA_operand_a_o,
    eFPGA_operand_b_o,
    eFPGA_result_a_i,
    eFPGA_result_b_i,
    eFPGA_result_c_i,
    uart_recv_error
);


  input clk_i;
  input debug_req_i;
  input fetch_enable_i;
  output irq_ack_o;
  input irq_i;
  input [4:0]irq_id_i;
  output [4:0]irq_id_o;
  input rx_i;
  output tx_o;
  input reset;


  output [31:0] eFPGA_operand_a_o;
  output [31:0] eFPGA_operand_b_o;
  input [31:0] eFPGA_result_a_i;
  input [31:0] eFPGA_result_b_i;
  input [31:0] eFPGA_result_c_i;

  output uart_recv_error;

    wire [31:0]ibex_core_w_0_data_addr_o;
    wire [3:0]ibex_core_w_0_data_be_o;
    wire ibex_core_w_0_data_req_o;
    wire [31:0]ibex_core_w_0_data_wdata_o;
    wire ibex_core_w_0_data_we_o;
    wire [31:0]ibex_core_w_0_instr_addr_o;
    wire ibex_core_w_0_instr_req_o;

    wire ram_w_0_ibex_data_gnt_o;
    wire [31:0]ram_w_0_ibex_data_rdata_o;
    wire ram_w_0_ibex_data_rvalid_o;
    wire ram_w_0_instr_gnt_o;
    wire [31:0]ram_w_0_instr_rdata_o;
    wire ram_w_0_instr_rvalid_o;
    wire ram_w_0_uart_data_gnt_o;
    wire [31:0]ram_w_0_uart_data_rdata_o;
    wire ram_w_0_uart_data_rvalid_o;
    wire [11:0]uart_to_mem_w_0_data_addr_o;
    wire [3:0]uart_to_mem_w_0_data_be_o;
    wire uart_to_mem_w_0_data_req_o;
    wire [31:0]uart_to_mem_w_0_data_wdata_o;
    wire uart_to_mem_w_0_data_we_o;


    uart_to_mem uart_to_mem_i
         (.clk_i(clk_i),
          .data_addr_o(uart_to_mem_w_0_data_addr_o),
          .data_be_o(uart_to_mem_w_0_data_be_o),
          .data_gnt_i(ram_w_0_uart_data_gnt_o),
          .data_rdata_i(ram_w_0_uart_data_rdata_o),
          .data_req_o(uart_to_mem_w_0_data_req_o),
          .data_rvalid_i(ram_w_0_uart_data_rvalid_o),
          .data_wdata_o(uart_to_mem_w_0_data_wdata_o),
          .data_we_o(uart_to_mem_w_0_data_we_o),
          .rst_i(reset),
          .rx_i(rx_i),
          .tx_o(tx_o),
          .uart_error(uart_recv_error));

  wire reset_ni;
  assign reset_ni = ~reset;


//need to set the debug vector
    ibex_core ibex_core_i
         (.boot_addr_i(32'd0),
          .clk_i(clk_i),
          .cluster_id_i(6'd0),
          .core_id_i(4'd0),
          .data_addr_o(ibex_core_w_0_data_addr_o),
          .data_be_o(ibex_core_w_0_data_be_o),
          .data_err_i(1'b0),
          .data_gnt_i(ram_w_0_ibex_data_gnt_o),
          .data_rdata_i(ram_w_0_ibex_data_rdata_o),
          .data_req_o(ibex_core_w_0_data_req_o),
          .data_rvalid_i(ram_w_0_ibex_data_rvalid_o),
          .data_wdata_o(ibex_core_w_0_data_wdata_o),
          .data_we_o(ibex_core_w_0_data_we_o),
          .debug_req_i(debug_req_i),
          .ext_perf_counters_i(1'b0),
          .fetch_enable_i(fetch_enable_i),
          .instr_addr_o(ibex_core_w_0_instr_addr_o),
          .instr_gnt_i(ram_w_0_instr_gnt_o),
          .instr_rdata_i(ram_w_0_instr_rdata_o),
          .instr_req_o(ibex_core_w_0_instr_req_o),
          .instr_rvalid_i(ram_w_0_instr_rvalid_o),
          .irq_ack_o(irq_ack_o),
          .irq_i(irq_i),
          .irq_id_i(irq_id_i),
          .irq_id_o(irq_id_o),
          .rst_ni(reset_ni),
          .test_en_i(1'b1),
          .cx_req_data0(eFPGA_operand_a_o),
          .cx_req_data1(eFPGA_operand_b_o),
          .cx_resp_data(eFPGA_result_a_i));



    ram ram_0
         (.clk(clk_i),
          .ibex_data_addr_i(ibex_core_w_0_data_addr_o[11:0]),
          .ibex_data_be_i(ibex_core_w_0_data_be_o),
          .ibex_data_gnt_o(ram_w_0_ibex_data_gnt_o),
          .ibex_data_rdata_o(ram_w_0_ibex_data_rdata_o),
          .ibex_data_req_i(ibex_core_w_0_data_req_o),
          .ibex_data_rvalid_o(ram_w_0_ibex_data_rvalid_o),
          .ibex_data_wdata_i(ibex_core_w_0_data_wdata_o),
          .ibex_data_we_i(ibex_core_w_0_data_we_o),
          .instr_addr_i(ibex_core_w_0_instr_addr_o[11:0]),
          .instr_gnt_o(ram_w_0_instr_gnt_o),
          .instr_rdata_o(ram_w_0_instr_rdata_o),
          .instr_req_i(ibex_core_w_0_instr_req_o),
          .instr_rvalid_o(ram_w_0_instr_rvalid_o),
          .uart_data_addr_i(uart_to_mem_w_0_data_addr_o),
          .uart_data_be_i(uart_to_mem_w_0_data_be_o),
          .uart_data_gnt_o(ram_w_0_uart_data_gnt_o),
          .uart_data_rdata_o(ram_w_0_uart_data_rdata_o),
          .uart_data_req_i(uart_to_mem_w_0_data_req_o),
          .uart_data_rvalid_o(ram_w_0_uart_data_rvalid_o),
          .uart_data_wdata_i(uart_to_mem_w_0_data_wdata_o),
          .uart_data_we_i(uart_to_mem_w_0_data_we_o));

  endmodule
