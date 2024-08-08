
  import ibex_defines::*;



module ibex_eFPGA  (
    input logic           clk,
    input logic           rst_n,
    input logic           en_i,
    input logic [1:0]     operator_i,
  //  input logic [31:0]    operand_a_i,
  //  input logic [31:0]    operand_b_i,
    output logic          ready_o, //should be 0 when working - Drives ex_ready_o to ID Stage
    output reg [31:0]     endresult_o,
    input  logic [31:0]           result_a_i,
    input  logic [31:0]           result_b_i,
    input  logic [31:0]           result_c_i,
    input  logic [3:0]            delay_i
);

typedef enum logic [1:0] { eIDLE, ePROCESSING, eFINISH  } eFPGA_fsm;
eFPGA_fsm eFPGA_fsm_r;

reg [3:0] count;

// TODO: sort this out for CX

//this is a placeholder for the eFPGA

always_ff @(posedge clk) begin
  if(!rst_n) begin
    eFPGA_fsm_r <= eIDLE;
    count <= 0;
  end
  else begin
      unique case (eFPGA_fsm_r)
        eIDLE: begin
          count <= 0;
          if(en_i == 1)
            eFPGA_fsm_r  <= ePROCESSING;
        end
        ePROCESSING: begin
          count <= count + 1;
          if (count == delay_i)
          begin
              eFPGA_fsm_r  <= eFINISH;
              unique case (operator_i)
                2'b00: begin
                  endresult_o <= result_a_i;
                end
                2'b01: begin
                  endresult_o <= result_b_i;
                end
                2'b10: begin
                  endresult_o <= result_c_i;
                end
                default: begin
                  endresult_o <= result_a_i;
                end
              endcase
          end
        end
        eFINISH: begin
          eFPGA_fsm_r  <= eIDLE;
        end
        default:;
      endcase
  end
end

assign ready_o  = (eFPGA_fsm_r == eFINISH);

endmodule
