module datapath(datapath_out, mdata, sximm5, sximm8, PC, vsel, readnum, writenum, loada, loadb, loadc, loads, ALUop, write, shift, asel, bsel, clk, Z_out,program_wire);

  input asel, bsel, clk, write, loada, loadb, loadc, loads;
  input [1:0] vsel;
  input [2:0] writenum, readnum;
  input [1:0] ALUop, shift;
  input [15:0] sximm5, sximm8, mdata;

  output [15:0] datapath_out;
  output[2:0] Z_out;
  output [15:0] program_wire;

 wire [15:0] program_wire;
  wire [15:0] data_in;
  wire [15:0] data_out, loada_out, in, sout, Ain, Bin, out;
  wire Z;
  wire V;
  wire N;

  input [8:0] PC;
  assign program_wire=data_out;
  Mux4 #(16) muxdatain(mdata, sximm8, {8'b0, PC}, datapath_out,vsel,data_in);
  regfile REGFILE(data_in,writenum,write,readnum,clk,data_out);
  vDFF1 #(16) loadaF(clk,loada, data_out, loada_out);
  Mux2 #(16) muxain(16'b0,loada_out,asel,Ain);
  vDFF1 #(16) loadbF(clk,loadb, data_out, in);
  shifter U1(in,shift,sout);
  Mux2 #(16) muxbin(sximm5,sout,bsel,Bin);
  ALU U2(Ain,Bin,ALUop,out,Z,V,N);
  vDFF1 #(16) loadcF(clk,loadc, out, datapath_out);
  vDFF1 #(3) loadsF(clk,loads, {Z,V,N}, Z_out);

endmodule

module vDFF1(clk, load, in, out);
  parameter n = 1;  // width
  input clk, load;
  input [n-1:0] in;
  output [n-1:0] out;
  reg [n-1:0] out;
  wire [n-1:0] next_out;

  assign next_out = (load ? in : out);

  always @(posedge clk)
    out = next_out;
endmodule 

module Mux2(a1, a0, sb, b) ;
  parameter k = 1;
  input [k-1:0] a0, a1;  // inputs
  input sb;          // binary select
  output[k-1:0] b;
  reg[k-1:0] b;
  always @(*) begin
   case(sb)
    0: b = a0;
    1: b = a1;
    default: b = {k{1'bx}};
   endcase
  end  
endmodule

//Mux 4 input binary select
module Mux4(a3, a2, a1, a0, sb, b) ;
  parameter k = 1;
  input [k-1:0] a0, a1, a2, a3;  // inputs
  input [1:0]   sb;          // binary select
  output[k-1:0] b;
  reg[k-1:0] b;

  always @(*) begin
   case(sb)
    2'b00: b = a0;
    2'b01: b = a1;
    2'b10: b = a2;
    2'b11: b = a3;
    default: b = {k{1'bx}};
   endcase
  end
endmodule
