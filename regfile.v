module regfile(data_in,writenum,write,readnum,clk,data_out);
  input [15:0] data_in;
  input [2:0] writenum, readnum;
  input write, clk;
  output [15:0] data_out;
  // fill out the rest

  `define SW 16

  wire [7:0] in;
  wire [7:0] andin;
  wire [15:0] R0, R1, R2, R3, R4, R5, R6, R7;
  wire [15:0] out;

  Dec #(3,8) U1(writenum, andin);

  // Register with Load Enable Circuit
  vDFF_reg #(`SW) Reg0(clk,(andin[0] & write),data_in,R0);
  vDFF_reg #(`SW) Reg1(clk,(andin[1] & write),data_in,R1);
  vDFF_reg #(`SW) Reg2(clk,(andin[2] & write),data_in,R2);
  vDFF_reg #(`SW) Reg3(clk,(andin[3] & write),data_in,R3);
  vDFF_reg #(`SW) Reg4(clk,(andin[4] & write),data_in,R4);
  vDFF_reg #(`SW) Reg5(clk,(andin[5] & write),data_in,R5);
  vDFF_reg #(`SW) Reg6(clk,(andin[6] & write),data_in,R6);
  vDFF_reg #(`SW) Reg7(clk,(andin[7] & write),data_in,R7);

  Dec #(3,8) U2(readnum,in); 

  Mux8 #(`SW) m(R7, R6, R5, R4, R3, R2, R1, R0, in, data_out) ;
endmodule

// 3:8 Decoder
// a - binary input   (n bits wide)
// b - one hot output (m bits wide)

module Dec(a, b);
  parameter n=2;
  parameter m=4;

  input  [n-1:0] a;
  output [m-1:0] b;

  wire [m-1:0] b = 1 << a; //shift 1 to the left by 'a' bit positons
endmodule

//D-Flip Flop Register
module vDFF_reg(clk, load, in, out);
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


//8:1 multiplexer with one-hot select
module Mux8(a7, a6, a5, a4, a3, a2, a1, a0, sb, b) ;
  parameter k = 1;
  input [k-1:0] a0, a1, a2, a3, a4, a5, a6, a7;  // inputs
  input [7:0]   sb;          // one-hot select
  output[k-1:0] b;
  wire[k-1:0] b;
  assign b = ({k{sb[0]}} & a0) | 
                          ({k{sb[1]}} & a1) |
                          ({k{sb[2]}} & a2) |
                          ({k{sb[3]}} & a3) |
			  ({k{sb[4]}} & a4) | 
                          ({k{sb[5]}} & a5) |
                          ({k{sb[6]}} & a6) |
                          ({k{sb[7]}} & a7) ;  
endmodule

