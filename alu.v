
module ALU(Ain,Bin,ALUop,out,Z, V, N);
input [15:0] Ain, Bin;
input [1:0] ALUop;
output [15:0] out;
output Z;
output V;
output N;

reg [15:0] out;
reg Z;
reg V;
reg N;
wire [15: 0] s1, s2;
wire V1, V2;

AddSub #(16) out1(Ain, Bin, 1'b0, s1, V1);
AddSub #(16) out2(Ain, Bin, 1'b1, s2, V2);


always @(*) begin
   V = 1'b0;
   case (ALUop)
      2'b00: begin out = s1; V = V1; end//if ALUop = 2'b00, add Ain and Bin
      2'b01: begin out = s2; V = V2; end//if ALUop = 2'b01, subtract Bin from Ain
      2'b10:out = Ain & Bin; //if AlUop = 2'b10, AND Ain and Bin
      2'b11:out = ~ Bin;     //
      default: out = {15{1'bx}}; //if ALUop is another value, set out to x's so we can debug
   endcase

   Z = 1'b0; //Z (the zero flag of the status register) is 1'b0 
   //if the below IF statement does not execute
   if (out == {16{1'b0}})begin //if out is EXACTLY all 0's...
      Z = 1'b1; //set Z to 1 (to implement if statements in later labs)
   end
   if (out[15] == 1'b1) begin
      N = 1'b1;
   end 
   else begin
      N = 1'b0;
   end
end
endmodule

// add a+b or subtract a-b, check for overflow
module AddSub(a,b,sub,s,ovf) ;
  parameter n = 8 ;
  input [n-1:0] a, b ;
  input sub ;           // subtract if sub=1, otherwise add
  output [n-1:0] s ;
  output ovf ;          // 1 if overflow
  wire c1, c2 ;         // carry out of last two bits
  wire ovf = c1 ^ c2 ;  // overflow if signs don't match

  // add non sign bits
  Adder1 #(n-1) ai(a[n-2:0],b[n-2:0]^{n-1{sub}},sub,c1,s[n-2:0]) ;
  // add sign bits
  Adder1 #(1)   as(a[n-1],b[n-1]^sub,c1,c2,s[n-1]) ;
endmodule

// multi-bit adder - behavioral
module Adder1(a,b,cin,cout,s) ;
  parameter n = 8 ;
  input [n-1:0] a, b ;
  input cin ;
  output [n-1:0] s ;
  output cout ;
  wire [n-1:0] s;
  wire cout ;

  assign {cout, s} = a + b + cin ;
endmodule 

