module shifter(in,shift,sout);
  input [15:0] in;
  input [1:0] shift;
  output [15:0] sout;

  reg [15:0] sout; //sout is set in the always block below, so it must be a reg as well

  always @(*)begin
    //cases for shift:
    //00 sout = in
    //01 in shifted left 1-bit, least significant bit is zero
    //10 in shifted right 1-bit, most significant bit, MSB, is 0
    //11 in shifted right 1-bit, MSB is copy of in[15]
    case (shift)
      2'b00: sout = in;
      2'b01: sout = {in[14:0],1'b0}; //least significant bit is zero 
      2'b10: sout = {1'b0,in[15:1]}; //MSB is 0, rest is in[15:1]
      2'b11: sout = {in[15],in[15:1]}; //same as above, except MSB is in[15]
      default: sout = {16{1'bx}};
    endcase
  end
endmodule
