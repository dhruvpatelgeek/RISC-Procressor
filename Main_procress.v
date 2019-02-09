module main(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,CLOCK_50);
  input [3:0] KEY;
  input [9:0] SW;
  input CLOCK_50;
  output [9:0] LEDR; 
  output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

  wire [15:0] datapath_out;
  wire Z, N, V;
  wire [1:0] mem_cmd;
  wire load_pc, reset_pc, load_addr, addr_sel, load_ir;
  wire control, write;
  wire [15:0] read_data; 
  wire [8:0] outCounter; // output of Program counter module
  wire [8:0] outDA; //output of Data Address reg
  wire [8:0] mem_addr;
  wire [15:0] dout;
  wire controlA, controlB;
wire [5:0] present;

  cpu CPU( .clk   (CLOCK_50), // recall from Lab 4 that KEY0 is 1 when NOT pushed
         .reset (~KEY[1]), 
         .in    (read_data),
         .out   (datapath_out),
         .Z     (Z),
         .N     (N),
         .V     (V),
	 .mem_cmd (mem_cmd),
	 .load_addr (load_addr),
	 .addr_sel (addr_sel),
	 .load_ir (load_ir),
	 .PC (outCounter),
.present_state(present));

/*
  assign HEX5[0] = ~Z;
  assign HEX5[6] = ~N;
  assign HEX5[3] = ~V;
*/
  // fill in sseg to display 4-bits in hexidecimal 0,1,2...9,A,B,C,D,E,F
sseg2 H4(present,{HEX5,HEX4});

  //assign HEX4 = 7'b1111111;
  //assign {HEX5[2:1],HEX5[5:4]} = 4'b1111; // disabled

assign mem_addr = addr_sel ? outCounter : outDA; // describes logic for choosing between DataAddress reg and ProgramCounter
// lab 8-------------
assign LEDR[8]= (present==6'd21) ? 1'b1 : 1'b0;// halt instruction linked to led 8

RAM MEM(CLOCK_50,mem_addr[7:0],mem_addr[7:0],write,datapath_out,dout);

triStateBuffer buff(dout, control, read_data);
vDFF_PC_DA DA(CLOCK_50, load_addr, datapath_out[8:0], outDA);
readOrWrite ROW(mem_cmd, mem_addr[8], write, control);


circuitA Switch (mem_cmd, mem_addr, controlA);
circuitB Leds (mem_cmd, mem_addr, controlB);

// insert circuitB output, outputs to LEDR on rising edge of the clock
vDFF1 #(8) LEDreg(CLOCK_50, controlB, datapath_out[7:0], LEDR[7:0]);
 
assign read_data = controlA ? {8'b0,SW[7:0]} : 16'bz;

  
endmodule


`define MWRITE 2'b01
`define MREAD 2'b11

  `define Sdecode 6'd22    //decode
  `define SgetB 6'd2    // GetA
  `define SgetA 6'd3      // GetB
  `define SloadC 6'd4   //LoadC
  `define SwriteReg 6'd5  //WriteReg
  `define SwriteImm 6'd6  //WriteImm
  `define Swritemove 6'd7  //WriteMOV
  `define Smove 6'd8       //
  `define Smovenext 6'd9      //
  `define Smovenextnext 6'd10 //
  `define ScMP 6'd11       //
  `define SmVN 6'd12       //MVN
  `define Reset 6'd1
  `define IF1 6'd13
  `define IF2 6'd14
  `define UpdatePC 6'd15

   `define Sload  6'd16
   `define SsetAload 6'd17
   `define SsetCload 6'd18
   `define Swriteload 6'd19
    `define StoreInRd 6'd23
	`define Readfrommemory 6'd24
  
   `define Sstore 6'd20
   `define ReadRd 6'd25
   `define GetRd 6'd26
	`define StoreMem 6'd27
   `define Shalt 6'd21

module readOrWrite (mem_cmd, mem_addr_bit, write, control);
input [1:0] mem_cmd;
input mem_addr_bit; 
output write; 
output control;

wire writeDecision = (mem_cmd == 2'b01);
wire readDecision = (mem_cmd == 2'b11);

wire allowedSize = (mem_addr_bit == 1'b0);

assign  write = writeDecision && allowedSize;
assign control = readDecision && allowedSize;

endmodule

module triStateBuffer (in, load, out);
input [15:0] in;
input load;
output [15:0] out;

assign out = load ? in : 16'bz;
endmodule        

module vDFF(clk,D,Q);
  parameter n=1;
  input clk;
  input [n-1:0] D;
  output [n-1:0] Q;
  reg [n-1:0] Q;
  always @(posedge clk)
    Q <= D;
endmodule


// circuit to design, feeds tristatt driver
module circuitA(mem_cmd, mem_addr, outA);
input [1:0] mem_cmd;
input [8:0] mem_addr;
wire [8:0] in; //0'h140 0x0140
output reg outA; //from

assign in = 9'h140;

//output to tristate driver
always @(*) begin
if ({mem_cmd == `MREAD} && {mem_addr == in})begin 
outA = 1'b1;
end else begin
outA = 1'b0;
end
end
endmodule

// circuit that loads register to LED
module circuitB(mem_cmd, mem_addr, fromB);
input [1:0] mem_cmd;
input [8:0] mem_addr;
output reg fromB;

wire [8:0] in;

assign in = 9'h100;

//output to LEDreg
always @(*) begin
if ({mem_cmd == `MWRITE} && {mem_addr == in})begin 
fromB = 1'b1;
end else begin
fromB = 1'b0;
end
end
endmodule

// The sseg module below can be used to display the value of datpath_out on
// the hex LEDS the input is a 4-bit value representing numbers between 0 and
// 15 the output is a 7-bit value that will print a hexadecimal digit.  You
// may want to look at the code in Figure 7.20 and 7.21 in Dally but note this
// code will not work with the DE1-SoC because the order of segments used in
// the book is not the same as on the DE1-SoC (see comments below).

module sseg(in,segs);
  input [3:0] in;
  output [6:0] segs;

  reg [6:0] segs;

  
  //   5    1
  //   5    1
  //    6666
  //   4    2
  //   4    2
  //    3333
  //
  // Decimal value | Binary | Hexadecimal symbol to render on (one) HEX display | Binary equivalent
  //             0 | 0000 | 0 | 1000000
  //             1 | 0001 | 1 | 1111001
  //             2 | 0010 | 2 | 0100100
  //             3 | 0011 | 3 | 0110000
  //             4 | 0100 | 4 | 0011001
  //             5 | 0101 | 5 | 0010010
  //             6 | 0110 | 6 | 0000010
  //             7 | 0111 | 7 | 1111000
  //             8 | 1000 | 8 | 0000000
  //             9 | 1001 | 9 | 0010000
  //            10 | 1010 | A | 0001000
  //            11 | 1011 | b | 0000011
  //            12 | 1100 | C | 1000110
  //            13 | 1101 | d | 0100001
  //            14 | 1110 | E | 0000110
  //            15 | 1111 | F | 0001110
  always @(*) begin
    case (in)
      4'b0000: segs = 7'b1000000; //0
      4'b0001: segs = 7'b1111001; //1
      4'b0010: segs = 7'b0100100; //2
      4'b0011: segs = 7'b0110000; //3
      4'b0100: segs = 7'b0011001; //4
      4'b0101: segs = 7'b0010010; //5
      4'b0110: segs = 7'b0000010; //6
      4'b0111: segs = 7'b1111000; //7
      4'b1000: segs = 7'b0000000; //8
      4'b1001: segs = 7'b0010000; //9
      4'b1010: segs = 7'b0001000; //A
      4'b1011: segs = 7'b0000011; //b
      4'b1100: segs = 7'b1000110; //C
      4'b1101: segs = 7'b0100001; //d
      4'b1110: segs = 7'b0000110; //E
      4'b1111: segs = 7'b0001110; //F
      default: segs = 7'bxxxxxxx; //error!
    endcase
  end

endmodule

// for testing purposes only
module sseg2(in, segs);
input [5:0] in; //present
output reg [13:0] segs;
always @(*) begin
    case (in)
      `Sdecode: segs = 14'b1000000_1000000; //00
      `SgetB: segs = 14'b1000000_1111001; //01
      `SgetA: segs = 14'b1000000_0100100; //02
      `SloadC: segs = 14'b1000000_0110000; //03
      `SwriteReg: segs = 14'b1000000_0011001; //04
      `SwriteImm: segs = 14'b1000000_0010010; //05
      `Swritemove: segs = 14'b1000000_0000010; //6
      `Smove: segs = 14'b1000000_1111000; //7
      `Smovenext: segs = 14'b1000000_0000000; //8
      `Smovenextnext: segs = 14'b1000000_0010000; //9
      `ScMP: segs = 14'b1111001_1000000; //10
      `SmVN: segs = 14'b1111001_0000011; //11
      `Reset: segs = 14'b1111001_0100100; //12
      `IF1: segs = 14'b1111001_0110000; //13
      `IF2: segs = 14'b1111001_0011001; //14
`UpdatePC: segs = 14'b1111001_0010010; //15
`Sload: segs = 14'b1111001_0000010; //16
`SsetAload: segs = 14'b1111001_1111000; //17
`SsetCload: segs = 14'b1111001_0000000; //18
`Swriteload: segs = 14'b1111001_0010000; //19
`StoreInRd: segs = 14'b0100100_1000000; //20
`Readfrommemory: segs = 14'b0100100_1111001; //21
`Sstore: segs = 14'b0100100_0100100; //22
`ReadRd: segs = 14'b0100100_0110000; //23
`GetRd: segs = 14'b0100100_0011001; //24
`StoreMem: segs = 14'b0100100_0010010; //25
`Shalt: segs = 14'b0100100_0000010; //26
      default: segs = 14'bxxxxxxx_xxxxxxx; //error!
    endcase
  end
endmodule
/*
4'b0000: segs = 14'b1000000; //0
      4'b0001: segs = 7'b1111001; //1
      4'b0010: segs = 7'b0100100; //2
      4'b0011: segs = 7'b0110000; //3
      4'b0100: segs = 7'b0011001; //4
      4'b0101: segs = 7'b0010010; //5
      4'b0110: segs = 7'b0000010; //6
      4'b0111: segs = 7'b1111000; //7
      4'b1000: segs = 7'b0000000; //8
      4'b1001: segs = 7'b0010000; //9
      4'b1010: segs = 7'b0001000; //A
      4'b1011: segs = 7'b0000011; //b
      4'b1100: segs = 7'b1000110; //C
      4'b1101: segs = 7'b0100001; //d
      4'b1110: segs = 7'b0000110; //E
      4'b1111: segs = 7'b0001110; //F
      default: segs = 7'bxxxxxxx; //error!
*/
