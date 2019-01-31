module cpu(clk,reset,in,out,N,V,Z,mem_cmd, load_addr, addr_sel, load_ir, PC, present_state);
  input clk, reset;
  input [15:0] in;
  output [15:0] out;
  output N, V, Z;
  output [1:0] mem_cmd;
  output  load_addr, addr_sel, load_ir;
  output [8:0] PC; //SHOULD BE CONNECTED TO THE OUTPUT OF ProgramCOUNTER because used by the checker

output [5:0] present_state;
wire [5:0] present;
assign present_state = present;
//   datapath inputs / outputs
  wire [15:0] sximm5, sximm8, mdata;
  wire [8:0] PCdatapath;
  wire [1:0] ALUop, shift, vsel; //vsel is still a binary select, but now it has 4 inputs
  wire [2:0] readnum, writenum,Z_out;
  wire asel, bsel, loada, loadb, loadc, loads, write, load_pc, reset_pc;
  //note: selects are binary!
  
//   FSM i/o
  wire [2:0] opcode;
  wire [1:0] op;

//   decoder
  
  // this is the wire connecting the output of the instruction register to
  // the input of the instruction decoder
  wire [15:0] decoder_in;
  
  // input to decoder from Controller
  wire [2:0] nsel; //nsel is a 3-bit one-hot code
  assign PCdatapath = PC;//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<hunt

  // since we concatenated N,V,Z into z_out in datapath, we have to separate them again
  assign Z = Z_out[2];
  assign V = Z_out[1];
  assign N = Z_out[0];

//lab 8 wires<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
wire [2:0]cond,Rd;
wire [15:0] program_wire;
// instruction register
  instruction_register #(16) Instruction_Register(clk, load_ir, in, decoder_in);
//Program counter
  programCounter programCount(clk, reset_pc, load_pc, PC, op, opcode, sximm8, cond, Rd,Z,N,V,program_wire);
//decoder:  
  decoder instruction_decoder (
  //  from instruction register:
    .in(decoder_in),
  //  to Controller FSM:
    .opcode(opcode),
    .op(op),

  //  to datapath:
  .ALUop(ALUop),
  .sximm5(sximm5),
  .sximm8(sximm8),
  .shift(shift),
  .readnum(readnum),  
  .writenum(writenum),
  //... input from FSM ... (to determine which of Rn, Rd, Rm) go to readnum or writenum
  .nsel(nsel),
  // lab 8------------
  .cond(cond),
  .Rd(Rd)
  );

  control FSM (
  // inputs 
   .clk(clk),
   .reset(reset),
    // from decoder
   .opcode(opcode),
   .op(op),
  // ...other outputs to datapath / decoder...
   .nsel(nsel), //to decoder
   .vsel(vsel),
   .loada(loada),
   .loadb(loadb),
   .loadc(loadc),
   .loads(loads),
   .asel(asel),
   .bsel(bsel),
   .write(write),
  // output to controller of cpu
  .load_pc(load_pc),
  .reset_pc(reset_pc),
  .load_addr(load_addr),
  .addr_sel(addr_sel),
  .mem_cmd(mem_cmd),
  .load_ir(load_ir),
.present(present));

  datapath DP(
    .Z_out(Z_out), //Z_out drives N, V, Z
    .datapath_out(out), //value of out should be contents of register C inside datapath
    .clk(clk),
    //unused for Lab 6
    .mdata(in),
    .PC(PCdatapath),
    // set by Instruction Decoder
    .ALUop(ALUop),
    .sximm8(sximm8),
    .sximm5(sximm5), //unused in Lab 6 (used in Lab 7)
    .shift(shift),
    .writenum(writenum),
    .readnum(readnum),
    //set by Controller FSM
    .write(write),
    .vsel(vsel),
    .loada(loada),
    .loadb(loadb),
    .loadc(loadc),
    .loads(loads),
    .asel(asel),
    .bsel(bsel),
    .program_wire(program_wire)
    );
endmodule

// Instruction Register block
module instruction_register (clk, load, in, out);
  parameter n=1;
  input clk, load;
  input [n-1:0] in;
  output reg [n-1:0] out;
  wire [n-1:0] next_out;

  assign next_out = load ? in : out;
  // if load = 1
  // value of in should be copied to instruction register on rising edge of clk
  // if load = 0, contents of instruction register should not change.
  always @(posedge clk)
      out = next_out;
  // if load = 1'b0 we want to keep the current value, so we do want a latch.
endmodule

// Instruction Decoder block
module decoder (in, ALUop, sximm5, sximm8, shift, readnum, writenum, opcode, op,nsel, // previous lab signals
                cond,Rd);// lab 8 signal
  input [15:0] in; // from Instruction Register
  input [2:0] nsel; // from Controller

  // to datapath and Controller
  // these are put directly into the Controller / datapath
  output [2:0] opcode,cond;
  output [1:0] op, ALUop, shift;
  output [15:0] sximm5, sximm8;
  

  // these signals are dependent on nsel
  output reg [2:0] readnum, writenum;
  
  output [2:0] Rd;

  wire [2:0] Rn, Rd, Rm;

  // as per specifications, ports of in are decoded into signals fed directly into the Controller / datapath
  assign cond = in[10:8];// condition for lab 8

  assign opcode = in[15:13];
  assign op = in[12:11];
  assign Rn = in[10:8];
  assign Rd = in[7:5];
  assign Rm = in[2:0];
  assign ALUop = in[12:11];
  assign shift = in[4:3]; //shift is sh in Table 1, and also sh_op

  //sximm5: in[4:0] (imm5) is sign-extended to 16 bits based on its MSB
  assign sximm5 = {{11{in[4]}},in[4:0]};
  //sximm8: in[7:0] (imm8) is sign-extended to 16 bits based on its MSB
  assign sximm8 = {{8{in[7]}},in[7:0]};

  always @(*) begin
    case (nsel)
      // chooses which register to connect to Rn, Rd, Rm
      3'b001: begin readnum = Rn; writenum = Rn; end //Rn 
      3'b010: begin readnum = Rd; writenum = Rd; end //Rd
      3'b100: begin readnum = Rm; writenum = Rm; end //Rm
      //-----------------------------------------------lab8
      3'b000: begin readnum = 3'b111; writenum = 3'b111; end //R7
      //-----------------------------------------------lab8
      default: begin readnum = 3'bzzz; writenum = 3'bzzz; end // have set writenum and readnum to zzz so that it can be driven by my program counter
    endcase
  end
endmodule

//Controller FSM
module control (
//inputs
  clk, 
  reset, 
  opcode, //from decoder
  op, //from decoder
  nsel, //to decoder
  asel,
  bsel,
  write,
  vsel, 
  loada,
  loadb,
  loadc,
  loads,
  load_pc,
  reset_pc,
  load_addr,
  addr_sel,
  mem_cmd,
  load_ir,
present); 


  input clk, reset;
  input [2:0] opcode;
  input [1:0] op;
  output [1:0] vsel; 
  //vsel: binary: 00: datapath_out / C, 01: PC, 10: sximm8, 11: mdata
  output [2:0] nsel; //to decoder
  output write, asel, bsel, loada, loadb, loadc, loads;
  output [1:0] mem_cmd;
  output load_pc, reset_pc, load_addr, addr_sel, load_ir;
output [5:0] present;

  `define SW2 6 // for now let's say there are at most 2^5 states...
  `define MEMREAD 11
  `define MEMWRITE 01
  //`define Swait 5'd0      //wait

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
   `define Shalt 6'd21//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  //states:
  // Wait: w = 1, all other states are default; wait until s = 1 to go to Decode (go to this if reset is ever 1).
  // Decode: choose which instruction to make
  // GetB: readnum = Rm (nsel = 100) loadb = 1; load the register Rm to register B
  // GetA: readnum = Rn (nsel = 001), loada = 1; load the register Rn to register A
  // LoadC: asel=bsel=0, loadc=1; put contents of A and B into ALU, write into register C
  // WriteReg: writenum = Rd (nsel = 010), write = 1, vsel = 00; write the value of register C to register Rd
  // WriteImm: writenum = Rn (nsel = 001), vsel = 10; write a sign extended imm8 to register Rn
  // WriteMOVs:

  // CMP: loads = 1.
  // MVN: asel = 1, bsel = 0, loadc =1; put contents of B into shifter

  //present_state is the state we are currently in (updated using DFF)
  //state_next_reset is the state we will go to, (if reset = 1 it's always the Wait state).
  wire [`SW2-1:0] present_state, state_next_reset, state_next;
  //next will set state_next, write, vsel, asel, bsel, loada, loadb, loadc, loads, w, nsel (IN THAT ORDER!)
  reg [(`SW2+12+7)-1:0] next;

  // state DFF for control FSM
  vDFF #(`SW2) STATE(clk, state_next_reset, present_state);
  // reset mux
  assign state_next_reset = reset ? `Reset : state_next;

  //this FSM is a Moore machine, so its current state is dependent on state. The NEXT state is driven by inputs in addition to current state, however.
  //see slide set 7's control for the form we are emulating
  //see Figure 4 for an example
  
  always @(*)
     casex({present_state, opcode, op}) 
     // to see which state we go to, we need to check opcode and op
     // this is a Moore machine, so outputs should be the same for a given state (see states description for more defails)
       {`Reset, 5'bxxxxx}: // next = {`IF1,        12'bz_10_0_0_0_0_0_0_000, 7'bxx_0_0_1_1_0};
                                            if(opcode==3'b010&((op==2'b11)|(op==2'b10)))begin
                                                next = {`IF1,12'b1_01_0_0_0_0_0_0_000, 7'bxx_0_0_1_1_0};// writing pc to r7
                                            end else begin
                                                next = {`IF1,12'b0_01_0_0_0_0_0_0_000, 7'bxx_0_0_1_1_0};
                                            end
       {`IF1, 5'bxxxxx}: // next = {`IF2,          12'b0_10_0_0_0_0_0_0_000, 7'b11_0_0_0_0_1};
                                          if(opcode==3'b010&((op==2'b11)|(op==2'b10)))begin                                              
                                                next = {`IF2,          12'b1_01_0_0_0_0_0_0_000, 7'b11_0_0_0_0_1};// writing pc to r7                                              
                                          end else begin
                                                next = {`IF2,          12'b0_01_0_0_0_0_0_0_000, 7'b11_0_0_0_0_1};
                                          end     // set nsel to xxx so that writenum can be high impedence.

      {`IF2, 5'bxxxxx}:    if(opcode==3'b010&((op==2'b11)|(op==2'b10)))begin
                             next =  {`UpdatePC,     12'b1_01_0_0_0_0_0_0_000, 7'b11_0_1_0_0_1};     // set nsel to xxx so that writenum can be high impedence.   
                              end else begin
                             next =  {`UpdatePC,     12'b0_01_0_0_0_0_0_0_000, 7'b11_0_1_0_0_1};
                              end
	    {`UpdatePC, 5'bxxxxx}: begin // updating pc 
                                                                  if((opcode==3'b001)|(opcode==3'b010))
                                                                   next = {`IF1, 12'b1_01_0_0_0_0_0_0_000, 7'b0_0_1_1_0_0};// THIS ONE WORKSSSS
                                                                  else
                                                                   next = {`Sdecode, 12'b0_01_0_0_0_0_0_0_001, 7'b00_0_0_1_0_0};
                                                                  end                      
                       /* {`Reset, 5'b1xxxx}: next = {`IF1,12'b0_10_0_0_0_0_0_0_000, 7'bxx_0_0_1_1_0};
                        {`Reset, 5'b00xxx}: next = {`IF1,12'b0_10_0_0_0_0_0_0_000, 7'bxx_0_0_1_1_0};

                                          {`Reset, 5'b0101x}: next = {`IF1,12'b1_10_0_0_0_0_0_0_000, 7'bxx_0_0_1_1_0};// writing pc to r7 

                        {`IF1, 5'b1xx_xx}: // next = {`IF2,          12'b0_10_0_0_0_0_0_0_000, 7'b11_0_0_0_0_1};   
                                           next = {`IF2,          12'b0_10_0_0_0_0_0_0_000, 7'b11_0_0_0_0_1}; // set nsel to xxx so that writenum can be high impedence.
                        {`IF1, 5'b00xxx}: // next = {`IF2,          12'b0_10_0_0_0_0_0_0_000, 7'b11_0_0_0_0_1};   
                                           next = {`IF2,          12'b0_10_0_0_0_0_0_0_000, 7'b11_0_0_0_0_1};// set nsel to xxx so that writenum can be high impedence.
                                          {`IF1, 5'b0101x}:  next = {`IF2, 12'b1_01_0_0_0_0_0_0_000, 7'b11_0_0_0_0_1};// set nsel to xxx so that writenum can be high impedence.
                                           
                                          
	                      {`IF2, 5'b1xxxx}:   next = {`UpdatePC,     12'b0_00_0_0_0_0_0_0_001, 7'b11_0_1_0_0_1};
                        {`IF2, 5'b00xxx}:   next = {`UpdatePC,     12'b0_00_0_0_0_0_0_0_001, 7'b11_0_1_0_0_1};
                                           {`IF2, 5'bxxxxx}:  next = {`UpdatePC,     12'b1_01_0_0_0_0_0_0_001, 7'b11_0_1_0_0_1};// set nsel to xxx so that writenum can be high impedence.
                                                                                       
                                            
                      	                                          {`UpdatePC, 5'bxxxxx}: begin // updating pc 
                                                                  if((opcode==3'b001)|(opcode==3'b010))
                                                                   next = {`IF1, 12'b1_00_0_0_0_0_0_0_010, 7'b0_0_1_1_0_0};// THIS ONE WORKSSSS
                                                                  else
                                                                   next = {`Sdecode, 12'b0_00_0_0_0_0_0_0_001, 7'b00_0_0_1_0_0};
                                                                  end  // set nsel to xxx so that writenum can be high impedence. but vsel to 00 to avoid erros   
*/

       {`Sdecode, 5'b11010}: next = {`SwriteImm, 12'b0_00_0_0_0_0_0_0_001, 7'b0}; //Sdecode -> SwriteImm if MOV Rn #<im8>
       {`Sdecode, 5'b11000}: next = {`Swritemove, 12'b0_00_0_0_0_0_0_0_001, 7'b0};//Sdecode -> Swritemove if MOV Rd, Rm{,<sh_op>}
       {`Sdecode, 5'b101x0}: next = {`SgetB, 12'b0_00_0_0_0_0_0_0_001, 7'b0}; //Sdecode -> Swriteadd if ADD Rd,Rn,Rm{,<sh_op>} /OR/ if AND Rd,Rn,Rm{,<sh_op>}
       {`Sdecode, 5'b10101}: next = {`SgetB, 12'b0_00_0_0_0_0_0_0_001, 7'b0};    //Sdecode -> Scmp if CMP Rn,Rm{,<sh_op>} 
       {`Sdecode, 5'b10111}: next = {`SgetB, 12'b0_00_0_0_0_0_0_0_001, 7'b0};      //Sdecode -> Smvn if MVN Rd, Rm{,<sh_op>}
	{`Sdecode, 5'b01100} : next = {`Sload, 12'b0_00_0_0_0_0_0_0_001, 7'b0}; //Sdecode -> SLDR if //LDR Rd, [Rn{,#<im5>}]
	{`Sdecode, 5'b10000} : next = {`Sstore, 12'b0_00_0_0_0_0_0_0_001, 7'b0};
	{`Sdecode, 5'b11100} : next = {`Shalt, 12'b0_00_0_0_0_0_0_0_001, 7'b0};
                                //LDR Rd, [Rn{,#<im5>}]
	{`Sload, 5'b01100}: next = {`SsetAload, 12'b0_10_0_0_0_0_0_0_100, 7'b0}; //Read from reg file
        {`SsetAload, 5'b01100}: next = {`SsetCload, 12'b0_10_0_0_1_0_0_0_001, 7'b00_1_0_0_0_0}; //Load value from Rn to regA
        {`SsetCload, 5'b01100}: next = {`Swriteload, 12'b0_10_0_1_0_0_1_0_001, 7'b11_1_0_0_0_0};  //Compute sixx5 + regA and store in C
	{`Swriteload, 5'b01100}: next = {`Readfrommemory, 12'b0_11_0_0_0_0_0_0_010, 7'b00_1_0_0_0_0};   //Load value to DataAddress reg
	{`Readfrommemory, 5'b01100}: next = {`StoreInRd, 12'b0_11_0_0_0_0_0_0_010, 7'b00_0_0_0_0_0};   //Wait for clock to go high so that mdata is updated
	{`StoreInRd, 5'b01100}: next = {`IF1, 12'b1_11_0_0_0_0_0_0_010, 7'b11_0_0_0_0_0};   //Change vsel to read OUT and store mdata in 010
	
        //REad from memory, read from DataAddress reg, set load of dataAddress to 1

	//STR Rd, [Rn{,#<im5>}]
	{`Sstore, 5'b10000}: next = {`SsetAload, 12'b0_10_0_0_0_0_0_0_001, 7'b0}; //Read from Rd
        {`SsetAload, 5'b10000}: next = {`SsetCload, 12'b0_10_0_0_1_0_0_0_001, 7'b00_0_0_0_0_0}; //Load value from Rn to regA
        {`SsetCload, 5'b10000}: next = {`Swriteload, 12'b0_10_0_1_0_0_1_0_001, 7'b00_0_0_0_0_0};  //Compute sixx5 + regA and store in C
	{`Swriteload, 5'b10000}: next = {`ReadRd, 12'b0_10_0_0_0_0_0_0_010, 7'b00_1_0_0_0_0};   //Reading rd
	{`ReadRd, 5'b10000}: next = {`SgetB, 12'b0_10_0_0_0_0_0_0_010, 7'b00_0_0_0_0_0};   //Reading rd
	{`SgetB, 5'b10000}: next = {`SloadC,  12'b0_10_1_0_0_1_0_0_010, 7'b0};      
	{`SloadC, 5'b10000}: next = {`StoreMem, 12'b0_10_1_0_0_0_1_0_010, 7'b00_0_0_0_0_0};  //
	{`StoreMem, 5'b10000}: next = {`IF1, 12'b0_10_0_0_0_0_0_0_010, 7'b01_0_0_0_0_0};  //
	
	
	
        
	//Write to memory, read from DataAddress reg, set load of dataAddress to 1

	//HALT
	{`Shalt, 5'b11100} : next = {`Shalt, 12'b0_00_0_0_0_0_0_0_001, 7'b0};
   
       //MOV Rn,#<im8>
       {`SwriteImm, 5'b11010}: next = {`IF1, 12'b1_10_0_0_0_0_0_0_001, 7'b0}; //SwriteImm ->Swait after executing instruction

       //MOV Rd, Rm{,<sh_op>
       {`Swritemove, 5'b11000}: next = {`Smove, 12'b0_10_0_0_0_0_0_0_100, 7'b0}; //Swritemove -> Smove
       {`Smove, 5'b11000}: next = {`Smovenext, 12'b0_10_0_0_0_1_0_0_100, 7'b0};  // Smove -> Smovenext
       {`Smovenext, 5'b11000}: next = {`Smovenextnext, 12'b0_10_1_0_0_0_1_0_100, 7'b0}; //Smovenext -> Smovenextnext
       {`Smovenextnext, 5'b11000}: next = {`IF1, 12'b1_00_0_0_0_0_0_0_010, 7'b0}; //Smovenextnext -> Swait

       //ADD Rd,Rn,Rm{,<sh_op>} (same as AND Rd,Rn,Rm{,<sh_op>)
       {`SgetB, 5'b101x0}: next = {`SgetA,  12'b0_10_0_0_0_1_0_0_100, 7'b0};     //SgetB     -> SgetA
       {`SgetA, 5'b101x0}: next = {`SloadC, 12'b0_10_0_0_1_0_0_0_001, 7'b0};      //SgetA     -> Sand
       {`SloadC, 5'b101x0}: next = {`SwriteReg, 12'b0_10_0_0_0_0_1_0_001, 7'b0};  //
       {`SwriteReg, 5'b101x0}: next = {`IF1, 12'b1_00_0_0_0_0_0_0_010, 7'b0};   //

       //CMP Rn, Rm{,<sh_op>}
       {`SgetB, 5'b10101}: next = {`SgetA,  12'b0_10_0_0_0_1_0_0_100, 7'b0};
       {`SgetA, 5'b10101}: next = {`ScMP, 12'b0_10_0_0_1_0_0_0_001, 7'b0};
       {`ScMP, 5'b10101}: next = {`IF1, 12'b0_10_0_0_0_0_0_1_001, 7'b0};
       //MVN Rd, Rm {,<sh_op>}
       {`SgetB, 5'b10111}: next = {`SmVN,  12'b0_10_0_0_0_1_0_0_100, 7'b0};      //SgetB -> SmVN
       {`SmVN, 5'b10111}: next = {`SloadC, 12'b0_10_1_0_0_0_1_0_100, 7'b0};      //SmVN  -> SloadC
       {`SloadC,5'b10111}: next = {`SwriteReg, 12'b0_10_0_0_0_0_1_0_001, 7'b0};  //SloadC -> SwriteReg
       {`SwriteReg, 5'b10111}: next = {`IF1, 12'b1_00_0_0_0_0_0_0_010, 7'b0}; //SwriteReg -> Wait
       
   	default : next = 25'b0;
      // default : next = {{`SW2{1'bx}},{12{1'bx}}, {7{1'bx}}};// only get here if present_state, s, opcode, or op are x's. 
   //default: next = {{`SW2{1'bx}},{13{1'bx}}};
     endcase
  //copy to module outputs
  assign {state_next, write, vsel, asel, bsel, loada, loadb, loadc, loads, nsel} = next[24:7];
  assign {mem_cmd, load_addr, load_ir, load_pc, reset_pc, addr_sel} = next[6:0];
assign present = present_state;
endmodule

// from Slide Set 7

// To ensure Quartus uses the embedded MLAB memory blocks inside the Cyclone
// V on your DE1-SoC we follow the coding style from in Altera's Quartus II
// Handbook (QII5V1 2015.05.04) in Chapter 12, ?Recommended HDL Coding Style?

module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 16; 
  parameter addr_width = 8;
  parameter filename = "data.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @ (posedge clk) begin
    if (write)
      mem[write_address] <= din;
    dout <= mem[read_address]; // dout doesn't get din in this clock cycle 
                               // (this is due to Verilog non-blocking assignment "<=")
  end 
endmodule

module vDFF_PC_DA(clk, load, in, out); // program counter
  parameter n = 9;  // width
  input clk, load;
  input [n-1:0] in;
  output [n-1:0] out;
  reg [n-1:0] out;
  wire [n-1:0] next_out;

  assign next_out = (load ? in : out);

  always @(posedge clk)
    out = next_out;
endmodule 

module programCounter (clk, reset_pc, load_pc, out,op,opcode,sximm8,cond,Rd,Z,N,V,program_wire);
input Z,N,V;
input reset_pc, load_pc, clk;
input [2:0] opcode,cond;
input [1:0] op;
input [15:0]sximm8,program_wire;
input [2:0] Rd;
output [8:0] out;
reg [8:0] next_pc;



always @(*)begin

  case({opcode,op,cond})
  8'b001_00_000: next_pc = reset_pc ? 9'b0 : (out + 1'b1+ sximm8);// b
  8'b001_00_001:begin// beq
                if (Z) begin
                  next_pc = reset_pc ? 9'b0 : (out + 1'b1+ sximm8);
                end else begin
                  next_pc = reset_pc ? 9'b0 : (out + 1'b1);
                end
                end
  8'b001_00_010:begin//bne
                if (Z) begin
                  next_pc = reset_pc ? 9'b0 : (out + 1'b1+ sximm8);
                end else begin
                  next_pc = reset_pc ? 9'b0 : (out + 1'b1);
                end
                end
  8'b001_00_011:begin//blt
                if (N!=V) begin
                  next_pc = reset_pc ? 9'b0 : (out + 1'b1+ sximm8);
                end else begin
                  next_pc = reset_pc ? 9'b0 : (out + 1'b1);
                end
                end

  8'b001_00_100:begin//ble
                if ((N!=V)|Z) begin
                  next_pc = reset_pc ? 9'b0 : (out + 1'b1+ sximm8);
                end else begin
                  next_pc = reset_pc ? 9'b0 : (out + 1'b1);
                end
                end
  8'b010_11_111:next_pc =  reset_pc ? 9'b0 : (out + 1'b1 + sximm8);// BL
                          
                          
  8'b010_00_000: next_pc=program_wire[9:0]+1'b1;//BX<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  8'b010_10_111: next_pc={Rd,5'b0};//BLX

  default: next_pc = reset_pc ? 9'b0 : (out + 1'b1);
  
  endcase
  /*
  if((opcode==3'b001)&(op==2'b00))begin
    if(cond==3'b000) begin
        next_pc = reset_pc ? 9'b0 : (out + 1'b1);
        end
    else begin
        next_pc = reset_pc ? 9'b0 : (out + 1'b1 + sximm8);
    end
  end else if(opcode==3'b010)begin
    if(op==2'b11)begin
      next_pc = reset_pc ? 9'b0 : (out + 1'b1 + sximm8);
    end
    else if (opcode==2'b00) begin
      next_pc={Rd,5'b0};
    end
    else if (opcode==2'b10)begin
      next_pc={Rd,5'b0};
    end
  else begin
    assign next_pc = reset_pc ? 9'b0 : (out + 1'b1);
  end
 end
  */
  end
  
//assign next_pc = reset_pc ? 9'b0 : (out + 1'b1);

vDFF_PC_DA #(9) register (clk, load_pc, next_pc, out);
endmodule



