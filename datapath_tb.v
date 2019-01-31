module datapath_tb;
  reg clk;
  reg [15:0] mdata, sximm5, sximm8;
  reg write, loada, loadb, asel, bsel, loadc, loads;
  reg [1:0] vsel;
  reg [2:0] readnum, writenum;
  reg [1:0] shift, ALUop;
  reg [7:0] PC;

  wire [15:0] datapath_out;
  wire [2:0] Z_out;

  reg err;

  datapath DUT ( .clk         (clk),
    // register operand fetch stage
                .readnum     (readnum),
                .vsel        (vsel),
                .loada       (loada),
                .loadb       (loadb),

                // computation stage (sometimes called "execute")
                .shift       (shift),
                .asel        (asel),
                .bsel        (bsel),
                .ALUop       (ALUop),
                .loadc       (loadc),
                .loads       (loads),
		.sximm5	     (sximm5),

                // set when "writing back" to register file
                .writenum    (writenum),
                .write       (write),  
                .mdata 	     (mdata),
		.sximm8      (sximm8),
		.PC 	     (PC),

                // outputs
                .Z_out       (Z_out),
                .datapath_out(datapath_out)
             );

  // for debugging purposes, we want to know the values of the registers
  // using the following statements (wire used as follows acts like assign)
  wire [15:0] R0 = DUT.REGFILE.R0;
  wire [15:0] R1 = DUT.REGFILE.R1;
  wire [15:0] R2 = DUT.REGFILE.R2;
  wire [15:0] R3 = DUT.REGFILE.R3;
  wire [15:0] R4 = DUT.REGFILE.R4;
  wire [15:0] R5 = DUT.REGFILE.R5;
  wire [15:0] R6 = DUT.REGFILE.R6;
  wire [15:0] R7 = DUT.REGFILE.R7;
 
  // The first initial block below generates the clock signal. The clock (clk)
  // starts with value 0, changes to 1 after 5 time units and changes again 0
  // after 10 time units.  This repeats "forever".  Rising edges of clk are at
  // time = 5, 15, 25, 35, ...  
  initial forever begin
    clk = 0; #5;
    clk = 1; #5;
  end

  initial begin
    // Plot err in your waveform to find out when first error occurs
    err = 0;
    
    // IMPORTANT: Set all control inputs to something at time=0 so not "undefined"
    //datapath_in = 0;
    mdata = 0; sximm5=0; sximm8=0; PC =0;
    write = 0; vsel=0; loada=0; loadb=0; asel=0; bsel=0; loadc=0; loads=0;
    readnum = 0; writenum=0;
    shift = 0; ALUop=0;

    // Now, wait for clk -- clock rises at time = 5, 15, 25, 35, ...  Thus, at 
    // time = 10 the clock is NOT rising so it is safe to change the inputs.
    #10; 
/*
    ////////////////////////////////////////////////////////////
    //First set of instructions:
    //MOV R0, #7 // this means, take the absolute number 7 and store it in R0
    //MOV R1, #2 // this means, take the absolute number 2 and store it in R1
    //ADD R2, R1, R0, LSL#1 // this means R2 = R1 + (R0 shifted left by 1) = 2+14=16
    //Encodings:
    //1101000000000111
    opcode =3'b110; op=2'b10; Rn=3'b000;im8=8'b00000111;#10;
    //1101000100000010
    opcode =3'b110; op=2'b10; Rn=3'b001;im8=8'b00000010;#10;
    //1010000101001000
    opcode =3'b101; ALUop=2'b00; Rn = 3'b001; Rd=3'b010; sh = 2'b01; Rm = 3'b000; #10;

    if (datapath_out !== 16'd16) begin 
      err = 1; 
      $display("FAILED: ADD R2, R1, R0, LSL#1 -- datapath_out=%h is wrong, expected %h", R2, 16'd16); 
      $stop; 
    end

    //Second set of instructions:
    //MOV R0, #7 // this means, take the absolute number 7 and store it in R0
    //MOV R1, #2 // this means, take the absolute number 2 and store it in R1
    //CMP R1, R0, LSL#1 // this means {Z,V,N} = R1 - (R0 shifted left by 1)=2-14=-12=001
    //Encodings:
    //1101000000000111
    opcode =3'b110; op=2'b10; Rn=3'b000;im8=8'b00000111;#10;
    //1101000100000010
    opcode =3'b110; op=2'b10; Rn=3'b001;im8=8'b00000010;#10;
    //1010100100001000
    opcode =3'b101; ALUop=2'b01; Rn = 3'b001; Rd=3'b000; sh = 2'b01; Rm = 3'b000; #10;

    if ({Z,V,N} !== 3'b001) begin 
      err = 1; 
      $display("FAILED: CMP R1, R0, LSL#1  -- {Z,V,N}=%h is wrong, expected %h", {Z,V,N}, 3'b001); 
      $stop; 
    end

    //Third set of instructions:
    //MOV R0, #7 // this means, take the absolute number 7 and store it in R0
    //MOV R1, #2 // this means, take the absolute number 2 and store it in R1
    //AND R2, R1, R0, LSL#1 // this means R2 = R1 & (R0 shifted left by 1) = 2&14=00000010&11111001=00000000
    //Encodings:
    //1101000000000111
    opcode =3'b110; op=2'b10; Rn=3'b000;im8=8'b00000111;#10;
    //1101000100000010
    opcode =3'b110; op=2'b10; Rn=3'b001;im8=8'b00000010;#10;
    //1011000101001000
    opcode =3'b101; ALUop=2'b01; Rn = 3'b001; Rd=3'b000; sh = 2'b01; Rm = 3'b000; #10;

    if (R2 !== 16'd0) begin
      err = 1; 
      $display("FAILED: AND R2, R1, R0, LSL#1 -- Regs[R2]=%h is wrong, expected %h", R2, 16'd0); 
      $stop; 
    end

    //Fourth set of instructions:
    //MOV R0, #7 // this means, take the absolute number 7 and store it in R0
    //MOV R1, #2 // this means, take the absolute number 2 and store it in R1
    //MVN R2, R0, LSL#1 // this means R2 = ~(R0 shifted left by 1) = ~14=~(11111001)=00000110
    //Encodings:
    //1101000000000111
    opcode =3'b110; op=2'b10; Rn=3'b000;im8=8'b00000111;#10;
    //1101000100000010
    opcode =3'b110; op=2'b10; Rn=3'b001;im8=8'b00000010;#10;
    //1010000101001000
    opcode =3'b101; ALUop=2'b01; Rn = 3'b001; Rd=3'b000; sh = 2'b01; Rm = 3'b000; #10;

    if (R2 !== 16'd6) begin
      err = 1; 
      $display("FAILED: MVN R2, R0, LSL#1 -- Regs[R2]=%h is wrong, expected %h", R2, 16'd6); 
      $stop; 
    end

    //MOV R1, #7 // this means, take the absolute number 7 and store it in R0
    //Check MOV R2, R1, LSL#1 //this means R2 = R1 shifted left by 1 = 14
    //Encodings:
    //1101000000000111
    opcode =3'b110; op=2'b10; Rn=3'b000;im8=8'b00000111;#10;
    //1100000001001001
    opcode =3'b101; ALUop=2'b01; Rn = 3'b001; Rd=3'b000; sh = 2'b01; Rm = 3'b001; #10;

    if (R2 !== 16'd14) begin
      err = 1; 
      $display("FAILED: MOV R2, R1, LSL#1 -- Regs[R2]=%h is wrong, expected %h", R2, 16'd14); 
      $stop; 
    end*/

    //MOV R0, #7
    sximm8 = 16'b0000000000000111;
    writenum = 3'b0; //R0 is the register we want to write to.
    write = 1'b1; //we want to write, so write should 1
    vsel = 2'b10; //we want to feed datapath_in through the mux, so vsel = 1.
    #10; // wait for clock

    //checks if MOV is executed correctly
    if (R0 !== 16'b0000000000000111) begin
      err = 1;
      $display("FAILED: MOV R0, #7 is wrong -- Regs[R0]=%h is wrong, expected %h", R0, 16'b0000000000000111);
    end
    ////////////////////////////////////////////////////////////

    // MOV R1, #2
    sximm8 = 16'b0000000000000010;
    writenum = 3'b001;
    write = 1'b1;
    vsel = 2'b10;
    #10; //wait for clock
    write = 0; // done writing, remember to set write to zero

    //checks if MOV is executed correctly
    if (R1 !== 16'b0000000000000010) begin 
       err = 1; 
       $display("FAILED: MOV R1, #2 wrong -- Regs[R1]=%h is wrong, expected %h", R1, 16'b0000000000000010); 
     
    end
    ////////////////////////////////////////////////////////////

    // AND R2, R1, R0, LSL#1 // this means R2 = R1 & (R0 shifted left by 1) = 2&14=00000010&00001110=00000010
    // step 1 - load contents of R0 into B reg
    readnum = 3'b000; //reading from R0
    loadb = 1'b1;
    #10; //wait for clock
    loadb = 1'b0; //done loading B, set loadb to zero so don't overwrite A

    // step 2 - load contents of R1 into A reg
    readnum = 3'b001;
    loada = 1'b1;
    #10; //wait for clock
    loada = 1'b0;

    // step 3 - perform addition of contents A and B registers (with B register shifted), load into C
    shift = 2'b01; // shift over 1 bit to the left.
    sximm5 = {11'b0,sximm8[4:0]};
    asel = 1'b0;
    bsel = 1'b0;
    ALUop = 2'b10;
    loadc = 1'b1;
    loads = 1'b1;
    #10; // wait for clock
    loadc = 1'b0; //reset load signals
    loads = 1'b0;

    //step 4 - store contents of C into R2
    write = 1'b1;
    writenum = 3'b010;
    vsel = 2'b00; //we're feeding datapath_out back into the mux so we can write to the register
    #10;
    write = 0;

    if (R2 !== 16'b0000000000000010) begin //2&14=2
      err = 1; 
      $display("FAILED:AND R2, R1, R0, LSL#1 -- Regs[R2]=%h is wrong, expected %h", R2, 16'b0000000000000010); 
     
    end

    if (datapath_out !== 16'b0000000000000010) begin 
      err = 1; 
      $display("FAILED: AND R2, R1, R0, LSL#1 -- datapath_out=%h is wrong, expected %h", R2, 16'b0000000000000010); 

    end

    if (Z_out !== 3'b000) begin
      err = 1; 
      $display("FAILED: AND R2, R1, R0, LSL#1 -- Z_out=%b is wrong, expected %b", Z_out, 3'b000); 

    end

    // ADD R2, R1, R0, LSL#1 ; this means R2 = R1 + (R0 shifted left by 1) = 2+14=16
    // step 1 - load contents of R0 into B reg
    readnum = 3'b0; //reading from R0
    loadb = 1'b1;
    #10; //wait for clock
    loadb = 1'b0; //done loading B, set loadb to zero so don't overwrite A

    // step 2 - load contents of R1 into A reg
    readnum = 3'b001;
    loada = 1'b1;
    #10; //wait for clock
    loada = 1'b0;

    // step 3 - perform addition of contents A and B registers (with B register shifted), load into C
    shift = 2'b01; // shift over 1 bit to the left.
    asel = 1'b0;
    bsel = 1'b0;
    ALUop = 2'b00;
    loadc =1'b1;
    loads = 1'b1;
    #10; // wait for clock
    loadc = 1'b0; //reset load signals
    loads = 1'b0;

    //step 4 - store contents of C into R2
    write = 1'b1;
    writenum = 3'b010;
    vsel = 2'b00; //we're feeding datapath_out back into the mux so we can write to the register
    #10;
    write = 0;

    if (R2 !== 16'd16) begin //2+14=16
      err = 1; 
      $display("FAILED: ADD R2, R1, R0, LSL#1 -- Regs[R2]=%h is wrong, expected %h", R2, 16'd16); 
  
    end

    if (datapath_out !== 16'b0000000000010000) begin 
      err = 1; 
      $display("FAILED: ADD R2, R1, R0, LSL#1 -- datapath_out=%h is wrong, expected %h", R2, 16'b0000000000010000); 

    end

    if (Z_out !== 3'b000) begin
      err = 1; 
      $display("FAILED: ADD R2, R1, R0, LSL#1 -- Z_out=%b is wrong, expected %b", Z_out, 3'b000); 

    end

    // CMP R1,R0{LSL#1} ; this means status = f(R1 - (R0 shifted left by 1)) = f(2-14)=010
    // step 1 - load contents of R0 into B reg
    readnum = 3'b0; //reading from R0
    loadb = 1'b1;
    #10; //wait for clock
    loadb = 1'b0; //done loading B, set loadb to zero so don't overwrite A

    // step 2 - load contents of R1 into A reg
    readnum = 3'b001;
    loada = 1'b1;
    #10; //wait for clock
    loada = 1'b0;

    // step 3 - CMP
    shift = 2'b01; // shift over 1 bit to the left.
    asel = 1'b0;
    bsel = 1'b0;
    ALUop = 2'b01;
    loadc = 1'b1;
    loads = 1'b1;
    #10; // wait for clock
    loadc = 1'b0; //reset load signals
    loads = 1'b0;


    if (Z_out !== 3'b001) begin
      err = 1; 
      $display("FAILED: CMP R1,R0{LSL#1} -- Z_out=%b is wrong, expected %b", Z_out, 3'b001); 

    end

    // MVN R2,R0{LSL#1} ; this means R2 = ~(R0 shifted left by 1) = ~14
    // step 1 - load contents of R0 into B reg
    readnum = 3'b0; //reading from R0
    loadb = 1'b1;
    #10; //wait for clock
    loadb = 1'b0; //done loading B, set loadb to zero so don't overwrite A


    // step 3 - perform addition of contents A and B registers (with B register shifted), load into C
    shift = 2'b01; // shift over 1 bit to the left.
    asel = 1'b1;
    bsel = 1'b0;
    ALUop = 2'b11;
    loadc = 1'b1;
    loads = 1'b1;
    #10; // wait for clock
    loadc = 1'b0; //reset load signals
    loads = 1'b0;

    //step 4 - store contents of C into R2
    write = 1'b1;
    writenum = 3'b010;
    vsel = 2'b00; //we're feeding datapath_out back into the mux so we can write to the register
    #10;
    write = 0;

    if (R2 !== 16'b1111111111110001) begin //~14
      err = 1; 
      $display("FAILED: MVN R2,R0{LSL#1} -- Regs[R2]=%h is wrong, expected %h", R2, 16'b1111111111110001); 
    end


    if (err === 0) begin
      $display("PASSED.");
    end 

    $stop;
  end
endmodule
