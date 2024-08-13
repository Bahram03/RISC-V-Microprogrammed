///////////////////////////////////////////////////////////////
// testbench
//
// Expect simulator to print "Simulation succeeded"
// when the value 25 (0x19) is written to address 100 (0x64)
///////////////////////////////////////////////////////////////


module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;
  logic [31:0] hash;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite);
  
  // initialize test
  initial
    begin
      hash <= 0;
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
        if(DataAdr === 100 & WriteData === 25) begin
          $display("Simulation succeeded");
 	   	  $display("hash = %h", hash);
          $stop;
        end else if (DataAdr !== 96) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end

  // Make 32-bit hash of instruction, PC, ALU
  always @(negedge clk)
    if (~reset) begin
      hash = hash ^ dut.rvuprog.dp.Instr ^ dut.rvuprog.dp.PC;
      if (MemWrite) hash = hash ^ WriteData;
      hash = {hash[30:0], hash[9] ^ hash[29] ^ hash[30] ^ hash[31]};
    end

endmodule

///////////////////////////////////////////////////////////////
// top
//
// Instantiates micro-programmed RISC-V processor and memory
///////////////////////////////////////////////////////////////

//hame ya voroodi va khorooji haye nahaeee toosh hastan!

module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);    

  logic [31:0] ReadData;
  
  // instantiate processor and memories   ->> risc v single
  riscvuprog rvuprog(clk, reset, MemWrite, DataAdr, 
                     WriteData, ReadData);
  mem mem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

///////////////////////////////////////////////////////////////
// mem
//
// Single-ported RAM with read and write ports
// Initialized with machine language program
///////////////////////////////////////////////////////////////

module mem(input  logic        clk, we,
           input  logic [31:0] a, wd,
           output logic [31:0] rd);

  logic [31:0] RAM[63:0];
  
  initial
      $readmemh("riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

///////////////////////////////////////////////////////////////
// riscvuprog




// Micro-programmed RISC-V microprocessor
///////////////////////////////////////////////////////////////

module riscvuprog(input  logic        clk, reset,
                  output logic        MemWrite,
                  output logic [31:0] Adr, WriteData,   // ADR ->> 
                  input  logic [31:0] ReadData);

    logic Zero , AdrSrc , IRWrite , PCWrite , RegWrite;
    logic [1:0] ImmSrc , ALUSrcA , ALUSrcB , ResultSrc;
    logic [2:0] AluControl;
    logic [31:0] Instr;

    controller controller (clk , reset , Instr[6:0] , Instr[14:12] , Instr[30] , Zero , ImmSrc , ALUSrcA , ALUSrcB , ResultSrc , AdrSrc , AluControl , IRWrite , PCWrite , RegWrite , MemWrite);

    datapath dp (clk, reset, ResultSrc, RegWrite, ImmSrc, AluControl, Zero, Instr, WriteData, Adr, ReadData);

  // Your code goes here
  // Instantiate controller and datapath 
endmodule

///////////////////////////////////////////////////////////////


//It is a data path

module datapath(input logic clk, reset, //
    input logic [1:0] ResultSrc,
    input logic RegWrite,
    input logic [1:0] ImmSrc,
    input logic [2:0] ALUControl,
    output logic Zero,
    output logic [31:0] Instr,
    output logic [31:0] WriteData,Adr,
    input logic [31:0] ReadData);


    logic [31:0] ImmExt , OldPC;
    logic [31:0] ALUSrcA, ALUSrcB;
    logic [31:0] Result;

    // output logic [31:0] ALUResult, RD1, RD2, WriteData, 


    flopenr #(32) reg_en1(clk, reset, IRWrite ,ReadData , Instr[31:0]); // it is a register

    flopenr #(32) reg_en2(clk, reset, IRWrite , PC , OldPC); // it is a register

    regfile rf(clk, RegWrite, Instr[19:15], Instr[24:20],
                    Instr[11:7], Result, RD1 , RD2); 

    flopr #(32) reg1(clk, reset, RD1 , A);

    flopr #(32) reg2(clk, reset, RD2 , WriteData);

    mux3 #(32) srcA_mux(PC , OldPC , A , // input of multiplexer
                                ALUSrcA, SrcA);  
                                
    extend ext_extend(Instr[31:7], ImmSrc, ImmExt); //instr 
                            

    mux3 #(32) srcB_mux(WriteData , ImmExt, 3'b100 , 
                                ALUSrcB, SrcB);

    alu alu(SrcA, SrcB, ALUControl, ALUResult, Zero);    //It is alu  

    flopr #(32) reg3(clk, reset, ALUResult , ALUOut);

    flopr #(32) reg4(clk, reset, ReadData , Data);

    mux3 #(32) resultmux(ALUOut , Data, ALUResult , //for result
                                ResultSrc, Result);

    mux2 #(32) adrSRC(PC, Result ,adrSrc, Adr); //َADR is output! 

    flopenr #(32) PCRegister(clk, reset, PCWrite , PCNext , PC); // it is a register that saved pc value

endmodule

// they are register with / without enable
module flopenr #(parameter WIDTH = 8)
                (input logic clk, reset, en,
                 input logic [WIDTH-1:0] d,
                 output logic [WIDTH-1:0] q);
    always_ff @(posedge clk, posedge reset)
        if (reset) q <= 0;
        else if (en) q <= d;
endmodule

module flopr #(parameter WIDTH = 8)
              (input logic clk, reset,
               input logic [WIDTH-1:0] d,
               output logic [WIDTH-1:0] q);

    always_ff @(posedge clk, posedge reset)
                if (reset) q <= 0;
                else q <= d;

endmodule

module extend(  input logic [31:7] instr,
                input logic [1:0] immsrc,
                output logic [31:0] immext);
    always_comb begin
        case(immsrc)
            // I−type
            2'b00: immext = {{20{instr[31]}}, instr[31:20]};
            // S−type (stores)
            2'b01: immext = {{20{instr[31]}}, instr[31:25],
                                instr[11:7]};
            // B−type (branches)
            2'b10: immext = {{20{instr[31]}}, instr[7],
                                instr[30:25], instr[11:8], 1'b0};
            // J−type (jal)
            2'b11: immext = {{12{instr[31]}}, instr[19:12],
                                instr[20], instr[30:21], 1'b0};
            default: immext = 32'bx; // undefined
        endcase
    end
endmodule

module alu (input logic [31:0] SrcA, SrcB,
            input logic [2:0] ALUControl,
            output logic [31:0] ALUResult,
            output logic Zero);

    always_comb begin
        case(ALUControl)
            3'b000: ALUResult = SrcA + SrcB;
            3'b001: ALUResult = SrcA - SrcB;
            3'b010: ALUResult = SrcA & SrcB;
            3'b011: ALUResult = SrcA | SrcB;
            3'b101: ALUResult = SrcA < SrcB;
            default: ALUResult = 32'bx;
        endcase
        
    end
    
    assign Zero = ALUResult == 0;
        
endmodule

module imem(input logic [31:0] a,
            output logic [31:0] rd);
    logic [31:0] RAM[63:0];
    
    initial
        $readmemh("D:\\Lessons\\Computer Architecture\\HW\\HW4\\riscvtest.txt",RAM);
    
    assign rd = RAM[a[31:2]]; // word aligned

endmodule

module regfile(input logic clk,
              input logic we3,
              input logic [4:0] a1, a2, a3,
              input logic [31:0] wd3,
              output logic [31:0] rd1, rd2);
    logic [31:0] rf[31:0];
    // three ported register file
    // read two ports combinationally (A1/RD1, A2/RD2)
    // write third port on rising edge of clock (A3/WD3/WE3)
    // register 0 hardwired to 0
    always_ff @(posedge clk)
        if (we3) rf[a3] <= wd3;
    
    assign rd1 = (a1 != 0) ? rf[a1] : 0;
    assign rd2 = (a2 != 0) ? rf[a2] : 0;

endmodule
typedef enum logic[6:0] {r_type_op=7'b0110011, i_type_alu_op=7'b0010011, lw_op=7'b0000011, sw_op=7'b0100011, beq_op=7'b1100011, jal_op=7'b1101111} opcodetype;

module controller(input  logic       clk,
                  input  logic       reset,  
                  input  opcodetype  op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       Zero,
                  output logic [1:0] ImmSrc,
                  output logic [1:0] ALUSrcA, ALUSrcB,
                  output logic [1:0] ResultSrc, 
                  output logic       AdrSrc,
                  output logic [2:0] ALUControl,
                  output logic       IRWrite, PCWrite, 
                  output logic       RegWrite, MemWrite);
    
    logic PCUpdate, Branch, s;
    logic [1:0] ALUOp;
    logic [4:0] CAR, nextCAR, nextAddress;  // Control Address Register
    logic [20:0]  CBR;  // Control Buffer  Register
    // 16 bits Control signals(Replaced ALUControl with ALUOp) + 5 bit branch target address

    always_comb begin
      if(reset) begin
         CAR = 5'b11110; // address of fetch in ROM
      end else begin
         CAR = nextCAR; 
      end
    end
    ROM controlMem(CAR, CBR);
    assign {PCUpdate , Branch, RegWrite, MemWrite, IRWrite, AdrSrc, ResultSrc, ALUSrcA, ALUSrcB, ImmSrc, ALUOp, nextAddress} = CBR;
    aludec ad(op[5], funct3, funct7b5, ALUOp, ALUControl);
    assign s =  op[6:2] == 5'b11111;
    mux2 #(5) decide(nextAddress, op[6:2], s, nextCAR);
    assign PCWrite = (Zero & Branch) | PCUpdate;

endmodule

module ROM (
  input [4:0] address,
  output [20:0] data
);

  reg [20:0] rom [31:0];

  initial begin
    /*
      Fetch   : 30
      Decode  : 31
      MemAdr  : 0 // for lw
      MemAdr  : 8 // for sw
      ExecuteR: 12
      ExecuteI: 4
      JAL     : 27
      BEQ     : 24
      MemRead : 14
      MemWB   : 15
      MemWrite: 28
      ALUWB   :10
    */
    /*
      Fetch   : 11110
      Decode  : 11111
      MemAdr  : 00000 // for lw
      MemAdr  : 01000 // for sw
      ExecuteR: 01100
      ExecuteI: 00100
      JAL     : 11011
      BEQ     : 11000
      MemRead : 01110
      MemWB   : 01111
      MemWrite: 11100
      ALUWB   : 01010
    */
    // format:
    // pcupdate_branch_regwrite_memwrite_irwrite_adrsrc_resultsrc [1:0]_alusrca [1:0]_alusrcb [1:0]_immsrc_[1:0]_aluop[1:0]_Micro-instruction Address [4:0]
    rom[30]  = 21'b0_0_0_0_1_0_00_00_00_00_00_11111; // Fetch    -> Decode
    rom[31]  = 21'b0_0_0_0_0_X_XX_XX_XX_00_00_11111; // Decode   -> unkown
    rom[0]   = 21'b0_0_0_0_0_X_XX_10_01_00_00_01110; // MemAdr for lw -> MemRead
    rom[8]   = 21'b0_0_0_0_0_X_XX_10_01_00_00_11100; // MemAdr for sw -> MemWrite
    rom[12]  = 21'b0_0_0_0_0_X_XX_10_00_XX_10_01010; // ExecuteR -> ALUWB
    rom[4]   = 21'b0_0_0_0_0_X_XX_10_01_XX_10_01010; // ExecuteI -> ALUWB
    rom[27]  = 21'b1_0_0_0_0_X_00_01_10_11_00_01010; // JAL      -> ALUWB
    rom[24]  = 21'b0_1_0_0_0_X_00_10_00_10_01_11110; // BEQ      -> Fetch      
    rom[14]  = 21'b0_0_0_0_0_1_00_XX_XX_00_XX_01111; // MemRead  -> MemWB
    rom[15]  = 21'b0_0_1_0_0_X_01_XX_XX_00_XX_11110; // MemWB    -> Fetch  
    rom[28]  = 21'b0_0_0_1_0_1_00_XX_XX_01_XX_11110; // MemWrite -> Fetch
    rom[10]  = 21'b0_0_1_0_0_X_00_XX_XX_XX_XX_11110; // ALUWB    -> Fetch
  end

  assign data = rom[address];

endmodule


module aludec(  input logic opb5,
                input logic [2:0] funct3,
                input logic funct7b5,
                input logic [1:0] ALUOp,
                output logic [2:0] ALUControl);
    logic RtypeSub;
    assign RtypeSub = funct7b5 & opb5; // TRUE for R–type subtract
    always_comb
        case(ALUOp)
            2'b00: ALUControl = 3'b000; // addition
            2'b01: ALUControl = 3'b001; // subtraction
            default: case(funct3) // R–type or I–type ALU
                        3'b000: if (RtypeSub)
                                    ALUControl = 3'b001; // sub
                                else
                                     ALUControl = 3'b000; // add, addi
                        3'b010: ALUControl = 3'b101; // slt, slti
                        3'b110: ALUControl = 3'b011; // or, ori
                        3'b111: ALUControl = 3'b010; // and, andi
                        default: ALUControl = 3'bxxx; // ???
                    endcase
        endcase
endmodule

///////////////////////////////////////////////////////////////

// Describe your non-leaf cells structurally
// Describe your lef cells (mux, flop, alu, etc.) behaviorally
// Exactly follow the micro-programmed processor diagram
// Remember to declare internal signals
// Be consistent with spelling and capitalization
// Be consistent with order of signals in module declarations and instantiations
