// controller.sv

// Place controller.tv in same computer directory as this file to test your multicycle/micro-programmed controller.


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

module mux2 #(parameter WIDTH = 8)
             (input logic [WIDTH-1:0] d0, d1,
              input logic s,
              output logic [WIDTH-1:0] y);
    
    assign y = s ? d1 : d0;

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


module testbench();

  logic        clk;
  logic        reset;
  
  opcodetype  op;
  logic [2:0] funct3;
  logic       funct7b5;
  logic       Zero;
  logic [1:0] ImmSrc;
  logic [1:0] ALUSrcA, ALUSrcB;
  logic [1:0] ResultSrc;
  logic       AdrSrc;
  logic [2:0] ALUControl;
  logic       IRWrite, PCWrite;
  logic       RegWrite, MemWrite;
  
  logic [31:0] vectornum, errors;
  logic [39:0] testvectors[10000:0];
  
  logic        new_error;
  logic [15:0] expected;
  logic [6:0]  hash;


  // instantiate device to be tested
  controller dut(clk, reset, op, funct3, funct7b5, Zero,
                 ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, ALUControl, IRWrite, PCWrite, RegWrite, MemWrite);
  
  // generate clock
  always 
    begin
      clk = 1; #5; clk = 0; #5;
    end

  // at start of test, load vectors and pulse reset
  initial
    begin
      $readmemb("D:\\Lessons\\Computer Architecture\\Final Project\\controller.tv", testvectors);
      vectornum = 0; errors = 0; hash = 0;
      reset = 1; #22; reset = 0;
    end
	 
  // apply test vectors on rising edge of clk
  always @(posedge clk)
    begin
      #1; {op, funct3, funct7b5, Zero, expected} = testvectors[vectornum];
    end

  // check results on falling edge of clk
  always @(negedge clk)
    if (~reset) begin // skip cycles during reset
      new_error=0; 

      if ((ImmSrc!==expected[15:14])&&(expected[15:14]!==2'bxx))  begin
        $display("   ImmSrc = %b      Expected %b", ImmSrc,     expected[15:14]);
        new_error=1;
      end
      if ((ALUSrcA!==expected[13:12])&&(expected[13:12]!==2'bxx)) begin
        $display("   ALUSrcA = %b     Expected %b", ALUSrcA,    expected[13:12]);
        new_error=1;
      end
      if ((ALUSrcB!==expected[11:10])&&(expected[11:10]!==2'bxx)) begin
        $display("   ALUSrcB = %b     Expected %b", ALUSrcB,    expected[11:10]);
        new_error=1;
      end
      if ((ResultSrc!==expected[9:8])&&(expected[9:8]!==2'bxx))   begin
        $display("   ResultSrc = %b   Expected %b", ResultSrc,  expected[9:8]);
        new_error=1;
      end
      if ((AdrSrc!==expected[7])&&(expected[7]!==1'bx))           begin
        $display("   AdrSrc = %b       Expected %b", AdrSrc,     expected[7]);
        new_error=1;
      end
      if ((ALUControl!==expected[6:4])&&(expected[6:4]!==3'bxxx)) begin
        $display("   ALUControl = %b Expected %b", ALUControl, expected[6:4]);
        new_error=1;
      end
      if ((IRWrite!==expected[3])&&(expected[3]!==1'bx))          begin
        $display("   IRWrite = %b      Expected %b", IRWrite,    expected[3]);
        new_error=1;
      end
      if ((PCWrite!==expected[2])&&(expected[2]!==1'bx))          begin
        $display("   PCWrite = %b      Expected %b", PCWrite,    expected[2]);
        new_error=1;
      end
      if ((RegWrite!==expected[1])&&(expected[1]!==1'bx))         begin
        $display("   RegWrite = %b     Expected %b", RegWrite,   expected[1]);
        new_error=1;
      end
      if ((MemWrite!==expected[0])&&(expected[0]!==1'bx))         begin
        $display("   MemWrite = %b     Expected %b", MemWrite,   expected[0]);
        new_error=1;
      end

      if (new_error) begin
        $display("Error on vector %d: inputs: op = %h funct3 = %h funct7b5 = %h", vectornum, op, funct3, funct7b5);
        errors = errors + 1;
      end
      vectornum = vectornum + 1;
      hash = hash ^ {ImmSrc&{2{expected[15:14]!==2'bxx}}, ALUSrcA&{2{expected[13:12]!==2'bxx}}} ^ {ALUSrcB&{2{expected[11:10]!==2'bxx}}, ResultSrc&{2{expected[9:8]!==2'bxx}}} ^ {AdrSrc&{expected[7]!==1'bx}, ALUControl&{3{expected[6:4]!==3'bxxx}}} ^ {IRWrite&{expected[3]!==1'bx}, PCWrite&{expected[2]!==1'bx}, RegWrite&{expected[1]!==1'bx}, MemWrite&{expected[0]!==1'bx}};
      hash = {hash[5:0], hash[6] ^ hash[5]};
      if (testvectors[vectornum] === 40'bx) begin 
        $display("%d tests completed with %d errors", vectornum, errors);
	      $display("hash = %h", hash);
        $stop;
      end
    end
endmodule

