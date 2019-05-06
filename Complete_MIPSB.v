`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/29/2019 09:38:30 PM
// Design Name: 
// Module Name: Complete_MIPS
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Complete_MIPS(CLK, RST, HALT, SW, B, sseg, an);
  
  input CLK;
  input RST;
  input HALT;
  input [2:0] SW;
  input B;
  output [6:0] sseg;
  output [3:0] an;

  wire CS, WE;
  wire [6:0] ADDR;
  wire [31:0] Mem_Bus;
  wire [6:0] sseg0, sseg1, sseg2, sseg3;
  wire [31:0] r2;
  wire [15:0] num_out;
  wire [3:0] dig3, dig2, dig1, dig0;
  
  binconverter b0 (num_out[13:0],dig3,dig2,dig1,dig0);

  hexto7segment h3 (dig3,sseg3);
  hexto7segment h2 (dig2,sseg2);
  hexto7segment h1 (dig1,sseg1);
  hexto7segment h0 (dig0,sseg0);
  
  displayLogic d0 (CLK,sseg0,sseg1,sseg2,sseg3,an[0],an[1],an[2],an[3],sseg);

  MIPS CPU(CLK, RST, CS, WE, ADDR, Mem_Bus, r2, SW, HALT);
  Memory MEM(CS, WE, CLK, ADDR, Mem_Bus);

  assign num_out = B ? r2[31:16] : r2[15:0];

endmodule

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

module Memory(CS, WE, CLK, ADDR, Mem_Bus);
  input CS;
  input WE;
  input CLK;
  input [6:0] ADDR;
  inout [31:0] Mem_Bus;

  reg [31:0] data_out;
  reg [31:0] RAM [0:127];


  initial
  begin
    RAM[0] = 32'h20C10000;
    RAM[1] = 32'h31080000;
    RAM[2] = 32'h3C047000;
    RAM[3] = 32'h3C057FFF;
    RAM[4] = 32'h3508000B;
    RAM[5] = 32'h10C1FFFF;
    RAM[6] = 32'h20C10000;
    RAM[7] = 32'h00013840;
    RAM[8] = 32'h01073820;
    RAM[9] = 32'h00E00008;
    RAM[10] = 32'h0BFFFFFA;
    RAM[11] = 32'h0C00000B;
    RAM[12] = 32'h0BFFFFF8;
    RAM[13] = 32'h0C00000B;
    RAM[14] = 32'h0BFFFFF6;
    RAM[15] = 32'h0C00000B;
    RAM[16] = 32'h0BFFFFF4;
    RAM[17] = 32'h0C00000B;
    RAM[18] = 32'h0BFFFFF2;
    RAM[19] = 32'h0C00000B;
    RAM[20] = 32'h0BFFFFF0;
    RAM[21] = 32'h0C00000B;
    RAM[22] = 32'h0BFFFFEE;
    RAM[23] = 32'h0085102D;
    RAM[24] = 32'h03E00008;
    RAM[25] = 32'h3C021000;
    RAM[26] = 32'h03E00008;
    RAM[27] = 32'h0045002F;
    RAM[28] = 32'h03E00008;
    RAM[29] = 32'h00440030;
    RAM[30] = 32'h03E00008;
    RAM[31] = 32'h00A51031;
    RAM[32] = 32'h03E00008;
    RAM[33] = 32'h00851032;
    RAM[34] = 32'h03E00008;
  end

  assign Mem_Bus = ((CS == 1'b0) || (WE == 1'b1)) ? 32'bZ : data_out;

  always @(negedge CLK)
  begin

    if((CS == 1'b1) && (WE == 1'b1))
      RAM[ADDR] <= Mem_Bus[31:0];

    data_out <= RAM[ADDR];
  end
endmodule

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

module REG(CLK, RegW, DR, SR1, SR2, Reg_In, ReadReg1, ReadReg2, r2, sw);
  input CLK;
  input RegW;
  input [4:0] DR;
  input [4:0] SR1;
  input [4:0] SR2;
  input [31:0] Reg_In;
  output reg [31:0] ReadReg1;
  output reg [31:0] ReadReg2;
  output [31:0] r2;
  input [2:0] sw;

  reg [31:0] REG [0:31];
  integer i;
  
  assign r2 = REG[2];

  initial begin
    ReadReg1 = 0;
    ReadReg2 = 0;
  end
  
  always@(*) REG[1] = sw;

  always @(posedge CLK)
  begin

    if(RegW == 1'b1)
      REG[DR] <= Reg_In[31:0];

    ReadReg1 <= REG[SR1];
    ReadReg2 <= REG[SR2];
  end
endmodule


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

`define opcode instr[31:26]
`define sr1 instr[25:21]
`define sr2 instr[20:16]
`define f_code instr[5:0]
`define numshift instr[10:6]

module MIPS (CLK, RST, CS, WE, ADDR, Mem_Bus, r2, switch, HALT);
  input CLK, RST;
  output reg CS, WE;
  output [6:0] ADDR;
  inout [31:0] Mem_Bus;
  output [31:0] r2;
  input [2:0] switch;
  input HALT;

  //special instructions (opcode == 000000), values of F code (bits 5-0):
  parameter add = 6'b100000;
  parameter sub = 6'b100010;
  parameter xor1 = 6'b100110;
  parameter and1 = 6'b100100;
  parameter or1 = 6'b100101;
  parameter slt = 6'b101010;
  parameter srl = 6'b000010;
  parameter sll = 6'b000000;
  parameter jr = 6'b001000;
  parameter rbit = 6'b101111;
  parameter rev = 6'b110000;
  parameter add8 = 6'b101101;
  parameter sadd = 6'b110001;
  parameter ssub = 6'b110010;

  //non-special instructions, values of opcodes:
  parameter addi = 6'b001000;
  parameter andi = 6'b001100;
  parameter ori = 6'b001101;
  parameter lw = 6'b100011;
  parameter sw = 6'b101011;
  parameter beq = 6'b000100;
  parameter bne = 6'b000101;
  parameter j = 6'b000010;
  parameter jal = 6'b000011;
  parameter lui = 6'b001111;

  //instruction format
  parameter R = 2'd0;
  parameter I = 2'd1;
  parameter J = 2'd2;

  //internal signals
  reg [5:0] op, opsave;
  wire [1:0] format;
  reg [31:0] instr, alu_result;
  reg [6:0] pc, npc;
  wire [31:0] imm_ext, alu_in_A, alu_in_B, reg_in, readreg1, readreg2;
  reg [31:0] alu_result_save;
  reg alu_or_mem, alu_or_mem_save, regw, writing, reg_or_imm, reg_or_imm_save;
  reg fetchDorI;
  wire [4:0] dr;
  reg [2:0] state, nstate;

  //combinational
  assign imm_ext = (instr[15] == 1)? {16'hFFFF, instr[15:0]} : {16'h0000, instr[15:0]};//Sign extend immediate field
  assign dr = ((`opcode == rbit) || (`opcode == rev)) ? instr[25:21] : 
                                    ((format == R)? instr[15:11] : 
                                    ((`opcode == jal) ? 5'd31 : instr[20:16])); //Destination Register MUX (MUX1)
  assign alu_in_A = readreg1;
  assign alu_in_B = (reg_or_imm_save)? imm_ext : readreg2; //ALU MUX (MUX2)
  assign reg_in = (alu_or_mem_save)? Mem_Bus : ((`opcode == jal) ? pc : ((`opcode == lui) ? (instr[15:0] << 16) : alu_result_save)); //Data MUX
  assign format = (`opcode == 6'd0)? R : ((`opcode == j || `opcode == jal)? J : I);
  assign Mem_Bus = (writing)? readreg2 : 32'bZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ;

  //drive memory bus only during writes
  assign ADDR = (fetchDorI)? pc : alu_result_save[6:0]; //ADDR Mux
  REG Register(CLK, regw, dr, `sr1, `sr2, reg_in, readreg1, readreg2, r2, switch);

  initial begin
    op = and1; opsave = and1;
    state = 3'b0; nstate = 3'b0;
    alu_or_mem = 0;
    regw = 0;
    fetchDorI = 0;
    writing = 0;
    reg_or_imm = 0; reg_or_imm_save = 0;
    alu_or_mem_save = 0;
  end

  integer i;
  
  always @(*)
  begin
    fetchDorI = 0; CS = 0; WE = 0; regw = 0; writing = 0; alu_result = 32'd0;
    npc = pc; op = jr; reg_or_imm = 0; alu_or_mem = 0; nstate = 3'd0;
    case (state)
      0: begin //fetch
        if(HALT) nstate = 3'd0;
        else begin
        npc = pc + 7'd1; CS = 1; nstate = 3'd1; 
        fetchDorI = 1;
        end
      end
      1: begin //decode
        nstate = 3'd2; reg_or_imm = 0; alu_or_mem = 0;
        if (format == J) begin //jump, jal
          npc = instr[6:0];
          if(`opcode == jal) nstate = 3'd2;
            else nstate = 3'd0;
        end
        else if (format == R) //register instructions
          op = `f_code;
        else if (format == I) begin //immediate instructions
          reg_or_imm = 1;
          if(`opcode == lw) begin
            op = add;
            alu_or_mem = 1;
          end
          else if ((`opcode == lw)||(`opcode == sw)||(`opcode == addi)) op = add;
          else if ((`opcode == beq)||(`opcode == bne)) begin
            op = sub;
            reg_or_imm = 0;
          end
          else if (`opcode == andi) op = and1;
          else if (`opcode == ori) op = or1;
        end
      end
      2: begin //execute
        nstate = 3'd3;
        if (opsave == and1) alu_result = alu_in_A & alu_in_B;
        else if (opsave == or1) alu_result = alu_in_A | alu_in_B;
        else if (opsave == add) alu_result = alu_in_A + alu_in_B;
        else if (opsave == sub) alu_result = alu_in_A - alu_in_B;
        else if (opsave == srl) alu_result = alu_in_B >> `numshift;
        else if (opsave == sll) alu_result = alu_in_B << `numshift;
        else if (opsave == slt) alu_result = (alu_in_A < alu_in_B)? 32'd1 : 32'd0;
        else if (opsave == xor1) alu_result = alu_in_A ^ alu_in_B;
        else if(opsave == rbit) begin for(i = 0;i < 32;i=i+1) alu_result[i] = alu_in_B[31-i]; end
        else if(opsave == rev) alu_result = {alu_in_B[7:0],alu_in_B[15:8],alu_in_B[23:16],alu_in_B[31:24]};
        else if(opsave == add8) begin alu_result[31:24] = alu_in_A[31:24] + alu_in_B[31:24]; 
                                      alu_result[23:16] = alu_in_A[23:16] + alu_in_B[23:16];
                                      alu_result[15:8] = alu_in_A[15:8] + alu_in_B[15:8];
                                      alu_result[7:0] = alu_in_A[7:0] + alu_in_B[7:0]; end
        else if(opsave == sadd) alu_result = ((alu_in_A + alu_in_B) > 32'hFFFFFFFF) ? 32'hFFFFFFFF : (alu_in_A + alu_in_B);
        else if(opsave == ssub) alu_result = ((alu_in_A - alu_in_B) < 32'h00000000) ? 32'h00000000 : (alu_in_A - alu_in_B);                                                 
        if (((alu_in_A == alu_in_B)&&(`opcode == beq)) || ((alu_in_A != alu_in_B)&&(`opcode == bne))) begin
          npc = pc + imm_ext[6:0];
          nstate = 3'd0;
        end
        else if ((`opcode == bne)||(`opcode == beq)) nstate = 3'd0;
        else if (opsave == jr) begin
          npc = alu_in_A[6:0];
          nstate = 3'd0;
        end
      end
      3: begin //prepare to write to mem
        nstate = 3'd0;
        if ((format == R)||(`opcode == addi)||(`opcode == andi)||(`opcode == ori)||(`opcode == jal)||(`opcode == lui)) regw = 1;
        else if (`opcode == sw) begin
          CS = 1;
          WE = 1;
          writing = 1;
        end
        else if (`opcode == lw) begin
          CS = 1;
          nstate = 3'd4;
        end
      end
      4: begin
        nstate = 3'd0;
        CS = 1;
        if (`opcode == lw) regw = 1;
      end
    endcase
  end //always

  always @(posedge CLK) begin

    if (RST) begin
      state <= 3'd0;
      pc <= 7'd0;
    end
    else begin
      state <= nstate;
      pc <= npc;
    end

    if (state == 3'd0) instr <= Mem_Bus;
    else if (state == 3'd1) begin
      opsave <= op;
      reg_or_imm_save <= reg_or_imm;
      alu_or_mem_save <= alu_or_mem;
    end
    else if (state == 3'd2) alu_result_save <= alu_result;

  end //always

endmodule

//converts binary number to 4 seperate digits
module binconverter(
    input [13:0] in,
    output [3:0] out3, 
    inout [3:0] out2, out1, out0
    );
    
    assign out0 = in%10;
    assign out1 = ((in%100) - out0)/10;
    assign out2 = ((in%1000) - (out1*10) - out0)/100;
    assign out3 = (in - (out2*100) - (out1*10) - out0)/1000;    
endmodule


//send a hex value, returns seven segment
module hexto7segment(
    input [3:0] x,
    output reg [6:0] r
    );
    always@(*)
        case(x)
            4'b0000 : r = 7'b1000000;
            4'b0001 : r = 7'b1111001;
            4'b0010 : r = 7'b0100100;
            4'b0011 : r = 7'b0110000;
            4'b0100 : r = 7'b0011001;
            4'b0101 : r = 7'b0010010;
            4'b0110 : r = 7'b0000010;
            4'b0111 : r = 7'b1111000;
            4'b1000 : r = 7'b0000000;
            4'b1001 : r = 7'b0010000;
            4'b1010 : r = 7'b0001000;
            4'b1011 : r = 7'b0000011;
            4'b1100 : r = 7'b1000110;
            4'b1101 : r = 7'b0100001;
            4'b1110 : r = 7'b0000110;
            4'b1111 : r = 7'b0001110;
        endcase   
endmodule


//rotates 4 digits on 4 seven segment displays
module displayLogic(
    input clk,
    input [6:0] sseg0, sseg1, sseg2, sseg3,
    output reg an0, an1, an2, an3, 
    output reg [6:0] sseg
    );
    reg [1:0] state, next_state;
    reg [9:0] counter;
    initial begin
        state = 2'b00;
        counter = 0;
    end 
    
    always@(*) begin
    case(state)
        2'b00 : begin {an3, an2, an1, an0} = 4'b1110; next_state = 2'b01; sseg = sseg0; end
        2'b01 : begin {an3, an2, an1, an0} = 4'b1101; next_state = 2'b10; sseg = sseg1; end
        2'b10 : begin {an3, an2, an1, an0} = 4'b1011; next_state = 2'b11; sseg = sseg2; end
        2'b11 : begin {an3, an2, an1, an0} = 4'b0111; next_state = 2'b00; sseg = sseg3; end
        endcase
    end
    
    always@(posedge clk) begin        
        if(counter == 999) begin
        state <= next_state;
        counter <= 0;
        end else counter <= counter + 1;
        end              
endmodule