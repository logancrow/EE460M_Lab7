`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/29/2019 09:40:30 PM
// Design Name: 
// Module Name: MIPS_Testbench
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


// You can use this skeleton testbench code, the textbook testbench code, or your own
module MIPS_Testbench ();
  reg CLK;
  reg RST;
  reg HALT;
  wire [7:0] r1;

  initial
  begin
    CLK = 0;
  end

  Complete_MIPS u1(CLK,RST,HALT,r1);

  always
  begin
    #5 CLK = !CLK;
  end

  initial
  begin
    #5 RST = 1'b1; //reset the processor

    //Notice that the memory is initialize in the in the memory module not here


    // driving reset low here puts processor in normal operating mode
    #5 RST = 1'b0;
    #5 HALT = 1'b0;

    /* add your testing code here */
    // you can add in a 'Halt' signal here as well to test Halt operation
    // you will be verifying your program operation using the
    // waveform viewer and/or self-checking operations


  end

endmodule

