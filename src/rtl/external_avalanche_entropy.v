//======================================================================
//
// external_avalanche_entropy.v
// ----------------------------
// Entropy provider for an external entropy source based on
// avalanche noise. (or any other source that ca toggle a single
// bit input).
//
// Currently the design consists of a free running counter. At every
// positive flank detected the LSB of the counter is pushed into
// a 32bit shift register.
//
// Author: Joachim Strombergson
// Copyright (c) 2013, 2014, Secworks Sweden AB
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or 
// without modification, are permitted provided that the following 
// conditions are met: 
// 
// 1. Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer. 
// 
// 2. Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in 
//    the documentation and/or other materials provided with the 
//    distribution. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module external_avalanche_entropy(
                                  input wire          clk,
                                  input wire          reset_n,

                                  input wire          entropy,
  
                                  output wire [7 : 0] debug
                                  );

  
  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg entropy_sample0_reg;
  reg entropy_sample1_reg;

  reg entropy_flank0_reg;
  reg entropy flang1_reg;

  reg [31 : 0] cycle_ctr_reg;

  reg [31 : 0] entropy_reg;
  
  
  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign debug = entropy_reg[7 : 0];
  
  
  //----------------------------------------------------------------
  // reg_update
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin
      if (!reset_n)
        begin
          entropy_sample0_reg <= 32'h00000000;
          entropy_sample1_reg <= 32'h00000000;
          entropy_flank0_reg  <= 32'h00000000;
          entropy flang1_reg  <= 32'h00000000;

          cycle_ctr_reg       <= 32'h00000000;
          
          entropy_reg         <= 32'h00000000;
        end
      else
        begin
          // Input register just to lock the external data.
          entropy_sample0_reg <= entropy;
          entropy_sample1_reg <= entropy_sample0_reg;

          // Flank detect registes. Could be done with the sample regs.
          entropy_flank0_reg <= entropy_sample1_reg;
          entropy flang1_reg <= entropy_flank0_reg;

          // Free running cycle counter.
          cycle_ctr_reg <= cycle_ctr_reg + 1'b1;
          
          // Shift register for entropy collection.
          if ((!flang1_reg) and (flang0_reg))
            begin
              entropy_reg <= {entropy_reg[30 : 0], cycle_ctr_reg[0]};
            end
        end
    end // reg_update

endmodule // external_avalanche_entropy

//======================================================================
// EOF external_avalanche_entropy.v
//======================================================================
