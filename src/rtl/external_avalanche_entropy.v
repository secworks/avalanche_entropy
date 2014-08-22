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
                                  input wire           clk,
                                  input wire           reset_n,

                                  input wire           cs,
                                  input wire           we,
                                  input wire  [7 : 0]  address,
                                  input wire  [31 : 0] write_data,
                                  output wire [31 : 0] read_data,
                                  output wire          error,

                                  input wire           noise,
                                  output wire [7 : 0]  debug
                                  );

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG_RATE = 32'h00300000;
  
  
  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg noise_sample0_reg;
  reg noise_sample_reg;

  reg flank0_reg;
  reg flank1_reg;

  reg [31 : 0] cycle_ctr_reg;

  reg [31 : 0] entropy_reg;
  reg [31 : 0] entropy_new;
  reg          entropy_we;
  
  reg [31 : 0] debug_ctr_reg;
  reg [31 : 0] debug_ctr_new;
  reg          debug_ctr_we;
  
  reg [7 : 0] debug_reg;
  reg [7 : 0] debug_new;
  reg         debug_we;

  
  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign debug = debug_reg;
  
  
  //----------------------------------------------------------------
  // reg_update
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin
      if (!reset_n)
        begin
          noise_sample0_reg <= 1'b0;
          noise_sample_reg  <= 1'b0;
          flank0_reg        <= 1'b0;
          flank1_reg        <= 1'b0;
          cycle_ctr_reg     <= 32'h00000000;
          debug_ctr_reg     <= 32'h00000000;
          entropy_reg       <= 32'h00000000;
          debug_reg         <= 8'h00;
        end
      else
        begin
          noise_sample0_reg <= noise;
          noise_sample_reg  <= noise_sample0_reg;

          flank0_reg <= noise_sample_reg;
          flank1_reg <= flank0_reg;

          cycle_ctr_reg <= cycle_ctr_reg + 1'b1;
          debug_ctr_reg <= debug_ctr_new;
          
          if (entropy_we)
            begin
              entropy_reg <= entropy_new;
            end

          if (debug_we)
            begin
              debug_reg <= debug_new;
            end
        end
    end // reg_update


  //----------------------------------------------------------------
  // entropy_collect
  //
  // This is where we collect entropy by adding the LSB of the
  // cycle counter to the entropy shift register every time
  // we detect a positive flank at the noise source.
  //----------------------------------------------------------------
  always @*
    begin : entropy_collect
      entropy_new = 32'h00000000;
      entropy_we  = 1'b0;

      // Update the entropy shift register every positive flank.
      if ((flank0_reg) && (!flank1_reg))
        begin
          entropy_new = {entropy_reg[30 : 0], cycle_ctr_reg[0]};
          entropy_we  = 1'b1;
        end
    end // entropy_collect
  

  //----------------------------------------------------------------
  // debug_update
  //
  // Sample the entropy register as debug value at the given
  // DEBUG_RATE.
  //----------------------------------------------------------------
  always @*
    begin : debug_update
      debug_ctr_new = debug_ctr_reg + 1'b1;
      debug_new     = entropy_reg[7 : 0];
      debug_we      = 1'b0;

      if (debug_ctr_reg == DEBUG_RATE)
        begin
          debug_ctr_new = 32'h00000000;
          debug_we      = 1'b1;
        end
    end // debug_update
  
endmodule // external_avalanche_entropy

//======================================================================
// EOF external_avalanche_entropy.v
//======================================================================
