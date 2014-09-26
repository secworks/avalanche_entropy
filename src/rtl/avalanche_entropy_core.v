//======================================================================
//
// avalanche_entropy_core.v
// ------------------------
// Entropy provider core for an external entropy source based on
// avalanche noise. (or any other source that ca toggle a single
// bit input).
//
// Currently the design consists of a free running counter. At every
// positive flank detected the LSB of the counter is pushed into
// a 32bit shift register.
//
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

module avalanche_entropy_core(
                              input wire           clk,
                              input wire           reset_n,

                              input wire           noise,
                              output wire          sampled_noise,
                              output wire          entropy,

                              input wire           entropy_ack,
                              output wire          entropy_syn,
                              output wire [31 : 0] entropy_data,

                              output wire [7 : 0]  led,
                              output wire [7 : 0]  debug_data,
                              output wire          debug_clk,

                              output wire [31 : 0] delta_data,
                              output wire          delta_clk
                             );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter ADDR_STATUS      = 8'h00;
  parameter ADDR_ENTROPY     = 8'h10;
  parameter ADDR_DELTA       = 8'h20;

  parameter DEBUG_DELAY  = 32'h002c4b40;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg          noise_sample0_reg;
  reg          noise_sample_reg;

  reg          flank0_reg;
  reg          flank1_reg;

  reg          entropy_bit_reg;

  reg [31 : 0] entropy_reg;
  reg [31 : 0] entropy_new;
  reg          entropy_we;

  reg          entropy_syn_reg;
  reg          entropy_syn_new;

  reg [5 :  0] bit_ctr_reg;
  reg [5 :  0] bit_ctr_new;
  reg          bit_ctr_inc;
  reg          bit_ctr_we;

  reg [31 : 0] cycle_ctr_reg;
  reg [31 : 0] cycle_ctr_new;

  reg [31 : 0] delta_reg;
  reg          delta_we;

  reg          delta_clk_reg;
  reg          delta_clk_new;

  reg [31 : 0] debug_delay_ctr_reg;
  reg [31 : 0] debug_delay_ctr_new;
  reg          debug_delay_ctr_we;

  reg [7 : 0]  debug_reg;
  reg          debug_we;

  reg          debug_update_reg;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign entropy_syn = entropy_syn_reg;
  assign entropy_data  = entropy_reg;

  assign sampled_noise = noise_sample_reg;
  assign entropy       = entropy_reg[0];

  assign delta_data    = delta_reg;
  assign delta_clk     = delta_clk_reg;

  assign debug         = debug_reg;


  //----------------------------------------------------------------
  // reg_update
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin
      if (!reset_n)
        begin
          noise_sample0_reg   <= 1'b0;
          noise_sample_reg    <= 1'b0;
          flank0_reg          <= 1'b0;
          flank1_reg          <= 1'b0;
          entropy_syn_reg   <= 1'b0;
          entropy_reg         <= 32'h00000000;
          entropy_bit_reg     <= 1'b0;
          bit_ctr_reg         <= 6'h00;
          debug_ctr_reg       <= 4'h0;
          debug_clk_reg       <= 1'b0;
          cycle_ctr_reg       <= 32'h00000000;
          delta_reg           <= 32'h00000000;
          delta_clk_reg       <= 1'b0;
          debug_delay_ctr_reg <= 32'h00000000;
          debug_reg           <= 8'h00;
          debug_update_reg    <= 0;
        end
      else
        begin
          noise_sample0_reg <= noise;
          noise_sample_reg  <= noise_sample0_reg;

          flank0_reg        <= noise_sample_reg;
          flank1_reg        <= flank0_reg;

          entropy_syn_reg   <= entropy_syn_new;

          entropy_bit_reg   <= ~entropy_bit_reg;

          delta_clk_reg     <= delta_clk_new;
          cycle_ctr_reg     <= cycle_ctr_new;

          debug_update_reg  <= debug_update;

          if (delta_we)
            begin
              delta_reg <= cycle_ctr_reg;
            end

          if (bit_ctr_we)
            begin
              bit_ctr_reg <= bit_ctr_new;
            end

          if (debug_ctr_we)
            begin
              debug_ctr_reg <= debug_ctr_new;
            end

          if (entropy_we)
            begin
              entropy_reg <= entropy_new;
            end

          if (debug_delay_ctr_we)
            begin
              debug_delay_ctr_reg <= debug_delay_ctr_new;
            end

          if (debug_we)
            begin
              debug_reg <= ent_shift_reg[7 : 0];
            end
        end
    end // reg_update


  //----------------------------------------------------------------
  // debug_out
  //
  // Logic that updates the debug port.
  //----------------------------------------------------------------
  always @*
    begin : debug_out
      debug_delay_ctr_new = 32'h00000000;
      debug_delay_ctr_we  = 0;
      debug_we            = 0;

      if (debug_update_reg)
        begin
          debug_delay_ctr_new = debug_delay_ctr_reg + 1'b1;
          debug_delay_ctr_we  = 1;
        end

      if (debug_delay_ctr_reg == DEBUG_DELAY)
        begin
          debug_delay_ctr_new = 32'h00000000;
          debug_delay_ctr_we  = 1;
          debug_we            = 1;
        end
    end


  //----------------------------------------------------------------
  // entropy_collect
  //
  // We collect entropy by adding the current state of the
  // entropy bit register the entropy shift register every time
  // we detect a positive flank in the noise source.
  //----------------------------------------------------------------
  always @*
    begin : entropy_collect
      entropy_new   = 32'h00000000;
      entropy_we    = 1'b0;
      bit_ctr_inc   = 1'b0;
      debug_ctr_inc = 1'b0;

      if ((flank0_reg) && (!flank1_reg))
        begin
          entropy_new   = {entropy_reg[30 : 0], entropy_bit_reg};
          entropy_we    = 1'b1;
          bit_ctr_inc   = 1'b1;
          debug_ctr_inc = 1'b1;
        end
    end // entropy_collect


  //----------------------------------------------------------------
  // delta_logic
  //
  // The logic implements the delta time measuerment system.
  //----------------------------------------------------------------
  always @*
    begin : delta_logic
      cycle_ctr_new      = cycle_ctr_reg + 1'b1;
      delta_clk_new      = 1'b0;
      delta_we           = 1'b0;

      if ((flank0_reg) && (!flank1_reg))
        begin
          cycle_ctr_new = 32'h00000000;
          delta_clk_new = 1'b1;
          delta_we      = 1'b1;
        end
    end // delta_logic


  //----------------------------------------------------------------
  // debug_ctr_logic
  //
  // The logic implements the counter needed to handle detection
  // that enough bis has been generated to output debug values.
  //----------------------------------------------------------------
  always @*
    begin : debug_ctr_logic
      debug_ctr_new = 4'h0;
      debug_ctr_we  = 0;
      debug_clk_new = 0;

      if (debug_ctr_reg == 4'h8)
        begin
          debug_ctr_new = 4'h0;
          debug_ctr_we  = 1;
          debug_clk_new = 1;
        end
      else if (debug_ctr_inc)
        begin
          debug_ctr_new = debug_ctr_reg + 1'b1;
          debug_ctr_we  = 1;
        end
      end // debug_ctr_logic


  //----------------------------------------------------------------
  // entropy_ack_logic
  //
  // The logic needed to handle detection that entropy has been
  // read and ensure that we collect more than 32 entropy
  // bits beforeproviding more entropy.
  //----------------------------------------------------------------
  always @*
    begin : entropy_ack_logic
      bit_ctr_new       = 6'h00;
      bit_ctr_we        = 1'b0;
      entropy_syn_new = 1'b0;

      if (bit_ctr_reg == 6'h20)
        begin
          entropy_syn_new = 1'b1;
        end

      if ((bit_ctr_inc) && (bit_ctr_reg < 6'h20))
        begin
          bit_ctr_new = bit_ctr_reg + 1'b1;
          bit_ctr_we  = 1'b1;
        end
      else if (entropy_ack)
        begin
          bit_ctr_new = 6'h00;
          bit_ctr_we  = 1'b1;
        end
      end // entropy_ack_logic

endmodule // avalanche_entropy_core

//======================================================================
// EOF avalanche_entropy_core.v
//======================================================================
