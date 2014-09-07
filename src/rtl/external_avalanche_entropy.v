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

//                                  input wire           cs,
//                                  input wire           we,
//                                  input wire  [7 : 0]  address,
//                                  input wire  [31 : 0] write_data,
//                                  output wire [31 : 0] read_data,
//                                  output wire          error,

                                  input wire           noise,
                                  output wire          sampled_noise,
                                  output wire          entropy,

                                  input wire           entropy_read,
                                  output wire          entropy_ready,
                                  output wire [31 : 0] entropy_data,

                                  output wire [7 : 0]  led,
                                  output wire [7 : 0]  debug_data,
                                  output wire          debug_clk,

                                  output wire [14 : 0] delta_data,
                                  output wire          delta_clk
                                 );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter ADDR_STATUS      = 8'h00;
  parameter ADDR_ENTROPY     = 8'h10;
  parameter ADDR_DELTA       = 8'h20;

  parameter LED_RATE     = 32'h00300000;
  parameter SECONDS_RATE = 32'h02faf080;


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

  reg          entropy_ready_reg;
  reg          entropy_ready_new;

  reg [5 :  0] bit_ctr_reg;
  reg [5 :  0] bit_ctr_new;
  reg          bit_ctr_inc;
  reg          bit_ctr_rst;
  reg          bit_ctr_we;

  reg [3 : 0]  debug_ctr_reg;
  reg [3 : 0]  debug_ctr_new;
  reg          debug_ctr_inc;
  reg          debug_ctr_we;

  reg          debug_clk_reg;
  reg          debug_clk_new;

  reg [7 : 0]  led_reg;
  reg [7 : 0]  led_new;
  reg          led_we;

  reg [31 : 0] led_ctr_reg;
  reg [31 : 0] led_ctr_new;

  reg [31 : 0] cycle_ctr_reg;
  reg [31 : 0] cycle_ctr_new;

  reg [14 : 0] delta_reg;
  reg          delta_we;

  reg          delta_clk_reg;
  reg          delta_clk_new;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
//  reg [31 : 0]   tmp_read_data;
//  reg            tmp_error;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign entropy_ready = entropy_ready_reg;
  assign entropy_data  = entropy_reg;

  assign led           = led_reg;
  assign debug_data    = entropy_reg[7 : 0];
  assign debug_clk     = debug_clk_reg;

  assign sampled_noise = noise_sample_reg;
  assign entropy       = entropy_reg[0];

//  assign read_data     = tmp_read_data;
//  assign error         = tmp_error;

  assign delta_data    = delta_reg;
  assign delta_clk     = delta_clk_reg;


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
          entropy_ready_reg   <= 1'b0;
          entropy_reg         <= 32'h00000000;
          entropy_bit_reg     <= 1'b0;
          bit_ctr_reg         <= 6'h00;
          led_reg             <= 8'h00;
          led_ctr_reg         <= 32'h00000000;
          debug_ctr_reg       <= 4'h0;
          debug_clk_reg       <= 1'b0;
          cycle_ctr_reg       <= 32'h00000000;
          prev_cycle_ctr_reg  <= 32'h00000000;
          delta_reg           <= 15'h0000;
          delta_clk_reg       <= 1'b0;
        end
      else
        begin
          noise_sample0_reg <= noise;
          noise_sample_reg  <= noise_sample0_reg;

          flank0_reg        <= noise_sample_reg;
          flank1_reg        <= flank0_reg;

          entropy_ready_reg <= entropy_ready_new;

          entropy_bit_reg   <= ~entropy_bit_reg;

          led_ctr_reg       <= led_ctr_new;
          debug_clk_reg     <= debug_clk_new;

          delta_clk_reg     <= delta_clk_new;
          cycle_ctr_reg     <= cycle_ctr_new;

          if (delta_we)
            begin
              delta_reg <= cycle_ctr_reg[14 : 0];
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

          if (led_we)
            begin
              led_reg <= entropy_reg[7 : 0];
            end
        end
    end // reg_update


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

      if (debug_ctr_reg == 4'h08)
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
  // entropy_read_logic
  //
  // The logic needed to handle detection that entropy has been
  // read and ensure that we collect more than 32 entropy
  // bits beforeproviding more entropy.
  //----------------------------------------------------------------
  always @*
    begin : entropy_read_logic
      bit_ctr_new       = 6'h00;
      bit_ctr_we        = 1'b0;
      entropy_ready_new = 1'b0;

      if (bit_ctr_reg == 6'h20)
        begin
          entropy_ready_new = 1'b1;
        end

      if ((bit_ctr_inc) && (bit_ctr_reg < 6'h20))
        begin
          bit_ctr_new = bit_ctr_reg + 1'b1;
          bit_ctr_we  = 1'b1;
        end
      else if (entropy_read || bit_ctr_rst)
        begin
          bit_ctr_new = 6'h00;
          bit_ctr_we  = 1'b1;
        end
      end // entropy_read_logic


  //----------------------------------------------------------------
  // led_update
  //
  // Sample the entropy register as LED output value at
  // the given LED_RATE.
  //----------------------------------------------------------------
  always @*
    begin : led_update
      led_ctr_new = led_ctr_reg + 1'b1;
      led_we      = 1'b0;

      if (led_ctr_reg == LED_RATE)
        begin
          led_ctr_new = 32'h00000000;
          led_we      = 1'b1;
        end
    end // led_update


  //----------------------------------------------------------------
  // api_logic
  //----------------------------------------------------------------
//  always @*
//    begin : api_logic
//      tmp_read_data = 32'h00000000;
//      tmp_error     = 1'b0;
//      bit_ctr_rst   = 1'b1;
//
//      if (cs)
//        begin
//          if (we)
//            begin
//              case (address)
//                // Write operations.
//
//                default:
//                  begin
//                    tmp_error = 1;
//                  end
//              endcase // case (address)
//            end // if (we)
//
//          else
//            begin
//              case (address)
//                // Read operations.
//                ADDR_STATUS:
//                  begin
//                    tmp_read_data = {31'h00000000, entropy_ready_reg};
//                   end
//
//                ADDR_ENTROPY:
//                  begin
//                    tmp_read_data = entropy_reg;
//                    bit_ctr_rst   = 1'b1;
//                  end
//
//                ADDR_POS_FLANKS:
//                  begin
//                    tmp_read_data = posflank_sample_reg;
//                  end
//
//                ADDR_NEG_FLANKS:
//                  begin
//                    tmp_read_data = negflank_sample_reg;
//                  end
//
//                ADDR_TOT_FLANKS:
//                  begin
//                    tmp_read_data = totflank_sample_reg;
//                  end
//
//                ADDR_DELTA:
//                  begin
//                    tmp_read_data = delta_reg;
//                  end
//
//                default:
//                  begin
//                    tmp_error = 1;
//                  end
//              endcase // case (address)
//            end // else: !if(we)
//        end // if (cs)
//    end // api_logic

endmodule // external_avalanche_entropy

//======================================================================
// EOF external_avalanche_entropy.v
//======================================================================
