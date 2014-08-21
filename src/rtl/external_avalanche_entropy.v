//======================================================================
//
// external_avalanche_entropy.v
// ----------------------------
// Entropy provider for an external entropy source based on
// avalanche noise. (or any other source that ca toggle a single
// bit input).
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
                                  // Clock and reset.
                                  input wire           clk,
                                  input wire           reset_n,
              
                                  // Control.
                                  input wire           cs,
                                  input wire           we,

                                  // Entropy input.
                                  input wire          entropy,
                                  
                                  // Data ports.
                                  input wire  [7 : 0]  address,
                                  input wire  [31 : 0] write_data,
                                  output wire [31 : 0] read_data,
                                  output wire          error
                                  );

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter ADDR_NAME0       = 8'h00;
  parameter ADDR_NAME1       = 8'h01;
  parameter ADDR_VERSION     = 8'h02;
  
  parameter ADDR_CTRL        = 8'h08;
  parameter CTRL_INIT_BIT    = 0;
  parameter CTRL_NEXT_BIT    = 1;

  parameter ADDR_STATUS      = 8'h09;
  parameter STATUS_READY_BIT = 0;
  parameter STATUS_VALID_BIT = 1;
                             
  parameter ADDR_RATE      = 8'h10;
  parameter ADDR_ENTROPY   = 8'h20;
  parameter ADDR_ZEROS     = 8'h21;
  parameter ADDR_ONES      = 8'h21;

  parameter CORE_NAME0     = 32'h73686132; // "ava "
  parameter CORE_NAME1     = 32'h2d323536; // "ent "
  parameter CORE_VERSION   = 32'h302e3031; // "0.01"

  
  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg entropy_sample0_reg;
  reg entropy_sample1_reg;

  reg entropy_flank0_reg;
  reg entropy flang1_reg;

  reg [31 : 0] entropy_ctr_reg;
  reg [31 : 0] entropy_ctr_new;
  reg          entropy_ctr_inc;
  reg          entropy_ctr_rst;
  reg          entropy_ctr_we;
  
  reg [31 : 0] zeros_ctr_reg;
  reg [31 : 0] zeros_ctr_new;
  reg          zeros_ctr_inc;
  reg          zeros_ctr_rst;
  reg          zeros_ctr_we;

  reg [31 : 0] ones_ctr_reg;
  reg [31 : 0] ones_ctr_new;
  reg          ones_ctr_inc;
  reg          ones_ctr_rst;
  reg          ones_ctr_we;
  
  reg [31 : 0] rate_ctr_reg;
  reg [31 : 0] rate_ctr_new;
  reg          rate_ctr_inc;
  reg          rate_ctr_rst;
  reg          rate_ctr_we;
  
  reg [31 : 0] rate_ctr_reg;
  reg [31 : 0] rate_ctr_new;
  reg          rate_ctr_inc;
  reg          rate_ctr_rst;
  reg          rate_ctr_we;
  
  reg [31 : 0] rate_reg;
  reg [31 : 0] rate_new;
  reg          rate_we;

  reg [255 : 0] digest_reg;

  reg digest_valid_reg;

  
  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0]   tmp_read_data;
  reg            tmp_error;
  
  
  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign read_data = tmp_read_data;
  assign error     = tmp_error;
  
  
  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with synchronous
  // active low reset. All registers have write enable.
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin
      if (!reset_n)
        begin
        end
      else
        begin
          ready_reg        <= core_ready;
          digest_valid_reg <= core_digest_valid;

        end
    end // reg_update


  //----------------------------------------------------------------
  // api_logic
  //
  // Implementation of the api logic. If cs is enabled will either 
  // try to write to or read from the internal registers.
  //----------------------------------------------------------------
  always @*
    begin : api_logic
      tmp_read_data = 32'h00000000;
      tmp_error     = 0;
      
      if (cs)
        begin
          if (we)
            begin
              case (address)
                // Write operations.
                ADDR_CTRL:
                  begin
                  end

                default:
                  begin
                    tmp_error = 1;
                  end
              endcase // case (address)
            end // if (we)

          else
            begin
              case (address)
                // Read operations.
                ADDR_NAME0:
                  begin
                    tmp_read_data = CORE_NAME0;
                  end

                ADDR_NAME1:
                  begin
                    tmp_read_data = CORE_NAME1;
                  end
                
                ADDR_VERSION:
                  begin
                    tmp_read_data = CORE_VERSION;
                  end

                ADDR_CTRL:
                  begin

                  end
                
                ADDR_STATUS:
                  begin

                  end

                default:
                  begin
                    tmp_error = 1;
                  end
              endcase // case (address)
            end
        end
    end // addr_decoder
endmodule // external_avalanche_entropy

//======================================================================
// EOF external_avalanche_entropy.v
//======================================================================
