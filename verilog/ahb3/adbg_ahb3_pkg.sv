//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_ahb3_pkg.sv                                            ////
////                                                              ////
////                                                              ////
////  This file is part of the SoC Advanced Debug Interface.      ////
////                                                              ////
////  Author(s):                                                  ////
////       Richard Herveille (richard.herveille@roalogic.com)     ////
////                                                              ////
////  Based on work by:                                           ////
////       Nathan Yawn (nathan.yawn@opencores.org)                ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2015 Authors                                   ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////


/*
 * Constants used 
 */
package adbg_ahb3_pkg;
  // The Wishbone debug module requires 53 bits
  parameter DBG_AHB_DATAREG_LEN = 53;

  // These relate to the number of internal registers, and how
  // many bits are required in the Reg. Select register
  parameter DBG_AHB_REGSELECT_SIZE = 1;
  parameter DBG_AHB_NUM_INTREG     = 1;

  // Register index definitions for module-internal registers
  // The WB module has just 1, the error register
  parameter DBG_AHB_INTREG_ERROR = 'b0;


  // Valid commands/opcodes for the wishbone debug module
  // 0000  NOP
  // 0001  Write burst, 8-bit access
  // 0010  Write burst, 16-bit access
  // 0011  Write burst, 32-bit access
  // 0100  Write burst, 64-bit access
  // 0101  Read burst, 8-bit access
  // 0110  Read burst, 16-bit access
  // 0111  Read burst, 32-bit access
  // 1000  Read burst, 64-bit access
  // 1001  Internal register select/write
  // 1010 - 1100 Reserved
  // 1101  Internal register select
  // 1110 - 1111 Reserved
  parameter DBG_AHB_CMD_BWRITE8  = 'h1;
  parameter DBG_AHB_CMD_BWRITE16 = 'h2;
  parameter DBG_AHB_CMD_BWRITE32 = 'h3;
  parameter DBG_AHB_CMD_BWRITE64 = 'h4;
  parameter DBG_AHB_CMD_BREAD8   = 'h5;
  parameter DBG_AHB_CMD_BREAD16  = 'h6;
  parameter DBG_AHB_CMD_BREAD32  = 'h7;
  parameter DBG_AHB_CMD_BREAD64  = 'h8;
  parameter DBG_AHB_CMD_IREG_WR  = 'h9;
  parameter DBG_AHB_CMD_IREG_SEL = 'hd;


  //AHB definitions
  parameter [1:0] HTRANS_IDLE   = 2'b00,
                  HTRANS_BUSY   = 2'b01,
                  HTRANS_NONSEQ = 2'b10,
                  HTRANS_SEQ    = 2'b11;

  parameter [2:0] HBURST_SINGLE = 3'b000,
                  HBURST_INCR   = 3'b001,
                  HBURST_WRAP4  = 3'b010,
                  HBURST_INCR4  = 3'b011,
                  HBURST_WRAP8  = 3'b100,
                  HBURST_INCR8  = 3'b101,
                  HBURST_WRAP16 = 3'b110,
                  HBURST_INCR16 = 3'b111;

  parameter [2:0] HSIZE8        = 3'b000,
                  HSIZE16       = 3'b001,
                  HSIZE32       = 3'b010,
                  HSIZE64       = 3'b011,
                  HSIZE128      = 3'b100,
                  HSIZE256      = 3'b101,
                  HSIZE512      = 3'b110,
                  HSIZE1024     = 3'b111;

  parameter [2:0] HSIZE_BYTE    = HSIZE8,
                  HSIZE_HWORD   = HSIZE16,
                  HSIZE_WORD    = HSIZE32,
                  HSIZE_DWORD   = HSIZE64,
                  HSIZE_4WLINE  = HSIZE128,
                  HSIZE_8WLINE  = HSIZE256;

  parameter       HRESP_OKAY    = 1'b0,
                  HRESP_ERR     = 1'b1;
endpackage

