//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_wb_pkg.sv                                              ////
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
package adbg_wb_pkg;
  // The Wishbone debug module requires 53 bits
  parameter DBG_WB_DATAREG_LEN = 53;

  // These relate to the number of internal registers, and how
  // many bits are required in the Reg. Select register
  parameter DBG_WB_REGSELECT_SIZE = 1;
  parameter DBG_WB_NUM_INTREG     = 1;

  // Register index definitions for module-internal registers
  // The WB module has just 1, the error register
  parameter DBG_WB_INTREG_ERROR = 'b0;


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
  parameter DBG_WB_CMD_BWRITE8  = 'h1;
  parameter DBG_WB_CMD_BWRITE16 = 'h2;
  parameter DBG_WB_CMD_BWRITE32 = 'h3;
  parameter DBG_WB_CMD_BWRITE64 = 'h4;
  parameter DBG_WB_CMD_BREAD8   = 'h5;
  parameter DBG_WB_CMD_BREAD16  = 'h6;
  parameter DBG_WB_CMD_BREAD32  = 'h7;
  parameter DBG_WB_CMD_BREAD64  = 'h8;
  parameter DBG_WB_CMD_IREG_WR  = 'h9;
  parameter DBG_WB_CMD_IREG_SEL = 'hd;
endpackage

