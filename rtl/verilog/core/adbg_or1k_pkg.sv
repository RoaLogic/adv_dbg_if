//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_or1k_pkg.sv                                            ////
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
package adbg_or1k_pkg;
  //  Size of data-shift-register
  //  1bit cmd
  //  4bit operation
  //  4bit core/thread select
  // 32bit address
  // 16bit count
  // Should we put this in a packed struct?!
  parameter DBG_OR1K_DATAREG_LEN = 57;

  // Size of the register-select register
  parameter DBG_OR1K_REGSELECT_LEN = 1;

  // Register index definitions for module-internal registers
  // Index 0 is the Status register, used for stall and reset
  parameter DBG_OR1K_INTREG_STATUS = 'h0;


  // Valid commands/opcodes for the or1k debug module
  // 0000        NOP
  // 0001 - 0010 Reserved
  // 0011        Write burst, 32-bit access
  // 0100 - 0110 Reserved
  // 0111        Read burst, 32-bit access
  // 1000        Reserved
  // 1001        Internal register select/write
  // 1010 - 1100 Reserved
  // 1101        Internal register select
  // 1110 - 1111 Reserved
  parameter DBG_OR1K_CMD_BWRITE32 = 'h3;
  parameter DBG_OR1K_CMD_BREAD32  = 'h7;
  parameter DBG_OR1K_CMD_IREG_WR  = 'h9;
  parameter DBG_OR1K_CMD_IREG_SEL = 'hd;
endpackage

