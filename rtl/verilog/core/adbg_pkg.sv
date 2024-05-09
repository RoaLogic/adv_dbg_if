//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_pkg.sv                                                 ////
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
//// Copyright (C) 2015-2016 Authors                              ////
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
package adbg_pkg;
  parameter DBG_TOP_DATAREG_LEN = 64;

  // How many modules can be supported by the module id length
  parameter DBG_TOP_MAX_MODULES = 4;

  // Length of the MODULE ID register
  parameter DBG_TOP_MODULE_ID_LENGTH = $clog2(DBG_TOP_MAX_MODULES);

  // Chains
  parameter DBG_TOP_BUSIF_DEBUG_MODULE  = 'h0;
  parameter DBG_TOP_CPU_DEBUG_MODULE    = 'h1;
  parameter DBG_TOP_JSP_DEBUG_MODULE    = 'h2;
  parameter DBG_TOP_RESERVED_DBG_MODULE = 'h3;
endpackage

