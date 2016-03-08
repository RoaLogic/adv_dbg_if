//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_ahb3_module.v                                          ////
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
//// Copyright (C) 2008-2015 Authors                              ////
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
//
// CVS Revision History
//
// $Log: $
//


module adbg_ahb3_module #(
  parameter ADDR_WIDTH = 32,
  parameter DATA_WIDTH = 32
)
(
  // JTAG signals
  input                     tck_i,
  output                    module_tdo_o,
  input                     tdi_i,

  // TAP states
  input                     tlr_i,
  input                     capture_dr_i,
  input                     shift_dr_i,
  input                     update_dr_i,

  input  [adbg_ahb3_pkg::DBG_AHB_DATAREG_LEN-1:0] data_register_i,  // the data register is at top level, shared between all modules
  input                     module_select_i,
  output                    top_inhibit_o,

  // AHB3 master interface
  input                     HCLK,
                            HRESETn,
  output                    HSEL,
  output [ADDR_WIDTH  -1:0] HADDR,
  output [DATA_WIDTH  -1:0] HWDATA,
  input  [DATA_WIDTH  -1:0] HRDATA,
  output                    HWRITE,
  output [             2:0] HSIZE,
  output [             2:0] HBURST,
  output [             3:0] HPROT,
  output [             1:0] HTRANS,
  output                    HMASTLOCK,
  input                     HREADY,
  input                     HRESP
);
  //////////////////////////////////////////////////////////////////
  //
  // Constants
  //
  import adbg_ahb3_pkg::*;


  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  logic                  biu_clk,
                         biu_rst;
  logic [DATA_WIDTH-1:0] biu_do,
                         biu_di;
  logic [ADDR_WIDTH-1:0] biu_addr;
  logic                  biu_strb,
                         biu_rw,
                         biu_rdy,
                         biu_err;
  logic [           3:0] biu_word_size;


  //////////////////////////////////////////////////////////////////
  //
  // Module Body
  //

  /*
   * Hookup Bus Debug Core
   */
  adbg_bus_module_core #(
    .ADDR_WIDTH     ( ADDR_WIDTH             ),
    .DATA_WIDTH     ( DATA_WIDTH             ),
    .DATAREG_LEN    ( DBG_AHB_DATAREG_LEN    ),
    .REGSELECT_SIZE ( DBG_AHB_REGSELECT_SIZE ),
  
    .BWRITE8        ( DBG_AHB_CMD_BWRITE8    ),
    .BWRITE16       ( DBG_AHB_CMD_BWRITE16   ),
    .BWRITE32       ( DBG_AHB_CMD_BWRITE32   ),
    .BWRITE64       ( DBG_AHB_CMD_BWRITE64   ),
    .BREAD8         ( DBG_AHB_CMD_BREAD8     ),
    .BREAD16        ( DBG_AHB_CMD_BREAD16    ),
    .BREAD32        ( DBG_AHB_CMD_BREAD32    ),
    .BREAD64        ( DBG_AHB_CMD_BREAD64    ),
    .IREG_WR        ( DBG_AHB_CMD_IREG_WR    ),
    .IREG_SEL       ( DBG_AHB_CMD_IREG_SEL   )
  )
  bus_module_core_inst (
    //Debug Module ports
    .dbg_rst       ( tlr_i ),
    .dbg_clk       ( tck_i ),
    .dbg_tdi       ( tdi_i ),
    .dbg_tdo       ( module_tdo_o ),

    .data_register ( data_register_i ), //data register from top-level
    .module_select ( module_select_i ),
    .inhibit       ( top_inhibit_o   ),

    .*
  );


  /*
   * Hookup AHB Bus Interface
   */
  adbg_ahb3_biu #(
    .ADDR_WIDTH ( ADDR_WIDTH ),
    .DATA_WIDTH ( DATA_WIDTH )
  )
  ahb3lite_biu_i (
    .*
  );
 
endmodule

