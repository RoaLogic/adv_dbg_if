//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_jsp_apb_module.sv                                      ////
////                                                              ////
////                                                              ////
////  This file is part of the SoC Advanced Debug Interface.      ////
////                                                              ////
////  Author(s):                                                  ////
////       Nathan Yawn (nathan.yawn@opencores.org)                ////
////       Richard Herveille (richard.herveille@roalogic.com)     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2010 - 2015 Authors                            ////
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


// Module interface
module adbg_jsp_apb_module 
(
  input                                         rst_i,

  // JTAG signals
  input                                         tck_i,
  input                                         tdi_i,
  output                                        module_tdo_o,

  // TAP states
  input                                         capture_dr_i,
                                                shift_dr_i,
                                                update_dr_i,

  input [adbg_jsp_pkg::DBG_JSP_DATAREG_LEN-1:0] data_register_i,  // the data register is at top level, shared between all modules
  input                                         module_select_i,
  output                                        top_inhibit_o,

  // AMBA APB interface
  input                                         PRESETn,
                                                PCLK,
										 
  input                                         PSEL,
  input                                         PENABLE,
  input                                         PWRITE,
  input  [                                 2:0] PADDR,
  input  [                                 7:0] PWDATA,
  output [                                 7:0] PRDATA,
  output                                        PREADY,
  output                                        PSLVERR,

  output                                        int_o
);
  
  //////////////////////////////////////////////////////////////////
  //
  // Constants
  //
  import adbg_jsp_pkg::*;


  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  logic       biu_clk,
              biu_rst;
  logic [7:0] biu_di,
              biu_do;
  logic [3:0] biu_bytes_available,
              biu_space_available;
  logic       biu_rd_strobe,
              biu_wr_strobe; 


  //////////////////////////////////////////////////////////////////
  //
  // Module body
  //

  /*
   * Hookup JSP Debug Core
   */
  adbg_jsp_module_core
  jsp_core_inst
  (
    .*
  );

  /*
   * Hookup JSP APB Interface
   */
  adbg_jsp_apb_biu
  jsp_biu_inst (
    // Debug interface signals
    .tck_i             ( biu_clk             ),
    .rst_i             ( biu_rst             ),
    .data_i            ( biu_di              ),
    .data_o            ( biu_do              ),
    .bytes_available_o ( biu_bytes_available ),
    .bytes_free_o      ( biu_space_available ),
    .rd_strobe_i       ( biu_rd_strobe       ),
    .wr_strobe_i       ( biu_wr_strobe       ),
		   
    // APB signals
    .*
   );

endmodule

