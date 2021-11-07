//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_or1k_status_reg.v                                      ////
////                                                              ////
////                                                              ////
////  This file is part of the SoC Debug Interface.               ////
////                                                              ////
////  Author(s):                                                  ////
////       Igor Mohor (igorm@opencores.org)                       ////
////       Nathan Yawn (nyawn@opencores.org)                      ////
////       richard.herveille@roalogic.com                         ////
////                                                              ////
////                                                              ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000 - 2011 Authors                            ////
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

module adbg_or1k_status_reg  #( 
  parameter NB_CORES = 4
)
(
  input                         tlr_i,
  input                         tck_i,
  input                         we_i,
  output logic [NB_CORES*2-1:0] ctrl_reg_o,


  input                         cpu_rstn_i,
  input                         cpu_clk_i,
  input  logic [NB_CORES*2-1:0] data_i,
  input  logic [NB_CORES  -1:0] bp_i,
  output logic [NB_CORES  -1:0] cpu_stall_o,
  output logic [NB_CORES  -1:0] cpu_rst_o
);

 reg [NB_CORES-1:0] stall_bp, stall_bp_csff, stall_bp_tck;
 reg [NB_CORES-1:0] stall_reg, stall_reg_csff, stall_reg_cpu;
 reg [NB_CORES-1:0] rst_reg, rst_reg_csff, rst_reg_cpu;


 // Breakpoint is latched and synchronized. Stall is set and latched.
 // This is done in the CPU clock domain, because the JTAG clock (TCK) is
 // irregular.  By only allowing bp_i to set (but not reset) the stall_bp
 // signal, we insure that the CPU will remain in the stalled state until
 // the debug host can read the state.
  always @(posedge cpu_clk_i,negedge cpu_rstn_i)
    if  (!cpu_rstn_i           ) stall_bp <= 'h0;
    else
    for (int i=0;i<NB_CORES;i++)
      if      (bp_i[i]         ) stall_bp[i] <= 1'b1;
      else if (stall_reg_cpu[i]) stall_bp[i] <= 1'b0;


   // Synchronizing
   always @(posedge tck_i,posedge tlr_i)
     if (tlr_i)
     begin
         stall_bp_csff <= 'h0;
         stall_bp_tck  <= 'h0;
     end
     else
     begin
         stall_bp_csff <= stall_bp;
         stall_bp_tck  <= stall_bp_csff;
     end


   always @(posedge cpu_clk_i,negedge cpu_rstn_i)
     if (!cpu_rstn_i)
     begin
         stall_reg_csff <= 'h0;
         stall_reg_cpu  <= 'h0;
     end
     else
     begin
         stall_reg_csff <= stall_reg;
         stall_reg_cpu  <= stall_reg_csff;
     end

     // bp_i forces a stall immediately on a breakpoint
     // stall_bp holds the stall until the debug host acts
     // stall_reg_cpu allows the debug host to control a stall.
     assign cpu_stall_o = bp_i | stall_bp | stall_reg_cpu;


     //Don't reset these synchronisation registers
     //These should drive cpu_rstn_i and hence may up resetting themselves
     always @(posedge cpu_clk_i)
     begin
         rst_reg_csff <= rst_reg;
         rst_reg_cpu  <= rst_reg_csff;
     end

     assign cpu_rst_o = rst_reg_cpu;


generate  
  genvar n;
  
  for (n=0; n < NB_CORES; n++)
  begin
      // Writing data to the control registers (stall)
      // This can be set either by the debug host, or by
      // a CPU breakpoint.  It can only be cleared by the host.
      always @(posedge tck_i,posedge tlr_i)
        if      (tlr_i          ) stall_reg[n] <= 1'b0;
        else if (stall_bp_tck[n]) stall_reg[n] <= 1'b1;
        else if (we_i           ) stall_reg[n] <= data_i[n*2];


  
      //Writing data to the control registers (reset)
      always @(posedge tck_i,posedge tlr_i)
        if      (tlr_i) rst_reg[n] <= 1'b0;
	else if (we_i ) rst_reg[n] <= data_i[n*2 +1];


      // Value for read back
      assign ctrl_reg_o[n +: 1] = {rst_reg[n], stall_reg[n]};
  end
endgenerate

endmodule

