//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_jsp_module.sv                                          ////
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


//`include "adbg_defines.v"

// Module interface
module adbg_jsp_module_core 
(
  input                                          rst_i,

  // JTAG signals
  input                                          tck_i,
  input                                          tdi_i,
  output                                         module_tdo_o,

  // TAP states
  input                                          capture_dr_i,
                                                 shift_dr_i,
                                                 update_dr_i,

  input  [adbg_jsp_pkg::DBG_JSP_DATAREG_LEN-1:0] data_register_i,  // the data register is at top level, shared between all modules
  input                                          module_select_i,
  output                                         top_inhibit_o,

  // JSP BIU interface
  output                                         biu_clk,
  output                                         biu_rst,
  output [                                  7:0] biu_di,               // data towards BIU
  input  [                                  7:0] biu_do,               // data from BIU
  input  [                                  3:0] biu_space_available,
  input  [                                  3:0] biu_bytes_available,
  output logic                                   biu_rd_strobe,        // Indicates that the BIU should ACK last read operation + start another
                                                 biu_wr_strobe         // Indicates BIU should latch input + begin a write operation
);
  


  // NOTE:  For the rest of this file, "input" and the "in" direction refer to bytes being transferred
  // from the PC, through the JTAG, and into the BIU FIFO.  The "output" direction refers to data being
  // transferred from the BIU FIFO, through the JTAG to the PC.
   
  // The read and write bit counts are separated to allow for JTAG chains with multiple devices.
  // The read bit count starts right away (after a single throwaway bit), but the write count
  // waits to receive a '1' start bit.


  //////////////////////////////////////////////////////////////////
  //
  // Constants
  //
  import adbg_jsp_pkg::*;

  //FSM states
  typedef enum logic [1:0] {STATE_wr_idle,STATE_wr_wait,STATE_wr_counts,STATE_wr_xfer} states_wr;
  typedef enum logic [1:0] {STATE_rd_idle,STATE_rd_counts,STATE_rd_rdack,STATE_rd_xfer} states_rd;


  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //

  // Registers to hold state etc.
  logic [3:0] read_bit_count;      // How many bits have been shifted out
  logic [3:0] write_bit_count;     // How many bits have been shifted in
  logic [3:0] input_word_count;    // space (bytes) remaining in input FIFO (from JTAG)
  logic [3:0] output_word_count;   // bytes remaining in output FIFO (to JTAG)
  logic [3:0] user_word_count;     // bytes user intends to send from PC
  logic [7:0] data_out_shift_reg;  // parallel-load output shift register


  // Control signals for the various counters / registers / state machines
  logic rd_bit_ct_en;     // enable bit counter
  logic rd_bit_ct_rst;    // reset (zero) bit count register
  logic wr_bit_ct_en;     // enable bit counter
  logic wr_bit_ct_rst;    // reset (zero) bit count register   
  logic in_word_ct_sel;   // Selects data for byte counter.  0 = data_register_i, 1 = decremented byte count
  logic out_word_ct_sel;  // Selects data for byte counter.  0 = data_register_i, 1 = decremented byte count
  logic in_word_ct_en;    // Enable input byte counter register
  logic out_word_ct_en;   // Enable output byte count register
  logic user_word_ct_en;  // Enable user byte count registere
  logic user_word_ct_sel; // selects data for user byte counter.  0 = user data, 1 = decremented byte count
  logic out_reg_ld_en;    // Enable parallel load of data_out_shift_reg
  logic out_reg_shift_en; // Enable shift of data_out_shift_reg
  logic out_reg_data_sel; // 0 = BIU data, 1 = byte count data (also from BIU)


  // Status signals
  logic in_word_count_zero;   // true when input byte counter is zero
  logic out_word_count_zero;  // true when output byte counter is zero
  logic user_word_count_zero; // true when user byte counter is zero
  logic rd_bit_count_max;     // true when bit counter is equal to current word size
  logic wr_bit_count_max;     // true when bit counter is equal to current word size


  // Intermediate signals
  logic [3:0] data_to_in_word_counter;     // output of the mux in front of the input byte counter reg
  logic [3:0] data_to_out_word_counter;    // output of the mux in front of the output byte counter reg
  logic [3:0] data_to_user_word_counter;   // output of mux in front of user word counter
  logic [3:0] count_data_in;               // from data_register_i
  logic [7:0] data_to_biu;                 // from data_register_i
  logic [7:0] data_from_biu;               // to data_out_shift_register
  logic [7:0] count_data_from_biu;         // combined space avail / bytes avail
  logic [7:0] out_reg_data;                // parallel input to the output shift register


  //Statemachine
  states_wr wr_module_state, wr_module_next_state;
  states_rd rd_module_state, rd_module_next_state;


  //////////////////////////////////////////////////////////////////
  //
  // Module body
  //


  /////////////////////////////////////////////////
  // Combinatorial assignments
  assign count_data_from_biu = {biu_bytes_available, biu_space_available};
  assign count_data_in       = {tdi_i, data_register_i[DBG_JSP_DATAREG_LEN-1 -: 3]}; // Second nibble of user data
  assign data_to_biu         = {tdi_i, data_register_i[DBG_JSP_DATAREG_LEN-1 -: 7]};
  assign top_inhibit_o       = 1'b0;


  //////////////////////////////////////
  // Input bit counter
  always @(posedge tck_i, posedge rst_i)
    if      (rst_i        ) write_bit_count <= 'h0;
    else if (wr_bit_ct_rst) write_bit_count <= 'h0;
    else if (wr_bit_ct_en ) write_bit_count <= write_bit_count + 'h1;

  assign wr_bit_count_max = write_bit_count == 4'h7;


  //////////////////////////////////////
  // Output bit counter
  always @(posedge tck_i,posedge rst_i)
    if      (rst_i        ) read_bit_count <= 'h0;
    else if (rd_bit_ct_rst) read_bit_count <= 'h0;
    else if (rd_bit_ct_en ) read_bit_count <= read_bit_count + 'h1;

   assign rd_bit_count_max = read_bit_count == 4'h7;


  ////////////////////////////////////////
  // Input word counter
  assign data_to_in_word_counter = in_word_ct_sel ? input_word_count - 'h1 : biu_space_available;

  always @ (posedge tck_i,posedge rst_i)
    if      (rst_i        ) input_word_count <= 'h0;
    else if (in_word_ct_en) input_word_count <= data_to_in_word_counter;

   assign in_word_count_zero = (input_word_count == 4'h0);
   

  ////////////////////////////////////////
  // Output word counter
  assign data_to_out_word_counter = out_word_ct_sel ? output_word_count - 'h1 : biu_bytes_available;

  always @ (posedge tck_i,posedge rst_i)
    if      (rst_i         ) output_word_count <= 'h0;
    else if (out_word_ct_en) output_word_count <= data_to_out_word_counter;

  assign out_word_count_zero = ~|output_word_count;


  ////////////////////////////////////////
  // User word counter
  assign data_to_user_word_counter = user_word_ct_sel ? user_word_count - 'h1 : count_data_in;

  always @(posedge tck_i,posedge rst_i)
    if      (rst_i          ) user_word_count <= 'h0;
    else if (user_word_ct_en) user_word_count <= data_to_user_word_counter;

  assign user_word_count_zero = ~|user_word_count;
    

  /////////////////////////////////////////////////////
  // Output register and TDO output MUX
   assign out_reg_data = (out_reg_data_sel) ? count_data_from_biu : data_from_biu;

  always @ (posedge tck_i or posedge rst_i)
    if      (rst_i           ) data_out_shift_reg <= 'h0;
    else if (out_reg_ld_en   ) data_out_shift_reg <= out_reg_data;
    else if (out_reg_shift_en) data_out_shift_reg <= {1'b0, data_out_shift_reg[$bits(data_out_shift_reg)-1:1]};

  assign module_tdo_o = data_out_shift_reg[0];

   
  ////////////////////////////////////////
  // Bus Interface Unit (to JTAG / WB UART)
  // It is assumed that the BIU has internal registers, and will
  // latch write data (and ack read data) on rising clock edge 
  // when strobe is asserted
  assign biu_clk       = tck_i;
  assign biu_rst       = rst_i;
  assign biu_di        = data_to_biu;
  assign data_from_biu = biu_do;

/*
   adbg_jsp_biu jsp_biu_i (
			   // Debug interface signals
			   .tck_i           (tck_i),
			   .rst_i           (rst_i),
			   .data_i          (data_to_biu),
			   .data_o          (data_from_biu),
			   .bytes_available_o (biu_bytes_available),
			   .bytes_free_o    (biu_space_available),
			   .rd_strobe_i     (biu_rd_strobe),
			   .wr_strobe_i     (biu_wr_strobe),
			   
			   // Wishbone slave signals
			   .wb_clk_i        (wb_clk_i),
			   .wb_rst_i        (wb_rst_i),
			   .wb_adr_i        (wb_adr_i),
			   .wb_dat_o        (wb_dat_o),
			   .wb_dat_i        (wb_dat_i),
			   .wb_cyc_i        (wb_cyc_i),
			   .wb_stb_i        (wb_stb_i),
			   .wb_sel_i        (wb_sel_i),
			   .wb_we_i         (wb_we_i),
			   .wb_ack_o        (wb_ack_o),
			   .wb_err_o        (wb_err_o),
			   .wb_cti_i        (wb_cti_i),
			   .wb_bte_i        (wb_bte_i),
			   .int_o           (int_o)
			   );
*/
 

  ////////////////////////////////////////
  // Input Control FSM

  // sequential part of the FSM
  always @ (posedge tck_i,posedge rst_i)
    if (rst_i) wr_module_state <= STATE_wr_idle;
    else       wr_module_state <= wr_module_next_state;


  // Determination of next state; purely combinatorial
  always_comb
    case(wr_module_state)
       STATE_wr_idle:
`ifdef ADBG_JSP_SUPPORT_MULTI
          if (module_select_i && capture_dr_i) wr_module_next_state = STATE_wr_wait;
`else
          if (module_select_i && capture_dr_i) wr_module_next_state = STATE_wr_counts;
`endif
          else                                 wr_module_next_state = STATE_wr_idle;
       STATE_wr_wait:
          if      (update_dr_i             ) wr_module_next_state = STATE_wr_idle;
          else if (module_select_i && tdi_i) wr_module_next_state = STATE_wr_counts;  // got start bit
          else                               wr_module_next_state = STATE_wr_wait;
       STATE_wr_counts:
          if      (update_dr_i     ) wr_module_next_state = STATE_wr_idle;
          else if (wr_bit_count_max) wr_module_next_state = STATE_wr_xfer;
          else                       wr_module_next_state = STATE_wr_counts;
       STATE_wr_xfer:
          if (update_dr_i) wr_module_next_state = STATE_wr_idle;
          else             wr_module_next_state = STATE_wr_xfer;
       default: wr_module_next_state = STATE_wr_idle;  // shouldn't actually happen...
    endcase
   

  // Outputs of state machine, pure combinatorial
  always_comb
    begin
        // Default everything to 0, keeps the case statement simple
        wr_bit_ct_en     = 1'b0; // enable bit counter
        wr_bit_ct_rst    = 1'b0; // reset (zero) bit count register
        in_word_ct_sel   = 1'b0; // Selects data for byte counter.  0 = data_register_i, 1 = decremented byte count
        user_word_ct_sel = 1'b0; // selects data for user byte counter, 0 = user data, 1 = decremented count
        in_word_ct_en    = 1'b0; // Enable input byte counter register
        user_word_ct_en  = 1'b0; // enable user byte count register
        biu_wr_strobe    = 1'b0; // Indicates BIU should latch input + begin a write operation

        case(wr_module_state)
           STATE_wr_idle:
           begin
               in_word_ct_sel = 1'b0;
	       
               // Going to transfer; enable count registers and output register
               if (wr_module_next_state != STATE_wr_idle)
               begin
                   wr_bit_ct_rst = 1'b1;
                   in_word_ct_en = 1'b1;
               end
           end

           // This state is only used when support for multi-device JTAG chains is enabled.
           STATE_wr_wait: wr_bit_ct_en = 1'b0;  // Don't do anything, just wait for the start bit.

           STATE_wr_counts:
           if (shift_dr_i)
           begin // Don't do anything in PAUSE or EXIT states...
               wr_bit_ct_en     = 1'b1;
               user_word_ct_sel = 1'b0;

               if (wr_bit_count_max)
               begin
                   wr_bit_ct_rst   = 1'b1;
                   user_word_ct_en = 1'b1;
               end
           end
    
           STATE_wr_xfer:
           if (shift_dr_i)
           begin  // Don't do anything in PAUSE or EXIT states
               wr_bit_ct_en     = 1'b1;
               in_word_ct_sel   = 1'b1;
               user_word_ct_sel = 1'b1;
	       
               if (wr_bit_count_max)
               begin  // Start biu transactions, if word counts allow
                   wr_bit_ct_rst = 1'b1;

                   if (!(in_word_count_zero || user_word_count_zero))
                   begin
                       biu_wr_strobe   = 1'b1;
                       in_word_ct_en   = 1'b1;
                       user_word_ct_en = 1'b1;
                   end
               end
           end

           default: ;
        endcase
    end


  ////////////////////////////////////////
  // Output Control FSM

  // We do not send the equivalent of a 'start bit' (like the one the input FSM
  // waits for when support for multi-device JTAG chains is enabled).  Since the
  // input and output are going to be offset anyway, why bother...

  // sequential part of the FSM
  always @(posedge tck_i,posedge rst_i)
    if  (rst_i) rd_module_state = STATE_rd_idle;
    else        rd_module_state = rd_module_next_state;


  // Determination of next state; purely combinatorial
  always_comb
    case (rd_module_state)
       STATE_rd_idle:
          if (module_select_i && capture_dr_i) rd_module_next_state = STATE_rd_counts;
          else                                 rd_module_next_state = STATE_rd_idle;
       STATE_rd_counts:
          if      (update_dr_i     ) rd_module_next_state = STATE_rd_idle;
          else if (rd_bit_count_max) rd_module_next_state = STATE_rd_rdack;
          else                       rd_module_next_state = STATE_rd_counts;
       STATE_rd_rdack:
          if (update_dr_i) rd_module_next_state = STATE_rd_idle;
          else             rd_module_next_state = STATE_rd_xfer;
       STATE_rd_xfer:
          if      (update_dr_i     ) rd_module_next_state = STATE_rd_idle;
          else if (rd_bit_count_max) rd_module_next_state = STATE_rd_rdack;
          else                       rd_module_next_state = STATE_rd_xfer;
	  
       default: rd_module_next_state = STATE_rd_idle;  // shouldn't actually happen...
    endcase
   

   // Outputs of state machine, pure combinatorial
   always_comb
    begin
        // Default everything to 0, keeps the case statement simple
        rd_bit_ct_en     = 1'b0; // enable bit counter
        rd_bit_ct_rst    = 1'b0; // reset (zero) bit count register
        out_word_ct_sel  = 1'b0; // Selects data for byte counter.  0 = data_register_i, 1 = decremented byte count
        out_word_ct_en   = 1'b0; // Enable output byte count register
        out_reg_ld_en    = 1'b0; // Enable parallel load of data_out_shift_reg
        out_reg_shift_en = 1'b0; // Enable shift of data_out_shift_reg
        out_reg_data_sel = 1'b0; // 0 = BIU data, 1 = byte count data (also from BIU)
        biu_rd_strobe    = 1'b0; // Indicates that the bus unit should ACK the last read operation + start another

        case (rd_module_state)
           STATE_rd_idle:
           begin
               out_reg_data_sel = 1'b1;
               out_word_ct_sel  = 1'b0;

               // Going to transfer; enable count registers and output register
               if (rd_module_next_state != STATE_rd_idle)
               begin
                   out_reg_ld_en  = 1'b1;
                   rd_bit_ct_rst  = 1'b1;
                   out_word_ct_en = 1'b1;
               end
           end

           STATE_rd_counts:
           if (shift_dr_i)
           begin // Don't do anything in PAUSE or EXIT states...
               rd_bit_ct_en     = 1'b1;
               out_reg_shift_en = 1'b1;
  
               if (rd_bit_count_max)
               begin
                   rd_bit_ct_rst = 1'b1;
     
                   // Latch the next output word, but don't ack until STATE_rd_rdack
                   if (!out_word_count_zero)
                   begin
                       out_reg_ld_en    = 1'b1;
                       out_reg_shift_en = 1'b0;
                   end
               end
           end
 
           STATE_rd_rdack:
           if (shift_dr_i)
           begin  // Don't do anything in PAUSE or EXIT states
               rd_bit_ct_en     = 1'b1;
               out_reg_shift_en = 1'b1;
               out_reg_data_sel = 1'b0;

               // Never have to worry about bit_count_max here.
               if(!out_word_count_zero) biu_rd_strobe = 1'b1;
           end
    
           STATE_rd_xfer:
           if (shift_dr_i)
           begin  // Don't do anything in PAUSE or EXIT states
               rd_bit_ct_en     = 1'b1;
               out_word_ct_sel  = 1'b1;
               out_reg_shift_en = 1'b1;
               out_reg_data_sel = 1'b0;

               if (rd_bit_count_max)
               begin  // Start biu transaction, if word count allows
                   rd_bit_ct_rst = 1'b1;

                   // Don't ack the read byte here, we do it in STATE_rdack
                   if (!out_word_count_zero)
                   begin
                       out_reg_ld_en    = 1'b1;
                       out_reg_shift_en = 1'b0;
                       out_word_ct_en   = 1'b1;
                   end
               end
           end

           default: ;
        endcase
    end

endmodule

