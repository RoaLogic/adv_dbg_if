//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_bus_module_core.sv                                     ////
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
// $Log: adbg_wb_module.v,v $
// Revision 1.5  2010-01-13 00:55:45  Nathan
// Created hi-speed mode for burst reads.  This will probably be most beneficial to the OR1K module, as GDB does a burst read of all the GPRs each time a microinstruction is single-stepped.
//
// Revision 1.2  2009/05/17 20:54:57  Nathan
// Changed email address to opencores.org
//
// Revision 1.1  2008/07/22 20:28:33  Nathan
// Changed names of all files and modules (prefixed an a, for advanced).  Cleanup, indenting.  No functional changes.
//
// Revision 1.12  2008/07/11 08:13:30  Nathan
// Latch opcode on posedge, like other signals.  This fixes a problem when 
// the module is used with a Xilinx BSCAN TAP.  Added signals to allow modules 
// to inhibit latching of a new active module by the top module.  This allows 
// the sub-modules to force the top level module to ignore the command present
// in the input shift register after e.g. a burst read.
//

module adbg_bus_module_core #(
  //parameter such that these can be pushed down from the higher level
  //higher level will either read these from a package or get them as parameters

  //Data + Address width
  parameter ADDR_WIDTH     = 32,
  parameter DATA_WIDTH     = 32,

  //Data register size (function of ADDR_WIDTH)
  parameter DATAREG_LEN    = 64,

  parameter REGSELECT_SIZE = 1,
  
  //Instructions  
  parameter BWRITE8        = 4'h1,
  parameter BWRITE16       = 4'h2,
  parameter BWRITE32       = 4'h3,
  parameter BWRITE64       = 4'h4,
  parameter BREAD8         = 4'h5,
  parameter BREAD16        = 4'h6,
  parameter BREAD32        = 4'h7,
  parameter BREAD64        = 4'h8,
  parameter IREG_WR        = 4'h9,
  parameter IREG_SEL       = 4'hd
)
(
  input                        dbg_clk,
  input                        dbg_rst,
  input                        dbg_tdi,
  output reg                   dbg_tdo,

  // TAP states
  input                        capture_dr_i,
  input                        shift_dr_i,
  input                        update_dr_i,

  input      [DATAREG_LEN-1:0] data_register,  // the data register is at top level, shared between all modules
  input                        module_select,
  output reg                   inhibit,

  //Bus Interface Unit ports
  output                       biu_clk,
                               biu_rst, //BIU reset
  output     [DATA_WIDTH -1:0] biu_di,  //data towards BIU
  input      [DATA_WIDTH -1:0] biu_do,  //data from BIU
  output     [ADDR_WIDTH -1:0] biu_addr,
  output                       biu_strb,
                               biu_rw,
  input                        biu_rdy,
                               biu_err,
  output     [            3:0] biu_word_size
);

  //////////////////////////////////////////////////////////////////
  //
  // Constants
  //
  //data register structure
  typedef struct packed {
    logic        selcmd;
    logic [ 3:0] operation;
    logic [31:0] address_data;
    logic [15:0] count_data;
  } data_struct;


  //error register
  typedef struct packed {
    logic [ADDR_WIDTH-1:0] addr;
    logic                  error;
  } error_reg_struct;

  typedef enum logic [3:0] {STATE_idle,STATE_Rbegin,STATE_Rready,STATE_Rstatus,STATE_Rburst,
                            STATE_Wready,STATE_Wwait,STATE_Wburst,STATE_Wstatus,STATE_Rcrc,STATE_Wcrc,STATE_Wmatch} states;

  localparam INTREG_ERROR = 1'b0;

  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //

  // Registers to hold state etc.
  logic [ADDR_WIDTH    -1:0] address_counter;           // Holds address for next Wishbone access
  logic [               5:0] bit_count;                 // How many bits have been shifted in/out
  logic [              15:0] word_count;                // bytes remaining in current burst command
  logic [DATA_WIDTH      :0] data_out_shift_reg;        // 32 bits to accomodate the internal_reg_error
  logic [REGSELECT_SIZE-1:0] internal_register_select;  // Holds index of currently selected register
  error_reg_struct           internal_reg_error;        // WB error module internal register.  32 bit address + error bit (LSB)


   // Control signals for the various counters / registers / state machines
   logic                     addr_sel;         // Selects data for address_counter. 0=data_register_i, 1=incremented address count
   logic                     addr_ct_en;       // Enable signal for address counter register
   logic                     op_reg_en;        // Enable signal for 'operation' register
   logic                     bit_ct_en;        // enable bit counter
   logic                     bit_ct_rst;       // reset (zero) bit count register
   logic                     word_ct_sel;      // Selects data for byte counter.  0=data_register_i, 1=decremented byte count
   logic                     word_ct_en;       // Enable byte counter register
   logic                     out_reg_ld_en;    // Enable parallel load of data_out_shift_reg
   logic                     out_reg_shift_en; // Enable shift of data_out_shift_reg
   logic                     out_reg_data_sel; // 0 = BIU data, 1 = internal register data
   reg [1:0]                 tdo_output_sel;   // Selects signal to send to TDO. 0=ready bit, 1=output register, 2=CRC match, 3=CRC shift reg.
   logic                     biu_strobe;       // Indicates that the bus unit should latch data and start a transaction
   logic                     crc_clr;          // resets CRC module
   logic                     crc_en;           // does 1-bit iteration in CRC module
   logic                     crc_in_sel;       // selects incoming write data (=0) or outgoing read data (=1)as input to CRC module
   logic                     crc_shift_en;     // CRC reg is also it's own output shift register; this enables a shift
   logic                     regsel_ld_en;     // Reg. select register load enable
   logic                     intreg_ld_en;     // load enable for internal registers
   logic                     error_reg_en;     // Tells the error register to check for and latch a bus error
   logic                     biu_clr_err;      // Allows FSM to reset BIU, to clear the biu_err bit which may have been set on the last transaction of the last burst.

   // Status signals
   wire                      word_count_zero;    // true when byte counter is zero
   wire                      bit_count_max;      // true when bit counter is equal to current word size
   wire                      module_cmd;         // inverse of MSB of data_register. 1 means current cmd not for top level (but is for us)
   logic                     burst_read;
   logic                     burst_write;
   wire                      intreg_instruction; // True when the input_data reg has a valid internal register instruction
   wire                      intreg_write;       // True when the input_data reg has an internal register write op
   reg                       rd_op;              // True when operation in the opcode reg is a read, false when a write
   wire                      crc_match;          // indicates whether data_register matches computed CRC
   wire                      bit_count_32;       // true when bit count register == 32, for CRC after burst writes

   // Intermediate signals
   logic [               5:0] word_size_bits;        // 8,16,32,64.  Decoded from 'operation'
   logic [               3:0] word_size_bytes;       // 1,2,4,8
   logic [              15:0] decremented_word_count;
   logic [ADDR_WIDTH    -1:0] address_data_in;       // from data_register_i
   logic [              15:0] count_data_in;         // from data_register_i
   logic [               3:0] operation_in;          // from data_register_i
   logic [DATA_WIDTH    -1:0] data_to_biu;           // from data_register_i
   logic [              31:0] crc_data_out;          // output of CRC module, to output shift register
   logic                      crc_data_in;           // input to CRC module, either data_register[52] or data_out_shift_reg[0]
   logic                      crc_serial_out;
   logic [REGSELECT_SIZE-1:0] reg_select_data; // from data_register_i, input to internal register select register
   logic [DATA_WIDTH      :0] data_from_internal_reg;  // data from internal reg. MUX to output shift register


   //Statemachine states
   states module_state, module_next_state;



  /////////////////////////////////////////////////
  // Combinatorial assignments
   assign module_cmd      =~data_register[DATAREG_LEN-1                         ];
   assign operation_in    = data_register[DATAREG_LEN-2            -:          4];
   assign address_data_in = data_register[DATAREG_LEN-6            -: ADDR_WIDTH];
   assign count_data_in   = data_register[DATAREG_LEN-6-ADDR_WIDTH -:         16];

   assign data_to_biu     = {dbg_tdi,data_register[DATAREG_LEN-1  -:  DATA_WIDTH-1]};

   assign reg_select_data = data_register[DATAREG_LEN-6  -: REGSELECT_SIZE];



  ////////////////////////////////////////////////
  // Operation decoder

  // These are only used before the operation is latched, so decode them from operation_in
  assign intreg_instruction = (operation_in == IREG_WR) | (operation_in == IREG_SEL);
  assign intreg_write       = (operation_in == IREG_WR);

  assign burst_write        = (operation_in == BWRITE8)  | 
                              (operation_in == BWRITE16) | 
                              (operation_in == BWRITE32) | 
                              (operation_in == BWRITE64); 

  assign burst_read         = (operation_in == BREAD8)  | 
                              (operation_in == BREAD16) | 
                              (operation_in == BREAD32) | 
                              (operation_in == BREAD64); 

  // This is decoded from the registered operation
  always @(posedge dbg_clk)
    if (op_reg_en)
      case(operation_in)
         BWRITE8 : begin
                       word_size_bits  <= 'd7;  // Bits is actually bits-1, to make the FSM easier
                       word_size_bytes <= 'd1;
                       rd_op           <= 'b0;
                   end
         BWRITE16: begin
                       word_size_bits  <= 'd15; // Bits is actually bits-1, to make the FSM easier
                       word_size_bytes <= 'd2;
                       rd_op           <= 'b0;
                   end
         BWRITE32: begin
                       word_size_bits  <= 'd31; // Bits is actually bits-1, to make the FSM easier
                       word_size_bytes <= 'd4;
                       rd_op           <= 'b0;
                   end
         BWRITE64: begin
                       word_size_bits  <= 'd63; // Bits is actually bits-1, to make the FSM easier
                       word_size_bytes <= 'd8;
                       rd_op           <= 'b0;
                   end
         BREAD8  : begin
                       word_size_bits  <= 'd7;  // Bits is actually bits-1, to make the FSM easier
                       word_size_bytes <= 'd1;
                       rd_op           <= 'b1;
                   end
         BREAD16 : begin
                       word_size_bits  <= 'd15; // Bits is actually bits-1, to make the FSM easier
                       word_size_bytes <= 'd2;
                       rd_op           <= 'b1;
                   end
         BREAD32 : begin
                       word_size_bits  <= 'd31; // Bits is actually bits-1, to make the FSM easier
                       word_size_bytes <= 'd4;
                       rd_op           <= 'b1;
                   end
         BREAD64 : begin
                       word_size_bits  <= 'd63; // Bits is actually bits-1, to make the FSM easier
                       word_size_bytes <= 'd8;
                       rd_op           <= 'b1;
                   end
         default:  begin
                       word_size_bits  <= 'hx;
                       word_size_bytes <= 'hx;
                       rd_op           <= 'bx;
                   end       
     endcase


   ////////////////////////////////////////////////
   // Module-internal register select register (no, that's not redundant.)
   // Also internal register output MUX
   always @(posedge dbg_clk,posedge dbg_rst)
     if      (dbg_rst     ) internal_register_select = 1'h0;
     else if (regsel_ld_en) internal_register_select = reg_select_data;


   // This is completely unnecessary here, since the WB module has only 1 internal register
   // However, to make the module expandable, it is included anyway.
   always_comb
     case(internal_register_select) 
        INTREG_ERROR: data_from_internal_reg = internal_reg_error;
        default     : data_from_internal_reg = internal_reg_error;
     endcase
   


   ////////////////////////////////////////////////////////////////////
   // Module-internal registers
   // These have generic read/write/select code, but
   // individual registers may have special behavior, defined here.

   // This is the bus error register, which traps WB errors
   // We latch every new BIU address in the upper 32 bits, so we always have the address for the transaction which
   // generated the error (the address counter might increment, esp. for writes)
   // We stop latching addresses when the error bit (bit 0) is set. Keep the error bit set until it is 
   // manually cleared by a module internal register write.
   // Note we use reg_select_data straight from data_register_i, rather than the latched version - 
   // otherwise, we would write the previously selected register.
   always @(posedge dbg_clk,posedge dbg_rst)
     if (dbg_rst) internal_reg_error <= 'h0;
     else if (intreg_ld_en && (reg_select_data == INTREG_ERROR))  // do load from data input register
     begin
         if (data_register[46]) internal_reg_error.error <= 1'b0;  // if write data is 1, reset the error bit  TODO:fix 46
     end
     else if (error_reg_en && !internal_reg_error.error)
     begin
         if      (biu_err || !biu_rdy) internal_reg_error.error <= 1'b1;	    
         else if (biu_strobe         ) internal_reg_error.addr  <= address_counter;
     end
     else if (biu_strobe && !internal_reg_error.error)
       internal_reg_error.addr <= address_counter;  // When no error, latch this whether error_reg_en or not


   ///////////////////////////////////////////////
   // Address counter

   // Technically, since this data (sometimes) comes from the input shift reg, we should latch on
   // negedge, per the JTAG spec. But that makes things difficult when incrementing.
   always @ (posedge dbg_clk,posedge dbg_rst)  // JTAG spec specifies latch on negative edge in UPDATE_DR state
     if      (dbg_rst   ) address_counter <= 'h0;
     else if (addr_ct_en) address_counter <= addr_sel ? address_counter + word_size_bytes : address_data_in;



  //////////////////////////////////////
  // Bit counter
   always @(posedge dbg_clk,posedge dbg_rst)
     if      (dbg_rst   ) bit_count <= 'h0;
     else if (bit_ct_rst) bit_count <= 'h0;
     else if (bit_ct_en ) bit_count <= bit_count + 'h1;

   assign bit_count_max = bit_count == word_size_bits;
   assign bit_count_32  = bit_count == 'd32;


  ////////////////////////////////////////
  // Word counter
  assign decremented_word_count = word_count - 'h1;

  // Technically, since this data (sometimes) comes from the input shift reg, we should latch on
  // negedge, per the JTAG spec. But that makes things difficult when incrementing.
  always @(posedge dbg_clk,posedge dbg_rst)  // JTAG spec specifies latch on negative edge in UPDATE_DR state
    if      (dbg_rst   ) word_count <= 'h0;
    else if (word_ct_en) word_count <= word_ct_sel ?  decremented_word_count : count_data_in; 

  assign word_count_zero = ~|word_count;


  /////////////////////////////////////////////////////
  // Output register and TDO output MUX
  always @(posedge dbg_clk,posedge dbg_rst)
    if      (dbg_rst         ) data_out_shift_reg <= 'h0;
    else if (out_reg_ld_en   ) data_out_shift_reg <= out_reg_data_sel ? data_from_internal_reg : {1'b0,biu_do};
    else if (out_reg_shift_en) data_out_shift_reg <= {1'b0, data_out_shift_reg[$bits(data_out_shift_reg)-1:1]};


   always_comb
     case (tdo_output_sel)
        2'h0   : dbg_tdo = biu_rdy;
        2'h1   : dbg_tdo = data_out_shift_reg[0];
        2'h2   : dbg_tdo = crc_match;
        default: dbg_tdo = crc_serial_out;
     endcase


  ////////////////////////////////////////
  // Bus Interface Unit
  // It is assumed that the BIU has internal registers, and will
  // latch address, operation, and write data on rising clock edge 
  // when strobe is asserted
  assign biu_clk       = dbg_clk;
  assign biu_rst       = dbg_rst | biu_clr_err;
  assign biu_di        = data_to_biu;
  assign biu_addr      = address_counter;
  assign biu_strb      = biu_strobe;
  assign biu_rw        = rd_op;
  assign biu_word_size = word_size_bytes;


  /////////////////////////////////////
  // CRC module

  assign crc_data_in = crc_in_sel ? dbg_tdi : data_out_shift_reg[0];  // MUX, write or read data

  adbg_crc32 wb_crc_i (
   .rstn       (~dbg_rst        ),
   .clk        ( dbg_clk        ), 
   .data       ( crc_data_in    ),
   .enable     ( crc_en         ),
   .shift      ( crc_shift_en   ),
   .clr        ( crc_clr        ),
   .crc_out    ( crc_data_out   ),
   .serial_out ( crc_serial_out ) );

  assign crc_match = data_register[DATAREG_LEN-1 -: 32] == crc_data_out;


   ////////////////////////////////////////
   // Control FSM

   // sequential part of the FSM
   always @(posedge dbg_clk,posedge dbg_rst)
     if   (dbg_rst) module_state <= STATE_idle;
     else           module_state <= module_next_state;


   // Determination of next state; purely combinatorial
   always_comb
     case(module_state)
       STATE_idle:
         if      (module_cmd && module_select && update_dr_i && burst_read ) module_next_state = STATE_Rbegin;
         else if (module_cmd && module_select && update_dr_i && burst_write) module_next_state = STATE_Wready;
         else                                                                module_next_state = STATE_idle;

       STATE_Rbegin:
         if (word_count_zero) module_next_state = STATE_idle;  // set up a burst of size 0, illegal.
         else                 module_next_state = STATE_Rready;
       STATE_Rready:
         if (module_select && capture_dr_i) module_next_state = STATE_Rstatus;
         else                               module_next_state = STATE_Rready;
       STATE_Rstatus:
         if      (update_dr_i) module_next_state = STATE_idle; 
         else if (biu_rdy    ) module_next_state = STATE_Rburst;
         else                  module_next_state = STATE_Rstatus;
       STATE_Rburst:
         if      (update_dr_i                     ) module_next_state = STATE_idle; 
         else if (bit_count_max && word_count_zero) module_next_state = STATE_Rcrc;
         else                                       module_next_state = STATE_Rburst;
       STATE_Rcrc:
         if (update_dr_i) module_next_state = STATE_idle;
         // This doubles as the 'recovery' state, so stay here until update_dr_i.
         else             module_next_state = STATE_Rcrc;    

       STATE_Wready:
         if      (word_count_zero              ) module_next_state = STATE_idle;
         else if (module_select && capture_dr_i) module_next_state = STATE_Wwait;
         else                                    module_next_state = STATE_Wready;
       STATE_Wwait:
         if      (update_dr_i                                  ) module_next_state = STATE_idle;  // client terminated early
         else if (module_select && data_register[DATAREG_LEN-1]) module_next_state = STATE_Wburst; // Got a start bit
         else                                                    module_next_state = STATE_Wwait;
       STATE_Wburst:
         if      (update_dr_i  )   module_next_state = STATE_idle;  // client terminated early
         else if (bit_count_max)
           if    (word_count_zero) module_next_state = STATE_Wcrc;
           else                    module_next_state = STATE_Wburst;
         else                      module_next_state = STATE_Wburst;
       STATE_Wstatus:
         if      (update_dr_i    ) module_next_state = STATE_idle;  // client terminated early    
         else if (word_count_zero) module_next_state = STATE_Wcrc;
         // can't wait until bus ready if multiple devices in chain...
         // Would have to read postfix_bits, then send another start bit and push it through
         // prefix_bits...potentially very inefficient.
         else                      module_next_state = STATE_Wburst;
	  
      STATE_Wcrc:
        if      (update_dr_i ) module_next_state = STATE_idle;  // client terminated early
        else if (bit_count_32) module_next_state = STATE_Wmatch;
        else                   module_next_state = STATE_Wcrc;    
  
       STATE_Wmatch:
         if (update_dr_i) module_next_state = STATE_idle;
         // This doubles as our recovery state, stay here until update_dr_i
         else             module_next_state = STATE_Wmatch;    

       default: module_next_state = STATE_idle;  // shouldn't actually happen...
     endcase


   // Outputs of state machine, pure combinatorial
   always_comb
     begin
         // Default everything to 0, keeps the case statement simple
         addr_sel         = 1'b1;  // Selects data for address_counter. 0 = data_register_i, 1 = incremented address count
         addr_ct_en       = 1'b0;  // Enable signal for address counter register
         op_reg_en        = 1'b0;  // Enable signal for 'operation' register
         bit_ct_en        = 1'b0;  // enable bit counter
         bit_ct_rst       = 1'b0;  // reset (zero) bit count register
         word_ct_sel      = 1'b1;  // Selects data for byte counter.  0 = data_register_i, 1 = decremented byte count
         word_ct_en       = 1'b0;  // Enable byte counter register
         out_reg_ld_en    = 1'b0;  // Enable parallel load of data_out_shift_reg
         out_reg_shift_en = 1'b0;  // Enable shift of data_out_shift_reg
         tdo_output_sel   = 2'b1;  // 1 = data reg, 0 = biu_rdy, 2 = crc_match, 3 = CRC data
         biu_strobe       = 1'b0;
         crc_clr          = 1'b0;
         crc_en           = 1'b0;  // add the input bit to the CRC calculation
         crc_in_sel       = 1'b0;  // 0 = tdo, 1 = tdi
         crc_shift_en     = 1'b0;
         out_reg_data_sel = 1'b1;  // 0 = BIU data, 1 = internal register data
         regsel_ld_en     = 1'b0;
         intreg_ld_en     = 1'b0;
         error_reg_en     = 1'b0;
         biu_clr_err      = 1'b0;  // Set this to reset the BIU, clearing the biu_err bit
         inhibit          = 1'b0;  // Don't disable the top-level module in the default case

         case (module_state)
           STATE_idle:
           begin
               addr_sel    = 1'b0;
               word_ct_sel = 1'b0;
       
               // Operations for internal registers - stay in idle state
               if (module_select & shift_dr_i) out_reg_shift_en = 1'b1; // For module regs

               if (module_select & capture_dr_i) 
               begin
                  out_reg_data_sel = 1'b1;  // select internal register data
                  out_reg_ld_en    = 1'b1;   // For module regs
               end

               if (module_select & module_cmd & update_dr_i)
               begin
                   if (intreg_instruction) regsel_ld_en = 1'b1;  // For module regs
                   if (intreg_write      ) intreg_ld_en = 1'b1;  // For module regs
               end
	       
               // Burst operations
               if (module_next_state != STATE_idle)
               begin  // Do the same to receive read or write opcode
                   addr_ct_en = 1'b1;
                   op_reg_en  = 1'b1;
                   bit_ct_rst = 1'b1;
                   word_ct_en = 1'b1;
                   crc_clr    = 1'b1;
               end
           end

           STATE_Rbegin:
           if(!word_count_zero)
           begin  // Start a biu read transaction
               biu_strobe = 1'b1;
               addr_sel   = 1'b1;
               addr_ct_en = 1'b1;
           end

           STATE_Rready: ; // Just a wait state
	  
           STATE_Rstatus:
           begin
               tdo_output_sel = 2'h0;
               inhibit        = 1'b1; // in case of early termination

               if (module_next_state == STATE_Rburst)
               begin
                   error_reg_en     = 1'b1; // Check the wb_error bit
                   out_reg_data_sel = 1'b0; // select BIU data
                   out_reg_ld_en    = 1'b1;
                   bit_ct_rst       = 1'b1;
                   word_ct_sel      = 1'b1;
                   word_ct_en       = 1'b1;

                   if (decremented_word_count != 0 && !word_count_zero)
                   begin  // Start a biu read transaction
                       biu_strobe = 1'b1;
                       addr_sel   = 1'b1;
                       addr_ct_en = 1'b1;
                   end
               end
           end

           STATE_Rburst:
           begin
               tdo_output_sel   = 2'h1;
               out_reg_shift_en = 1'b1;
               bit_ct_en        = 1'b1;
               crc_en           = 1'b1;
               crc_in_sel       = 1'b0;  // read data in output shift register LSB (tdo)
               inhibit          = 1'b1;  // in case of early termination
      
               if (bit_count_max)
               begin
                   error_reg_en     = 1'b1; // Check the wb_error bit
                   out_reg_data_sel = 1'b0; // select BIU data
                   out_reg_ld_en    = 1'b1;
                   bit_ct_rst       = 1'b1;
                   word_ct_sel      = 1'b1;
                   word_ct_en       = 1'b1;

                   if (decremented_word_count != 0 && !word_count_zero)  // Start a biu read transaction
                   begin
                       biu_strobe = 1'b1;
                       addr_sel   = 1'b1;
                       addr_ct_en = 1'b1;
                   end
               end
           end

           STATE_Rcrc:
           begin
               // Just shift out the data, don't bother counting, we don't move on until update_dr_i
               tdo_output_sel = 2'h3;
               crc_shift_en   = 1'b1;
               inhibit        = 1'b1;
           end

           STATE_Wready: ; // Just a wait state

           STATE_Wwait:
           begin
               tdo_output_sel = 2'h1;
               inhibit        = 1'b1;  // in case of early termination

               if (module_next_state == STATE_Wburst)
               begin
                   biu_clr_err = 1'b1; // If error occurred on last transaction of last burst, biu_err is still set.  Clear it.
                   bit_ct_en   = 1'b1;
                   word_ct_sel = 1'b1; // Pre-decrement the byte count
                   word_ct_en  = 1'b1;
                   crc_en      = 1'b1; // CRC gets dbg_tdi, which is 1 cycle ahead of data_register_i, so we need the bit there now in the CRC
                   crc_in_sel  = 1'b1; // read data from dbg_tdi
               end
           end

           STATE_Wburst:
           begin
               bit_ct_en      = 1'b1;
               tdo_output_sel = 2'h1;
               crc_en         = 1'b1;
               crc_in_sel     = 1'b1;   // read data from tdi_i
               inhibit        = 1'b1;   // in case of early termination

               if (bit_count_max)
               begin
                   error_reg_en = 1'b1; // Check the wb_error bit
                   bit_ct_rst   = 1'b1; // Zero the bit count

                   // start transaction. Can't do this here if not hispeed, biu_rdy
                   // is the status bit, and it's 0 if we start a transaction here.
                   biu_strobe   = 1'b1; // Start a BIU transaction
                   addr_ct_en   = 1'b1; // Increment thte address counter

                   // Also can't dec the byte count yet unless hispeed,
                   // that would skip the last word.
                   word_ct_sel  = 1'b1; // Decrement the byte count
                   word_ct_en   = 1'b1;
               end
           end

           STATE_Wstatus:
           begin
               tdo_output_sel = 2'h0; // Send the status bit to TDO
               error_reg_en   = 1'b1; // Check the wb_error bit

               // start transaction
               biu_strobe     = 1'b1; // Start a BIU transaction
               word_ct_sel    = 1'b1; // Decrement the byte count
               word_ct_en     = 1'b1;
               bit_ct_rst     = 1'b1; // Zero the bit count
               addr_ct_en     = 1'b1; // Increment thte address counter
               inhibit        = 1'b1; // in case of early termination
           end
	  
           STATE_Wcrc:
           begin
               bit_ct_en = 1'b1;
               inhibit   = 1'b1;    // in case of early termination
               if (module_next_state == STATE_Wmatch) tdo_output_sel = 2'h2;  // This is when the 'match' bit is actually read
           end
	  
           STATE_Wmatch:
           begin
               tdo_output_sel = 2'h2;
               inhibit        = 1'b1;

               // Bit of a hack here...an error on the final write won't be detected in STATE_Wstatus like the rest, 
               // so we assume the bus transaction is done and check it / latch it into the error register here.
               if (module_next_state == STATE_idle) error_reg_en = 1'b1;
           end

           default: ;
         endcase
     end
endmodule

