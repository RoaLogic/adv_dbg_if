//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_jsp_apb_biu.v                                          ////
////                                                              ////
////                                                              ////
////  This file is part of the SoC Debug Interface.               ////
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
//
// This is where the magic happens in the JTAG Serial Port.  The serial
// port FIFOs and counters are kept in the WishBone clock domain.
// 'Syncflop' elements are used to synchronize strobe lines across
// clock domains, and 'syncreg' elements keep the byte and free count
// as current as possible in the JTAG clock domain.  Also in the APB
// clock domain is an APB slave interface, which more or less
// tries to emulate a 16550 without FIFOs (despite the fact that
// FIFOs are actually present, they are opaque to the APB interface.)
//


// Top module
module adbg_jsp_apb_biu
(
  // Debug interface signals
  input         tck_i,
  input         rst_i,
  input  [ 7:0] data_i,
  output [ 7:0] data_o,
  output [ 3:0] bytes_available_o,
  output [ 3:0] bytes_free_o,
  input         rd_strobe_i,
                wr_strobe_i,

  // APB signals
  input            PRESETn,
                   PCLK,
										 
  input            PSEL,
  input            PENABLE,
  input            PWRITE,
  input      [2:0] PADDR,
  input      [7:0] PWDATA,
  output reg [7:0] PRDATA,
  output           PREADY,
  output           PSLVERR,

  output           int_o
);

  //////////////////////////////////////////////////////////////////
  //
  // Constants
  //
  import adbg_jsp_16550_pkg::*;

  typedef enum logic [1:0] {RD_IDLE,RD_PUSH,RD_POP,RD_LATCH} rd_states;
  typedef enum logic [1:0] {WR_IDLE,WR_PUSH,WR_POP         } wr_states;

 
  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //

  // Registers
  reg  [7:0] data_in;
  reg  [7:0] rdata;
  reg        wen_tff;
  reg        ren_tff;
 
  // Wires  
  wire       fifo_ack;
  wire [3:0] wr_bytes_free;
  wire [3:0] rd_bytes_avail;
  wire [3:0] wr_bytes_avail;  // used to generate wr_fifo_not_empty
  wire       rd_bytes_avail_not_zero;
  wire       ren_sff_out;   
  wire [7:0] rd_fifo_data_out;
  wire [7:0] data_to_extbus;
  wire [7:0] data_from_extbus;
  wire       wr_fifo_not_empty;  // this is for the WishBone interface LSR register
  wire       rx_fifo_rst;  // rcvr in the APB sense, opposite most of the rest of this file
  wire       tx_fifo_rst;  // ditto
   
  // Control Signals (FSM outputs)
  reg        wda_rst;   // reset wdata_avail SFF
  reg        wpp;       // Write FIFO PUSH (1) or POP (0)
  reg        w_fifo_en; // Enable write FIFO
  reg        ren_rst;   // reset 'pop' SFF
  reg        rdata_en;  // enable 'rdata' register
  reg        rpp;       // read FIFO PUSH (1) or POP (0)
  reg        r_fifo_en; // enable read FIFO    
  reg        r_wb_ack;  // read FSM acks APB transaction
  reg        w_wb_ack;  // write FSM acks APB transaction

  // Indicators to FSMs
  wire       wdata_avail; // JTAG side has data available
  wire       fifo_rd;     // ext.bus requests read
  wire       fifo_wr;     // ext.bus requests write
  wire       pop;         // JTAG side received a byte, pop and get next
  wire       rcz;         // zero bytes available in read FIFO
  

  rd_states  rd_fsm_state, next_rd_fsm_state;
  wr_states  wr_fsm_state, next_wr_fsm_state;
 
   
  //////////////////////////////////////////////////////////////////
  //
  // Module body
  //

  //////////////////////////////////////////////////////
  // TCK clock domain
  // There is no FSM here, just signal latching and clock
  // domain synchronization

  assign data_o = rdata;

  // Write enable (WEN) toggle FF
  always @(posedge tck_i,posedge rst_i)
    if      (rst_i      ) wen_tff <= 'b0;
    else if (wr_strobe_i) wen_tff <= ~wen_tff;


  // Read enable (REN) toggle FF
  always @(posedge tck_i,posedge rst_i)
    if      (rst_i      ) ren_tff <= 'b0;
    else if (rd_strobe_i) ren_tff <= ~ren_tff;

  // Write data register
  always @(posedge tck_i,posedge rst_i)
    if      (rst_i      ) data_in <= 'h0;
    else if (wr_strobe_i) data_in <= data_i;
   

  ///////////////////////////////////////////////////////
  // APB clock domain

  // Combinatorial assignments
  assign rd_bytes_avail_not_zero = |rd_bytes_avail;
  assign pop                     =  ren_sff_out & rd_bytes_avail_not_zero;
  assign rcz                     = ~rd_bytes_avail_not_zero;
  assign fifo_ack                =  r_wb_ack | w_wb_ack;
  assign wr_fifo_not_empty       = |wr_bytes_avail;
       
  // rdata register
  always @(posedge PCLK,negedge PRESETn)
    if      (!PRESETn ) rdata <= 'h0;
    else if ( rdata_en) rdata <= rd_fifo_data_out;
        
  // WEN SFF
  syncflop wen_sff (
    .RESET     ( rst_i       ),
    .DEST_CLK  ( PCLK        ),
    .D_SET     ( 1'b0        ),
    .D_RST     ( wda_rst     ),
    .TOGGLE_IN ( wen_tff     ),
    .D_OUT     ( wdata_avail )
  );
   
  // REN SFF
  syncflop ren_sff (
    .RESET     ( rst_i       ),
    .DEST_CLK  ( PCLK        ),
    .D_SET     ( 1'b0        ),
    .D_RST     ( ren_rst     ),
    .TOGGLE_IN ( ren_tff     ),
    .D_OUT     ( ren_sff_out )
  );
 
//TODO syncreg.RST should be synchronised to DFF clock domain
  // 'free space available' syncreg
  syncreg freespace_syncreg (
    .RST      ( rst_i         ),
    .CLKA     ( PCLK          ),
    .CLKB     ( tck_i         ),
    .DATA_IN  ( wr_bytes_free ),
    .DATA_OUT ( bytes_free_o  )
   );
   
  // 'bytes available' syncreg
  syncreg bytesavail_syncreg (
    .RST      ( rst_i             ),
    .CLKA     ( PCLK              ),
    .CLKB     ( tck_i             ),
    .DATA_IN  ( rd_bytes_avail    ),
    .DATA_OUT ( bytes_available_o )
  );

  //TODO synch. FIFO resets
  // write FIFO
  bytefifo wr_fifo (
    .RST         ( rst_i | rx_fifo_rst ), // rst_i from JTAG clk domain, rx_fifo_rst from APB, RST is async reset
    .CLK         ( PCLK                ),
    .DATA_IN     ( data_in             ),
    .DATA_OUT    ( data_to_extbus      ),
    .PUSH_POPn   ( wpp                 ),
    .EN          ( w_fifo_en           ),
    .BYTES_AVAIL ( wr_bytes_avail      ),
    .BYTES_FREE  ( wr_bytes_free       )
  );
   
  // read FIFO
  bytefifo rd_fifo (
    .RST         ( rst_i | tx_fifo_rst ), // rst_i from JTAG clk domain, tx_fifo_rst from APB, RST is async reset
    .CLK         ( PCLK                ),
    .DATA_IN     ( data_from_extbus    ),
    .DATA_OUT    ( rd_fifo_data_out    ),
    .PUSH_POPn   ( rpp                 ),
    .EN          ( r_fifo_en           ),
    .BYTES_AVAIL ( rd_bytes_avail      ),
    .BYTES_FREE  ( )
  );			      


  /////////////////////////////////////////////////////
  // State machine for the read FIFO

  // Sequential bit
  always @(posedge PCLK,negedge PRESETn)
    if (!PRESETn) rd_fsm_state <= RD_IDLE;
    else          rd_fsm_state <= next_rd_fsm_state; 


  // Determination of next state (combinatorial)
  always_comb
    case (rd_fsm_state)
      RD_IDLE:
        if      (fifo_wr) next_rd_fsm_state = RD_PUSH;
        else if (pop    ) next_rd_fsm_state = RD_POP;
        else              next_rd_fsm_state = RD_IDLE;

      RD_PUSH:
        if      (rcz    ) next_rd_fsm_state = RD_LATCH;  // putting first item in fifo, move to rdata in state LATCH
        else if (pop    ) next_rd_fsm_state = RD_POP;
        else              next_rd_fsm_state = RD_IDLE;

      RD_POP:             next_rd_fsm_state = RD_LATCH; // new data at FIFO head, move to rdata in state LATCH

      RD_LATCH:
        if      (fifo_wr) next_rd_fsm_state = RD_PUSH;
        else if (pop    ) next_rd_fsm_state = RD_POP;
        else              next_rd_fsm_state = RD_IDLE;

      default:            next_rd_fsm_state = RD_IDLE;
    endcase


   // Outputs of state machine (combinatorial)
   always_comb
    begin
        ren_rst   = 1'b0;
        rpp       = 1'b0;
        r_fifo_en = 1'b0;
        rdata_en  = 1'b0;
        r_wb_ack  = 1'b0;

        case (rd_fsm_state)
          RD_PUSH:
          begin
              rpp       = 1'b1;
              r_fifo_en = 1'b1;
              r_wb_ack  = 1'b1;
          end
	  
          RD_POP:
          begin
              ren_rst   = 1'b1;
              r_fifo_en = 1'b1;
          end
	  
          RD_LATCH: rdata_en = 1'b1;

          default: ;
        endcase
    end


  /////////////////////////////////////////////////////
  // State machine for the write FIFO

  // Sequential bit
  always @(posedge PCLK,negedge PRESETn)
    if (!PRESETn) wr_fsm_state <= WR_IDLE;
    else          wr_fsm_state <= next_wr_fsm_state; 


  // Determination of next state (combinatorial)
  always_comb
    case (wr_fsm_state)
      WR_IDLE:
        if      (fifo_rd    ) next_wr_fsm_state = WR_POP;
        else if (wdata_avail) next_wr_fsm_state = WR_PUSH;
        else                  next_wr_fsm_state = WR_IDLE;

      WR_PUSH:
        if      (fifo_rd    ) next_wr_fsm_state = WR_POP;
        else                  next_wr_fsm_state = WR_IDLE;

      WR_POP:
        if      (wdata_avail) next_wr_fsm_state = WR_PUSH;
        else                  next_wr_fsm_state = WR_IDLE;

      default:                next_wr_fsm_state = WR_IDLE;
    endcase


   // Outputs of state machine (combinatorial)
   always_comb
    begin
        wda_rst   = 1'b0;
        wpp       = 1'b0;
        w_fifo_en = 1'b0;
        w_wb_ack  = 1'b0;

        case (wr_fsm_state)
           WR_PUSH:
           begin
               wda_rst   = 1'b1;
               wpp       = 1'b1;
               w_fifo_en = 1'b1;
           end
  
           WR_POP:
           begin
               w_wb_ack  = 1'b1;
               w_fifo_en = 1'b1;
           end

           default: ;
  
        endcase
    end


  ////////////////////////////////////////////////////////////
  // Interface hardware & 16550 registers
  // Interface signals to read and write fifos:
  // fifo_rd : read strobe
  // fifo_wr : write strobe
  // fifo_ack: fifo has completed operation

  //16550 registers
  ier_struct ier;
  iir_struct iir;
//  fcr_struct fcr;
  lcr_struct lcr;
  mcr_struct mcr;
  lsr_struct lsr;
  msr_struct msr;
  scr_struct scr;


  reg  reg_ack;
  wire rd_fifo_not_full;  // "rd fifo" is the one the APB writes to
  wire rd_fifo_becoming_empty;
  reg  thr_int_arm;       // used so that an IIR read can clear a transmit interrupt
  wire iir_read;
   

  // These 16550 registers are not implemented
  assign mcr = 'h0;
  assign msr = 'hb;

  // Create the simple / combinatorial registers
  assign rd_fifo_not_full = !(rd_bytes_avail == 4'h8);
  assign lsr              = {1'b0, rd_fifo_not_full, rd_fifo_not_full, 4'h0, wr_fifo_not_empty};   

  // Create writeable registers
  always @(posedge PCLK,negedge PRESETn)
    if (!PRESETn)
    begin
        ier <= 'h0;
        lcr <= 'h0;
        scr <= 'h0;
    end
    else if (PSEL & PWRITE & PENABLE)
      case (PADDR)
         3'b001: if (!lcr.dlab) ier <= PWDATA[3:0];
         3'b011:                lcr <= PWDATA;
         3'b111:                scr <= PWDATA;
      endcase

    
  // Create handshake signals to/from the FIFOs
  // Access FIFO during APB-Setup Phase, so we acknowledge during APB-Access phase
  assign fifo_rd  = PSEL & ~PENABLE & ~PWRITE & (PADDR == 3'b000) & ~lcr.dlab;
  assign fifo_wr  = PSEL & ~PENABLE &  PWRITE & (PADDR == 3'b000) & ~lcr.dlab;

  // APB responses
  assign PREADY  = fifo_ack | reg_ack;
  assign PSLVERR = 1'b0;

  // acknowledge all accesses, except to FIFOs
  always @(posedge PCLK)
    reg_ack <= PSEL & ~PENABLE & (lcr.dlab | PADDR != 3'b000);


  // Create FIFO reset signals
  assign rx_fifo_rst = PSEL & PENABLE & PWRITE & (PADDR == 3'b010) & PWDATA[1];
  assign tx_fifo_rst = PSEL & PENABLE & PWRITE & (PADDR == 3'b010) & PWDATA[2];


  // Create IIR (and THR INT arm bit)
  assign rd_fifo_becoming_empty = r_fifo_en & (~rpp) & (rd_bytes_avail == 4'h1);  // "rd fifo" is the ext.bus write FIFO...

  assign iir_read = PSEL & PENABLE & ~PWRITE & (PADDR == 3'b010);

   
  always @(posedge PCLK,negedge PRESETn)
    if      (!PRESETn                           ) thr_int_arm <= 1'b0;
    else if (fifo_wr  ||  rd_fifo_becoming_empty) thr_int_arm <= 1'b1;  // Set when APB write fifo becomes empty, or on a write to it
    else if (iir_read && !wr_fifo_not_empty     ) thr_int_arm <= 1'b0;


  always_comb
    if      (wr_fifo_not_empty              ) iir = 'b100;
    else if (thr_int_arm && rd_fifo_not_full) iir = 'b010;
    else                                      iir = 'b001;
   
   
  // Create ext.bus Data Out
  always_comb
    case (PADDR)
      3'b000 : PRDATA = data_to_extbus;
      3'b001 : PRDATA = {4'h0, ier};
      3'b010 : PRDATA = iir;
      3'b011 : PRDATA = lcr;
      3'b100 : PRDATA = mcr;
      3'b101 : PRDATA = lsr;
      3'b110 : PRDATA = msr;
      3'b111 : PRDATA = scr;
      default: PRDATA = 'h0;
    endcase

   assign data_from_extbus = PWDATA;  // Data to the FIFO

   // Generate interrupt output
   assign int_o = (rd_fifo_not_full & thr_int_arm & ier.etbei) | (wr_fifo_not_empty & ier.erbfi);
endmodule

