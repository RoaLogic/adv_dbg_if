//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_jsp_biu.v                                              ////
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
// as current as possible in the JTAG clock domain.  Also in the WB
// clock domain is a WishBone target interface, which more or less
// tries to emulate a 16550 without FIFOs (despite the fact that
// FIFOs are actually present, they are opaque to the WB interface.)
//


// Top module
module adbg_jsp_wb_biu
#(
  parameter ADDR_WIDTH = 32,
  parameter DATA_WIDTH = 32
)
(
// Debug interface signals
  input                     tck_i,
  input                     rst_i,
  input  [             7:0] data_i,
  output [             7:0] data_o,
  output [             3:0] bytes_available_o,
  output [             3:0] bytes_free_o,
  input                     rd_strobe_i,
                            wr_strobe_i,

  // Wishbone signals
  input                     wb_clk_i,
  input                     wb_rst_i,
  input                     wb_cyc_i,
  input                     wb_stb_i,
  input                     wb_we_i,
  input  [ADDR_WIDTH  -1:0] wb_adr_i,
  input  [DATA_WIDTH/8-1:0] wb_sel_i,
  input  [DATA_WIDTH  -1:0] wb_dat_i,
  output [DATA_WIDTH  -1:0] wb_dat_o,
  output                    wb_ack_o,
  output                    wb_err_o,
  output                    int_o
);

  //////////////////////////////////////////////////////////////////
  //
  // Constants
  //

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
  wire wb_fifo_ack;
  wire [3:0] wr_bytes_free;
  wire [3:0] rd_bytes_avail;
  wire [3:0] wr_bytes_avail;  // used to generate wr_fifo_not_empty
  wire       rd_bytes_avail_not_zero;
  wire       ren_sff_out;   
  wire [7:0] rd_fifo_data_out;
  wire [7:0] data_to_wb;
  wire [7:0] data_from_wb;
  wire       wr_fifo_not_empty;  // this is for the WishBone interface LSR register
  wire       rx_fifo_rst;  // rcvr in the WB sense, opposite most of the rest of this file
  wire       tx_fifo_rst;  // ditto
   
  // Control Signals (FSM outputs)
  reg        wda_rst;   // reset wdata_avail SFF
  reg        wpp;       // Write FIFO PUSH (1) or POP (0)
  reg        w_fifo_en; // Enable write FIFO
  reg        ren_rst;   // reset 'pop' SFF
  reg        rdata_en;  // enable 'rdata' register
  reg        rpp;       // read FIFO PUSH (1) or POP (0)
  reg        r_fifo_en; // enable read FIFO    
  reg        r_wb_ack;  // read FSM acks WB transaction
  reg        w_wb_ack;  // write FSM acks WB transaction

  // Indicators to FSMs
  wire       wdata_avail; // JTAG side has data available
  wire       wb_rd;       // WishBone requests read
  wire       wb_wr;       // WishBone requests write
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
  // Wishbone clock domain

  // Combinatorial assignments
  assign rd_bytes_avail_not_zero = |rd_bytes_avail;
  assign pop                     =  ren_sff_out & rd_bytes_avail_not_zero;
  assign rcz                     = ~rd_bytes_avail_not_zero;
  assign wb_fifo_ack             =  r_wb_ack | w_wb_ack;
  assign wr_fifo_not_empty       = |wr_bytes_avail;
       
  // rdata register
  always @(posedge wb_clk_i,posedge rst_i)
    if      (rst_i   ) rdata = 8'h0;
    else if (rdata_en) rdata <= rd_fifo_data_out;
        
  // WEN SFF
  syncflop wen_sff (
    .RESET     ( rst_i       ),
    .DEST_CLK  ( wb_clk_i    ),
    .D_SET     ( 1'b0        ),
    .D_RST     ( wda_rst     ),
    .TOGGLE_IN ( wen_tff     ),
    .D_OUT     ( wdata_avail )
  );
   
  // REN SFF
  syncflop ren_sff (
    .RESET     ( rst_i       ),
    .DEST_CLK  ( wb_clk_i    ),
    .D_SET     ( 1'b0        ),
    .D_RST     ( ren_rst     ),
    .TOGGLE_IN ( ren_tff     ),
    .D_OUT     ( ren_sff_out )
  );
 
//TODO syncreg.RST should be synchronised to DFF clock domain
  // 'free space available' syncreg
  syncreg freespace_syncreg (
    .RST      ( rst_i         ),
    .CLKA     ( wb_clk_i      ),
    .CLKB     ( tck_i         ),
    .DATA_IN  ( wr_bytes_free ),
    .DATA_OUT ( bytes_free_o  )
   );
   
  // 'bytes available' syncreg
  syncreg bytesavail_syncreg (
    .RST      ( rst_i             ),
    .CLKA     ( wb_clk_i          ),
    .CLKB     ( tck_i             ),
    .DATA_IN  ( rd_bytes_avail    ),
    .DATA_OUT ( bytes_available_o )
  );

  //TODO synch. FIFO resets
  // write FIFO
  bytefifo wr_fifo (
    .RST         ( rst_i | rx_fifo_rst ), // rst_i from JTAG clk domain, rx_fifo_rst from WB, RST is async reset
    .CLK         ( wb_clk_i            ),
    .DATA_IN     ( data_in             ),
    .DATA_OUT    ( data_to_wb          ),
    .PUSH_POPn   ( wpp                 ),
    .EN          ( w_fifo_en           ),
    .BYTES_AVAIL ( wr_bytes_avail      ),
    .BYTES_FREE  ( wr_bytes_free       )
  );
   
  // read FIFO
  bytefifo rd_fifo (
    .RST         ( rst_i | txt_fifo_rst), // rst_i from JTAG clk domain, tx_fifo_rst from WB, RST is async reset
    .CLK         ( wb_clk_i            ),
    .DATA_IN     ( data_from_wb        ),
    .DATA_OUT    ( rd_fifo_data_out    ),
    .PUSH_POPn   ( rpp                 ),
    .EN          ( r_fifo_en           ),
    .BYTES_AVAIL ( rd_bytes_avail      ),
    .BYTES_FREE  ( )
  );			      


  /////////////////////////////////////////////////////
  // State machine for the read FIFO

  // Sequential bit
  always @(posedge wb_clk_i,posedge rst_i)
    if (rst_i) rd_fsm_state <= RD_IDLE;
    else       rd_fsm_state <= next_rd_fsm_state; 


  // Determination of next state (combinatorial)
  always_comb
    case (rd_fsm_state)
      RD_IDLE:
        if      (wb_wr) next_rd_fsm_state = RD_PUSH;
        else if (pop  ) next_rd_fsm_state = RD_POP;
        else            next_rd_fsm_state = RD_IDLE;

      RD_PUSH:
        if      (rcz  ) next_rd_fsm_state = RD_LATCH;  // putting first item in fifo, move to rdata in state LATCH
        else if (pop  ) next_rd_fsm_state = RD_POP;
        else            next_rd_fsm_state = RD_IDLE;

      RD_POP:           next_rd_fsm_state = RD_LATCH; // new data at FIFO head, move to rdata in state LATCH

      RD_LATCH:
        if      (wb_wr) next_rd_fsm_state = RD_PUSH;
        else if (pop  ) next_rd_fsm_state = RD_POP;
        else            next_rd_fsm_state = RD_IDLE;

      default:          next_rd_fsm_state = RD_IDLE;
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
  always @(posedge wb_clk_i,posedge rst_i)
    if (rst_i) wr_fsm_state <= WR_IDLE;
    else       wr_fsm_state <= next_wr_fsm_state; 


  // Determination of next state (combinatorial)
  always_comb
    case (wr_fsm_state)
      WR_IDLE:
        if      (wb_rd      ) next_wr_fsm_state = WR_POP;
        else if (wdata_avail) next_wr_fsm_state = WR_PUSH;
        else                  next_wr_fsm_state = WR_IDLE;

      WR_PUSH:
        if      (wb_rd      ) next_wr_fsm_state = WR_POP;
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
  // WishBone interface hardware
  // Interface signals to read and write fifos:
  // wb_rd:  read strobe
  // wb_wr:  write strobe
  // wb_fifo_ack: fifo has completed operation

  wire [DATA_WIDTH-1:0] bus_data_lo,
                        bus_data_hi;
  wire                  wb_reg_ack;
  wire                  rd_fifo_not_full;  // "rd fifo" is the one the WB writes to
  reg  [           2:0] iir_gen;           // actually combinatorial
  wire                  rd_fifo_becoming_empty;
   
  // These 16550 registers are at least partly implemented
  reg                   reg_dlab_bit;      // part of the LCR
  reg  [           3:0] reg_ier;
  wire [           2:0] reg_iir;
  reg                   thr_int_arm;       // used so that an IIR read can clear a transmit interrupt
  wire [           7:0] reg_lsr;
  wire                  reg_dlab_bit_wren;
  wire                  reg_ier_wren;
  wire                  reg_iir_rden;
  wire [           7:0] reg_lcr;           // the DLAB bit above is the 8th bit
  wire                  reg_fcr_wren;      // FCR is WR-only, at the same address as the IIR (contains SW reset bits)
   
  // These 16550 registers are not implemented
  wire [           7:0] reg_mcr = 'h00;  // These bits control modem control lines, unused here
  wire [           7:0] reg_msr = 'hb0;  // CD, DSR, CTS true, RI false, indicate no changes
  wire [           7:0] reg_scr = 'h00;  // scratch register


  // Create handshake signals to/from the FIFOs
  assign wb_rd    = wb_cyc_i & wb_stb_i & ~wb_we_i & wb_sel_i[3] & (wb_adr_i[1:0] == 2'b00) & ~reg_dlab_bit;
  assign wb_wr    = wb_cyc_i & wb_stb_i &  wb_we_i & wb_sel_i[3] & (wb_adr_i[1:0] == 2'b00) & ~reg_dlab_bit;
  assign wb_ack_o = wb_fifo_ack | wb_reg_ack;
  assign wb_err_o = 1'b0;


  // Create the simple / combinatorial registers
  assign rd_fifo_not_full = !(rd_bytes_avail == 4'h8);
  assign reg_lcr          = {reg_dlab_bit, 7'h03};  // Always set for 8n1
  assign reg_lsr          = {1'b0, rd_fifo_not_full, rd_fifo_not_full, 4'h0, wr_fifo_not_empty};   


  // Create enable bits for the 16550 registers that we actually implement
  assign reg_dlab_bit_wren = wb_cyc_i & wb_stb_i &  wb_we_i & wb_sel_i[0] & (wb_adr_i[2:0] == 3'b011);
  assign reg_ier_wren      = wb_cyc_i & wb_stb_i &  wb_we_i & wb_sel_i[2] & (wb_adr_i[2:0] == 3'b001) & ~reg_dlab_bit;
  assign reg_iir_rden      = wb_cyc_i & wb_stb_i & ~wb_we_i & wb_sel_i[1] & (wb_adr_i[2:0] == 3'b010);
  assign wb_reg_ack        = wb_cyc_i & wb_stb_i &           |wb_sel_i    & (reg_dlab_bit | (wb_adr_i[2:0] != 3'b000));
  assign reg_fcr_wren      = wb_cyc_i & wb_stb_i &  wb_we_i & wb_sel_i[1] & (wb_adr_i[2:0] == 3'b010);
  assign rx_fifo_rst       = reg_fcr_wren & wb_dat_i[9];
  assign tx_fifo_rst       = reg_fcr_wren & wb_dat_i[10];
   
  // Create DLAB bit
  always @(posedge wb_clk_i)
    if      (wb_rst_i         ) reg_dlab_bit <= 1'b0;
    else if (reg_dlab_bit_wren) reg_dlab_bit <= wb_dat_i[7];


  // Create IER.  We only use the two LS bits...
  always @(posedge wb_clk_i)
    if      (wb_rst_i    ) reg_ier <= 4'h0;
    else if (reg_ier_wren) reg_ier <= wb_dat_i[19:16];


  // Create IIR (and THR INT arm bit)
  assign rd_fifo_becoming_empty = r_fifo_en & (~rpp) & (rd_bytes_avail == 4'h1);  // "rd fifo" is the WB write FIFO...

   
  always @(posedge wb_clk_i)
    if      (wb_rst_i                          ) thr_int_arm <= 1'b0;
    else if (wb_wr || rd_fifo_becoming_empty   ) thr_int_arm <= 1'b1;  // Set when WB write fifo becomes empty, or on a write to it
    else if (reg_iir_rden && !wr_fifo_not_empty) thr_int_arm <= 1'b0;


  always @(thr_int_arm or rd_fifo_not_full or wr_fifo_not_empty)
    if      (wr_fifo_not_empty              ) iir_gen <= 3'b100;
    else if (thr_int_arm && rd_fifo_not_full) iir_gen <= 3'b010;
    else                                      iir_gen <= 3'b001;
   
   assign reg_iir = iir_gen;
   

   // Create the data lines out to the WB.
   // Always put all 4 bytes on the WB data lines, let the master pick out what it wants.   
   assign bus_data_lo  = {data_to_wb, {4'h0, reg_ier}, {5'h0, reg_iir}, reg_lcr};
   assign bus_data_hi  = {reg_mcr, reg_lsr, reg_msr, reg_scr};
   assign wb_dat_o     = (wb_adr_i[2]) ? bus_data_hi : bus_data_lo;

   assign data_from_wb = wb_dat_i[31:24];  // Data to the FIFO


   // Generate interrupt output
   assign int_o = (rd_fifo_not_full & thr_int_arm & reg_ier[1]) | (wr_fifo_not_empty & reg_ier[0]);
endmodule

