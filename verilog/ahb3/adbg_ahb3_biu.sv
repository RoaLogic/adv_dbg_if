//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_ahb3_biu.sv                                            ////
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
//// Copyright (C) 2008-2016  Authors                             ////
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

// Top module
module adbg_ahb3_biu #(
  parameter LITTLE_ENDIAN = 1,
  parameter ADDR_WIDTH    = 32,
  parameter DATA_WIDTH    = 32
)
(
  // Debug interface signals
  input                       biu_clk,
  input                       biu_rst,
  input      [DATA_WIDTH-1:0] biu_di,
  output reg [DATA_WIDTH-1:0] biu_do,
  input      [ADDR_WIDTH-1:0] biu_addr,
  input                       biu_strb,
  input                       biu_rw,
  output reg                  biu_rdy,
  output reg                  biu_err,
  input      [           3:0] biu_word_size,


  // AHB Master signals
  input                       HCLK,
                              HRESETn,
  output                      HSEL,
  output reg [ADDR_WIDTH-1:0] HADDR,
  output reg [DATA_WIDTH-1:0] HWDATA,
  input      [DATA_WIDTH-1:0] HRDATA,
  output reg                  HWRITE,
  output reg [           2:0] HSIZE,
  output     [           2:0] HBURST,
  output     [           3:0] HPROT,
  output reg [           1:0] HTRANS,
  output                      HMASTLOCK,
  input                       HREADY,
  input                       HRESP
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
  logic [DATA_WIDTH-1:0] data_out_reg;  // AHB->dbg
  logic                  str_sync;      // This is 'active-toggle' rather than -high or -low.
  logic                  rdy_sync;      // ditto, active-toggle


   // Sync registers.  TFF indicates TCK domain, AFF indicates AHB domain
   logic [1:0] ahb_rstn_sync;
   logic       ahb_rstn;
   logic       rdy_sync_tff1,
               rdy_sync_tff2,
               rdy_sync_tff2q,   // used to detect toggles
               str_sync_aff1,
               str_sync_aff2,
               str_sync_aff2q;   // used to detect toggles

   // Internal signals
   logic       start_toggle,      // AHB domain, indicates a toggle on the start strobe
               start_toggle_hold, // hold start_toggle if AHB bus busy (not-ready)
               ahb_transfer_ack;  // AHB bus responded to data transfer

   //AHB FSM
   enum logic [1:0] {IDLE,ADDRESS,DATA} ahb_fsm_state;


  //////////////////////////////////////////////////////////////////
  //
  // Module Body
  //


  //////////////////////////////////////////////////////
  // TCK clock domain
  //
  // There is no FSM here, just signal latching and clock domain synchronization

  // Create byte enable signals from word_size and address
  always @(posedge biu_clk)
    if (biu_strb && biu_rdy)
      case (biu_word_size)
         'h1    : HSIZE <= HSIZE_BYTE;
         'h2    : HSIZE <= HSIZE_HWORD;
         'h4    : HSIZE <= HSIZE_WORD;
         default: HSIZE <= HSIZE_DWORD;
      endcase


generate
  if (DATA_WIDTH == 32)
  begin
    always @(posedge biu_clk)
      if (biu_strb && biu_rdy)
        case (biu_word_size)
           'h1    : HWDATA <= {4{biu_di[31 -:  8]}};
           'h2    : HWDATA <= {2{biu_di[31 -: 16]}};
           default: HWDATA <= biu_di;
        endcase
  end
  else //DATA_WIDTH == 64
  begin
    always @(posedge biu_clk)
      if (biu_strb && biu_rdy)
        case (biu_word_size)
           'h1    : HWDATA <= {8{biu_di[63 -:  8]}};
           'h2    : HWDATA <= {4{biu_di[63 -: 16]}};
           'h4    : HWDATA <= {2{biu_di[63 -: 32]}};
           default: HWDATA <= biu_di;
        endcase
  end
endgenerate


  // Latch input data on 'start' strobe, if ready.
  always @(posedge biu_clk,posedge biu_rst)
    if(biu_rst)
    begin
        HADDR  <= 'h0;
        HWRITE <= 'b0;
    end
    else if (biu_strb && biu_rdy)
    begin
        HADDR  <=  biu_addr;
        HWRITE <= ~biu_rw;
    end 


  // Create toggle-active strobe signal for clock sync.  This will start a transaction
  // on the AHB once the toggle propagates to the FSM in the AHB domain.
  always @(posedge biu_clk,posedge biu_rst)
    if      (biu_rst            ) str_sync <= 1'b0;
    else if (biu_strb && biu_rdy) str_sync <= ~str_sync;


  // Create biu_rdy output.  Set on reset, clear on strobe (if set), set on input toggle
  always @ (posedge biu_clk,posedge biu_rst)
    if(biu_rst)
    begin
        rdy_sync_tff1  <= 1'b0;
        rdy_sync_tff2  <= 1'b0;
        rdy_sync_tff2q <= 1'b0;
        biu_rdy        <= 1'b1;
    end
    else
    begin  
        rdy_sync_tff1  <= rdy_sync;       // Synchronize the ready signal across clock domains
        rdy_sync_tff2  <= rdy_sync_tff1;
        rdy_sync_tff2q <= rdy_sync_tff2;  // used to detect toggles

        if      (biu_strb && biu_rdy            ) biu_rdy <= 1'b0;
        else if (rdy_sync_tff2 != rdy_sync_tff2q) biu_rdy <= 1'b1;
    end



  ///////////////////////////////////////////////////////
  // AHB clock domain
  //

  // synchronize asynchronous active high reset
  always @(posedge HCLK,posedge biu_rst)
    if (biu_rst) ahb_rstn_sync <= {$bits(ahb_rstn_sync){1'b0}};
    else         ahb_rstn_sync <= {1'b1,ahb_rstn_sync[$bits(ahb_rstn_sync)-1:1]};

  assign ahb_rstn = ~(~HRESETn | ~ahb_rstn_sync[0]);


  // synchronize the start strobe
  always @(posedge HCLK,negedge ahb_rstn)
    if(!ahb_rstn)
    begin
        str_sync_aff1  <= 1'b0;
        str_sync_aff2  <= 1'b0;
        str_sync_aff2q <= 1'b0;      
   end
   else
   begin
       str_sync_aff1  <= str_sync;
       str_sync_aff2  <= str_sync_aff1;
       str_sync_aff2q <= str_sync_aff2;  // used to detect toggles
   end

  assign start_toggle = (str_sync_aff2 != str_sync_aff2q);


  always @(posedge HCLK,negedge ahb_rstn)
    if (!ahb_rstn) start_toggle_hold <= 1'b0;
    else           start_toggle_hold <= ~ahb_transfer_ack & (start_toggle | start_toggle_hold);


  // Bus Error register
  always @(posedge HCLK,negedge ahb_rstn)
    if      (!ahb_rstn        ) biu_err <= 1'b0;
    else if ( ahb_transfer_ack) biu_err <= HRESP; 


  // Received data register
generate
  if (DATA_WIDTH == 32)
  begin
    always @(posedge HCLK)
      if (ahb_transfer_ack)
        case (biu_word_size)
           'h1    : case (HADDR[1:0])
                       2'b00: biu_do <= LITTLE_ENDIAN ? {24'h0, HRDATA[ 7 -: 8]} : {24'h0, HRDATA[31 -: 8]};
                       2'b01: biu_do <= LITTLE_ENDIAN ? {24'h0, HRDATA[15 -: 8]} : {24'h0, HRDATA[23 -: 8]};
                       2'b10: biu_do <= LITTLE_ENDIAN ? {24'h0, HRDATA[23 -: 8]} : {24'h0, HRDATA[15 -: 8]};
                       2'b11: biu_do <= LITTLE_ENDIAN ? {24'h0, HRDATA[31 -: 8]} : {24'h0, HRDATA[ 7 -: 8]};
                    endcase
           'h2    : case (HADDR[1])
                       1'b0 : biu_do <= LITTLE_ENDIAN ? {16'h0, HRDATA[15 -: 16]} : {16'h0, HRDATA[31 -: 16]};
                       2'b1 : biu_do <= LITTLE_ENDIAN ? {16'h0, HRDATA[31 -: 16]} : {16'h0, HRDATA[15 -: 16]};
                    endcase
           default:           biu_do <= HRDATA;
        endcase
  end
  else //DATA_WIDTH == 64
  begin
    always @(posedge HCLK)
        if (ahb_transfer_ack)
        case (biu_word_size)
           'h1    : case (HADDR[2:0])
                       3'b000: biu_do <= LITTLE_ENDIAN ? {56'h0, HRDATA[ 7 -: 8]} : {56'h0, HRDATA[63 -: 8]};
                       3'b001: biu_do <= LITTLE_ENDIAN ? {56'h0, HRDATA[15 -: 8]} : {56'h0, HRDATA[55 -: 8]};
                       3'b010: biu_do <= LITTLE_ENDIAN ? {56'h0, HRDATA[23 -: 8]} : {56'h0, HRDATA[47 -: 8]};
                       3'b011: biu_do <= LITTLE_ENDIAN ? {56'h0, HRDATA[31 -: 8]} : {56'h0, HRDATA[39 -: 8]};
                       3'b100: biu_do <= LITTLE_ENDIAN ? {56'h0, HRDATA[39 -: 8]} : {56'h0, HRDATA[31 -: 8]};
                       3'b101: biu_do <= LITTLE_ENDIAN ? {56'h0, HRDATA[47 -: 8]} : {56'h0, HRDATA[23 -: 8]};
                       3'b110: biu_do <= LITTLE_ENDIAN ? {56'h0, HRDATA[55 -: 8]} : {56'h0, HRDATA[15 -: 8]};
                       3'b111: biu_do <= LITTLE_ENDIAN ? {56'h0, HRDATA[63 -: 8]} : {56'h0, HRDATA[ 7 -: 8]};
                    endcase
           'h2    : case (HADDR[2:1])
                       2'b00 : biu_do <= LITTLE_ENDIAN ? {48'h0, HRDATA[15 -: 16]} : {48'h0, HRDATA[63 -: 16]};
                       2'b01 : biu_do <= LITTLE_ENDIAN ? {48'h0, HRDATA[31 -: 16]} : {48'h0, HRDATA[47 -: 16]};
                       2'b10 : biu_do <= LITTLE_ENDIAN ? {48'h0, HRDATA[47 -: 16]} : {48'h0, HRDATA[31 -: 16]};
                       2'b11 : biu_do <= LITTLE_ENDIAN ? {48'h0, HRDATA[63 -: 16]} : {48'h0, HRDATA[15 -: 16]};
                    endcase
           'h4    : case (HADDR[2])
                       1'b0  : biu_do <= LITTLE_ENDIAN ? {32'h0, HRDATA[31 -: 32]} : {16'h0, HRDATA[63 -: 32]};
                       2'b1  : biu_do <= LITTLE_ENDIAN ? {32'h0, HRDATA[63 -: 32]} : {16'h0, HRDATA[31 -: 32]};
                    endcase
           default:            biu_do <= HRDATA;
        endcase
  end
endgenerate


  // Create a toggle-active ready signal to send to the TCK domain
  always @(posedge HCLK,negedge ahb_rstn)
    if      (!ahb_rstn        ) rdy_sync <= 1'b0;
    else if ( ahb_transfer_ack) rdy_sync <= ~rdy_sync;


  /////////////////////////////////////////////////////
  // State machine to create AHB accesses

  assign ahb_transfer_ack = HREADY & (ahb_fsm_state == DATA);

  assign HSEL      = 1'b1;
  assign HPROT     = HPROT_DATA | HPROT_PRIVILEGED | HPROT_NON_BUFFERABLE | HPROT_NON_CACHEABLE;
  assign HMASTLOCK = 1'b0;
  
  always @ (posedge HCLK,negedge ahb_rstn)
    if (!ahb_rstn)
    begin
        HTRANS <= HTRANS_IDLE;
        ahb_fsm_state <= IDLE;
    end
    else
      case (ahb_fsm_state)
         IDLE   : if (start_toggle || start_toggle_hold)
                  begin
                      HTRANS        <= HTRANS_NONSEQ;
                      ahb_fsm_state <= ADDRESS;
                  end

         ADDRESS: begin
                      HTRANS        <= HTRANS_IDLE;
                      ahb_fsm_state <= DATA;
                  end

         DATA   : if (HREADY) ahb_fsm_state <= IDLE;

         default : begin
                       HTRANS        <= HTRANS_IDLE;
                       ahb_fsm_state <= IDLE;
                   end
      endcase


  //Only single accesses; no bursts
  assign HBURST = HBURST_SINGLE;
endmodule

