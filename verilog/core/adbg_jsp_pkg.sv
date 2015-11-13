//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_jsp_pkg.sv                                             ////
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
package adbg_jsp_pkg;
  parameter DBG_JSP_DATAREG_LEN = 64;

endpackage


package adbg_jsp_16550_pkg;
  /*
   adr	DLAB	name	bits	description
   0	0	RBR	7:0	Receiver Buffer Register - read-only
   0	0	THR	7:0	Transmitter Holding Register - write-only
   0	1	DLL	7:0	Divisor Latch (LS)
   1	0	IER	3:0	Interrupt Enable Register
   1	1	DLM	7:0	Divisor Latch (MS)
   2	-	IIR	7:6,3:0	Interrupt Ident Register - read-only
   2	-	FCR	7:6,3:0	FIFO Control Register - write-only
   3	-	LCR	7:0	Line Control Register
   4	-	MCR	4:0	Modem Control Register
   5	-	LSR	7:0	Line Status Register
   6	-	SCR	7:0	Scratch Register
   */

  typedef struct packed {
    logic       edssi,      // Enable Modem Status Interrupt
                elsi,       // Enable Receiver Line Status Interrupt
                etbei,      // Enable Transmitter Holding Register Empty Interrupt
                erbfi;      // Enable Received Data Available Interrupt
  } ier_struct;

  typedef struct packed {
    logic [1:0] fe;         // FIFOs Enabled
    logic [1:0] zeroes;
    logic [2:0] id;         // Interrupt ID
    logic       ip;         // '0' if interrupt pending
  } iir_struct;

  typedef struct packed {
    logic [1:0] rx_trigger; // Rx Trigger
    logic       dma,        // DMA mode selected
                txfifo_rst, // Tx FIFO reset
                rxfifo_rst, // Rx FIFO reset
                fifo_ena;   // Enable FIFOs
  } fcr_struct;

  typedef struct packed {
    logic       dlab,       // Divisor Latch Access bit
                setbreak,
                stickparity,
                eps,        // Even parity select
                pen,        // parity enable
                stb;        // number of stop bits
    logic [1:0] wls;        // Word Length Select
  } lcr_struct;

  typedef struct packed {
    logic       loop,
                out2,
                out1,
                rts,        // Request To Send
                dtr;        // Data Terminal Ready
  } mcr_struct;

  typedef struct packed {
    logic       rxfifo_err, // Error in Rx FIFO
                tempt,      // Transmitter empty
                thre,       // Transmitter Hold Register empty
                bi,         // Break Interrupt
                fe,         // Framing Error
                pe,         // Parity Error
                oe,         // Overrun Error
                dr;         // Data Ready
  } lsr_struct;

  typedef struct packed {
    logic       dcd,        // Data Carrier Detect
                ri,         // Ring Indicator
                dsr,        // Data Set Ready
                cts,        // Clear To Send
                ddcd,       // Delta Data Carrier Detect
                teri,       // Trailing Edge Ring Indicator
                ddsr,       // Delta Data Set Ready
                dcts;       // Delta Clear To Send
  } msr_struct;

  typedef logic [7:0] scr_struct;

endpackage
