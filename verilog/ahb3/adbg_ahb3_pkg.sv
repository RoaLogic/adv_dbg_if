//////////////////////////////////////////////////////////////////////
////                                                              ////
////  adbg_ahb3_pkg.sv                                            ////
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
////      THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ////
////  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ////
////  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ////
////  FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ////
////  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ////
////  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ////
////  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ////
////  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ////
////  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ////
////  LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ////
////  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ////
////  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ////
////  POSSIBILITY OF SUCH DAMAGE.                                 ////
////                                                              ////
//////////////////////////////////////////////////////////////////////

/*
 * Constants used 
 */
package adbg_ahb3_pkg;
  // The AHB3 debug module requires 53 bits
  parameter DBG_AHB_DATAREG_LEN = 53;

  // These relate to the number of internal registers, and how
  // many bits are required in the Reg. Select register
  parameter DBG_AHB_REGSELECT_SIZE = 1;
  parameter DBG_AHB_NUM_INTREG     = 1;

  // Register index definitions for module-internal registers
  // The AHB module has just 1, the error register
  parameter DBG_AHB_INTREG_ERROR = 'b0;


  // Valid commands/opcodes for the AHB debug module (same as Wishbone)
  // 0000  NOP
  // 0001  Write burst, 8-bit access
  // 0010  Write burst, 16-bit access
  // 0011  Write burst, 32-bit access
  // 0100  Write burst, 64-bit access
  // 0101  Read burst, 8-bit access
  // 0110  Read burst, 16-bit access
  // 0111  Read burst, 32-bit access
  // 1000  Read burst, 64-bit access
  // 1001  Internal register select/write
  // 1010 - 1100 Reserved
  // 1101  Internal register select
  // 1110 - 1111 Reserved
  parameter DBG_AHB_CMD_BWRITE8  = 'h1;
  parameter DBG_AHB_CMD_BWRITE16 = 'h2;
  parameter DBG_AHB_CMD_BWRITE32 = 'h3;
  parameter DBG_AHB_CMD_BWRITE64 = 'h4;
  parameter DBG_AHB_CMD_BREAD8   = 'h5;
  parameter DBG_AHB_CMD_BREAD16  = 'h6;
  parameter DBG_AHB_CMD_BREAD32  = 'h7;
  parameter DBG_AHB_CMD_BREAD64  = 'h8;
  parameter DBG_AHB_CMD_IREG_WR  = 'h9;
  parameter DBG_AHB_CMD_IREG_SEL = 'hd;


  //AHB definitions
  parameter [1:0] HTRANS_IDLE          = 2'b00,
                  HTRANS_BUSY          = 2'b01,
                  HTRANS_NONSEQ        = 2'b10,
                  HTRANS_SEQ           = 2'b11;

  parameter [2:0] HBURST_SINGLE        = 3'b000,
                  HBURST_INCR          = 3'b001,
                  HBURST_WRAP4         = 3'b010,
                  HBURST_INCR4         = 3'b011,
                  HBURST_WRAP8         = 3'b100,
                  HBURST_INCR8         = 3'b101,
                  HBURST_WRAP16        = 3'b110,
                  HBURST_INCR16        = 3'b111;

  parameter [2:0] HSIZE8               = 3'b000,
                  HSIZE16              = 3'b001,
                  HSIZE32              = 3'b010,
                  HSIZE64              = 3'b011,
                  HSIZE128             = 3'b100,
                  HSIZE256             = 3'b101,
                  HSIZE512             = 3'b110,
                  HSIZE1024            = 3'b111;

  parameter [2:0] HSIZE_BYTE           = HSIZE8,
                  HSIZE_HWORD          = HSIZE16,
                  HSIZE_WORD           = HSIZE32,
                  HSIZE_DWORD          = HSIZE64,
                  HSIZE_4WLINE         = HSIZE128,
                  HSIZE_8WLINE         = HSIZE256;

  parameter [3:0] HPROT_OPCODE         = 4'b0000,
                  HPROT_DATA           = 4'b0001,
                  HPROT_USER           = 4'b0000,
                  HPROT_PRIVILEGED     = 4'b0010,
                  HPROT_NON_BUFFERABLE = 4'b0000,
                  HPROT_BUFFERABLE     = 4'b0100,
                  HPROT_NON_CACHEABLE  = 4'b0000,
                  HPROT_CACHEABLE      = 4'b1000;

  parameter       HRESP_OKAY           = 1'b0,
                  HRESP_ERR            = 1'b1;
endpackage

/*
 * AHB3 Lite Interface
 */
 `ifndef AHB3_INTERFACES
 `define AHB3_INTERFACES
interface ahb3lite_bus #(
    parameter HADDR_SIZE = 32,
    parameter HDATA_SIZE = 32
  )
  (
    input logic HCLK,HRESETn
  );
    logic                   HSEL;
    logic [HADDR_SIZE -1:0] HADDR;
    logic [HDATA_SIZE -1:0] HWDATA;
    logic [HDATA_SIZE -1:0] HRDATA;
    logic                   HWRITE;
    logic [            2:0] HSIZE;
    logic [            3:0] HBURST;
    logic [            3:0] HPROT;
    logic [            1:0] HTRANS;
    logic                   HMASTLOCK;
    logic                   HREADY;
    logic                   HREADYOUT;
    logic                   HRESP;

    modport master (
      input  HRESETn,
      input  HCLK,
      output HSEL,
      output HADDR,
      output HWDATA,
      input  HRDATA,
      output HWRITE,
      output HSIZE,
      output HBURST,
      output HPROT,
      output HTRANS,
      output HMASTLOCK,
      input  HREADY,
      input  HRESP
    );

    modport slave (
      input  HRESETn,
      input  HCLK,
      input  HSEL,
      input  HADDR,
      input  HWDATA,
      output HRDATA,
      input  HWRITE,
      input  HSIZE,
      input  HBURST,
      input  HPROT,
      input  HTRANS,
      input  HMASTLOCK,
      input  HREADY,
      output HREADYOUT,
      output HRESP
    );
endinterface

/*
 * AHB3 Lite Interface
 */
interface apb_bus #(
    parameter PADDR_SIZE = 6,
    parameter PDATA_SIZE = 8
  )
  (
    input logic PCLK,PRESETn
  );
    logic                    PSEL;
    logic                    PENABLE;
    logic [             2:0] PPROT;
    logic                    PWRITE;
    logic [PDATA_SIZE/8-1:0] PSTRB;
    logic [PADDR_SIZE  -1:0] PADDR;
    logic [PDATA_SIZE  -1:0] PWDATA;
    logic [PDATA_SIZE  -1:0] PRDATA;
    logic                    PREADY;
    logic                    PSLVERR;

    modport master (
      input  PRESETn,
      input  PCLK,
      output PSEL,
      output PENABLE,
      output PPROT,
      output PADDR,
      output PWRITE,
      output PSTRB,
      output PWDATA,
      input  PRDATA,
      input  PREADY,
      input  PSLVERR
    );

    modport slave (
      input  PRESETn,
      input  PCLK,
      input  PSEL,
      input  PENABLE,
      input  PPROT,
      input  PADDR,
      input  PWRITE,
      input  PSTRB,
      input  PWDATA,
      output PRDATA,
      output PREADY,
      output PSLVERR
    );
endinterface
`endif
