//////////////////////////////////////////////////////////////////////
// File:  CRC32.v                             
// Date:  Thu Nov 27 13:56:49 2003                                                      
//                                                                     
// Copyright (C) 1999-2003 Easics NV.                 
// This source file may be used and distributed without restriction    
// provided that this copyright statement is not removed from the file 
// and that any derivative work contains the original copyright notice
// and the associated disclaimer.
//
// THIS SOURCE FILE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
// WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Purpose: Verilog module containing a synthesizable CRC function
//   * polynomial: (0 1 2 4 5 7 8 10 11 12 16 22 23 26 32)
//   * data width: 1
//                                                                     
// Info: janz@easics.be (Jan Zegers)                           
//       http://www.easics.com
//
// Modified by Nathan Yawn for the Advanced Debug Module
// Changes (C) 2008 - 2010 Nathan Yawn                                 
///////////////////////////////////////////////////////////////////////

module adbg_crc32 (rstn, clk, data, enable, shift, clr, crc_out, serial_out);
input         rstn;
input         clk;
input         data;
input         enable;
input         shift;
input         clr;
output [31:0] crc_out;
output        serial_out;


reg    [31:0] crc;
wire   [31:0] new_crc;


// You may notice that the 'poly' in this implementation is backwards.
// This is because the shift is also 'backwards', so that the data can
// be shifted out in the same direction, which saves on logic + routing.
assign new_crc[0] = crc[1];
assign new_crc[1] = crc[2];
assign new_crc[2] = crc[3];
assign new_crc[3] = crc[4];
assign new_crc[4] = crc[5];
assign new_crc[5] = crc[6] ^ data ^ crc[0];
assign new_crc[6] = crc[7];
assign new_crc[7] = crc[8];
assign new_crc[8] = crc[9] ^ data ^ crc[0];
assign new_crc[9] = crc[10] ^ data ^ crc[0];
assign new_crc[10] = crc[11];
assign new_crc[11] = crc[12];
assign new_crc[12] = crc[13];
assign new_crc[13] = crc[14];
assign new_crc[14] = crc[15];
assign new_crc[15] = crc[16] ^ data ^ crc[0];
assign new_crc[16] = crc[17];
assign new_crc[17] = crc[18];
assign new_crc[18] = crc[19];
assign new_crc[19] = crc[20] ^ data ^ crc[0];
assign new_crc[20] = crc[21] ^ data ^ crc[0];
assign new_crc[21] = crc[22] ^ data ^ crc[0];
assign new_crc[22] = crc[23];
assign new_crc[23] = crc[24] ^ data ^ crc[0];
assign new_crc[24] = crc[25] ^ data ^ crc[0];
assign new_crc[25] = crc[26];
assign new_crc[26] = crc[27] ^ data ^ crc[0];
assign new_crc[27] = crc[28] ^ data ^ crc[0];
assign new_crc[28] = crc[29];
assign new_crc[29] = crc[30] ^ data ^ crc[0];
assign new_crc[30] = crc[31] ^ data ^ crc[0];
assign new_crc[31] =           data ^ crc[0];

always @ (posedge clk or negedge rstn)
  if(!rstn)
    crc[31:0] <= 32'hffffffff;
  else if(clr)
    crc[31:0] <= 32'hffffffff;
  else if(enable)
    crc[31:0] <= new_crc;
  else if (shift)
    crc[31:0] <= {1'b0, crc[31:1]};


//assign crc_match = (crc == 32'h0);
assign crc_out = crc; //[31];
assign serial_out = crc[0];

endmodule
