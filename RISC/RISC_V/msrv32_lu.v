/***********************************
//
//File Name: msrv32_lu.v
//
//Module Name: msrv32_lu
//
//Description: Load unit loads the data from the memory to register file via
//             the write back MUX.
//
//Dependencies: 
//
//Version: 1.0
//
//Engineer: Alistair
//
//Email: tech_support@maven-silicon.com
//
************************************/

`timescale 1ns / 1ps

module msrv32_lu(
                 input  [1:0] load_size_in,         //size of data
                 input  load_unsigned_in,           //type of data (signed or unsigend)
                 input  [31:0] data_in,             //data from memory
                 input  [1:0] iadder_1_to_0_in,     //instruction address [1:0]
                 input  ahb_resp_in,                //AHB Response
                 output reg [31:0] lu_output        //data driven to the register (via write-back MUX)
                 ); 
        
    
   reg [7:0] data_byte; 
   reg [15:0] data_half;    
   wire [23:0] byte_ext;
   wire [15:0] half_ext;
   
   
wire a,b,c,d;
assign a = load_unsigned_in? 0 : data_in[7]; 
assign b = load_unsigned_in? 0 : data_in[15];
assign c = load_unsigned_in? 0 : data_in[23]; 
assign d = load_unsigned_in? 0 : data_in[31];

always@(*)
begin
lu_output = 0;
if(~ahb_resp_in)
begin
if(load_size_in == 2'b00) //Load byte 
begin
lu_output = 0;
case (iadder_1_to_0_in[1:0])
2'b00: lu_output = {{24{a}},data_in[7:0]};

2'b01: lu_output = {{24{b}},data_in[15:8]};

2'b10: lu_output = {{25{c}},data_in[23:16]};

2'b11: lu_output = {{24{d}},data_in[31:24]};

endcase
end

else if(load_size_in  == 2'b01)
begin
lu_output = 0;
case(iadder_1_to_0_in[1])
1'b0: lu_output = {{16{b}},data_in[15:0]};

1'b1: lu_output = {{16{d}},data_in[31:16]};
endcase
end

else
      lu_output = data_in;  
	
end
end
    
  
   
endmodule
