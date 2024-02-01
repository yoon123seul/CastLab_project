
`timescale 1ns/1ps

package dma_pkg;
	
    parameter REG_ADDR_WIDTH			= 32;
    parameter REG_DATA_WIDTH			= 32;
	parameter MEM_ADDR_WIDTH			= 16;
	parameter MEM_DATA_WIDTH			= 32;
	parameter MEM_STRB_WIDTH			= MEM_DATA_WIDTH/8;
	parameter MODE						= 2;
	parameter MAX_TRANS_SIZE			= 5;					// Maximum Transfer Size = 16 (Represent in 5byte)

endpackage
