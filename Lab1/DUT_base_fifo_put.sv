module DUT /////////// 가장 최근 버전
        import dma_pkg::*;
(
    //Global
    input  logic                         CLK,
    input  logic                         RSTN,
    output logic                         INTR,

    //APB
    input  logic                         PSEL,
    input  logic                         PENABLE,
    output logic                         PREADY,
    input  logic                         PWRITE,
    input  logic [MEM_ADDR_WIDTH-1:0]    PADDR,
    input  logic [MEM_DATA_WIDTH-1:0]    PWDATA,
    output logic [MEM_DATA_WIDTH-1:0]    PRDATA,
    
    //Memory 0
    output logic                        mem0_en,
    output logic [3:0]                  mem0_we,
    output logic [MEM_ADDR_WIDTH-1:0]   mem0_addr,
    output logic [MEM_DATA_WIDTH-1:0]   mem0_wdata,
    input  logic [MEM_DATA_WIDTH-1:0]   mem0_rdata,
    
    //Memory 1
    output logic                        mem1_en,
    output logic [3:0]                  mem1_we,
    output logic [MEM_ADDR_WIDTH-1:0]   mem1_addr,
    output logic [MEM_DATA_WIDTH-1:0]   mem1_wdata,
    input  logic [MEM_DATA_WIDTH-1:0]   mem1_rdata
);

FIFO fifo (
	.clk							( CLK						),//input
	.rstn							( RSTN					    ),//input
	.full							( fifo_full					),//output
	.empty							( fifo_empty		        ),//output
	.push							( fifo_push	    			),//input
	.push_data						( fifo_wdata				),//input
    .pop                            ( fifo_pop                  ),//input
    .pop_data                       ( fifo_rdata                ) //output
);

// DMA registers
logic [REG_DATA_WIDTH:0]                reg_src_addr;
logic [REG_DATA_WIDTH:0]                reg_dst_addr;
logic [REG_DATA_WIDTH:0]                reg_size;
logic [REG_DATA_WIDTH:0]                reg_mode;
logic [REG_DATA_WIDTH:0]                reg_interrput;

// Internal Signals
logic                                   fifo_full;
logic                                   fifo_empty;
logic [MEM_DATA_WIDTH-1:0]              fifo_rdata;
logic [MEM_DATA_WIDTH-1:0]              fifo_wdata;
logic                                   fifo_push;
logic                                   fifo_pop;
logic [15:0]                            count;


// local param
// localparam RW = 1;
localparam SRC_ADDR                  = 32'h00000000;
localparam DEST_ADDR                 = 32'h00000004;
localparam SIZE_ADDR                 = 32'h00000008;
localparam MODE_ADDR                 = 32'h0000000c;
localparam INT_ADDR                  = 32'h00000010;


// APB regigster setting


always_ff @ (posedge CLK, negedge RSTN) begin
    PREADY <= 0;
    if(~RSTN) begin
        reg_src_addr <= 32'h0;
        reg_dst_addr <= 32'h0;
        reg_size <= 32'h0;
        reg_mode <= 32'h0;
        reg_interrput <= 32'h0;
        INTR <= 0;
        count <= 0;
    end 
    else if(PENABLE & PSEL) begin
        case(PADDR)
            SRC_ADDR : begin
                reg_src_addr <= PWDATA;
                PREADY <= 1;
            end
            DEST_ADDR : begin
                reg_dst_addr <= PWDATA;
                PREADY <= 1;
            end
            SIZE_ADDR : begin
                reg_size <= PWDATA;
                PREADY <= 1;
            end
            MODE_ADDR : begin
                if (INTR) reg_mode <= 0;
                else reg_mode <= PWDATA;
                PREADY <= 1;
            end
            INT_ADDR : begin
                reg_interrput <= PWDATA;
                PREADY <= 1;
                reg_mode <= 32'h0;
            end
            default : begin
                reg_src_addr <= 32'h0;
                reg_dst_addr <= 32'h0;
                reg_size <= 32'h0;
                reg_mode <= 32'h0;
                reg_interrput <= 32'h0;
            end
        endcase
    end
end

//////////////////////////////
// DMA State Machine 
//////////////////////////////
enum logic [2:0] {
    IDLE,
    EQUAL,
    LEFT,
    RIGHT,
    OPERATION,
    WRITE,
    DONE
} next_state, curr_state;


always_ff @ (posedge CLK, negedge RSTN) begin
    if(~RSTN) begin
        curr_state <= IDLE;
    end
    else begin
        curr_state <= next_state;
    end
end

///////////////////////////////
// register setting & operation
///////////////////////////////
logic [15:0]                                    read_count;
logic [1:0]                                     read_offset;
logic [1:0]                                     write_offset;
logic [31:0]                                    buffer_in;
logic [31:0]                                    buffer_out;



logic [31:0] mem_rdata; //메모리에서 읽어온 데이터
assign mem_rdata = reg_src_addr[20] ? mem0_rdata : mem1_rdata;

integer R_m_L;


always_comb begin
    // default
    next_state = curr_state;

    // main
    if (reg_mode == 1)
    begin
        case(curr_state)
            IDLE: begin
                if(reg_src_addr[20] == 1'b1) begin
                    mem0_en = 1'b1;
                    // mem0_we = 4'b0;
                    mem0_wdata = 0;
                    mem0_addr = reg_src_addr[17:2];
                    
                end
                else if (reg_src_addr[20] == 1'b0) begin
                    mem1_en = 1'b1;
                    // mem1_we = 4'b0;
                    mem1_wdata = 0;
                    mem1_addr = reg_src_addr[17:2];
                    
                end

                write_offset = reg_dst_addr[1:0];
                read_offset = reg_src_addr[1:0];
                if ((reg_size + read_offset) % 'd4 == 0) begin 
                    read_count = (reg_size + read_offset) / 'd4;    
                end
                else read_count = (reg_size + read_offset) / 'd4 + 'd1;
                R_m_L = read_offset - write_offset;
                if (count == read_count) begin
                    next_state = OPERATION;
                    count = 0;
                end
                else if ((R_m_L) == 0)
                    next_state = EQUAL;
                else if ((R_m_L) > 0)
                    next_state = LEFT;
                else if ((R_m_L) < 0)
                    next_state = RIGHT;
            end



            EQUAL: begin
                 fifo_push = 1'b1;
                 fifo_wdata = mem_rdata;
                 count = count + 1;
                if (count == read_count) begin
                    next_state = OPERATION;
                    fifo_push = 1'b0;
                end
                
                else begin next_state = EQUAL;
                    if (mem0_en) begin 
                        mem0_addr = mem0_addr + 1;
                    end
                    else begin 
                        mem1_addr = mem1_addr + 1;
                    end
                end
            end


            LEFT: begin //Read_offset이 더 크다
                fifo_push = 1'b0;
                index_write(read_offset, write_offset, mem_rdata, buffer_in, fifo_wdata, buffer_out);
                $display("buffer_out %h", buffer_out[23:8]);
                buffer_in = buffer_out;

                write_offset = write_offset + ('d4 - read_offset);
                read_offset = 0;
                count = count + 1;

                if (count == read_count + 1) begin
                    next_state = DONE; ///////////
                    fifo_push = 1'b0;
                end
                
                else begin next_state = RIGHT;
                    if (mem0_en) begin 
                        mem0_addr = mem0_addr + 1;
                    end
                    else begin 
                        mem1_addr = mem1_addr + 1;
                    end
                end
            end



            RIGHT: begin  //Write_offset이 더 크다 
                fifo_push = 1'b1;

                index_write(read_offset, write_offset, mem_rdata, buffer_in, fifo_wdata, buffer_out);
                buffer_in = buffer_out;

                write_offset = write_offset + ('d4 - read_offset);
                read_offset = 0;
                count = count + 1;
                if (count == read_count + 1) begin
                    next_state = DONE;  ///////////////
                    fifo_push = 1'b0;   ///////////////여기가 문제 
                    // fifo_pop = 1'b1;
                end
                else begin next_state = RIGHT;

                    if (mem0_en) begin 
                        mem0_addr = mem0_addr + 1;
                    end
                    else begin 
                        mem1_addr = mem1_addr + 1;
                    end
                end
    
            end
            
            
            OPERATION: begin                
            end

            WRITE: begin
            end

            DONE: begin
                INTR = 'b1;
                fifo_push = 'b0;
                // next_state = IDLE;
                mem1_en = 1'b0;
                mem1_we = 4'b0;
                mem1_en = 1'b0;
                mem1_we = 4'b0;
                // reg_mode = 0;
                // next_state = DONE;
            end
        endcase
    end

end




task automatic index_write(
		input [1:0] read_offset,
		input [3:0] write_offset,
        input [31:0] mem_rdata,
        input [31:0] buffer_in,

        output [31:0] fifo_wdata,
		output [31:0] buffer_out
        // output []
	);
    begin
        case (read_offset)
            0: begin
                case (write_offset)
                1: begin
                    fifo_wdata[7:0] = buffer_in[7:0];
                    fifo_wdata[31:8] = mem_rdata[23:0];
                    buffer_out[7:0] = mem_rdata[31:24]; 
                end
                2:begin
                    fifo_wdata[15:0] = buffer_in[15:0];
                    fifo_wdata[31:16] = mem_rdata[15:0];
                    buffer_out[15:0] = mem_rdata[31:16]; 
                end
                3:begin
                    fifo_wdata[23:0] = buffer_in[23:0];
                    fifo_wdata[31:24] = mem_rdata[7:0];
                    buffer_out[23:0] = mem_rdata[31:8]; 
                end
                endcase
            end
            1: begin
                case (write_offset)
                0: begin
                    buffer_out[23:0] = mem_rdata[31:8]; 
                end
                2:begin
                    fifo_wdata[15:0] = buffer_in[15:0];
                    fifo_wdata[31:16] = mem_rdata[23:8];
                    buffer_out[7:0] = mem_rdata[31:24]; 
                end
                3:begin
                    fifo_wdata[23:0] = buffer_in[23:0];
                    fifo_wdata[31:24] = mem_rdata[15:8];
                    buffer_out[15:0] = mem_rdata[31:16]; 
                end
                endcase
            end
            2: begin
                case (write_offset)
                0: begin
                    buffer_out[15:0] = mem_rdata[31:16]; 
                end
                1: begin
                    buffer_out[23:8] = mem_rdata[31:16]; 
                end
                3:begin
                    fifo_wdata[23:0] = buffer_in[23:0];
                    fifo_wdata[31:24] = mem_rdata[23:16];
                    buffer_out[7:0] = mem_rdata[31:24];
                end
                endcase
            end
            3:begin
                case (write_offset)
                0: begin
                    buffer_out[7:0] = mem_rdata[31:24]; 
                end
                1: begin
                    buffer_out[15:8] = mem_rdata[31:24]; 
                end
                2:begin
                    buffer_out[23:16] = mem_rdata[31:24];
                end
                endcase
            end
        endcase
    end
	endtask
endmodule
