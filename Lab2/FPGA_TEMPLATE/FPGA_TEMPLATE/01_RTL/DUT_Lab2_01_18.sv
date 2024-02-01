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
    input  logic [MEM_DATA_WIDTH-1:0]   mem1_rdata,

    output logic [1:0]                  led
);

FIFO fifo (
	.clk							( CLK						),//input
	.rstn							( fifo_RSTN					),//input
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

// fifo control 
logic                                   fifo_full;
logic                                   fifo_empty;
logic [MEM_DATA_WIDTH-1:0]              fifo_rdata;
logic [MEM_DATA_WIDTH-1:0]              fifo_wdata;
logic                                   fifo_push;
logic                                   fifo_pop;
logic                                   fifo_RSTN;
logic                                   reset_fifo;

// Internal Signals for FIFO write & memory read
// logic [15:0]                            count;
logic [15:0]                            curr_count; //
logic [15:0]                            next_count; //
logic [15:0]                            read_count;
logic [1:0]                             read_offset;
logic [1:0]                             write_offset;
logic [31:0]                            buffer_in;
logic [31:0]                            buffer_out;
logic [32:0]                            w_buffer;
integer                                 R_m_L;
logic [31:0]                            mem_rdata; //메모리에서 읽어온 데이터
logic [31:0]                            test_data;


// Internal Signals for FIFO read & memory write
// logic [15:0]                            count_w;
logic [15:0]                            curr_w_count;
logic [15:0]                            next_w_count;
logic [15:0]                            last_loop;
logic [1:0]                             last_mask;

// local param
localparam SRC_ADDR                  = 32'h00000000;
localparam DEST_ADDR                 = 32'h00000004;
localparam SIZE_ADDR                 = 32'h00000008;
localparam MODE_ADDR                 = 32'h0000000c;
localparam INT_ADDR                  = 32'h00000010;


assign mem_rdata = reg_src_addr[20] ? mem0_rdata : mem1_rdata;
assign test_data = reg_dst_addr[20] ? mem1_rdata : mem0_rdata;
assign fifo_RSTN = RSTN & reset_fifo; //FIFO seset은 DMA 연산 1개가 끝나면 바로 수행



//////////////////////////////
// APB regigster setting
//////////////////////////////
always_ff @ (posedge CLK, negedge RSTN) begin
    PREADY <= 0;
    if (INTR) begin 
        reg_mode <= 0;
        INTR <= 0;
    end
    if(~RSTN) begin
        reg_src_addr <= 32'h0;
        reg_dst_addr <= 32'h0;
        reg_size <= 32'h0;
        reg_mode <= 32'h0;
        reg_interrput <= 32'h0;
        INTR <= 0;
    end 
    else if(PENABLE & PSEL) begin
        case(PADDR)
            SRC_ADDR : begin
                if(PWRITE) reg_src_addr <= PWDATA;
                else PRDATA <= reg_src_addr;
                PREADY <= 1;
            end
            DEST_ADDR : begin
                if(PWRITE) reg_dst_addr <= PWDATA;
                else PRDATA <= reg_dst_addr;
                PREADY <= 1;
            end
            SIZE_ADDR : begin
                if(PWRITE) reg_size <= PWDATA;
                else PRDATA <= reg_size;
                PREADY <= 1;
            end
            // MODE_ADDR : begin
            //     if (INTR) begin 
            //         reg_mode <= 'b0;
            //         PRDATA <= 'b0;
            //     end
            //     else begin
            //         if(PWRITE) reg_mode <= PWDATA;
            //         else PRDATA <= reg_mode;
            //     end
            //     PREADY <= 1;
            MODE_ADDR : begin
                if(PWRITE) reg_mode <= PWDATA;
                else PRDATA <= reg_mode;
                PREADY <= 1;
            end
            INT_ADDR : begin
                if(PWRITE) reg_interrput <= PWDATA;
                else PRDATA <= reg_interrput;
                PREADY <= 1;
                reg_mode <= 32'h0;
                
            end
            default : begin
                PRDATA <= PWRITE;
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
    EQUAL,      //read_offset과 write_offset이 같을 때
    LEFT,       //첫 실행에 전부다 버퍼에만 넣기 (원래는 버퍼에 넣을 게 없음)
    RIGHT,      //첫 실행부터 버퍼에 넣을 게 생김
    OPERATION,  //FIFO에서 읽을 준비 하기 
    WRITE,      //FIFO에서 읽기 
    DONE,       //모든 상태 마침 & 초기화
    TEST        //test mode
} next_state, curr_state;


always_ff @ (posedge CLK, negedge RSTN) begin
    if(~RSTN) begin
        curr_state <= IDLE;
        curr_count <= 0;
        curr_w_count <= 0;
    end
    else begin
        curr_state <= next_state;
        curr_count <= next_count;
        curr_w_count <= next_w_count;
    end
end

///////////////////////////////
// register setting & operation
///////////////////////////////
always_comb begin
    // default
    next_state = curr_state;
    next_count = curr_count;
    next_w_count = curr_w_count;

    // main
    if (reg_mode == 1)
    begin
        case(curr_state)
            IDLE: begin
                if(reg_src_addr[20] == 1'b1) begin
                    mem0_en = 1'b1;
                    mem0_we = 4'b0000;
                    mem0_wdata = 0;
                    // mem0_rdata = 0;
                    mem0_addr = reg_src_addr[17:2];
                    
                end
                else if (reg_src_addr[20] == 1'b0) begin
                    mem1_en = 1'b1;
                    mem1_we = 4'b0000;
                    mem1_wdata = 0;
                    // mem1_rdata = 0;
                    mem1_addr = reg_src_addr[17:2];
                    
                end
                reset_fifo = 'b1;
                write_offset = reg_dst_addr[1:0];
                read_offset = reg_src_addr[1:0];
                if ((reg_size + read_offset) % 'd4 == 0) begin 
                    read_count = (reg_size + read_offset) / 'd4 +'d1;    
                end
                else read_count = (reg_size + read_offset) / 'd4 + 'd2;
                R_m_L = read_offset - write_offset;
                if (curr_count >= read_count) begin
                    next_state = DONE;
                    next_count = 0;
                end
                else if ((R_m_L) == 0) begin
                    next_state = EQUAL;
                    next_count = 0;
                end
                else if ((R_m_L) > 0) begin
                    next_state = LEFT;
                    next_count = 0;
                end
                else if ((R_m_L) < 0) begin
                    next_state = RIGHT;
                    next_count = 0;
                end
            end



            EQUAL: begin
                 fifo_push = 1'b1;
                 fifo_wdata = mem_rdata;
                 next_count = curr_count + 1;
                if (next_count == read_count - 1) begin
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
                buffer_in = buffer_out;

                write_offset = write_offset + ('d4 - read_offset);
                read_offset = 0;
                next_count = curr_count + 1;

                if (next_count == read_count) begin
                    next_state = OPERATION;
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
                next_count = curr_count + 1;
                if (next_count == read_count) begin
                    next_state = OPERATION;
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
            
            
            OPERATION: begin                
                fifo_pop = 1'b1;
                next_w_count = 0;
                last_mask = (reg_size + reg_dst_addr[1:0]) % 4;
                if (last_mask == 0) last_loop = (reg_size + reg_dst_addr[1:0]) / 4; 
                else last_loop = (reg_size + reg_dst_addr[1:0]) / 4 + 1;
                next_state = WRITE;
                if(reg_dst_addr[20] == 1'b1) begin //mem0에 쓰기
                    mem0_en = 1'b1;
                    case (reg_dst_addr[1:0])
                        2'b00: mem0_we = 4'b1111;
                        2'b01: mem0_we = 4'b1110;
                        2'b10: mem0_we = 4'b1100;
                        2'b11: mem0_we = 4'b1000;
                    endcase
                    mem0_addr = reg_dst_addr[17:2];                  
                end
                else if (reg_dst_addr[20] == 1'b0) begin //mem1에 쓰기
                    mem1_en = 1'b1;
                    case (reg_dst_addr[1:0])
                        2'b00: mem1_we = 4'b1111;
                        2'b01: mem1_we = 4'b1110;
                        2'b10: mem1_we = 4'b1100;
                        2'b11: mem1_we = 4'b1000;
                    endcase
                    mem1_addr = reg_dst_addr[17:2];
                    mem1_wdata = fifo_rdata;
                end
            end

            WRITE: begin
                if (curr_w_count != 0) begin
                    if (reg_dst_addr[20]) begin 
                        mem0_we = 4'b1111;
                        mem0_addr = mem0_addr + 1;
                    end
                    else begin 
                        mem1_we = 4'b1111;
                        mem1_addr = mem1_addr + 1;
                    end

                end

                if(reg_dst_addr[20] == 1'b1) begin
                    mem0_wdata = fifo_rdata;
                end
                else mem1_wdata = fifo_rdata;
                next_w_count = curr_w_count + 1;
                if (next_w_count == 1) next_state = WRITE;
                else if (next_w_count >= last_loop) begin 
                    next_state = DONE;
                    if (reg_dst_addr[20]) begin //////////////
                        case (last_mask)
                            2'b00: mem0_we = 4'b1111;
                            2'b01: mem0_we = 4'b0001;
                            2'b10: mem0_we = 4'b0011;
                            2'b11: mem0_we = 4'b0111;
                        endcase
                    end
                    else begin 
                        case (last_mask)
                            2'b00: mem1_we = 4'b1111;
                            2'b01: mem1_we = 4'b0001;
                            2'b10: mem1_we = 4'b0011;
                            2'b11: mem1_we = 4'b0111;
                        endcase
                    end
                end
            end

            DONE: begin
                INTR = 'b1;
                mem0_en = 1'b0;
                mem0_we = 4'b0000;
                mem1_en = 1'b0;
                mem1_we = 4'b0000;
                reset_fifo = 'b0;
                fifo_pop = 1'b0;
                fifo_push = 1'b0;
                next_state = IDLE;
                next_count = 0;
                // count = 0;
                next_w_count = 0;
                // @ (posedge CLK);
                // reg_mode = 0;
            end
        endcase
    end
    else if (reg_mode == 2) begin
        
        case (curr_state)
            TEST: begin
                if (test_data == 22'h00_005f5e5d_5c5b5a59_5857) led = 2'b10;
                else led = 2'b01;
                next_state = IDLE;
                INTR = 1;
            end
            default: begin
                next_state = TEST;
                if (reg_dst_addr[20]) begin
                    mem0_addr = reg_dst_addr[17:2];
                end
                else begin
                    mem1_addr = reg_dst_addr[17:2];
                end
            end

        endcase

        

    
    end

end

// task for byte re-aligned
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

