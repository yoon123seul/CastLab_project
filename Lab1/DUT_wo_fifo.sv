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
// logic                                   RW_state; //R : 0, W : 1
// logic                                   IDEL_state; //Idel : 0, Operation : 1
logic                                   fifo_full;
logic                                   fifo_empty;
logic [MEM_DATA_WIDTH-1:0]              fifo_rdata;
logic [MEM_DATA_WIDTH-1:0]              fifo_wdata;
logic                                   fifo_push;
logic                                   fifo_pop;


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
enum logic [1:0] {
    IDLE,
    OPERATION,
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

// logic [31:0] temp_reg0_r;
// logic [31:0] temp_reg1_r;
// logic [31:0] temp_reg0_w;
// logic [31:0] temp_reg1_w;
///////////////////////////////
// register setting & operation
///////////////////////////////
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
                    mem0_we = 4'b0;
                    mem0_wdata = 0;
                    mem0_addr = reg_src_addr[17:2];
                end
                else if (reg_src_addr[20] == 1'b0) begin
                    mem1_en = 1'b1;
                    mem1_we = 4'b0;
                    mem1_wdata = 0;
                    mem1_addr = reg_src_addr[17:2];
                end
                next_state = OPERATION;
            end
            OPERATION: begin
                if(reg_dst_addr[20] == 1'b1) begin //mem0에 쓰기
                    mem0_en = 1'b1;
                    mem0_we = 4'b1111;
                    mem0_addr = reg_dst_addr[17:2];
                    mem0_wdata = mem1_rdata; //////// 쓸 데이터 여기에 준비 
                    // mem0_rdata = 0;                   
                end
                else if (reg_dst_addr[20] == 1'b0) begin //mem1에 쓰기
                    mem1_en = 1'b1;
                    mem1_we = 4'b1111;
                    mem1_addr = reg_dst_addr[17:2];
                    mem1_wdata = mem0_rdata; //////// 쓸 데이터 여기에 준비                    
                    // mem1_rdata = 0;
                end
                next_state = DONE;
            end
            DONE: begin
                INTR = 'b1;
                next_state = IDLE;
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




endmodule
