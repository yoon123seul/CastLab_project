module FIFO  //////////�??�� 최근 버전
    import dma_pkg::*;
(
    input   logic                           clk,
    input   logic                           rstn,
    output  logic                           full,
    output  logic                           empty,
    input   logic                           push,
    input   logic  [MEM_DATA_WIDTH-1:0]     push_data,
    input   logic                           pop,
    output  logic  [MEM_DATA_WIDTH-1:0]     pop_data

    // input  logic [3:1]                      num_input,
    // output logic                            push_finish,
    // output logic                            pop_finish
);
logic                                       wr_en;
logic                                       rd_en;
// logic                                       count;

logic [15:0]                                wr_ptr, rd_ptr;
logic [MEM_DATA_WIDTH-1:0]                  mem [15:0];

assign wr_en = push & ~full;
assign rd_en = pop & ~empty;

always_ff @ (posedge clk) begin 
    if (~rstn) begin 
        wr_ptr <= 16'd0;
        rd_ptr <= 16'd0;
        full <= 1'b0;
        empty <= 1'b1;
    end
    else begin
        if (wr_en) begin 
            
            wr_ptr <= wr_ptr + 16'd1;
        end
        else wr_ptr <= wr_ptr;
        if (rd_en) begin 
            pop_data = mem[rd_ptr];
            rd_ptr <= rd_ptr + 16'd1;
        end
        else rd_ptr <= rd_ptr;
    end
end

always_comb begin
    mem[wr_ptr] = push_data;
end

always_ff @ (posedge clk) begin
    if (~rstn) begin
        full = 1'b0;
        empty = 1'b1;
    end
    else if (wr_en & rd_en) begin
        full <= full;
        empty <= empty;
    end
    else if (wr_en & !rd_en) begin
        full <= (wr_ptr + 'd1) == rd_ptr;
        empty <= 1'b0;
    end
    else if (!wr_en & rd_en) begin
        empty <= (rd_ptr) == wr_ptr;
        full <= 1'b0;
    end
end

endmodule