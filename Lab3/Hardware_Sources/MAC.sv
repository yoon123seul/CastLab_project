// Code your design here



`timescale 1 ns / 1 ps

module MAC #(
  parameter A_BITWIDTH      = 16,
  parameter A_FRAC_BIT      = 8,
  parameter W_BITWIDTH      = 8,
  parameter W_FRAC_BIT      = 6,
  parameter P_BITWIDTH      = 40,
  parameter P_FRAC_BIT      = 14
)
(
  input  logic                                             clk,
  input  logic                                             rst,

  input  logic                                             prefetch,
  input  logic                                             conv,

  input  logic signed [A_BITWIDTH-1:0]                            a_i,
  input  logic signed [W_BITWIDTH-1:0]                            w_i,
  input  logic signed [P_BITWIDTH-1:0]                            p_i,
  output logic signed [A_BITWIDTH-1:0]                            a_o,
  output logic signed [W_BITWIDTH-1:0]                            w_o,
  output logic signed [P_BITWIDTH-1:0]                            p_o
);

logic signed [A_BITWIDTH-1:0] a_curr, a_next;
logic signed [W_BITWIDTH-1:0] w_curr, w_next;
logic signed [P_BITWIDTH-1:0] p_curr, p_next;

always_ff @ (posedge clk, posedge rst) begin
    if (rst) begin
        {a_curr, w_curr, p_curr} <= 0; // initialize 
        // {a_curr, a_next, w_curr, w_next} <= 0; // initialize 
    end
    else begin
        a_curr <= a_next;
        w_curr <= w_next;
        p_curr <= p_next;
    end
end

always_comb begin
    w_next = w_curr;
    a_next = 0;
    p_next = 0;
    if (prefetch) begin
        w_next = w_i;
    end
    else if (conv) begin
        a_next = a_i;
        p_next = $signed(p_i + w_curr * a_curr);
    end
end

always_comb begin
    a_o = a_curr;
    p_o = p_curr;
    w_o = w_curr;
end

// assign a_o = a_curr;
// assign p_o = p_curr;
// assign w_o = w_curr;




endmodule
