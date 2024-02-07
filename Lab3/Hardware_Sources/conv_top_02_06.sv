/////////////////////////////////////////////////////////////////////
//
// Title: conv_top.sv
// Author: 
//
/////////////////////////////////////////////////////////////////////

`timescale 1 ns / 1 ps

module conv_top #(
  parameter IF_WIDTH     = 128,
  parameter IF_HEIGHT    = 128,
  parameter IF_CHANNEL   = 3,
  parameter IF_BITWIDTH  = 16,
  parameter IF_FRAC_BIT  = 8,
  parameter IF_PORT      = 27,

  parameter K_WIDTH      = 3,
  parameter K_HEIGHT     = 3,
  parameter K_CHANNEL    = 3,
  parameter K_BITWIDTH   = 8,
  parameter K_FRAC_BIT   = 6,
  parameter K_PORT       = 1,
  parameter K_NUM        = 3,

  parameter OF_WIDTH     = 128,
  parameter OF_HEIGHT    = 128,
  parameter OF_CHANNEL   = 3,
  parameter OF_BITWIDTH  = 16,
  parameter OF_FRAC_BIT  = 8,
  parameter OF_PORT      = 1,
  parameter OF_NUM       = 3
)
(
  input  logic                                             clk,
  input  logic                                             rst,

  input  logic                                             if_start,
  input  logic                                             k_prefetch,
  output logic                                             of_done,

  input  logic [IF_PORT-1:0][IF_BITWIDTH-1:0]              if_i_data,
  input  logic [IF_PORT-1:0]                               if_i_valid,
  input  logic [K_NUM-1:0][K_PORT-1:0][K_BITWIDTH-1:0]     k_i_data,
  input  logic [K_NUM-1:0][K_PORT-1:0]                     k_i_valid,
  output logic [OF_NUM-1:0][OF_PORT-1:0][OF_BITWIDTH-1:0]  of_o_data,
  output logic [OF_NUM-1:0][OF_PORT-1:0]                   of_o_valid
);

/////////////////////////////////////////////////////////////////////

localparam A_BITWIDTH  = IF_BITWIDTH;
localparam A_FRAC_BIT  = IF_FRAC_BIT;
localparam W_BITWIDTH  = K_BITWIDTH;
localparam W_FRAC_BIT  = K_FRAC_BIT;
localparam P_BITWIDTH  = A_BITWIDTH*2 + W_BITWIDTH;
localparam P_FRAC_BIT  = A_FRAC_BIT + W_FRAC_BIT;

/////////////////////////////////////////////////////////////////////
logic                                                      prefetch;
logic                                                      conv;
logic signed [IF_PORT-1:0][K_NUM:0][A_BITWIDTH-1:0]        pipe_a;
logic signed [IF_PORT:0][K_NUM-1:0][K_BITWIDTH-1:0]        pipe_w;
logic signed [IF_PORT:0][K_NUM-1:0][P_BITWIDTH-1:0]        pipe_p;

genvar a,b;
for ( a=0 ; a<IF_PORT ; a=a+1 ) begin
  assign pipe_a[a][0] = if_i_data[a];
end

for ( b=0 ; b<K_NUM ; b=b+1 ) begin
  assign pipe_w[0][b] = k_i_data[b][0];
  assign pipe_p[0][b] = 40'b0;
  assign of_o_data[b] = {pipe_p[IF_PORT][b][39], pipe_p[IF_PORT][b][20:14], pipe_p[IF_PORT][b][13:6]};
end

genvar i,j;
generate  
  for ( i=0 ; i<IF_PORT ; i=i+1 ) begin : MAC_outer
    for ( j=0 ; j<K_NUM ; j=j+1 ) begin : MAC_inner
      MAC #(
        .A_BITWIDTH      ( A_BITWIDTH       ),
        .A_FRAC_BIT      ( A_FRAC_BIT       ),
        .W_BITWIDTH      ( W_BITWIDTH       ),
        .W_FRAC_BIT      ( W_FRAC_BIT       ),
        .P_BITWIDTH      ( P_BITWIDTH       ),
        .P_FRAC_BIT      ( P_FRAC_BIT       )
      )
      u_mac (
        .clk       ( clk              ),
        .rst       ( rst              ),

        .prefetch  ( prefetch         ), 
        .conv      ( conv             ), 

        .a_i       ( pipe_a[i][j]     ), //
        .w_i       ( pipe_w[i][j]     ),//
        .p_i       ( pipe_p[i][j]     ),//

        .a_o       ( pipe_a[i][j+1]   ),//
        .w_o       ( pipe_w[i+1][j]   ),//
        .p_o       ( pipe_p[i+1][j]   )//
      );
    end
  end
endgenerate

enum logic [3:0] {
  IDLE,
  READY1,
  READY2,
  READY3,
  READY4,
  PREF,
  CONV,
  CONV1_1,
  CONV1_2,
  CONV2,
  CONV3,
  CONV3_1,
  CONV3_2,
  CONV4
} next_state, curr_state;

always_ff @ (posedge clk, posedge rst) begin
  if (rst) begin
    curr_state <= IDLE;
  end 
  else begin 
    curr_state <= next_state;
  end
end

always_comb begin
  next_state = curr_state;
  
  case (curr_state)
    IDLE: begin
      of_o_valid = 0;
      of_done = 0;
      conv = 0;
      if (k_prefetch) begin
        next_state = READY1;
      end
      else if (if_start) begin
        next_state = READY3;
      end
    end
    READY1: begin 
      next_state = READY2;
    end
    READY2: begin 
      next_state = PREF;
    end
    PREF: begin
      if (!prefetch) begin
        next_state = IDLE;
      end
      else begin
        next_state = PREF;
      end
    end
    READY3: begin 
      next_state = READY4;
    end
    READY4: begin 
      next_state = CONV;
    end
    CONV: begin 
      if (out_start) begin
        next_state = CONV1_1;
      end
      else begin 
        next_state = CONV;
      end
    end
    CONV1_1: begin
      next_state = CONV1_2;
      end
    CONV1_2: begin
      next_state = CONV2;
      of_o_valid = 3'b001;
    end
    CONV2: begin 
      next_state = CONV3;
      of_o_valid = 3'b011;
    end
    CONV3: begin 
      of_o_valid = 3'b111;
      if (out_end) begin
        next_state = CONV3_1;
      end
      else begin 
        next_state = CONV3;
      end
    end
    CONV3_1: begin 
      next_state = CONV3_2;
    end
    CONV3_2: begin 
      of_o_valid = 3'b110;
      next_state = CONV4;
    end
    CONV4: begin 
      next_state = IDLE;
      of_o_valid = 3'b100;
      of_done = 1;
    end
  endcase 
end
  

logic out_start;
logic out_end;

always_comb begin 
  if (curr_state == PREF & k_i_valid[1]) begin
    prefetch = 1;
  end
  else begin
    prefetch = 0;
  end
end

always_comb begin 
  out_start = 0;
  out_end = 0;
  conv = 0;
  if (curr_state == CONV & (if_i_valid[0] & !if_i_valid[IF_PORT-1])) begin
    conv = 1;
  end
  else if (curr_state == CONV & (if_i_valid[IF_PORT-1])) begin 
    conv = 1;
    out_start = 1;
  end
  else if (curr_state == CONV3 & (!if_i_valid[IF_PORT-1])) begin
    conv = 1;
    out_end = 1;
  end
end

endmodule
