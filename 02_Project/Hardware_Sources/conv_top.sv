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
logic [IF_PORT-1:0][K_NUM:0][A_BITWIDTH-1:0]               pipe_a;
logic [IF_PORT:0][K_NUM-1:0][K_BITWIDTH-1:0]               pipe_w;
logic [IF_PORT:0][K_NUM-1:0][K_BITWIDTH-1:0]               pipe_p;

genvar a,b,c;
for ( a=0 ; a<IF_PORT ; a=a+1 ) begin
  assign pipe_a[a][0] = if_i_data[a];
end

for ( b=0 ; b<K_NUM ; b=b+1 ) begin
  assign pipe_w[0][b] = k_i_data[b][0];
end

for ( c=0 ; c<K_NUM ; c=c+1 ) begin
  assign pipe_p[0][c] = 0;
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




endmodule
