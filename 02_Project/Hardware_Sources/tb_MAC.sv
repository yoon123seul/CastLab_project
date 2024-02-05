// Code your testbench here
// or browse Examples
`timescale 1 ns / 1 ps


module tb_MAC_top ();

logic                                             clk;
logic                                             rst;

logic                                             prefetch;
logic                                             conv;

logic [16-1:0]                                    a_i;
logic [8-1:0]                                     w_i;
logic [40-1:0]                                    p_i;
logic [16-1:0]                                    a_o;
logic [8-1:0]                                     w_o;
logic [40-1:0]                                    p_o;

MAC u_MAC(
  .clk       ( clk              ),
  .rst       ( rst              ),

  .prefetch  ( prefetch         ), // No Write
  .conv      ( conv             ), // No Write

  .a_i       ( a_i              ),
  .w_i       ( w_i              ),
  .p_i       ( p_i              ),

  .a_o       ( a_o              ),
  .w_o       ( w_o              ),
  .p_o       ( p_o              )
);

initial begin
  clk = 0;
  forever begin
    clk = #((1000/100)/2) ~clk;
  end
end

initial begin
    rst = 0;
    prefetch = 0;
    conv = 0;
    a_i = 0;
    w_i = 0;
    p_i = 0;
    @ (posedge clk);
    rst = 1;
  	@ (posedge clk);
  	@ (posedge clk);
    rst = 0;
    prefetch = 1;
    w_i = 3;
    @ (posedge clk);
    w_i = 2;
    @ (posedge clk);
    w_i = 1;
    @ (posedge clk);
  	prefetch = 0;
    @ (posedge clk);
  
  	conv = 1;
  	a_i = 1;
  	p_i = 1;
  	@ (posedge clk);
  	a_i = 2;
  	p_i = 2;
  	@ (posedge clk);
  	a_i = 3;
  	p_i = 3;
  	@ (posedge clk);
  	@ (posedge clk);
  	
  
	$finish;

end



initial begin
  $dumpfile("dump.vcd"); 
  $dumpvars(0, tb_MAC_top);
end






endmodule