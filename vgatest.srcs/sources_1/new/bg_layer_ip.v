// bg_layer_ip.v -- Ö»¶Á±³¾°£¨µ÷É«°åË÷Òý£©
module bg_layer_ip #(
  parameter H = 640, V = 480, BITS = 4
)(
  input  wire           clk,
  input  wire [9:0]     pix_x,
  input  wire [9:0]     pix_y,
  output reg  [BITS-1:0] bg_idx  // ±³¾°ÏñËØË÷Òý
);
  localparam integer DEPTH = H*V;
  (* ram_style="block" *) reg [BITS-1:0] rom [0:DEPTH-1];  // BRAM
  initial $readmemh("bg_idx_640x480.hex", rom);

  wire [18:0] addr = pix_y*H + pix_x; // 640*480 < 2^19
  always @(posedge clk) bg_idx <= rom[addr];
endmodule
