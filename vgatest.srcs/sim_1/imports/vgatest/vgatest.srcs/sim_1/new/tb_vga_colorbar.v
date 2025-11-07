`timescale 1ns / 1ps

module tb_vga_colorbar();

reg  sys_clk;
wire sys_rst_n = 1'b1;      // 固定拉高
wire hsync;
wire vsync;
wire [3:0] vga_r;
wire [3:0] vga_g;
wire [3:0] vga_b;

// 只产生时钟
initial begin
    sys_clk = 1'b1;
end
always #5 sys_clk = ~sys_clk;  // 100MHz

// DUT
vga_colorbar vga_colorbar_inst
(
    .sys_clk   (sys_clk),
    .sys_rst_n (sys_rst_n),   // 始终为1
    .hsync     (hsync),
    .vsync     (vsync),
    .vga_r     (vga_r),
    .vga_g     (vga_g),
    .vga_b     (vga_b)
);

// 可选：跑一段时间后自动停
// initial #2_000_000 $stop;  // 2 ms

endmodule

