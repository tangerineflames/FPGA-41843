## R[3:0]
set_property PACKAGE_PIN U15 [get_ports {vga_r[3]}]
set_property PACKAGE_PIN U16 [get_ports {vga_r[2]}]
set_property PACKAGE_PIN U17 [get_ports {vga_r[1]}]
set_property PACKAGE_PIN V15 [get_ports {vga_r[0]}]

## G[3:0]
set_property PACKAGE_PIN AA17 [get_ports {vga_g[3]}]
set_property PACKAGE_PIN AB17 [get_ports {vga_g[2]}]
set_property PACKAGE_PIN AA16 [get_ports {vga_g[1]}]
set_property PACKAGE_PIN AB16 [get_ports {vga_g[0]}]

## B[3:0]
set_property PACKAGE_PIN W15  [get_ports {vga_b[3]}]
set_property PACKAGE_PIN Y15  [get_ports {vga_b[2]}]
set_property PACKAGE_PIN AA14 [get_ports {vga_b[1]}]
set_property PACKAGE_PIN AB15 [get_ports {vga_b[0]}]
## VGA ͬ���ź�
set_property PACKAGE_PIN W16 [get_ports hsync]
set_property PACKAGE_PIN Y16 [get_ports vsync]

set_property IOSTANDARD LVCMOS33 [get_ports hsync]
set_property IOSTANDARD LVCMOS33 [get_ports vsync]


set_property IOSTANDARD LVCMOS33 [get_ports {vga_r[*] vga_g[*] vga_b[*] hsync vsync sys_clk}]
# ϵͳʱ�ӣ�PL 100MHz��
set_property PACKAGE_PIN AA18 [get_ports sys_clk]
set_property IOSTANDARD LVCMOS33 [get_ports sys_clk]
# ���� Vivado�����ǵ���ʱ�����룬����ר�ò�ֲ���?
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets -of_objects [get_ports sys_clk]]
create_clock -period 10.000 [get_ports sys_clk]

## === Mode switch button on H17 ===
set_property PACKAGE_PIN H17       [get_ports {key_in}]
set_property IOSTANDARD LVCMOS33   [get_ports {key_in}]
set_property PULLUP true           [get_ports {key_in}]

## === Mode switch button on C19 ===
set_property PACKAGE_PIN C19     [get_ports {key_model}]
set_property IOSTANDARD LVCMOS33   [get_ports {key_model}]
set_property PULLUP true           [get_ports {key_model}]

## key_count1 on A22
set_property PACKAGE_PIN A22 [get_ports {ped_sw_1}]
set_property IOSTANDARD LVCMOS33 [get_ports {ped_sw_1}]
set_property PULLUP true [get_ports {ped_sw_1}]
## key_count1 on D22
set_property PACKAGE_PIN D22 [get_ports {car_sw}]
set_property IOSTANDARD LVCMOS33 [get_ports {car_sw}]
set_property PULLUP true [get_ports {car_sw}]
## key_count1 on D22
set_property PACKAGE_PIN C22 [get_ports {car_sw2}]
set_property IOSTANDARD LVCMOS33 [get_ports {car_sw2}]
set_property PULLUP true [get_ports {car_sw2}]

set_property PACKAGE_PIN E21 [get_ports {mode_adapt_sw}]
set_property IOSTANDARD LVCMOS33 [get_ports {mode_adapt_sw}]
set_property PULLUP true [get_ports {mode_adapt_sw}]
# UART接收：FPGA从CP2102的TXD(引脚26)接收数据，连接到V17
set_property PACKAGE_PIN V17 [get_ports {uart_rx_pin}]
set_property IOSTANDARD LVCMOS33 [get_ports {uart_rx_pin}]
set_property PULLUP true [get_ports {uart_rx_pin}]
set_false_path -from [get_ports {uart_rx_pin}]
## 7-seg segment pins
set_property PACKAGE_PIN U14  [get_ports SEG_CA]
set_property PACKAGE_PIN V13  [get_ports SEG_CB]
set_property PACKAGE_PIN V14  [get_ports SEG_CC]
set_property PACKAGE_PIN W13  [get_ports SEG_CD]
set_property PACKAGE_PIN Y13  [get_ports SEG_CE]
set_property PACKAGE_PIN Y14  [get_ports SEG_CF]
set_property PACKAGE_PIN AA13 [get_ports SEG_CG]
set_property PACKAGE_PIN AB14 [get_ports SEG_DP]

## 7-seg digit select pins
set_property PACKAGE_PIN R15 [get_ports SEG_BIT1]
set_property PACKAGE_PIN R16 [get_ports SEG_BIT2]
set_property PACKAGE_PIN P16 [get_ports SEG_BIT3]
set_property PACKAGE_PIN M20 [get_ports SEG_BIT4]

## IO standard
set_property IOSTANDARD LVCMOS33 [get_ports {SEG_CA SEG_CB SEG_CC SEG_CD SEG_CE SEG_CF SEG_CG SEG_DP}]
set_property IOSTANDARD LVCMOS33 [get_ports {SEG_BIT1 SEG_BIT2 SEG_BIT3 SEG_BIT4}]
