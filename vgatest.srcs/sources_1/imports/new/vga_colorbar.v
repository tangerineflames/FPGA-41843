`timescale 1ns / 1ps
module vga_colorbar(
    input  wire       sys_clk,
    input  wire       key_in,      // ԭ������/ģʽ�л���
    input  wire       key_model,   // �£�ģ���л�����0=ԭ������1=·�ڳ�����
    input wire ped_sw_1,
    input wire car_sw,
    input wire car_sw2,
    input wire mode_adapt_sw,
    input  wire uart_rx_pin,
    output wire       hsync,
    output wire       vsync,
    output wire [3:0] vga_r,
    output wire [3:0] vga_g,
    output wire [3:0] vga_b
);
    // ================== ʱ�Ӻ͸�λ ==================
    wire vga_clk;
    wire locked;
    wire rst_n = locked;

    clk_wiz_0 u_clk (
        .clk_out1 (vga_clk),
        .reset    (1'b0),
        .locked   (locked),
        .clk_in1  (sys_clk)
    );

    // ================== VGA ɨ���ź� ==================
    wire [9:0]  pix_x, pix_y;
    wire [15:0] vga_rgb_int;

    // ================== �������ģ�~30Hz�� ==================
    wire tick_anim;
    tick_anim u_tanim (
        .clk       (vga_clk),
        .rst_n     (rst_n),
        .tick_anim (tick_anim)
    );

    // ================== ������ģʽ������/���������� ==================
    wire key_flag;  // ԭ����ȥ����ĵ����壨sys_clk��
    key_filter #(
        .CLK_HZ     (100_000_000),
        .DEBOUNCE_MS(20),
        .ACTIVE_LOW (1)
    ) u_key (
        .sys_clk   (sys_clk),
        .sys_rst_n (rst_n),
        .key_in    (key_in),
        .key_flag  (key_flag)
    );

    wire [1:0] mode;
    Periodchange u_mode (
        .clk             (sys_clk),
        .rst_n           (rst_n),
        .key_second_level(key_flag),
        .modenum         (mode)
    );

wire key_count1_flag;
key_filter #(
    .CLK_HZ     (100_000_000),
    .DEBOUNCE_MS(20),
    .ACTIVE_LOW (1)
) u_key_count1 (
    .sys_clk   (sys_clk),
    .sys_rst_n (rst_n),
    .key_in    (ped_sw_1),
    .key_flag  (key_count1_flag)
);
wire key_count2_flag;
key_filter #(
    .CLK_HZ     (100_000_000),
    .DEBOUNCE_MS(20),
    .ACTIVE_LOW (1)
) u_key_count2 (
    .sys_clk   (sys_clk),
    .sys_rst_n (rst_n),
    .key_in    (car_sw),
    .key_flag  (key_count2_flag)
);
wire key_count3_flag;
key_filter #(
    .CLK_HZ     (100_000_000),
    .DEBOUNCE_MS(20),
    .ACTIVE_LOW (1)
) u_key_count3 (
    .sys_clk   (sys_clk),
    .sys_rst_n (rst_n),
    .key_in    (car_sw2),
    .key_flag  (key_count3_flag)
);
wire key_count4_flag;
key_filter #(
    .CLK_HZ     (100_000_000),
    .DEBOUNCE_MS(20),
    .ACTIVE_LOW (1)
) u_key_count4 (
    .sys_clk   (sys_clk),
    .sys_rst_n (rst_n),
    .key_in    (mode_adapt_sw),
    .key_flag  (key_count4_flag)
);
wire        uart_rx_valid;
wire [7:0]  uart_rx_byte;

// ================== �????化方案：直接用vga_clk ==================
// 优点：无�????CDC，简单直�????
// 缺点：波特率误差-2.5%（可接受范围内，但不是最优）
uart_rx #(
  .CLK_HZ(25_175_000),
  .BAUD  (115200)
) u_uart_rx (
  .clk   (vga_clk),
  .rst_n (rst_n),
  .rx    (uart_rx_pin),
  .valid (uart_rx_valid),
  .data  (uart_rx_byte)
);

    // 直接使用，无�????跨时钟域处理
    wire uart_rx_valid_vga = uart_rx_valid;
    wire [7:0] uart_rx_byte_vga = uart_rx_byte;

    // ================== ����ӳ�䣨sys_clk �� ==================
    wire [7:0] car_count_up_sc;
    wire [7:0] car_count_down_sc;
    wire [7:0] people_count_sc;

    traffic_count_map #(
        .UP_MORNING_PEAK(6),     .DN_MORNING_PEAK(6),
        .UP_NORMAL(3),           .DN_NORMAL(3),
        .UP_EVENING_PEAK(6),     .DN_EVENING_PEAK(6),
        .PEOPLE_MORNING_PEAK(7), .PEOPLE_NORMAL(2), .PEOPLE_EVENING_PEAK(7)
    ) u_count (
        .mode          (mode),
        .car_count_up  (car_count_up_sc),
        .car_count_down(car_count_down_sc),
        .people_count  (people_count_sc)
    );

    // ================== ����ͬ���� vga_clk ==================
    reg [7:0] car_count_up_s0,   car_count_up_s1;
    reg [7:0] car_count_down_s0, car_count_down_s1;
    reg [7:0] people_count_s0,   people_count_s1;

    always @(posedge vga_clk or negedge rst_n) begin
        if(!rst_n) begin
            car_count_up_s0   <= 8'd0; car_count_up_s1   <= 8'd0;
            car_count_down_s0 <= 8'd0; car_count_down_s1 <= 8'd0;
            people_count_s0   <= 8'd0; people_count_s1   <= 8'd0;
        end else begin
            car_count_up_s0   <= car_count_up_sc;   car_count_up_s1   <= car_count_up_s0;
            car_count_down_s0 <= car_count_down_sc; car_count_down_s1 <= car_count_down_s0;
            people_count_s0   <= people_count_sc;   people_count_s1   <= people_count_s0;
        end
    end

    reg [1:0] mode_s0, mode_s1;
    always @(posedge vga_clk or negedge rst_n) begin
        if(!rst_n) begin
            mode_s0 <= 2'd1; // Ĭ�� NORMAL
            mode_s1 <= 2'd1;
        end else begin
            mode_s0 <= mode;
            mode_s1 <= mode_s0;
        end
    end
    wire [1:0] mode_vga = mode_s1;

    wire [7:0] car_count_up   = car_count_up_s1;
    wire [7:0] car_count_down = car_count_down_s1;
    wire [7:0] people_count   = people_count_s1;

    // ================== ���˼�⣨���˾ۺϣ�????? ==================
    wire [9:0] ped_wait_y;
    wire       wait_zone, in_crossing;

    // ����λ��
    wire [9:0] car_x;
    wire [9:0] ped_y;

    ped_sensor #(
        .Y_TOP       (10'd190),
        .Y_BOTTOM    (10'd310),
        .WAIT_MARGIN (10'd8),
        .CLEAR_MARGIN(10'd4),
        .R           (10'd5),
        .MAX_PEOPLE  (32)
    ) u_pedsensor (
        .clk         (vga_clk),
        .rst_n       (rst_n),
        .ped_y       (ped_y),
        .people_count(people_count_s1),
        .phase_step  (10'd16),

        .ped_wait_y  (ped_wait_y),
        .wait_zone   (wait_zone),
        .in_crossing (in_crossing)
    );

    // ��·��⣺�����ڵȴ��������ڴ�Խ����?????"������"
    wire det_main = 1'b1;
    wire det_side = wait_zone | in_crossing;

    // ================== ����Ӧ�źŵ� ==================
    wire main_R, main_Y, main_G;
    wire side_R, side_Y, side_G;
    wire [2:0] tl_state;
    wire [7:0] walk_cur_sec;
    
    // ��������Ч����������½��ʱ����
    reg in_crossing_d1;
    reg [7:0] walk_final_sec;
    reg [3:0] eff_tens, eff_ones, eff_frac;  // Ч�ʲ�ֳɵ�λ��ʡ��?? vga_pic �еĳ���
    reg [9:0] eff_val;  // Ч��м����
    
    always @(posedge vga_clk or negedge rst_n) begin
        if (!rst_n) begin
            in_crossing_d1 <= 1'b0;
            walk_final_sec <= 8'd0;
            eff_tens <= 4'd0;
            eff_ones <= 4'd0;
            eff_frac <= 4'd0;
            eff_val <= 10'd0;
        end else begin
            in_crossing_d1 <= in_crossing;
            // ���½�ţ�������һ��½��ʱ����??
            if (in_crossing_d1 && !in_crossing && (walk_cur_sec > 0)) begin
                walk_final_sec <= walk_cur_sec;
                // ����Ч�ʣ���������10��/walktime�����ͱ�99.9������ֳɵ��?
                if ((people_count * 10) / walk_cur_sec > 999) begin
                    eff_tens <= 4'd9;
                    eff_ones <= 4'd9;
                    eff_frac <= 4'd9;
                end else begin
                    eff_val <= (people_count * 10) / walk_cur_sec;
                    eff_tens <= eff_val / 100;
                    eff_ones <= (eff_val % 100) / 10;
                    eff_frac <= eff_val % 10;
                end
            end
        end
    end

        traffic_ctrl_adaptive #(
        .CLK_HZ(25_175_000),
        .MIN_GREEN_S(3),
        .MAX_GREEN_S(12),
        .YELLOW_S(1),
        .ALL_RED_S(1),
        .GAP_S(1)
    ) u_tl (
        .clk      (vga_clk),
        .rst_n    (rst_n),

        //��������ģʽ����ȥ��0=��߷�,1=����,2=���߷壩
        .mode     (mode_vga),

        .tick     (1'b0),
        .det_main (det_main),
        .det_side (det_side),
        .in_crossing(in_crossing),
        .main_G   (main_G), .main_Y(main_Y), .main_R(main_R),
        .side_G   (side_G), .side_Y(side_Y), .side_R(side_R),
        .cur_sec  (walk_cur_sec), .state(tl_state)
    );


    // ================== ��������/���ˣ� ==================
    wire [3:0] car_speed = 4'd8;   // ����ÿ tick 8 ����
    wire [3:0] ped_speed = 4'd1;   // �ˣ�ÿ tick 1 ����

    scene_anim u_anim (
        .clk      (vga_clk),
        .rst_n    (rst_n),
        .tick     (tick_anim),
        .main_R   (main_R),
        .main_G   (main_G),
        .side_G   (side_G),
        .car_speed(car_speed),
        .ped_speed(ped_speed),
        .car_x    (car_x),
        .ped_y    (ped_y)
    );

    // =========================================================
    // =============== ������ģ���л�����߼�????? ===================
    // =========================================================
    // 1) �ڶ�������ȥ�����л�ģ��
    wire key_flag_model; // sys_clk ������
    key_filter #(
        .CLK_HZ     (100_000_000),
        .DEBOUNCE_MS(20),
        .ACTIVE_LOW (1)
    ) u_key_model (
        .sys_clk   (sys_clk),
        .sys_rst_n (rst_n),
        .key_in    (key_model),
        .key_flag  (key_flag_model)
    );

    // 2) sys_clk ��ת��ѡ��λ
    reg scene_sel_sc;  // 0=ԭ����(vga_pic)  1=·��(scene_cross)
    always @(posedge sys_clk or negedge rst_n) begin
        if(!rst_n) scene_sel_sc <= 1'b0;
        else if (key_flag_model) scene_sel_sc <= ~scene_sel_sc;
    end

    // 3) ͬ���� vga_clk ��
    reg scene_sel_s0, scene_sel_s1;
    always @(posedge vga_clk or negedge rst_n) begin
        if(!rst_n) begin scene_sel_s0 <= 1'b0; scene_sel_s1 <= 1'b0; end
        else begin scene_sel_s0 <= scene_sel_sc; scene_sel_s1 <= scene_sel_s0; end
    end
    wire scene_sel = scene_sel_s1;

    // ================== ͼ�����ɣ���·�� ==================
    // ԭ���ĳ���
    wire [15:0] pix_pic;
    vga_pic u_pic (
        .vga_clk   (vga_clk),
        .sys_rst_n (rst_n),
        .pix_x     (pix_x),
        .pix_y     (pix_y),
        .main_R    (main_R), .main_Y(main_Y), .main_G(main_G),
        .side_R    (side_R), .side_Y(side_Y), .side_G(side_G),
        .car_x     (car_x),
        .ped_y     (ped_y),
        .tl_state  (tl_state),

        .car_count_up   (car_count_up),
        .car_count_down (car_count_down),
        .people_count   (people_count),
        .walk_time_sec  (walk_final_sec),
        .eff_tens       (eff_tens),
        .eff_ones       (eff_ones),
        .eff_frac       (eff_frac),

        .modenum        (mode_vga),       // 右上角模式色�???
        .ped_phase_step (10'd16),         // 行人间距

        .pix_data       (pix_pic)
    );

    // ������·��ģ�ͣ���̬��ͼ�����????? scene_cross.v �ӵ����̣�
    wire [15:0] pix_cross;

scene_cross #(
  .ZB_OFF(35)
) u_cross(
  .vga_clk (vga_clk),
  .sys_rst_n (rst_n),
  .pix_x   (pix_x),
  .pix_y   (pix_y),
  .ped_sw_1(ped_sw_1),
  .car_sw(car_sw),  
  .car_sw2(car_sw2),
  .mode_adapt_sw(mode_adapt_sw),
    .uart_rx_valid (uart_rx_valid_vga),     // 使用同步后的信号
  .uart_rx_byte  (uart_rx_byte_vga),      // 使用同步后的信号
  .pix_cross(pix_cross)
);

    // ѡ�������һ�׻���?????
    wire [15:0] pix_data_mux = scene_sel ? pix_cross : pix_pic;

    // ================== VGA ���ƣ�ע���????? mux ������أ�????? ==================
    vga_ctrl u_ctrl (
        .vga_clk   (vga_clk),
        .sys_rst_n (rst_n),
        .pix_data  (pix_data_mux),
        .pix_x     (pix_x),
        .pix_y     (pix_y),
        .hsync     (hsync),
        .vsync     (vsync),
        .vga_rgb   (vga_rgb_int)
    );

    // ================== RGB 444 ���????? ==================
    assign vga_r = vga_rgb_int[15:12];
    assign vga_g = vga_rgb_int[10:7];
    assign vga_b = vga_rgb_int[4:1];
endmodule