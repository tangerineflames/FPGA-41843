`timescale 1ns / 1ps
module vga_colorbar(
    input  wire       sys_clk,
    input  wire       key_in,      // Ô­ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½/Ä£Ê½ï¿½Ğ»ï¿½ï¿½ï¿½
    input  wire       key_model,   // ï¿½Â£ï¿½Ä£ï¿½ï¿½ï¿½Ğ»ï¿½ï¿½ï¿½ï¿½ï¿½0=Ô­ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½1=Â·ï¿½Ú³ï¿½ï¿½ï¿½ï¿½ï¿½
    input wire ped_sw_1,
    input wire car_sw,
    input wire car_sw2,
    input wire mode_adapt_sw,
    input  wire uart_rx_pin,
    output wire       hsync,
    output wire       vsync,
    output wire [3:0] vga_r,
    output wire [3:0] vga_g,
    output wire [3:0] vga_b,
    output wire SEG_CA,
    output wire SEG_CB,
    output wire SEG_CC,
    output wire SEG_CD,
    output wire SEG_CE,
    output wire SEG_CF,
    output wire SEG_CG,
    output wire SEG_DP,
    output wire SEG_BIT1,
    output wire SEG_BIT2,
    output wire SEG_BIT3,
    output wire SEG_BIT4
);
    // ================== Ê±ï¿½ÓºÍ¸ï¿½Î» ==================
    wire vga_clk;
    wire locked;
    wire rst_n = locked;

    clk_wiz_0 u_clk (
        .clk_out1 (vga_clk),
        .reset    (1'b0),
        .locked   (locked),
        .clk_in1  (sys_clk)
    );

    // ================== VGA É¨ï¿½ï¿½ï¿½Åºï¿½ ==================
    wire [9:0]  pix_x, pix_y;
    wire [15:0] vga_rgb_int;

    // ================== ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£ï¿½~30Hzï¿½ï¿½ ==================
    wire tick_anim;
    tick_anim u_tanim (
        .clk       (vga_clk),
        .rst_n     (rst_n),
        .tick_anim (tick_anim)
    );

    // ================== ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½/ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ==================
    wire key_flag;  // Ô­ï¿½ï¿½ï¿½ï¿½È¥ï¿½ï¿½ï¿½ï¿½Äµï¿½ï¿½ï¿½ï¿½å£¨sys_clkï¿½ï¿½
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

// ================== ï¿????åŒ–æ–¹æ¡ˆï¼šç›´æ¥ç”¨vga_clk ==================
// ä¼˜ç‚¹ï¼šæ— ï¿????CDCï¼Œç®€å•ç›´ï¿????
// ç¼ºç‚¹ï¼šæ³¢ç‰¹ç‡è¯¯å·®-2.5%ï¼ˆå¯æ¥å—èŒƒå›´å†…ï¼Œä½†ä¸æ˜¯æœ€ä¼˜ï¼‰
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

    // ç›´æ¥ä½¿ç”¨ï¼Œæ— ï¿????è·¨æ—¶é’ŸåŸŸå¤„ç†
    wire uart_rx_valid_vga = uart_rx_valid;
    wire [7:0] uart_rx_byte_vga = uart_rx_byte;

    // ================== ï¿½ï¿½ï¿½ï¿½Ó³ï¿½ä£¨sys_clk ï¿½ï¿½ ==================
    wire [7:0] car_count_up_sc;
    wire [7:0] car_count_down_sc;
    wire [7:0] people_count_sc;
    wire ped_cfg_normal_sel = ped_sw_1; 
    traffic_count_map #(
        .UP_MORNING_PEAK(6),     .DN_MORNING_PEAK(6),
        .UP_NORMAL(3),           .DN_NORMAL(3),
        .UP_EVENING_PEAK(6),     .DN_EVENING_PEAK(6),
        .PEOPLE_MORNING_PEAK(7), 
        .PEOPLE_NORMAL(2),       // NORMAL Ä£Ê½"ÉÙÈË"£º2 ÈË
        .PEOPLE_NORMAL_ALT(5),   // NORMAL Ä£Ê½"¶àÈË"£º5 ÈË
        .PEOPLE_EVENING_PEAK(7)
    ) u_count (
        .mode          (mode),
        .ped_cfg_normal_sel (ped_cfg_normal_sel),
        .car_count_up  (car_count_up_sc),
        .car_count_down(car_count_down_sc),
        .people_count  (people_count_sc)
    );

    // ================== ï¿½ï¿½ï¿½ï¿½Í¬ï¿½ï¿½ï¿½ï¿½ vga_clk ==================
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
            mode_s0 <= 2'd1; // Ä¬ï¿½ï¿½ NORMAL
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

    // ================== ï¿½ï¿½ï¿½Ë¼ï¿½â£¨ï¿½ï¿½ï¿½Ë¾ÛºÏ£ï¿????? ==================
    wire [9:0] ped_wait_y;
    wire       wait_zone, in_crossing;

    // ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½
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

    // ï¿½ï¿½Â·ï¿½ï¿½â£ºï¿½ï¿½ï¿½ï¿½ï¿½ÚµÈ´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú´ï¿½Ô½ï¿½ï¿½ï¿½ï¿?????"ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½"
    wire det_main = 1'b1;
    wire det_side = wait_zone | in_crossing;

    // ================== ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ÅºÅµï¿½ ==================
    wire main_R, main_Y, main_G;
    wire side_R, side_Y, side_G;
    wire [2:0] tl_state;
    wire [7:0] walk_cur_sec;
    wire [7:0] peak_main_green_s;
    wire [7:0] peak_side_green_s;
    wire [7:0]  green_left_s;
    wire        green_on_main;

    reg in_crossing_d1;
    reg [7:0] walk_final_sec;
    reg [3:0] eff_tens, eff_ones, eff_frac;  // Ğ§ÂÊ xx.x
    reg [9:0] eff_val;                       // Ö»ÊÇÎªÁËµ÷ÊÔ±£Áô

    integer eff_raw;  // ÓÃÀ´×öÁÙÊ±¼ÆËãµÄÕûĞÍ

    always @(posedge vga_clk or negedge rst_n) begin
        if (!rst_n) begin
            in_crossing_d1 <= 1'b0;
            walk_final_sec <= 8'd0;
            eff_tens       <= 4'd0;
            eff_ones       <= 4'd0;
            eff_frac       <= 4'd0;
            eff_val        <= 10'd0;
        end else begin
            in_crossing_d1 <= in_crossing;

            // ¡ï Ö»ÔÚ NORMAL Ä£Ê½ÏÂÍ³¼Æ walktime / efficiency
            if (mode_vga == 2'd1) begin
                // "ÉÏÒ»ÅÄÔÚ°ßÂíÏß£¬ÕâÒ»ÅÄÃ»ÈËÁË" -> Ò»Åú¹ı½Ö½áÊø
                if (in_crossing_d1 && !in_crossing && (walk_cur_sec > 0)) begin
                    walk_final_sec <= walk_cur_sec;

                    // ÁÙÊ±Ëã³öÔ­Ê¼Ğ§ÂÊ people_count*10 / walk_cur_sec
                    eff_raw = (people_count * 10) / walk_cur_sec;

                    // ±¥ºÍµ½ 0~999
                    if (eff_raw > 999)
                        eff_raw = 999;
                    else if (eff_raw < 0)
                        eff_raw = 0;

                    eff_val  <= eff_raw[9:0];         // ·½±ãÄãÒÔºóµ÷ÊÔ
                    eff_tens <= eff_raw / 100;
                    eff_ones <= (eff_raw % 100) / 10;
                    eff_frac <= eff_raw % 10;
                end
            end else begin
                // ¡ï ¸ß·åÄ£Ê½£ºÕâĞ©Êı²»ÓÃ£¬Í³Ò»ÇåÁã
                walk_final_sec <= 8'd0;
                eff_tens       <= 4'd0;
                eff_ones       <= 4'd0;
                eff_frac       <= 4'd0;
                eff_val        <= 10'd0;
            end
        end
    end


    traffic_ctrl_adaptive #(
    .CLK_HZ       (25_175_000),
    .SEC_SCALE    (4),      // ¡ï ĞÂÔö

    .MIN_GREEN_S  (20),
    .MAX_GREEN_S  (48),
    .YELLOW_S     (4),
    .ALL_RED_S    (4),
    .GAP_S        (4),

    .MORN_MAIN_S  (48),
    .MORN_SIDE_S  (28),
    .EVEN_MAIN_S  (48),
    .EVEN_SIDE_S  (28)
    ) u_tl (
        .clk      (vga_clk),
        .rst_n    (rst_n),
        .mode     (mode_vga),
        .tick     (1'b0),
        .det_main (det_main),
        .det_side (det_side),
        .in_crossing (in_crossing),

        .main_G   (main_G), .main_Y(main_Y), .main_R(main_R),
        .side_G   (side_G), .side_Y(side_Y), .side_R(side_R),
        .cur_sec  (walk_cur_sec),
        .state    (tl_state),

        .peak_main_green_s (peak_main_green_s),
        .peak_side_green_s (peak_side_green_s),
        
        .green_left_s      (green_left_s),
        .green_on_main     (green_on_main)
    );

    // ================== ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½/ï¿½ï¿½ï¿½Ë£ï¿½ ==================
    wire [3:0] car_speed = 4'd8;   // ï¿½ï¿½ï¿½ï¿½Ã¿ tick 8 ï¿½ï¿½ï¿½ï¿½
    wire [3:0] ped_speed = 4'd1;   // ï¿½Ë£ï¿½Ã¿ tick 1 ï¿½ï¿½ï¿½ï¿½

    scene_anim u_anim (
        .clk      (vga_clk),
        .rst_n    (rst_n),
        .tick     (tick_anim),
        .main_R   (main_R),
        .main_G   (main_G),
        .side_G   (side_G),
        .car_speed(car_speed),
        .ped_speed(ped_speed),
        
        .people_count  (people_count),
        .ped_phase_step(10'd16), 
        
        .car_x    (car_x),
        .ped_y    (ped_y)
    );

    // =========================================================
    // =============== ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£ï¿½ï¿½ï¿½Ğ»ï¿½ï¿½ï¿½ï¿½ï¿½ß¼ï¿????? ===================
    // =========================================================
    // 1) ï¿½Ú¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È¥ï¿½ï¿½ï¿½ï¿½ï¿½Ğ»ï¿½Ä£ï¿½ï¿½
    wire key_flag_model; // sys_clk ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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

    // 2) sys_clk ï¿½ï¿½×ªï¿½ï¿½Ñ¡ï¿½ï¿½Î»
    reg scene_sel_sc;  // 0=Ô­ï¿½ï¿½ï¿½ï¿½(vga_pic)  1=Â·ï¿½ï¿½(scene_cross)
    always @(posedge sys_clk or negedge rst_n) begin
        if(!rst_n) scene_sel_sc <= 1'b0;
        else if (key_flag_model) scene_sel_sc <= ~scene_sel_sc;
    end

    // 3) Í¬ï¿½ï¿½ï¿½ï¿½ vga_clk ï¿½ï¿½
    reg scene_sel_s0, scene_sel_s1;
    always @(posedge vga_clk or negedge rst_n) begin
        if(!rst_n) begin scene_sel_s0 <= 1'b0; scene_sel_s1 <= 1'b0; end
        else begin scene_sel_s0 <= scene_sel_sc; scene_sel_s1 <= scene_sel_s0; end
    end
    wire scene_sel = scene_sel_s1;
    
    // À´×Ô traffic_ctrl_adaptive µÄÊıÂë¹ÜÖµ£¨½ÖµÀ³¡¾°£©
    wire [7:0] green_left_s_street = green_left_s;
    wire       green_on_main_street = green_on_main;

    
    // À´×Ô traffic_adapt2 µÄÊıÂë¹ÜÖµ£¨Ê®×ÖÂ·¿Ú³¡¾°£©
    wire [7:0] phase_left_s_cross;
    wire [2:0] phase_id_cross;
    
    // seg7 ×îÖÕÏÔÊ¾µÄÖµ£º¸ù¾İ scene_sel Ñ¡Ôñ
    wire [7:0] seg_value = scene_sel ? phase_left_s_cross : green_left_s_street;
    seg7_4digit u_seg7 (
        .clk      (sys_clk),      // ÓÃ 100MHz£¬Ë¢ĞÂ»á±È½ÏÎÈ¶¨
        .rst_n    (rst_n),
        .value    (seg_value), // µ±Ç°ÂÌµÆÊ£Óà"ĞéÄâÃë" 0~99

        .SEG_CA   (SEG_CA),
        .SEG_CB   (SEG_CB),
        .SEG_CC   (SEG_CC),
        .SEG_CD   (SEG_CD),
        .SEG_CE   (SEG_CE),
        .SEG_CF   (SEG_CF),
        .SEG_CG   (SEG_CG),
        .SEG_DP   (SEG_DP),
        .SEG_BIT1 (SEG_BIT1),
        .SEG_BIT2 (SEG_BIT2),
        .SEG_BIT3 (SEG_BIT3),
        .SEG_BIT4 (SEG_BIT4)
    );

    // ================== Í¼ï¿½ï¿½ï¿½ï¿½ï¿½É£ï¿½ï¿½ï¿½Â·ï¿½ï¿½ ==================
    // Ô­ï¿½ï¿½ï¿½Ä³ï¿½ï¿½ï¿½
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

        .modenum        (mode_vga),       // å³ä¸Šè§’æ¨¡å¼è‰²ï¿???
        .ped_phase_step (10'd16),         // è¡Œäººé—´è·
        .peak_main_green_s (peak_main_green_s),
        .peak_side_green_s (peak_side_green_s),
        .pix_data       (pix_pic)
    );

    // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â·ï¿½ï¿½Ä£ï¿½Í£ï¿½ï¿½ï¿½Ì¬ï¿½ï¿½Í¼ï¿½ï¿½ï¿½ï¿½ï¿????? scene_cross.v ï¿½Óµï¿½ï¿½ï¿½ï¿½Ì£ï¿½
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
    .uart_rx_valid (uart_rx_valid_vga),     // ä½¿ç”¨åŒæ­¥åçš„ä¿¡å·
  .uart_rx_byte  (uart_rx_byte_vga),      // ä½¿ç”¨åŒæ­¥åçš„ä¿¡å·
  .pix_cross(pix_cross),
  // ĞÂÔö£º°ÑÊ®×ÖÂ·¿ÚµÄµ¹¼ÆÊ±½Ó³öÀ´
  .phase_left_s_out(phase_left_s_cross),
  .phase_id_out    (phase_id_cross) 
);

    // Ñ¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½×»ï¿½ï¿½ï¿?????
    wire [15:0] pix_data_mux = scene_sel ? pix_cross : pix_pic;

    // ================== VGA ï¿½ï¿½ï¿½Æ£ï¿½×¢ï¿½ï¿½ï¿????? mux ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ø£ï¿????? ==================
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

    // ================== RGB 444 ï¿½ï¿½ï¿????? ==================
    assign vga_r = vga_rgb_int[15:12];
    assign vga_g = vga_rgb_int[10:7];
    assign vga_b = vga_rgb_int[4:1];
endmodule