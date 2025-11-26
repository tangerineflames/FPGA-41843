// scene_cross_right_turn_lanecontrol.v

`timescale 1ns/1ps
module scene_cross #(
    // 锟斤拷幕锟街憋拷锟斤�??/锟斤拷锟斤拷
    parameter H_VALID = 640,
    parameter V_VALID = 480,
    parameter XC = 320,
    parameter YC = 240,
    parameter MAIN_W = 120,   // 锟斤拷锟缴碉拷锟斤拷锟斤�??
    parameter SIDE_W = 120,   // 锟斤拷锟斤拷锟斤拷龋锟斤拷锟斤拷锟斤拷傻锟斤拷锟酵拷锟斤拷纬锟斤拷锟斤拷锟斤拷锟铰凤拷冢锟?
    // 锟竭斤拷/路缘/锟斤拷锟斤拷锟斤�??/锟斤拷锟斤拷
    parameter EDGE_W = 3,
    parameter CURB_R = 40,
    parameter XW = 40,
    parameter STRIPE_W = 6,
    parameter STRIPE_G = 6,
    parameter ZB_OFF   = 35,
    parameter DASH_PERIOD = 32,
    parameter DASH_ON     = 12,
    parameter LANE_T        = 1,
    parameter LANE_DASH_PER = 16,
    parameter LANE_DASH_ON  = 8,
    // 锟斤拷锟斤拷锟竭达拷/锟劫讹拷
    parameter CAR_W    = 10,
    parameter CAR_L    = 18,
    parameter CAR_SPD  = 3,
    parameter GAP_L2R  = 180,
    parameter GAP_R2L  = 180,
    parameter MOVE_DIV_W   = 19,
    // 锟斤拷转锟斤拷锟斤拷锟斤拷锟斤拷锟斤�?
    parameter TURN_N   = 3,   // 锟斤拷转锟斤拷锟斤拷锟斤�??(2~4)
    parameter TURN_DY  = 3,   // 锟斤拷转锟斤拷锟斤拷锟洁（锟斤拷锟斤拷CAR_SPD 锟斤�?? CAR_SPD>>1)
    // 直锟叫筹拷锟斤拷锟狡匡拷锟截ｏ拷1=锟斤拷锟斤拷锟斤�??0=锟截闭ｏ拷锟斤拷锟节诧拷锟皆ｏ拷
    parameter PAUSE_R2L     = 0,  // 锟斤拷锟斤拷(锟斤�??->锟斤�??)锟角凤拷锟斤拷停
    parameter PAUSE_L2R_TOP = 1,  // 锟斤拷色锟斤�??(锟斤�??->锟斤�??)锟较筹拷锟斤�??(YD2C)
    parameter PAUSE_L2R_MID = 0,  // 锟斤拷色锟斤�??(锟斤�??->锟斤�??)锟叫筹拷锟斤�??(YD1C)
    // ==== 锟斤拷锟教灯碉拷锟斤�??(锟斤拷锟斤拷锟斤拷锟斤拷)锟斤拷锟斤拷 ====
    parameter TL_BOX_W   = 20, // 锟斤拷锟斤拷锟斤拷龋锟斤拷锟街憋拷锟斤拷�???
    parameter TL_BOX_H   = 48, // 锟斤拷锟斤拷叨龋�???3锟斤拷锟斤拷锟斤拷锟脚凤拷锟斤拷+锟斤拷锟?
    parameter TL_BOX_PAD = 8,  // 锟斤拷锟斤拷锟斤拷锟铰凤拷锟?/路缘锟侥硷拷�???
    parameter TL_RING_T  = 2,  // 锟斤拷锟斤拷呖锟斤拷锟斤�??
    parameter TL_LIGHT_SIZE = 12,  // 锟斤拷锟斤拷锟狡的尺达拷12x12锟斤拷锟斤拷
    parameter TL_LIGHT_GAP  = 3,   // 锟斤拷锟斤拷锟街拷锟斤拷锟?3锟斤拷锟斤拷
    // 锟斤拷锟教碉拷时锟斤拷锟斤拷锟斤拷锟斤拷锟缴版本锟斤拷锟斤拷锟斤拷锟斤拷锟节碉拷锟斤拷锟斤拷锟狡ｏ拷锟斤拷锟斤拷统一锟斤�??
    parameter TL_T_RED     = 40,   // 锟斤拷锟绞憋拷锟?
    parameter TL_T_YEL     = 12,   // 锟狡碉拷时锟斤拷
    parameter TL_T_GRN_S   = 40,   // 直锟斤拷锟教碉拷时锟斤拷
    parameter TL_T_GRN_L   = 30,   // 锟斤拷转锟教碉拷时锟斤拷
    parameter TL_T_GRN_R   = 20    // 锟斤拷转锟教碉拷时锟斤拷
)(
    input  wire        vga_clk,
    input  wire        sys_rst_n,
    input  wire [9:0]  pix_x,
    input  wire [9:0]  pix_y,
    input  wire [15:0] l2r_head_ext, // 外部输入预留
    input  wire [15:0] r2l_head_ext, // 外部输入预留
    input  wire        rst_key,     // 按键复位：低有效=0
    // 行人控制参数（类似vga_pic.v�??
    input  wire [7:0]  people_count, // 显示的行人数�?? (0..MAX_PED)
    input wire ped_sw_1,    // 1=4人，0=10�??
    input wire car_sw,
    input wire car_sw2,
    input  wire mode_adapt_sw,
    output reg  [15:0] pix_cross,
    
      // 行人数量
  input wire [4:0] PED_N_LR, // 行人数量在LR方向
  input wire [4:0] PED_N_TB, // 行人数量在TB方向
  input  wire        uart_rx_valid,
input  wire [7:0]  uart_rx_byte,

  
  // 传�?�给 traffic_adapt2 的信�??
  output wire [4:0] ped_n_lr_out, 
  output wire [4:0] ped_n_tb_out, 
  output reg ped_has_lr_out, 
  output reg ped_has_tb_out,
  
    //  新增：过街占用信号（用于判断行人是否正在过马路）
  output reg  ped_cross_lr_out,
  output reg  ped_cross_tb_out,
    // �������� traffic_adapt2 �ĵ���ʱ͸����
  output wire [7:0]  phase_left_s_out,
  output wire [2:0]  phase_id_out
);
    // 颜色
    localparam [15:0] BLACK  = 16'h0000;
    localparam [15:0] WHITE  = 16'hFFFF;
    localparam [15:0] YEL    = 16'hFFE0;
    localparam [15:0] RED    = 16'h4A49;  // 深灰�? (原来是红�? F800)
    localparam [15:0] GRN    = 16'h07E0;
    localparam [15:0] BLUE = 16'h001F;

localparam integer MAX_PED = 10;

reg ped_sw_s0, ped_sw_s1;
always @(posedge vga_clk or negedge sys_rst_n) begin
  if(!sys_rst_n) begin ped_sw_s0 <= 1'b0; ped_sw_s1 <= 1'b0; end
  else begin ped_sw_s0 <= ped_sw_1; ped_sw_s1 <= ped_sw_s0; end
end
wire [4:0] PED_N = ped_sw_s1 ? 5'd4 : 5'd10;

reg car_sw_s0, car_sw_s1;
always @(posedge vga_clk or negedge sys_rst_n) begin
  if(!sys_rst_n) begin car_sw_s0 <= 1'b0; car_sw_s1 <= 1'b0; end
  else begin car_sw_s0 <= car_sw; car_sw_s1 <= car_sw_s0; end
end

reg car_sw2_s0, car_sw2_s1;
always @(posedge vga_clk or negedge sys_rst_n) begin
  if(!sys_rst_n) begin
    car_sw2_s0 <= 1'b0;
    car_sw2_s1 <= 1'b0;
  end else begin
    car_sw2_s0 <= car_sw2;
    car_sw2_s1 <= car_sw2_s0;
  end
end

// ===== 同步 mode_adapt_sw �??关信�?? =====
reg mode_adapt_sw_s0, mode_adapt_sw_s1;
always @(posedge vga_clk or negedge sys_rst_n) begin
  if(!sys_rst_n) begin
    mode_adapt_sw_s0 <= 1'b0;
    mode_adapt_sw_s1 <= 1'b0;
  end else begin
    mode_adapt_sw_s0 <= mode_adapt_sw;
    mode_adapt_sw_s1 <= mode_adapt_sw_s0;
  end
end

// ===== ????????????1=??? ROM ??????0=??????????????????/????/???????=====
parameter USE_BG = 1;

// ===== ??????????? bg_layer_ip ????? ROM ????=====
reg [9:0] pix_x_d, pix_y_d;
always @(posedge vga_clk or negedge sys_rst_n) begin
  if(!sys_rst_n) begin pix_x_d <= 10'd0; pix_y_d <= 10'd0; end
  else begin pix_x_d <= pix_x; pix_y_d <= pix_y; end
end

// ***** 统一用当前拍坐标，避免行�??/车道/图形命中错位 *****
wire [9:0] PX = pix_x;
wire [9:0] PY = pix_y;

// ===== ???? IP??????????? pix_x/pix_y ?????????? bg_idx ???????? =====
wire [2:0] bg_idx;
bg_layer_ip #(
  .H(640), .V(480), .BITS(3)
) u_bg (
  .clk   (vga_clk),
  .pix_x (pix_x),
  .pix_y (pix_y),
  .bg_idx(bg_idx)     // ?? PX/PY ???
);

// ===== 3bit ????? ?? RGB565?????????????????=====
function [15:0] pal3_to_rgb565;
  input [2:0] i;
  begin
    case(i)
      3'd0: pal3_to_rgb565 = 16'h0000; // ????/????????/???
      3'd1: pal3_to_rgb565 = 16'hFFFF; // ???????/????/?????
      3'd2: pal3_to_rgb565 = 16'hFFE0; // ?????????????
      3'd3: pal3_to_rgb565 = 16'h7BEF; // ?????????
      3'd4: pal3_to_rgb565 = 16'h39E7; // ????????
      3'd5: pal3_to_rgb565 = 16'h001F; // ???????
      3'd6: pal3_to_rgb565 = 16'hF800; // ???????
      3'd7: pal3_to_rgb565 = 16'h07E0; // ???????
      default: pal3_to_rgb565 = 16'h0000;
    endcase
  end
endfunction
wire [15:0] BG_RGB = pal3_to_rgb565(bg_idx);


// ========= ???????????? PX/PY??=========
function is_rect;
  input integer x0,x1,y0,y1; begin
    is_rect = (PX>=x0 && PX<x1 && PY>=y0 && PY<y1);
  end
endfunction

// ??????????????????????????????
function ring_band_exact;
  input integer cx, cy, r, thk; integer dx,dy,d2,r_in,r_in2,r_out2; begin
    dx=PX-cx; dy=PY-cy; d2=dx*dx+dy*dy;
    r_in=(r>thk)?(r-thk):0; r_in2=r_in*r_in; r_out2=(r+thk)*(r+thk);
    ring_band_exact=(d2>=r_in2)&&(d2<=r_out2);
  end
endfunction

// ???????????????????????????????
function in_circle;
  input integer cx, cy, r; integer dx, dy;
  begin
    dx = PX - cx; dy = PY - cy;
    in_circle = (dx*dx + dy*dy) <= (r*r);
  end
endfunction

function car_rect_xy;
  input integer px,py;
  begin
    car_rect_xy = is_rect(px-(CAR_L>>1), px+(CAR_L>>1), py-(CAR_W>>1), py+(CAR_W>>1));
  end
endfunction

function car_rect_h; input integer px,py; begin
  car_rect_h = is_rect(px-(CAR_L>>1), px+(CAR_L>>1),
                       py-(CAR_W>>1), py+(CAR_W>>1));
end endfunction

function car_rect_v; input integer px,py; begin
  car_rect_v = is_rect(px-(CAR_W>>1), px+(CAR_W>>1),
                       py-(CAR_L>>1), py+(CAR_L>>1));
end endfunction


// ------------ ??????????????????=6 ????? ------------
function in_circle_fast_r6;
  input integer cx, cy;
  integer dx, dy, adx, ady, halfw;
  begin
    dx  = pix_x - cx;
    dy  = pix_y - cy;
    adx = (dx<0)?-dx:dx;
    ady = (dy<0)?-dy:dy;

    // ----------- ??????????????????????? false -----------
    if (adx>6 || ady>6) begin
      in_circle_fast_r6 = 1'b0;
    end else begin
      // ???
      case(ady)
        0: halfw = 6;
        1,2: halfw = 5;
        3,4: halfw = 4;
        5:   halfw = 3;
        default: halfw = 0;
      endcase
      in_circle_fast_r6 = (adx <= halfw);
    end
  end
endfunction


// 锟脚伙拷锟斤拷圆锟轿硷拷猓拷刖? 6 时使锟矫匡拷锟劫版本锟斤拷锟斤拷锟斤拷锟斤拷帽锟阶硷拷姹撅拷�???
function in_circle_fast;
  input integer cx, cy, r;
  begin
    if (r==6) in_circle_fast = in_circle_fast_r6(cx,cy);
    else      in_circle_fast = in_circle(cx,cy,r); // 锟斤拷锟斤拷锟诫�??
  end
endfunction


// =================== 锟斤拷锟斤拷锟斤拷锟叫ｏ拷锟斤拷锟斤拷锟劫撅拷锟诫） ===================
// 锟斤拷锟斤拷 1 锟斤拷示锟斤拷锟斤拷 (PX,PY) 锟斤拷锟斤拷锟斤�?? (cx,cy) 为锟斤拷锟侥★拷锟诫�?? r 锟斤�??"锟斤拷锟斤拷"
// 锟斤拷锟斤拷锟斤拷|PX-cx| + |PY-cy| <= r
function is_diamond_hit;
  input integer cx, cy, r;
  integer dx, dy, adx, ady, sum;
  begin
    dx  = PX - cx;
    dy  = PY - cy;
    adx = (dx < 0) ? -dx : dx;
    ady = (dy < 0) ? -dy : dy;
    sum = adx + ady;
    is_diamond_hit = (sum <= r);
  end
endfunction

    // ========= 路锟节边斤拷锟斤拷锟斤拷 =========
    localparam integer XL = XC - SIDE_W/2;
    localparam integer XR = XC + SIDE_W/2;
    localparam integer YT = YC - MAIN_W/2;
    localparam integer YB = YC + MAIN_W/2;

    wire in_center_box = is_rect(XL, XR, YT, YB);
    wire in_h_road     = is_rect(0, H_VALID, YT, YB);
    wire in_v_road     = is_rect(XL, XR, 0, V_VALID);

    // 锟斤拷锟斤拷锟斤拷锟斤拷
    localparam integer HW_HALF = MAIN_W/2;   // 锟斤拷锟铰凤拷锟?
    localparam integer HW_LW   = HW_HALF/3;  // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    localparam integer VW_HALF = SIDE_W/2;   // 锟斤拷锟斤拷锟斤�??
    localparam integer VH_LW   = VW_HALF/3;  // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷�???

    // 锟斤拷锟缴碉拷锟斤拷锟斤拷锟斤拷锟斤拷Y
    localparam integer YU0C = YT + (HW_LW/2);
    localparam integer YU1C = YT + (HW_LW + HW_LW/2);
    localparam integer YU2C = YC - (HW_LW/2);
    localparam integer YD0C = YB - (HW_LW/2);          // 锟斤拷锟斤拷锟斤拷锟斤拷直锟叫筹拷锟斤拷锟节筹拷锟斤�??
    localparam integer YD1C = YB - (HW_LW + HW_LW/2);  // 锟叫间车锟斤�??
    localparam integer YD2C = YC + (HW_LW/2);          // 锟较凤拷锟斤拷锟斤拷

    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟絏锟斤拷锟斤拷直锟斤拷锟斤拷3锟斤�?? + 水平3锟斤拷锟斤拷
    localparam integer XL_1 = XL + (VH_LW>>1);         // 锟斤拷转 锟斤�??1锟斤拷锟斤拷锟斤�??
    localparam integer XL_2 = XL + VH_LW + (VH_LW>>1);
    localparam integer XL_3 = XC - (VH_LW>>1);
    localparam integer XR_1 = XC + (VH_LW>>1);
    localparam integer XR_2 = XC + VH_LW + (VH_LW>>1);
    localparam integer XR_3 = XR - (VH_LW>>1);

    // ?????????
    wire h_dash_on_core = ((pix_x % DASH_PERIOD) < DASH_ON) && (pix_y >= (YC-1)) && (pix_y <= (YC+1)) && !in_center_box;
    wire v_dash_on_core = ((pix_y % DASH_PERIOD) < DASH_ON) && (pix_x >= (XC-1)) && (pix_x <= (XC+1)) && !in_center_box;

    // ???????
    localparam integer YU1 = YT + HW_LW;
    localparam integer YU2 = YT + 2*HW_LW;
    localparam integer YD1 = YC + HW_LW;
    localparam integer YD2 = YC + 2*HW_LW;

    wire lane_h_u1 = (pix_y >= (YU1-LANE_T) && pix_y <= (YU1+LANE_T)) && in_h_road && ((pix_x % LANE_DASH_PER) < LANE_DASH_ON) && !in_center_box;
    wire lane_h_u2 = (pix_y >= (YU2-LANE_T) && pix_y <= (YU2+LANE_T)) && in_h_road && ((pix_x % LANE_DASH_PER) < LANE_DASH_ON) && !in_center_box;
    wire lane_h_d1 = (pix_y >= (YD1-LANE_T) && pix_y <= (YD1+LANE_T)) && in_h_road && ((pix_x % LANE_DASH_PER) < LANE_DASH_ON) && !in_center_box;
    wire lane_h_d2 = (pix_y >= (YD2-LANE_T) && pix_y <= (YD2+LANE_T)) && in_h_road && ((pix_x % LANE_DASH_PER) < LANE_DASH_ON) && !in_center_box;

    // ????????????
    wire lane_v_lsep1 = (pix_x >= (XL + VH_LW - LANE_T)     && pix_x <= (XL + VH_LW + LANE_T))
                      && in_v_road && ((pix_y % LANE_DASH_PER) < LANE_DASH_ON) && !in_center_box;
    wire lane_v_lsep2 = (pix_x >= (XL + (2*VH_LW) - LANE_T) && pix_x <= (XL + (2*VH_LW) + LANE_T))
                      && in_v_road && ((pix_y % LANE_DASH_PER) < LANE_DASH_ON) && !in_center_box;
    wire lane_v_rsep1 = (pix_x >= (XC + VH_LW - LANE_T)     && pix_x <= (XC + VH_LW + LANE_T))
                      && in_v_road && ((pix_y % LANE_DASH_PER) < LANE_DASH_ON) && !in_center_box;
    wire lane_v_rsep2 = (pix_x >= (XC + (2*VH_LW) - LANE_T) && pix_x <= (XC + (2*VH_LW) + LANE_T))
                      && in_v_road && ((pix_y % LANE_DASH_PER) < LANE_DASH_ON) && !in_center_box;

    // ????????????
    wire zb_top_band   = is_rect(XL, XR, YT - ZB_OFF - XW, YT - ZB_OFF);
    wire zb_bot_band   = is_rect(XL, XR, YB + ZB_OFF,      YB + ZB_OFF + XW);
    wire zb_left_band  = is_rect(XL - ZB_OFF - XW, XL - ZB_OFF, YT, YB);
    wire zb_right_band = is_rect(XR + ZB_OFF,      XR + ZB_OFF + XW, YT, YB);

    wire zb_top   = zb_top_band   && (((pix_x - XL) % (STRIPE_W + STRIPE_G)) < STRIPE_W);
    wire zb_bot   = zb_bot_band   && (((pix_x - XL) % (STRIPE_W + STRIPE_G)) < STRIPE_W);
    wire zb_left  = zb_left_band  && (((pix_y - YT) % (STRIPE_W + STRIPE_G)) < STRIPE_W);
    wire zb_right = zb_right_band && (((pix_y - YT) % (STRIPE_W + STRIPE_G)) < STRIPE_W);

    // ????????????????
    wire edge_top_L_raw = is_rect(0,  XL, YT-EDGE_W, YT);
    wire edge_top_R_raw = is_rect(XR, H_VALID, YT-EDGE_W, YT);
    wire edge_bot_L_raw = is_rect(0,  XL, YB,        YB+EDGE_W);
    wire edge_bot_R_raw = is_rect(XR, H_VALID, YB,  YB+EDGE_W);

    wire edge_left_T_raw  = is_rect(XL-EDGE_W, XL, 0,   YT);
    wire edge_left_B_raw  = is_rect(XL-EDGE_W, XL, YB,  V_VALID);
    wire edge_right_T_raw = is_rect(XR, XR+EDGE_W, 0,   YT);
    wire edge_right_B_raw = is_rect(XR, XR+EDGE_W, YB,  V_VALID);

    // ????????
    localparam integer C_TL_X = XL - CURB_R, C_TL_Y = YT - CURB_R;
    localparam integer C_TR_X = XR + CURB_R, C_TR_Y = YT - CURB_R;
    localparam integer C_BL_X = XL - CURB_R, C_BL_Y = YB + CURB_R;
    localparam integer C_BR_X = XR + CURB_R, C_BR_Y = YB + CURB_R;

    wire cut_top_tl   = is_rect(XL-CURB_R, XL,         YT-EDGE_W, YT);
    wire cut_top_tr   = is_rect(XR,         XR+CURB_R, YT-EDGE_W, YT);
    wire cut_bot_bl   = is_rect(XL-CURB_R, XL,         YB,        YB+EDGE_W);
    wire cut_bot_br   = is_rect(XR,         XR+CURB_R, YB,        YB+EDGE_W);
    wire cut_left_tl  = is_rect(XL-EDGE_W,  XL,        YT-CURB_R, YT);
    wire cut_left_bl  = is_rect(XL-EDGE_W,  XL,        YB,        YB+CURB_R);
    wire cut_right_tr = is_rect(XR,         XR+EDGE_W, YT-CURB_R, YT);
    wire cut_right_br = is_rect(XR,         XR+EDGE_W, YB,        YB+CURB_R);

    wire edge_top_L   = edge_top_L_raw   && !cut_top_tl;
    wire edge_top_R   = edge_top_R_raw   && !cut_top_tr;
    wire edge_bot_L   = edge_bot_L_raw   && !cut_bot_bl;
    wire edge_bot_R   = edge_bot_R_raw   && !cut_bot_br;
    wire edge_left_T  = edge_left_T_raw  && !cut_left_tl;
    wire edge_left_B  = edge_left_B_raw  && !cut_left_bl;
    wire edge_right_T = edge_right_T_raw && !cut_right_tr;
    wire edge_right_B = edge_right_B_raw && !cut_right_br;

    wire box_tl = (pix_x >= XL-CURB_R && pix_x <  XL && pix_y >= YT-CURB_R && pix_y <  YT);
    wire box_tr = (pix_x >  XR        && pix_x <= XR+CURB_R && pix_y >= YT-CURB_R && pix_y <  YT);
    wire box_bl = (pix_x >= XL-CURB_R && pix_x <  XL && pix_y >  YB        && pix_y <= YB+CURB_R);
    wire box_br = (pix_x >  XR        && pix_x <= XR+CURB_R && pix_y >  YB        && pix_y <= YB+CURB_R);

    wire c_tl_arc = box_tl && ring_band_exact(C_TL_X, C_TL_Y, CURB_R, EDGE_W);
    wire c_tr_arc = box_tr && ring_band_exact(C_TR_X, C_TR_Y, CURB_R, EDGE_W);
    wire c_bl_arc = box_bl && ring_band_exact(C_BL_X, C_BL_Y, CURB_R, EDGE_W);
    wire c_br_arc = box_br && ring_band_exact(C_BR_X, C_BR_Y, CURB_R, EDGE_W);
    
    // ================== Zebra Traffic Lights Final Positions ==================
// TOP (X+40 done)
localparam integer ZT_CX = 400;
localparam integer ZT_G_CY = 55;
localparam integer ZT_Y_CY = 73;
localparam integer ZT_R_CY = 91;

// BOTTOM (X+40 done)
localparam integer ZB_CX = 400;
localparam integer ZB_G_CY = 389;
localparam integer ZB_Y_CY = 407;
localparam integer ZB_R_CY = 425;

// LEFT  (Y-40 done)
localparam integer ZL_CY = 160;
localparam integer ZL_G_CX = 135;
localparam integer ZL_Y_CX = 153;
localparam integer ZL_R_CX = 171;

// RIGHT (Y-40 done)
localparam integer ZR_CY = 160;
localparam integer ZR_G_CX = 469;
localparam integer ZR_Y_CX = 487;
localparam integer ZR_R_CX = 505;

// 锟斤拷锟截猴拷锟斤拷锟斤拷锟诫车锟矫碉拷�??锟铰ｏ拷--锟斤拷锟斤拷锟斤拷锟斤拷锟窖讹拷锟藉，锟斤拷锟斤拷锟截革拷锟斤拷锟斤�??
function [9:0] wrap480; input [10:0] v;
begin
  wrap480 = (v>=11'd480) ? (v-11'd480) : v[9:0];
end
endfunction

localparam [1:0] C_RED = 2'd0, C_YEL = 2'd1, C_GRN = 2'd2;
 // 计算主路LR的灯色（直行/左转/右转�??
wire [1:0] col_LR_S, col_LR_L, col_LR_R;
// 计算次路TB的灯色（直行/左转/右转），与LR相位�??90�??
wire [1:0] col_TB_S, col_TB_L, col_TB_R;
wire [1:0] car_bias;
// 绿灯时长信号（在 traffic_adapt2 实例化之前声明）
wire [7:0] green_sec_LR_S, green_sec_LR_L, green_sec_LR_R;
wire [7:0] green_sec_TB_S, green_sec_TB_L, green_sec_TB_R;

// car_sw_s1 = TB ���򳵶�
// car_sw2_s1 = LR ���򳵶�
// Լ�����䣺01 = LR more, 10 = TB more
assign car_bias =
    (car_sw_s1 & car_sw2_s1) ? 2'b00 :   // ������ 1 -> ����
    (car_sw2_s1)             ? 2'b01 :   // LR more���� car_sw2_s1 ���ƣ�
    (car_sw_s1)              ? 2'b10 :   // TB more���� car_sw_s1 ���ƣ�
                               2'b00 ;   // ������ -> ����



    // ========= ??????? =========
    reg [MOVE_DIV_W-1:0] div;  wire move_tick = (div=={MOVE_DIV_W{1'b0}});
    always @(posedge vga_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) div <= {MOVE_DIV_W{1'b0}};
        else            div <= div + 1'b1;
    end
    
    // ???????move_tick ???????1Hz
    // move_tick ?? 47.7Hz??????VGA???25MHz??MOVE_DIV_W=19??
    // ?????????1Hz ?? ?48??move_tick = 1??
    localparam integer SEC_DIV = 12;  // ?????48??move_tick ?? 1??
    
    reg [7:0] sec_div_cnt;
    wire sec_tick = move_tick && (sec_div_cnt == 0);
    
    always @(posedge vga_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            sec_div_cnt <= 8'd0;
        end
        else if (move_tick) begin
            if (sec_div_cnt < SEC_DIV-1)
                sec_div_cnt <= sec_div_cnt + 1'b1;
            else
                sec_div_cnt <= 8'd0;
        end
    end

    // 倒计时功能已删除以节省LUT
// ===================== 行人（靠路外排队，分批过街，整路�??3�??/批） =====================
// 依赖：XL,XR,YT,YB,XW,ZB_OFF, col_LR_S/col_TB_S/C_GRN, vga_clk, sys_rst_n, pix_x,pix_y, sec_tick
// 约定：外部提�?? MAX_PED、PED_N（绘制上�?? <= MAX_PED）�?�PED_N_LR、PED_N_TB
// 端口类型务必�?? output wire [4:0] ped_n_lr_out, ped_n_tb_out;  output reg ped_has_lr_out, ped_has_tb_out;

// ---------- 参数 ----------
localparam integer PED_D        = 7;
localparam integer PED_R        = PED_D;
localparam integer PED_SPD      = 1;            // 1 px / tick
localparam integer PED_TICK_DIV = 624_999;      // 25MHz -> 40Hz
localparam [9:0]  PED_STEP      = 10'd26;       // 队伍间距
localparam integer MARGIN       = PED_D;        // 等待线外�??
localparam [9:0]  WRAP_GAP      = 10'd12;       // 屏外微偏移（封存时用�??

// ---------- 斑马�??/等待�?? ----------
localparam integer PED_TOP_Y = YT - ZB_OFF - (XW>>1);
localparam integer PED_BOT_Y = YB + ZB_OFF + (XW>>1);
localparam integer PED_L_X   = XL - ZB_OFF - (XW>>1);
localparam integer PED_R_X   = XR + ZB_OFF + (XW>>1);

localparam [9:0] WAIT_L_X = (XL > (PED_D + MARGIN))            ? (XL - (PED_D + MARGIN))          : 10'd0;
localparam [9:0] WAIT_R_X = (XR + (PED_D + MARGIN) < 10'd639)  ? (XR + (PED_D + MARGIN))          : 10'd639;
localparam [9:0] WAIT_T_Y = (YT > (PED_D + MARGIN))            ? (YT - (PED_D + MARGIN))          : 10'd0;
localparam [9:0] WAIT_B_Y = (YB + (PED_D + MARGIN) < 10'd479)  ? (YB + (PED_D + MARGIN))          : 10'd479;

// ---------- 信号与边�?? ----------
wire lr_green = (col_LR_S == C_GRN);
wire tb_green = (col_TB_S == C_GRN);

reg lr_green_q, tb_green_q;
always @(posedge vga_clk or negedge sys_rst_n) begin
  if(!sys_rst_n) begin lr_green_q<=1'b0; tb_green_q<=1'b0; end
  else begin        lr_green_q<=lr_green; tb_green_q<=tb_green; end
end
wire lr_green_rise =  lr_green & ~lr_green_q;
wire tb_green_rise =  tb_green & ~tb_green_q;
wire lr_green_fall = ~lr_green &  lr_green_q;   // 用于"复活"
wire tb_green_fall = ~tb_green &  tb_green_q;

// ---------- �??次绿灯只放行�??批：令牌 ----------
reg lr_token, tb_token;
reg lr_batch_started, tb_batch_started;

// ---------- 方向/进入锁存 ----------
reg signed [1:0] dir_top, dir_bot;    // +1：向右；-1：向�??
reg signed [1:0] dir_left, dir_right; // +1：向下；-1：向�??
reg entered_top, entered_bot, entered_left, entered_right;

initial begin
  dir_top   =  1;  // 顶部：左->�??
  dir_bot   = -1;  // 底部：右->�??
  dir_left  =  1;  // 左侧：上->�??
  dir_right = -1;  // 右侧：下->�??
end


// ---------- 40Hz 行人步进时钟 ----------
reg [31:0] ped_div;  wire ped_tick = (ped_div == PED_TICK_DIV);
always @(posedge vga_clk or negedge sys_rst_n) begin
  if(!sys_rst_n) ped_div <= 0;
  else           ped_div <= ped_tick ? 0 : ped_div + 1'b1;
end

// ---------- "封存/复活" 管理 ----------
reg top_wrapped,  bot_wrapped;
reg left_wrapped, right_wrapped;
reg respawn_arm_lr, respawn_arm_tb;   // 有侧封存后举手，等待允许复活

// 允许进入（封存期间禁止）
wire ALLOW_ENTER_LR = lr_green && !top_wrapped && !bot_wrapped
                    && (lr_token || entered_top || entered_bot);
wire ALLOW_ENTER_TB = tb_green && !left_wrapped && !right_wrapped
                    && (tb_token || entered_left || entered_right);
// ---------- 关键位置 ----------
reg [9:0] ped_x_top, ped_x_bot;
reg [9:0] ped_y_left, ped_y_right;

localparam integer ENTER_GUARD = PED_D;
localparam integer EXIT_GUARD  = PED_D;

// ===== TB direction now uses direct entry logic like LR (no will_enter signals) =====
// ===== ͳһ�� LR/TB ���ƹ��� =====
always @(posedge vga_clk or negedge sys_rst_n) begin
  if(!sys_rst_n) begin
    lr_token         <= 1'b0;
    tb_token         <= 1'b0;
    lr_batch_started <= 1'b0;
    tb_batch_started <= 1'b0;
  end else begin
    // -- LR���̵����ط����ƣ���һ�����˽�������ѣ��������
    if (lr_green_rise) begin
      lr_token         <= 1'b1;
      lr_batch_started <= 1'b0;
    end
    if (!lr_green)
      lr_token <= 1'b0;
    if (lr_token && (entered_top || entered_bot)) begin
      lr_token         <= 1'b0;   // ����
      lr_batch_started <= 1'b1;
    end

    // -- TB direction token control, identical to LR
    if (tb_green_rise) begin
      tb_token         <= 1'b1;
      tb_batch_started <= 1'b0;
    end
    if (!tb_green)
      tb_token <= 1'b0;
    if (tb_token && (entered_left || entered_right)) begin
      tb_token         <= 1'b0;
      tb_batch_started <= 1'b1;
    end
  end
end


// =====================================================
// 统一�?? respawn_req_* 生成（唯�??驱动者）
// =====================================================
reg [7:0] wrap_secs_lr, wrap_secs_tb;
always @(posedge vga_clk or negedge sys_rst_n) begin
  if (!sys_rst_n) begin wrap_secs_lr<=8'd0; wrap_secs_tb<=8'd0; end
  else if (sec_tick) begin
    wrap_secs_lr <= (top_wrapped  || bot_wrapped ) ? (wrap_secs_lr + 1'b1) : 8'd0;
    wrap_secs_tb <= (left_wrapped || right_wrapped) ? (wrap_secs_tb + 1'b1) : 8'd0;
  end
end
localparam integer WRAP_FORCE_RESPAWN_S = 0; // 0=关闭绿↑强制复活

// 仅此处写 respawn_req_*
reg respawn_req_lr, respawn_req_tb;

// �??"执行�??"传回的一拍完成脉冲（在执行块里产生成寄存器，然后这里只读�??
wire respawn_done_lr_p, respawn_done_tb_p;

always @(posedge vga_clk or negedge sys_rst_n) begin
  if (!sys_rst_n) begin
    respawn_req_lr <= 1'b0;
    respawn_req_tb <= 1'b0;
  end else begin
if (respawn_done_lr_p) begin
  respawn_req_lr <= 1'b0;
end else if (respawn_arm_lr) begin
  if (lr_green_fall ||
      (lr_green_rise && (wrap_secs_lr >= WRAP_FORCE_RESPAWN_S)) ||
      (lr_green      && (wrap_secs_lr >= WRAP_FORCE_RESPAWN_S))) begin
    respawn_req_lr <= 1'b1;
  end
end

if (respawn_done_tb_p) begin
  respawn_req_tb <= 1'b0;
end else if (respawn_arm_tb) begin
  // �� LR һ�����ȿ��̵��½��أ�Ҳ��"�Ѿ��Ǻ�� + wrap ��һ��ʱ��"
  if ( tb_green_fall ||
      (!tb_green && (wrap_secs_tb >= WRAP_FORCE_RESPAWN_S)) ) begin
    respawn_req_tb <= 1'b1;
  end
end

  end
end

// ====================== 水平（LR）运动与复活执行 ======================
reg respawn_done_lr_z;  assign respawn_done_lr_p = respawn_done_lr_z; // 只在本块里写 _z
always @(posedge vga_clk or negedge sys_rst_n) begin
  if(!sys_rst_n) begin
    ped_x_top<=WAIT_L_X; ped_x_bot<=WAIT_R_X;
    entered_top<=1'b0; entered_bot<=1'b0;
    top_wrapped<=1'b0; bot_wrapped<=1'b0;
    respawn_arm_lr<=1'b0;
    respawn_done_lr_z<=1'b0;
  end else if (ped_tick) begin
    respawn_done_lr_z <= 1'b0; // 默认拉低

    // 进入�??�?? - 只在绿灯时允许进�??
    if(!entered_top && !top_wrapped && lr_green) begin
      if (dir_top==1) begin
        if((ped_x_top < (XL - ENTER_GUARD)) && (ped_x_top + PED_SPD >= (XL - ENTER_GUARD))) entered_top <= 1'b1;
      end else begin
        if((ped_x_top > (XR + ENTER_GUARD)) && (ped_x_top - PED_SPD <= (XR + ENTER_GUARD))) entered_top <= 1'b1;
      end
    end
    if(!entered_bot && !bot_wrapped && lr_green) begin
      if (dir_bot==-1) begin
        if((ped_x_bot > (XR + ENTER_GUARD)) && (ped_x_bot - PED_SPD <= (XR + ENTER_GUARD))) entered_bot <= 1'b1;
      end else begin
        if((ped_x_bot < (XL - ENTER_GUARD)) && (ped_x_bot + PED_SPD >= (XL - ENTER_GUARD))) entered_bot <= 1'b1;
      end
    end

    // 移动（未封存侧）
    if (!top_wrapped) begin
      if (lr_green) begin
        if (ALLOW_ENTER_LR || entered_top) ped_x_top <= ped_x_top + (dir_top==1 ? PED_SPD : -PED_SPD);
      end else begin
        if (entered_top) ped_x_top <= ped_x_top + (dir_top==1 ? PED_SPD : -PED_SPD);
        else begin
          if (dir_top==1) begin
            if(ped_x_top < WAIT_L_X) ped_x_top <= (ped_x_top + PED_SPD <= WAIT_L_X)?(ped_x_top + PED_SPD):WAIT_L_X;
          end else begin
            if(ped_x_top > WAIT_R_X) ped_x_top <= (ped_x_top > WAIT_R_X + PED_SPD)?(ped_x_top - PED_SPD):WAIT_R_X;
          end
        end
      end
    end else begin
      ped_x_top <= (dir_top==1) ? (10'd639 - WRAP_GAP) : (10'd0 + WRAP_GAP); // 封存：屏外固�??
    end

    if (!bot_wrapped) begin
      if (lr_green) begin
        if (ALLOW_ENTER_LR || entered_bot) ped_x_bot <= ped_x_bot + (dir_bot==1 ? PED_SPD : -PED_SPD);
      end else begin
        if (entered_bot) ped_x_bot <= ped_x_bot + (dir_bot==1 ? PED_SPD : -PED_SPD);
        else begin
          if (dir_bot==-1) begin
            if(ped_x_bot > WAIT_R_X) ped_x_bot <= (ped_x_bot > WAIT_R_X + PED_SPD)?(ped_x_bot - PED_SPD):WAIT_R_X;
          end else begin
            if(ped_x_bot < WAIT_L_X) ped_x_bot <= (ped_x_bot + PED_SPD <= WAIT_L_X)?(ped_x_bot + PED_SPD):WAIT_L_X;
          end
        end
      end
    end else begin
      ped_x_bot <= (dir_bot==1) ? (10'd639 - WRAP_GAP) : (10'd0 + WRAP_GAP);
    end

    // 队尾越界 �?? 封存 + 举手
    if (entered_top) begin
      if (dir_top==1) begin
        if (($signed({1'b0,ped_x_top}) - $signed((PED_N-1)*PED_STEP)) >= $signed(XR + EXIT_GUARD)) begin
          entered_top<=1'b0; top_wrapped<=1'b1; respawn_arm_lr<=1'b1;
        end
      end else begin
        if (($signed({1'b0,ped_x_top}) + $signed((PED_N-1)*PED_STEP)) <= $signed(XL - EXIT_GUARD)) begin
          entered_top<=1'b0; top_wrapped<=1'b1; respawn_arm_lr<=1'b1;
        end
      end
    end
    if (entered_bot) begin
      if (dir_bot==-1) begin
        if (($signed({1'b0,ped_x_bot}) + $signed((PED_N-1)*PED_STEP)) <= $signed(XL - EXIT_GUARD)) begin
          entered_bot<=1'b0; bot_wrapped<=1'b1; respawn_arm_lr<=1'b1;
        end
      end else begin
        if (($signed({1'b0,ped_x_bot}) - $signed((PED_N-1)*PED_STEP)) >= $signed(XR + EXIT_GUARD)) begin
          entered_bot<=1'b0; bot_wrapped<=1'b1; respawn_arm_lr<=1'b1;
        end
      end
    end

    // 复活执行（只�?? respawn_req_lr；这里不修改 req 本体�??
    if (respawn_req_lr) begin
      // 回到"起始侧等待线"
      if (top_wrapped) ped_x_top <= (dir_top==1) ? WAIT_L_X : WAIT_R_X;
      if (bot_wrapped) ped_x_bot <= (dir_bot==1) ? WAIT_L_X : WAIT_R_X;
      top_wrapped<=1'b0; bot_wrapped<=1'b0; respawn_arm_lr<=1'b0;
      respawn_done_lr_z <= 1'b1;  // 单拍完成脉冲
    end
  end
end



// ====================== TB �����˶�/����/���磨�ָ��棬�������� tb_token�� ======================
localparam integer CLEARANCE_Y = EXIT_GUARD + PED_R;

reg respawn_done_tb_z;
assign respawn_done_tb_p = respawn_done_tb_z;

always @(posedge vga_clk or negedge sys_rst_n) begin
  if (!sys_rst_n) begin
    // ��ֵ�ص��ȴ���
    ped_y_left     <= WAIT_T_Y;
    ped_y_right    <= WAIT_B_Y;
    entered_left   <= 1'b0;
    entered_right  <= 1'b0;
    left_wrapped   <= 1'b0;
    right_wrapped  <= 1'b0;
    respawn_arm_tb <= 1'b0;
    respawn_done_tb_z <= 1'b0;
  end else if (ped_tick) begin
    respawn_done_tb_z <= 1'b0; // Ĭ������

    // -- ����������λ�����������ѽ��tb_token ֻ��"���ƹ�����"��ģ�?
    // ���?
    if(!entered_left && !left_wrapped && tb_green) begin
      if (dir_left== 1) begin
        if((ped_y_left < (YT - ENTER_GUARD)) && (ped_y_left + PED_SPD >= (YT - ENTER_GUARD))) entered_left <= 1'b1;
      end else begin
        if((ped_y_left > (YB + ENTER_GUARD)) && (ped_y_left - PED_SPD <= (YB + ENTER_GUARD))) entered_left <= 1'b1;
      end
    end
    // �Ҳ�
    if(!entered_right && !right_wrapped && tb_green) begin
      if (dir_right==-1) begin
        if((ped_y_right > (YB + ENTER_GUARD)) && (ped_y_right - PED_SPD <= (YB + ENTER_GUARD))) entered_right <= 1'b1;
      end else begin
        if((ped_y_right < (YT - ENTER_GUARD)) && (ped_y_right + PED_SPD >= (YT - ENTER_GUARD))) entered_right <= 1'b1;
      end
    end

// ================= ���˳��� + wrap���ĳɺ� LR ͬ��񣬵����������ӳ��ȣ� =================
// ˼·��ֻҪ"��ͷ"Խ�����ڱ����ߣ�����Ϊ��һ���Ѿ�ͨ�������� wrap��

// ��ࣺdir_left = +1 ��ʾ �� -> �£��������õķ���
if (entered_left) begin
  if (dir_left ==  1) begin
    // ��ͷ ped_y_left �����ߣ����� YB + EXIT_GUARD + CLEARANCE_Y ��������
    if ($signed({1'b0,ped_y_left}) >= $signed(YB + EXIT_GUARD + CLEARANCE_Y)) begin
      entered_left   <= 1'b0;
      left_wrapped   <= 1'b1;
      respawn_arm_tb <= 1'b1;
    end
  end else begin
    // Ԥ��������Ժ���ĳ��� -> �ϣ������Ǿ�������
    if ($signed({1'b0,ped_y_left}) <= $signed(YT - EXIT_GUARD - CLEARANCE_Y)) begin
      entered_left   <= 1'b0;
      left_wrapped   <= 1'b1;
      respawn_arm_tb <= 1'b1;
    end
  end
end

// �Ҳࣺdir_right = -1 ��ʾ �� -> �ϣ������ڵķ���
if (entered_right) begin
  if (dir_right == -1) begin
    // ��ͷ ped_y_right �����ߣ����� YT - EXIT_GUARD - CLEARANCE_Y ��������
    if ($signed({1'b0,ped_y_right}) <= $signed(YT - EXIT_GUARD - CLEARANCE_Y)) begin
      entered_right  <= 1'b0;
      right_wrapped  <= 1'b1;
      respawn_arm_tb <= 1'b1;
    end
  end else begin
    // Ԥ������������� -> �� / �� -> �£��Գ�д��
    if ($signed({1'b0,ped_y_right}) >= $signed(YB + EXIT_GUARD + CLEARANCE_Y)) begin
      entered_right  <= 1'b0;
      right_wrapped  <= 1'b1;
      respawn_arm_tb <= 1'b1;
    end
  end
end

    // ================= λ�� =================
    // ���?
    if (!left_wrapped) begin
      if (tb_green) begin
        // �̵��ڣ�δ�볡ʱ������ ALLOW_ENTER_TB�����볡�����ǰ��?
        if (ALLOW_ENTER_TB || entered_left)
          ped_y_left <= ped_y_left + (dir_left==1 ? PED_SPD : -PED_SPD);
      end else begin
        // ����ڣ��볡�ļ����ߣ�δ�볡�Ŀ����ȴ���?
        if (entered_left) begin
          ped_y_left <= ped_y_left + (dir_left==1 ? PED_SPD : -PED_SPD);
        end else begin
          if (dir_left==1) begin
            // ��->�£��� WAIT_T_Y
            if (ped_y_left < WAIT_T_Y)
              ped_y_left <= (ped_y_left + PED_SPD <= WAIT_T_Y) ? (ped_y_left + PED_SPD) : WAIT_T_Y;
          end else begin
            // ��->�ϣ��� WAIT_B_Y
            if (ped_y_left > WAIT_B_Y)
              ped_y_left <= (ped_y_left > WAIT_B_Y + PED_SPD) ? (ped_y_left - PED_SPD) : WAIT_B_Y;
          end
        end
      end
    end else begin
      // wrapped���ŵ����⻺��λ
      ped_y_left <= (dir_left==1) ? (10'd479 - WRAP_GAP) : (10'd0 + WRAP_GAP);
    end

    // �Ҳ�
    if (!right_wrapped) begin
      if (tb_green) begin
        if (ALLOW_ENTER_TB || entered_right)
          ped_y_right <= ped_y_right + (dir_right==1 ? PED_SPD : -PED_SPD);
      end else begin
        if (entered_right) begin
          ped_y_right <= ped_y_right + (dir_right==1 ? PED_SPD : -PED_SPD);
        end else begin
          if (dir_right==-1) begin
            // ��->�ϣ��� WAIT_B_Y
            if (ped_y_right > WAIT_B_Y)
              ped_y_right <= (ped_y_right > WAIT_B_Y + PED_SPD) ? (ped_y_right - PED_SPD) : WAIT_B_Y;
          end else begin
            // ��->�£��� WAIT_T_Y
            if (ped_y_right < WAIT_T_Y)
              ped_y_right <= (ped_y_right + PED_SPD <= WAIT_T_Y) ? (ped_y_right + PED_SPD) : WAIT_T_Y;
          end
        end
      end
    end else begin
      ped_y_right <= (dir_right==1) ? (10'd479 - WRAP_GAP) : (10'd0 + WRAP_GAP);
    end

    // ================= ���λ��£�����Ӧ respawn_req_tb�� =================
    if (respawn_req_tb) begin
      // �ص�"��ʼ�ȴ���"
      if (left_wrapped)  ped_y_left  <= (dir_left==1)  ? WAIT_T_Y : WAIT_B_Y;
      if (right_wrapped) ped_y_right <= (dir_right==1) ? WAIT_T_Y : WAIT_B_Y;

      left_wrapped      <= 1'b0;
      right_wrapped     <= 1'b0;
      respawn_arm_tb    <= 1'b0;
      respawn_done_tb_z <= 1'b1; // �����������?
    end
  end
end

// ================= 命中�??测：菱形 =================
function [0:0] diamond_hit_xy;
  input [9:0] cx, cy; reg [10:0] dx, dy, md;
  begin
    dx = (pix_x > cx) ? (pix_x - cx) : (cx - pix_x);
    dy = (pix_y > cy) ? (pix_y - cy) : (cy - pix_y);
    md = dx + dy;
    diamond_hit_xy = (md <= PED_R);
  end
endfunction

function hit_top_idx; input integer idx; reg [11:0] xt;
  begin
    if (top_wrapped) hit_top_idx = 1'b0;
    else begin
      xt = {2'b00,ped_x_top} - (idx * PED_STEP);
      hit_top_idx = (xt[11]==1'b0 && xt<12'd640) ? diamond_hit_xy(xt[9:0], PED_TOP_Y[9:0]) : 1'b0;
    end
  end
endfunction

function hit_bot_idx; input integer idx; reg [11:0] xt;
  begin
    if (bot_wrapped) hit_bot_idx = 1'b0;
    else begin
      xt = {2'b00,ped_x_bot} + (idx * PED_STEP);
      hit_bot_idx = (xt<12'd640) ? diamond_hit_xy(xt[9:0], PED_BOT_Y[9:0]) : 1'b0;
    end
  end
endfunction

function hit_left_idx;  input integer idx; reg [11:0] yt;
  begin
    if (left_wrapped) hit_left_idx = 1'b0;
    else begin
      yt = {2'b00,ped_y_left} - (idx * PED_STEP);
      hit_left_idx = (yt[11]==1'b0 && yt<12'd480) ? diamond_hit_xy(PED_L_X[9:0], yt[9:0]) : 1'b0;
    end
  end
endfunction

function hit_right_idx; input integer idx; reg [11:0] yt;
  begin
    if (right_wrapped) hit_right_idx = 1'b0;
    else begin
      yt = {2'b00,ped_y_right} + (idx * PED_STEP);
      hit_right_idx = (yt<12'd480) ? diamond_hit_xy(PED_R_X[9:0], yt[9:0]) : 1'b0;
    end
  end
endfunction

// ---------- 聚合 ----------
reg ped_hit_r_h, ped_hit_r_v; integer i;
always @* begin
  ped_hit_r_h = 1'b0;
  for(i=0;i<MAX_PED;i=i+1) begin
    if (i < PED_N) begin
      if(hit_top_idx(i)) ped_hit_r_h = 1'b1;
      if(hit_bot_idx(i)) ped_hit_r_h = 1'b1;
    end
  end
  ped_hit_r_v = 1'b0;
  for(i=0;i<MAX_PED;i=i+1) begin
    if (i < PED_N) begin
      if(hit_left_idx(i))  ped_hit_r_v = 1'b1;
      if(hit_right_idx(i)) ped_hit_r_v = 1'b1;
    end
  end
end

wire ped_hit = ped_hit_r_h | ped_hit_r_v;  // 若你外面有�?�像素混色，这里接入

// ===================== 占用输出（只算斑马线内，等待线不算） =====================
// ֻͳ�ư������ڲ��Ƿ����ˣ��ȴ��߲��㣩
wire ped_active_lr = (entered_top  || entered_bot );
wire ped_active_tb = (entered_left || entered_right);

// �������ռ�ã����پ�? sec_tick ����/���֣����� 1~2 ���ӳ�
// ע��: ֻͳ�����ѽ��밮������������ȴ������볡ʱ��ת���Է�������?
always @(posedge vga_clk or negedge sys_rst_n) begin
  if (!sys_rst_n) begin
    ped_has_lr_out <= 1'b0;
    ped_has_tb_out <= 1'b0;
  end else begin
    ped_has_lr_out <= ped_active_lr;
    ped_has_tb_out <= ped_active_tb;
  end
end

// --- 过街占用信号输出（表示行人是否正在过马路�?? ---
always @(posedge vga_clk or negedge sys_rst_n) begin
  if (!sys_rst_n) begin
    ped_cross_lr_out <= 1'b0;
    ped_cross_tb_out <= 1'b0;
  end else begin
    ped_cross_lr_out <= ped_active_lr;
    ped_cross_tb_out <= ped_active_tb;
  end
end

// --- 人数输出（保持与原接口一致；端口�?? output wire�?? ---
// 使用内部 PED_N（由 ped_sw_1 控制）来显示，确保拨码开关能控制显示的人�??
// LR方向有top和bot两条路，每条路PED_N人，�??以LR方向总共 = PED_N * 2
// TB方向有left和right两条路，每条路PED_N人，�??以TB方向总共 = PED_N * 2
// 总人�?? = (PED_N * 2) + (PED_N * 2) = PED_N * 4 = 16 �?? 40
assign ped_n_lr_out = PED_N * 2;  // LR方向总人数：1=8�??(4*2)�??0=20�??(10*2)
assign ped_n_tb_out = PED_N * 2;  // TB方向总人数：1=8�??(4*2)�??0=20�??(10*2)

// --- 车辆参数定义（必须在wire和数组声明之前） ---
// LR方向车辆参数
localparam integer N_ST = 8;
localparam integer N_RT = 6; 
localparam integer N_LT = 4;
localparam integer N_ST_R = 8;
localparam integer N_RT_R = 6;
localparam integer N_LT_R = 4;

// TB方向车辆参数
localparam integer N_TSD = 10;
localparam integer N_TL = 4;
localparam integer N_TR_ALWAYS   = 6;
localparam integer N_BS_ST = 10;
localparam integer N_BR = 6;
localparam integer N_BL = 4;

// --- 车辆参数和数组定义（必须在统计代码之前） ---
// 0=保持现状�??1=�??有流量强制为2（用同步后的 car_sw_s1�??
wire [4:0] NUM_ST    = car_sw_s1 ? 5'd2 : N_ST;     // L→R 直行
wire [2:0] NUM_RT    = car_sw_s1 ? 3'd2 : N_RT;     // L→R 右转
wire [2:0] NUM_LT    = car_sw_s1 ? 3'd2 : N_LT;     // L→R 左转
wire [4:0] NUM_ST_R  = car_sw_s1 ? 5'd2 : N_ST_R;   // R→L 直行
wire [2:0] NUM_RT_R  = car_sw_s1 ? 3'd2 : N_RT_R;   // R→L 右转
wire [2:0] NUM_LT_R  = car_sw_s1 ? 3'd2 : N_LT_R;   // R→L 左转

// 竖直方向 - 使用 car_sw2_s1 强制 =2
wire [4:0] NUM_TSD    = car_sw2_s1 ? 5'd2 : N_TSD;
wire [2:0] NUM_TL     = car_sw2_s1 ? 3'd2 : N_TL;
wire [2:0] NUM_TR     = car_sw2_s1 ? 3'd2 : N_TR_ALWAYS;
wire [4:0] NUM_BS_ST  = car_sw2_s1 ? 5'd2 : N_BS_ST;
wire [2:0] NUM_BR     = car_sw2_s1 ? 3'd2 : N_BR;
wire [2:0] NUM_BL     = car_sw2_s1 ? 3'd2 : N_BL;

// 车辆状�?�数组声明（用于统计�??
reg        st_active [0:N_ST-1];
reg        rt_active [0:N_RT-1];
reg        lt_active [0:N_LT-1];
reg        stR_active [0:N_ST_R-1];
reg        rt2_active [0:N_RT_R-1];
reg        lt2_active [0:N_LT_R-1];
reg        tsd_active [0:N_TSD-1];
reg        tl_active [0:N_TL-1];
reg        tr_active [0:N_TR_ALWAYS-1];
reg        bs_st_active [0:N_BS_ST-1];
reg        br_active [0:N_BR-1];
reg        bl_active [0:N_BL-1];

// --- 车辆数量统计（LR和TB方向�?? ---
// LR方向：st_active, rt_active, lt_active, stR_active, rt2_active, lt2_active
// TB方向：tsd_active, tl_active, tr_active, bs_st_active, br_active, bl_active
integer car_lr_count, car_tb_count;
integer car_i;
always @(*) begin
    car_lr_count = 0;
    car_tb_count = 0;
    
    // LR方向车辆统计
    for (car_i = 0; car_i < N_ST; car_i = car_i + 1)
        if (car_i < NUM_ST && st_active[car_i]) car_lr_count = car_lr_count + 1;
    for (car_i = 0; car_i < N_RT; car_i = car_i + 1)
        if (car_i < NUM_RT && rt_active[car_i]) car_lr_count = car_lr_count + 1;
    for (car_i = 0; car_i < N_LT; car_i = car_i + 1)
        if (car_i < NUM_LT && lt_active[car_i]) car_lr_count = car_lr_count + 1;
    for (car_i = 0; car_i < N_ST_R; car_i = car_i + 1)
        if (car_i < NUM_ST_R && stR_active[car_i]) car_lr_count = car_lr_count + 1;
    for (car_i = 0; car_i < N_RT_R; car_i = car_i + 1)
        if (car_i < NUM_RT_R && rt2_active[car_i]) car_lr_count = car_lr_count + 1;
    for (car_i = 0; car_i < N_LT_R; car_i = car_i + 1)
        if (car_i < NUM_LT_R && lt2_active[car_i]) car_lr_count = car_lr_count + 1;
    
    // TB方向车辆统计（需要检�?? NUM_* 限制�??
    for (car_i = 0; car_i < N_TSD; car_i = car_i + 1)
        if (car_i < NUM_TSD && tsd_active[car_i]) car_tb_count = car_tb_count + 1;
    for (car_i = 0; car_i < N_TL; car_i = car_i + 1)
        if (car_i < NUM_TL && tl_active[car_i]) car_tb_count = car_tb_count + 1;
    for (car_i = 0; car_i < N_TR_ALWAYS; car_i = car_i + 1)
        if (car_i < NUM_TR && tr_active[car_i]) car_tb_count = car_tb_count + 1;
    for (car_i = 0; car_i < N_BS_ST; car_i = car_i + 1)
        if (car_i < NUM_BS_ST && bs_st_active[car_i]) car_tb_count = car_tb_count + 1;
    for (car_i = 0; car_i < N_BR; car_i = car_i + 1)
        if (car_i < NUM_BR && br_active[car_i]) car_tb_count = car_tb_count + 1;
    for (car_i = 0; car_i < N_BL; car_i = car_i + 1)
        if (car_i < NUM_BL && bl_active[car_i]) car_tb_count = car_tb_count + 1;
end

// 车辆数量输出（限制在5位，�??�??31�??
wire [4:0] car_n_lr_out = (car_lr_count > 31) ? 5'd31 : car_lr_count[4:0];
wire [4:0] car_n_tb_out = (car_tb_count > 31) ? 5'd31 : car_tb_count[4:0];
// 速度建议：根据LR和TB两条车道的车流密�?
localparam [4:0] CAR_HIGH_TH = 5'd15;  // 单条车道车多的阈�?
localparam [7:0] SPEED_BOTH_LOW  = 8'd60;  // 两条车道都少�?60 km/h
localparam [7:0] SPEED_ONE_HIGH  = 8'd40;  // �?条车道多�?40 km/h
localparam [7:0] SPEED_BOTH_HIGH = 8'd30;  // 两条车道都多�?30 km/h

wire lr_busy = (car_n_lr_out >= CAR_HIGH_TH);  // LR车道拥堵标志
wire tb_busy = (car_n_tb_out >= CAR_HIGH_TH);  // TB车道拥堵标志

always @(posedge vga_clk or negedge sys_rst_n) begin
  if (!sys_rst_n) begin
    speed_sugg <= SPEED_BOTH_LOW;
  end else if (sec_tick) begin
    case ({lr_busy, tb_busy})
      2'b00: speed_sugg <= SPEED_BOTH_LOW;   // 两条都少�?60
      2'b01,
      2'b10: speed_sugg <= SPEED_ONE_HIGH;   // �?条多�?40
      2'b11: speed_sugg <= SPEED_BOTH_HIGH;  // 两条都多�?30
      default: speed_sugg <= SPEED_BOTH_LOW;
    endcase
  end
end
wire [7:0] cfg_LRS, cfg_LRL, cfg_LRR;
wire [7:0] cfg_TBS, cfg_TBL, cfg_TBR;
wire       pLRS, pLRL, pLRR, pTBS, pTBL, pTBR;

// 2) ����������ļ���� cfg_uart_simple
cfg_uart_simple #(
  .DEF_LRS(8'd40), .DEF_LRL(8'd30), .DEF_LRR(8'd20),
  .DEF_TBS(8'd40), .DEF_TBL(8'd30), .DEF_TBR(8'd20)
) u_cfg (
  .clk     (vga_clk),
  .rst_n   (sys_rst_n),
  .rx_valid(uart_rx_valid),   // ���㴫������������
  .rx_data (uart_rx_byte),

  .LRS(cfg_LRS), .LRL(cfg_LRL), .LRR(cfg_LRR),
  .TBS(cfg_TBS), .TBL(cfg_TBL), .TBR(cfg_TBR),

  .LRS_up(pLRS), .LRL_up(pLRL), .LRR_up(pLRR),
  .TBS_up(pTBS), .TBL_up(pTBL), .TBR_up(pTBR)
);

traffic_adapt2 #(
  .BASE_S (40), .BASE_L (30), .BASE_R (20), .T_YEL (12),
  .EXT_BIG(10), .EXT_SMALL(5), .PED_RT_TH(9)
) u_adapt (
  .clk          (vga_clk),
  .rst_n        (sys_rst_n),
  .sec_tick     (sec_tick),
  .car_bias     (car_bias),

  // 来自 scene_cross 的行人统�??/等待�??
  .ped_n_lr     (ped_n_lr_out),   // [4:0]
  .ped_n_tb     (ped_n_tb_out),   // [4:0]
  .ped_has_lr   (ped_has_lr_out), // 1bit（加入相位保持）
  .ped_has_tb   (ped_has_tb_out), // 1bit��������λ���֣�
  // ʵʱ�����źţ������������̼��?
  .ped_active_lr(ped_active_lr),  // ʵʱ�źţ���ǰ�Ƿ���������LR�����������?
  .ped_active_tb(ped_active_tb),  // ʵʱ�źţ���ǰ�Ƿ���������TB�����������?

  // 自�?�应�??�??
  .mode_adapt_sw(mode_adapt_sw_s1),  // 使用同步后的信号

  // 灯输�??
  .col_LR_S     (col_LR_S),
  .col_LR_L     (col_LR_L),
  .col_LR_R     (col_LR_R),
  .col_TB_S     (col_TB_S),
  .col_TB_L     (col_TB_L),
  .col_TB_R     (col_TB_R),
  // 绿灯时长输出
  .green_sec_LR_S (green_sec_LR_S),
  .green_sec_LR_L (green_sec_LR_L),
  .green_sec_LR_R (green_sec_LR_R),
  .green_sec_TB_S (green_sec_TB_S),
  .green_sec_TB_L (green_sec_TB_L),
  .green_sec_TB_R (green_sec_TB_R),
  // ֻ������6������
  .cfg_LRS(cfg_LRS), .cfg_LRL(cfg_LRL), .cfg_LRR(cfg_LRR),
  .cfg_TBS(cfg_TBS), .cfg_TBL(cfg_TBL), .cfg_TBR(cfg_TBR),
  .phase_left_s (phase_left_s_out),
  .phase_id     (phase_id_out)
  
);

// ===================== 模式标签显示模块 =====================
wire mode_label_on;
mode_label #(
  .CHAR_W   (8),
  .CHAR_H   (8),
  .SPACE_W  (1),
  .SCREEN_W (640),
  .SCREEN_H (480),
  .PAD_X    (8),
  .PAD_Y    (8)
) u_mode_label (
  .pix_clk      (vga_clk),
  .x             (pix_x),
  .y             (pix_y),
  .adaptive_mode (mode_adapt_sw_s1),  // 使用同步后的信号
  .mode_on      (mode_label_on)
);

// ===================== 左上角显示：people: 总人数�?�LRCar: �?? TBCar: 车辆数（使用 word_renderer�?? =====================
wire people_label_on, people_num_on;
wire lrcar_label_on, lrcar_num_on;
wire tbcar_label_on, tbcar_num_on;
wire speed_label_on, speed_num_on;
// 绿灯时长显示输出信号（在 word_renderer 实例化之前声明）
wire tb_green_label_on, tb_s_label_on, tb_s_num_on;
wire tb_l_label_on, tb_l_num_on;
wire tb_r_label_on, tb_r_num_on;
wire lr_green_label_on, lr_s_label_on, lr_s_num_on;
wire lr_l_label_on, lr_l_num_on;
wire lr_r_label_on, lr_r_num_on;
// 倒计时已删除
reg  [7:0] speed_sugg;
word_renderer #(
  .CHAR_W  (8),
  .CHAR_H  (8),
  .SPACE_W (1)
) u_word_renderer (
  .pix_clk    (vga_clk),
  .x          (pix_x),
  .y          (pix_y),
  .people_on  (),  // 未使�??
  .carM_on    (),  // 未使�??
  .carS_on    (),  // 未使�??
  .ped_num_lr (ped_n_lr_out),
  .ped_num_tb (ped_n_tb_out),
  .car_num_lr (car_n_lr_out),
  .car_num_tb (car_n_tb_out),
  .people_label_on(people_label_on),
  .people_num_on  (people_num_on),
  .lrcar_label_on (lrcar_label_on),
  .lrcar_num_on   (lrcar_num_on),
  .tbcar_label_on (tbcar_label_on),
  .tbcar_num_on   (tbcar_num_on),
  // 绿灯时长输入
  .green_sec_LR_S (green_sec_LR_S),
  .green_sec_LR_L (green_sec_LR_L),
  .green_sec_LR_R (green_sec_LR_R),
  .green_sec_TB_S (green_sec_TB_S),
  .green_sec_TB_L (green_sec_TB_L),
  .green_sec_TB_R (green_sec_TB_R),
  .mode_adapt_sw  (mode_adapt_sw_s1),
  .speed_sugg     (speed_sugg),
  // 绿灯时长显示输出
  .tb_green_label_on(tb_green_label_on),
  .tb_s_label_on    (tb_s_label_on),
  .tb_s_num_on      (tb_s_num_on),
  .tb_l_label_on    (tb_l_label_on),
  .tb_l_num_on      (tb_l_num_on),
  .tb_r_label_on    (tb_r_label_on),
  .tb_r_num_on      (tb_r_num_on),
  .lr_green_label_on(lr_green_label_on),
  .lr_s_label_on    (lr_s_label_on),
  .lr_s_num_on      (lr_s_num_on),
  .lr_l_label_on    (lr_l_label_on),
  .lr_l_num_on      (lr_l_num_on),
  .lr_r_label_on    (lr_r_label_on),
  .lr_r_num_on      (lr_r_num_on),
  .speed_label_on   (speed_label_on),
  .speed_num_on     (speed_num_on),
  // 倒计时已删除以节省LUT
  .countdown_LR_S   (8'd0),
  .countdown_LR_L   (8'd0),
  .countdown_LR_R   (8'd0),
  .countdown_TB_S   (8'd0),
  .countdown_TB_L   (8'd0),
  .countdown_TB_R   (8'd0),
  .cd_tb_s_num_on   (),
  .cd_tb_l_num_on   (),
  .cd_tb_r_num_on   (),
  .cd_lr_s_num_on   (),
  .cd_lr_l_num_on   (),
  .cd_lr_r_num_on   ()
);

    // ========= 6?????????????????? + ?????? =========
    // ??????LR?????????40s?? -> ?????30s?? -> ?????20s??
    // ??????TB?????????40s?? -> ?????30s?? -> ?????20s??????????90??
    
    // ?????????????????????????????????
    localparam integer LR_S_GREEN  = 4;   // ?????? 4???
    localparam integer LR_S_YELLOW = 1;   // ?????? 1???
    
    localparam integer LR_L_GREEN  = 3;   // ?????? 3???
    localparam integer LR_L_YELLOW = 1;   // ?????? 1???
    
    localparam integer LR_R_GREEN  = 2;   // ?????? 2???
    localparam integer LR_R_YELLOW = 1;   // ?????? 1???
    
    // LR??????????????
    localparam integer LR_T0  = 0;
    localparam integer LR_T1  = LR_T0  + LR_S_GREEN;
    localparam integer LR_T2  = LR_T1  + LR_S_YELLOW;
    localparam integer LR_T3  = LR_T2;
    localparam integer LR_T4  = LR_T3  + LR_L_GREEN;
    localparam integer LR_T5  = LR_T4  + LR_L_YELLOW;
    localparam integer LR_T6  = LR_T5;
    localparam integer LR_T7  = LR_T6  + LR_R_GREEN;
    localparam integer LR_T8  = LR_T7  + LR_R_YELLOW;
    localparam integer LR_END = LR_T8;
    
    // TB???????????????LR???
    localparam integer TB_T0  = LR_END;
    localparam integer TB_T1  = TB_T0  + LR_S_GREEN;
    localparam integer TB_T2  = TB_T1  + LR_S_YELLOW;
    localparam integer TB_T3  = TB_T2;
    localparam integer TB_T4  = TB_T3  + LR_L_GREEN;
    localparam integer TB_T5  = TB_T4  + LR_L_YELLOW;
    localparam integer TB_T6  = TB_T5;
    localparam integer TB_T7  = TB_T6  + LR_R_GREEN;
    localparam integer TB_T8  = TB_T7  + LR_R_YELLOW;
    localparam integer TB_END = TB_T8;
    
    localparam integer FULL_CYCLE = TB_END;  // ???????? = LR + TB

    // 范围判断函数
    function in_range;
        input integer s0, s1;
        input [15:0] cnt;
        begin
            in_range = (cnt >= s0) && (cnt < s1);
        end
    endfunction
    
    // ========= 车辆位置（车头）=========
    reg   signed [15:0] l2r_mid_head_int, l2r_top_head_int; // 左到右中/�??
    reg   signed [15:0] r2l_head_int;                       // 右到左车�??
  
// === LightL 红绿灯左侧灯箱位置（LS直行灯�?�LL左转灯�?�LR右转灯）===
localparam integer L_x0 = XL + TL_BOX_PAD;
localparam integer L_x1 = XL + TL_BOX_PAD + TL_BOX_W;
localparam integer L_y0 = YB - TL_BOX_PAD - TL_BOX_H;
localparam integer L_y1 = YB - TL_BOX_PAD;

// 灯箱边框
wire lightL_border =
    is_rect(L_x0, L_x1, L_y0,              L_y0 + TL_RING_T) |
    is_rect(L_x0, L_x1, L_y1 - TL_RING_T,  L_y1) |
    is_rect(L_x0, L_x0 + TL_RING_T, L_y0,  L_y1) |
    is_rect(L_x1 - TL_RING_T, L_x1, L_y0,  L_y1);

// 灯箱内部圆形灯位置（横向居中，纵向从上到下排列）
localparam integer L_light_x0 = L_x0 + 2;  // 灯的左边�??+2像素边距
localparam integer L_light_x1 = L_light_x0 + TL_LIGHT_SIZE;

// 左侧灯箱 L，从上到下依次为 �?? �?? �?? 三个�??
// 第一个灯：LL (Left) - 左转
localparam integer L_L_y0 = L_y0 + TL_LIGHT_GAP;
localparam integer L_L_y1 = L_L_y0 + TL_LIGHT_SIZE;
wire L_L_rect = is_rect(L_light_x0, L_light_x1, L_L_y0, L_L_y1);

// 第二个灯：LS (Straight) - 直行�??
localparam integer L_S_y0 = L_L_y1 + TL_LIGHT_GAP;
localparam integer L_S_y1 = L_S_y0 + TL_LIGHT_SIZE;
wire L_S_rect = is_rect(L_light_x0, L_light_x1, L_S_y0, L_S_y1);

// 第三个灯：LR (Right) - 右转�??
localparam integer L_R_y0 = L_S_y1 + TL_LIGHT_GAP;
localparam integer L_R_y1 = L_R_y0 + TL_LIGHT_SIZE;
wire L_R_rect = is_rect(L_light_x0, L_light_x1, L_R_y0, L_R_y1);
    
// === LightR 红绿灯右侧灯箱位置（RR右转灯�?�RL左转灯�?�RS直行灯）===    
localparam integer R_x0 = XR - TL_BOX_PAD - TL_BOX_W;
localparam integer R_x1 = XR - TL_BOX_PAD;
localparam integer R_y0 = YT + TL_BOX_PAD;
localparam integer R_y1 = YT + TL_BOX_PAD + TL_BOX_H;

// 灯箱边框
wire lightR_border =
    is_rect(R_x0, R_x1, R_y0,              R_y0 + TL_RING_T) |
    is_rect(R_x0, R_x1, R_y1 - TL_RING_T,  R_y1) |
    is_rect(R_x0, R_x0 + TL_RING_T, R_y0,  R_y1) |
    is_rect(R_x1 - TL_RING_T, R_x1, R_y0,  R_y1);

// 灯箱内部圆形灯位置（横向居中，纵向从上到下排列）
localparam integer R_light_x0 = R_x0 + 2;
localparam integer R_light_x1 = R_light_x0 + TL_LIGHT_SIZE;

// 第一个灯：RR (Right)
localparam integer R_R_y0 = R_y0 + TL_LIGHT_GAP;
localparam integer R_R_y1 = R_R_y0 + TL_LIGHT_SIZE;
wire R_R_rect = is_rect(R_light_x0, R_light_x1, R_R_y0, R_R_y1);

// 第二个灯：RL (Left)
localparam integer R_L_y0 = R_R_y1 + TL_LIGHT_GAP;
localparam integer R_L_y1 = R_L_y0 + TL_LIGHT_SIZE;
wire R_L_rect = is_rect(R_light_x0, R_light_x1, R_L_y0, R_L_y1);

// 第三个灯：RS (Straight)
localparam integer R_S_y0 = R_L_y1 + TL_LIGHT_GAP;
localparam integer R_S_y1 = R_S_y0 + TL_LIGHT_SIZE;
wire R_S_rect = is_rect(R_light_x0, R_light_x1, R_S_y0, R_S_y1);
// ============== LightT ?????????????????TR???????TL???????TS?????==============
// ???????????=TL_BOX_H(48)??????=TL_BOX_W(16)
localparam integer T_x1 = XC - TL_BOX_PAD;
localparam integer T_x0 = T_x1 - TL_BOX_H;  // ???????????
localparam integer T_y1 = YT - TL_BOX_PAD;
localparam integer T_y0 = T_y1 - TL_BOX_W;  // ???????????

// ??????
wire lightT_border =
    is_rect(T_x0,T_x1,           T_y0,           T_y0+TL_RING_T) |
    is_rect(T_x0,T_x1,           T_y1-TL_RING_T, T_y1) |
    is_rect(T_x0,T_x0+TL_RING_T, T_y0,           T_y1)           |
    is_rect(T_x1-TL_RING_T,T_x1, T_y0,           T_y1);

// ???????????????????????????????????????
localparam integer T_light_y0 = T_y0 + 2;
localparam integer T_light_y1 = T_light_y0 + TL_LIGHT_SIZE;

// ???????? TR (Right)
localparam integer T_R_x0 = T_x0 + TL_LIGHT_GAP;
localparam integer T_R_x1 = T_R_x0 + TL_LIGHT_SIZE;
wire T_R_rect = is_rect(T_R_x0, T_R_x1, T_light_y0, T_light_y1);

// ????????? TL (Left)
localparam integer T_L_x0 = T_R_x1 + TL_LIGHT_GAP;
localparam integer T_L_x1 = T_L_x0 + TL_LIGHT_SIZE;
wire T_L_rect = is_rect(T_L_x0, T_L_x1, T_light_y0, T_light_y1);

// ???????? TS (Straight)
localparam integer T_S_x0 = T_L_x1 + TL_LIGHT_GAP;
localparam integer T_S_x1 = T_S_x0 + TL_LIGHT_SIZE;
wire T_S_rect = is_rect(T_S_x0, T_S_x1, T_light_y0, T_light_y1);



// ============== LightB ??????????????????BS???????BL???????BR?????==============
// ???????????=TL_BOX_H(48)??????=TL_BOX_W(16)
localparam integer B_x1 = XR - TL_BOX_PAD;
localparam integer B_x0 = B_x1 - TL_BOX_H;  // ???????????
localparam integer B_y0 = YB + TL_BOX_PAD;
localparam integer B_y1 = B_y0 + TL_BOX_W;  // ???????????

// ??????
wire lightB_border =
    is_rect(B_x0,B_x1,           B_y0,           B_y0+TL_RING_T) |
    is_rect(B_x0,B_x1,           B_y1-TL_RING_T, B_y1)           |
    is_rect(B_x0,B_x0+TL_RING_T, B_y0,           B_y1)           |
    is_rect(B_x1-TL_RING_T,B_x1, B_y0,           B_y1);

// ???????????????????????????????????????
localparam integer B_light_y0 = B_y0 + 2;
localparam integer B_light_y1 = B_light_y0 + TL_LIGHT_SIZE;

// ???????? BS (Straight)
localparam integer B_S_x0 = B_x0 + TL_LIGHT_GAP;
localparam integer B_S_x1 = B_S_x0 + TL_LIGHT_SIZE;
wire B_S_rect = is_rect(B_S_x0, B_S_x1, B_light_y0, B_light_y1);

// ????????? BL (Left)
localparam integer B_L_x0 = B_S_x1 + TL_LIGHT_GAP;
localparam integer B_L_x1 = B_L_x0 + TL_LIGHT_SIZE;
wire B_L_rect = is_rect(B_L_x0, B_L_x1, B_light_y0, B_light_y1);

// ???????? BR (Right)
localparam integer B_R_x0 = B_L_x1 + TL_LIGHT_GAP;
localparam integer B_R_x1 = B_R_x0 + TL_LIGHT_SIZE;
wire B_R_rect = is_rect(B_R_x0, B_R_x1, B_light_y0, B_light_y1);

// ================= ???????????????????????? =================
// ????????????????????????????????
localparam integer ZTL_R = 6;  // ???????????????????????

// =========== ????????????????????????===========
// ????????????????????????????????????????????
// ??????????
localparam integer L_CX  = (L_x0 + L_x1) >> 1;
localparam integer L_CY0 = (L_L_y0 + L_L_y1) >> 1;  // ?????????
localparam integer L_CY1 = (L_S_y0 + L_S_y1) >> 1;  // ?????????

localparam integer L_L_X = L_CX;
localparam integer L_L_Y = L_CY0;
localparam integer L_S_X = L_CX;
localparam integer L_S_Y = L_CY1;

// ??????????
localparam integer R_CX  = (R_x0 + R_x1) >> 1;
localparam integer R_CY0 = (R_R_y0 + R_R_y1) >> 1;  // ?????????
localparam integer R_CY1 = (R_L_y0 + R_L_y1) >> 1;  // ?????????

localparam integer R_L_X = R_CX;
localparam integer R_L_Y = R_CY1;
localparam integer R_S_X = R_CX;
localparam integer R_S_Y = R_CY0;

// ????????????
localparam integer B_CXc = (B_x0 + B_x1) >> 1;
localparam integer B_CY  = (B_y0 + B_y1) >> 1;
localparam integer B_CX0 = (B_L_x0 + B_L_x1) >> 1;  // ?????????
localparam integer B_CX1 = (B_S_x0 + B_S_x1) >> 1;  // ?????????

localparam integer B_L_X = B_CX0;
localparam integer B_L_Y = B_CY;
localparam integer B_S_X = B_CX1;
localparam integer B_S_Y = B_CY;

// ???????????
localparam integer T_CYc = (T_y0 + T_y1) >> 1;
localparam integer T_CY  = T_CYc;
localparam integer T_CX0 = (T_S_x0 + T_S_x1) >> 1;  // ?????????
localparam integer T_CX1 = (T_L_x0 + T_L_x1) >> 1;  // ?????????

localparam integer T_L_X = T_CX1;
localparam integer T_L_Y = T_CY;
localparam integer T_S_X = T_CX0;
localparam integer T_S_Y = T_CY;


// ===== ?????????????? =====
localparam integer ZBOX_PAD_X = 6;         // ?????????????????????
localparam integer ZBOX_PAD_Y = 6;         // ??????????????????????
localparam integer ZBOX_BORDER_T = 2;      // ???????????

// ===== Precompute TL box rectangles =====
// --- TOP (vertical)
localparam integer ZT_x0 = ZT_CX - (ZTL_R + ZBOX_PAD_X);
localparam integer ZT_x1 = ZT_CX + (ZTL_R + ZBOX_PAD_X);
localparam integer ZT_y0 = ZT_G_CY - (ZTL_R + ZBOX_PAD_Y);
localparam integer ZT_y1 = ZT_R_CY + (ZTL_R + ZBOX_PAD_Y);

// --- BOTTOM (vertical)
localparam integer ZB_x0 = ZB_CX - (ZTL_R + ZBOX_PAD_X);
localparam integer ZB_x1 = ZB_CX + (ZTL_R + ZBOX_PAD_X);
localparam integer ZB_y0 = ZB_G_CY - (ZTL_R + ZBOX_PAD_Y);
localparam integer ZB_y1 = ZB_R_CY + (ZTL_R + ZBOX_PAD_Y);

// --- LEFT (horizontal)
localparam integer ZL_x0 = ZL_G_CX - (ZTL_R + ZBOX_PAD_X);
localparam integer ZL_x1 = ZL_R_CX + (ZTL_R + ZBOX_PAD_X);
localparam integer ZL_y0 = ZL_CY   - (ZTL_R + ZBOX_PAD_Y);
localparam integer ZL_y1 = ZL_CY   + (ZTL_R + ZBOX_PAD_Y);

// --- RIGHT (horizontal)
localparam integer ZR_x0 = ZR_G_CX - (ZTL_R + ZBOX_PAD_X);
localparam integer ZR_x1 = ZR_R_CX + (ZTL_R + ZBOX_PAD_X);
localparam integer ZR_y0 = ZR_CY   - (ZTL_R + ZBOX_PAD_Y);
localparam integer ZR_y1 = ZR_CY   + (ZTL_R + ZBOX_PAD_Y);

// ===== ???????????????????????? =====
localparam integer ZTL_SIZE = 10;  // ????????10x10

// ZT - ????????????????????????????
localparam integer ZT_G_x0 = ZT_CX - (ZTL_SIZE>>1);
localparam integer ZT_G_x1 = ZT_CX + (ZTL_SIZE>>1);
localparam integer ZT_G_y0 = ZT_G_CY - (ZTL_SIZE>>1);
localparam integer ZT_G_y1 = ZT_G_CY + (ZTL_SIZE>>1);

localparam integer ZT_Y_x0 = ZT_CX - (ZTL_SIZE>>1);
localparam integer ZT_Y_x1 = ZT_CX + (ZTL_SIZE>>1);
localparam integer ZT_Y_y0 = ZT_Y_CY - (ZTL_SIZE>>1);
localparam integer ZT_Y_y1 = ZT_Y_CY + (ZTL_SIZE>>1);

localparam integer ZT_R_x0 = ZT_CX - (ZTL_SIZE>>1);
localparam integer ZT_R_x1 = ZT_CX + (ZTL_SIZE>>1);
localparam integer ZT_R_y0 = ZT_R_CY - (ZTL_SIZE>>1);
localparam integer ZT_R_y1 = ZT_R_CY + (ZTL_SIZE>>1);

// ZB - ?????????????????????????????
localparam integer ZB_G_x0 = ZB_CX - (ZTL_SIZE>>1);
localparam integer ZB_G_x1 = ZB_CX + (ZTL_SIZE>>1);
localparam integer ZB_G_y0 = ZB_G_CY - (ZTL_SIZE>>1);
localparam integer ZB_G_y1 = ZB_G_CY + (ZTL_SIZE>>1);

localparam integer ZB_Y_x0 = ZB_CX - (ZTL_SIZE>>1);
localparam integer ZB_Y_x1 = ZB_CX + (ZTL_SIZE>>1);
localparam integer ZB_Y_y0 = ZB_Y_CY - (ZTL_SIZE>>1);
localparam integer ZB_Y_y1 = ZB_Y_CY + (ZTL_SIZE>>1);

localparam integer ZB_R_x0 = ZB_CX - (ZTL_SIZE>>1);
localparam integer ZB_R_x1 = ZB_CX + (ZTL_SIZE>>1);
localparam integer ZB_R_y0 = ZB_R_CY - (ZTL_SIZE>>1);
localparam integer ZB_R_y1 = ZB_R_CY + (ZTL_SIZE>>1);

// ZL - ??????????????????????????
localparam integer ZL_G_x0 = ZL_G_CX - (ZTL_SIZE>>1);
localparam integer ZL_G_x1 = ZL_G_CX + (ZTL_SIZE>>1);
localparam integer ZL_G_y0 = ZL_CY - (ZTL_SIZE>>1);
localparam integer ZL_G_y1 = ZL_CY + (ZTL_SIZE>>1);

localparam integer ZL_Y_x0 = ZL_Y_CX - (ZTL_SIZE>>1);
localparam integer ZL_Y_x1 = ZL_Y_CX + (ZTL_SIZE>>1);
localparam integer ZL_Y_y0 = ZL_CY - (ZTL_SIZE>>1);
localparam integer ZL_Y_y1 = ZL_CY + (ZTL_SIZE>>1);

localparam integer ZL_R_x0 = ZL_R_CX - (ZTL_SIZE>>1);
localparam integer ZL_R_x1 = ZL_R_CX + (ZTL_SIZE>>1);
localparam integer ZL_R_y0 = ZL_CY - (ZTL_SIZE>>1);
localparam integer ZL_R_y1 = ZL_CY + (ZTL_SIZE>>1);

// ZR - ???????????????????????????
localparam integer ZR_G_x0 = ZR_G_CX - (ZTL_SIZE>>1);
localparam integer ZR_G_x1 = ZR_G_CX + (ZTL_SIZE>>1);
localparam integer ZR_G_y0 = ZR_CY - (ZTL_SIZE>>1);
localparam integer ZR_G_y1 = ZR_CY + (ZTL_SIZE>>1);

localparam integer ZR_Y_x0 = ZR_Y_CX - (ZTL_SIZE>>1);
localparam integer ZR_Y_x1 = ZR_Y_CX + (ZTL_SIZE>>1);
localparam integer ZR_Y_y0 = ZR_CY - (ZTL_SIZE>>1);
localparam integer ZR_Y_y1 = ZR_CY + (ZTL_SIZE>>1);

localparam integer ZR_R_x0 = ZR_R_CX - (ZTL_SIZE>>1);
localparam integer ZR_R_x1 = ZR_R_CX + (ZTL_SIZE>>1);
localparam integer ZR_R_y0 = ZR_CY - (ZTL_SIZE>>1);
localparam integer ZR_R_y1 = ZR_CY + (ZTL_SIZE>>1);


/// ============= 锟斤拷锟教碉拷状态锟斤拷2=锟教ｏ拷1=锟狡ｏ拷0=锟斤�?? =============


// 锟斤拷锟教碉拷锟斤拷色锟斤拷锟藉（RGB565锟斤拷式锟斤�??
localparam [15:0] COL_RED = 16'hF800;
localparam [15:0] COL_YEL = 16'hFFE0;
localparam [15:0] COL_GRN = 16'h07E0;
localparam integer RAD    = 6; // 锟斤拷锟教碉拷圆锟轿半径锟斤拷锟斤拷锟节匡拷锟劫伙拷锟狡ｏ拷

//======================================================================
// 直锟叫筹拷锟斤拷锟斤拷锟斤拷�??/锟斤拷色锟斤拷锟斤拷 + 90锟斤拷锟斤拷位锟筋，锟街憋拷锟斤拷锟斤拷医锟斤拷耄癸拷锟絍erilog-2001锟斤�??
// 通锟斤拷锟斤拷锟斤拷PAUSE_*锟缴匡拷锟斤拷锟角凤拷锟斤拷锟矫革拷锟斤拷锟斤拷
//======================================================================

/************ 锟狡讹拷时锟接凤拷频锟斤拷锟斤拷锟狡筹拷锟斤拷锟劫度ｏ拷 ************/
initial begin
  div = {MOVE_DIV_W{1'b0}};  // 锟斤拷始锟斤拷锟斤拷频锟斤拷锟斤拷锟斤拷锟斤�?
end
// 使锟斤拷initial锟斤拷锟斤拷锟絘lways锟斤拷锟斤拷锟斤拷始锟斤拷锟斤拷频锟斤拷锟斤拷锟斤拷锟斤拷initial锟斤拷锟节诧拷锟斤拷锟斤拷X�??

/************ 停止锟斤拷位锟矫讹拷锟斤�?? ************/
localparam integer STOP_L2R_X = XL - ZB_OFF - XW;   // 锟斤拷锟斤拷锟斤拷锟揭筹拷锟斤拷停止锟斤拷X锟斤拷锟斤拷
localparam integer STOP_R2L_X = XR + ZB_OFF + XW;

localparam integer GAP_SPAWN  = CAR_L + 60;    // 锟斤拷锟斤拷锟斤拷锟缴硷拷锟斤拷锟斤拷锟斤拷�???+60锟斤拷锟截ｏ�??
localparam integer GAP_QUEUE  = CAR_L + 20; // 锟斤拷锟斤拷锟斤拷啵拷锟斤拷锟?+20锟斤拷锟截ｏ�??

function is_out_right_x; input integer x; begin
  is_out_right_x = (x - (CAR_L>>1)) > H_VALID; // 锟斤拷锟斤拷锟角否超筹拷锟揭边斤�??
end endfunction
function is_out_top_y; input integer y; begin
  is_out_top_y = (y + (CAR_L>>1)) < 0;         // 锟斤拷锟斤拷锟角否超筹拷锟较边斤�??
end endfunction

// ================= 锟斤拷转锟斤拷锟斤拷锟斤拷撞锟斤拷猓ㄊ癸拷锟�??8.8锟斤拷锟斤拷锟斤拷锟斤拷 =================
function car_rect_rot_q;
  input integer cx, cy, halfL, halfW;      // 锟斤拷锟斤拷锟斤拷锟斤拷 & 锟诫长锟斤拷�???
  input signed [15:0] cos_q, sin_q;        // cos/sin值锟斤拷Q8.8锟斤拷锟斤拷锟斤拷锟斤拷
  reg   signed [17:0] dx, dy;
  reg   signed [31:0] lx, ly;              // Q8.8
  begin
    dx = $signed(pix_x) - $signed(cx);
    dy = $signed(pix_y) - $signed(cy);
    // x' =  dx*cos + dy*sin
    // y' = -dx*sin + dy*cos
    lx = $signed(dx)*$signed(cos_q) + $signed(dy)*$signed(sin_q);
    ly = -$signed(dx)*$signed(sin_q) + $signed(dy)*$signed(cos_q);
    car_rect_rot_q =
      ( ($signed(lx>>>8) >= -halfL) && ($signed(lx>>>8) < halfL) &&
        ($signed(ly>>>8) >= -halfW) && ($signed(ly>>>8) < halfW) );
  end
endfunction

// ============ 0..90?? ?? cos/sin ?????Q8.8?????? 95/01?? ============
localparam integer ANG_STEPS = 8;  // ????????0..16 (?????????LUT??12??8)

// 注意：N_ST, N_RT, N_LT, N_ST_R, N_RT_R, N_LT_R, N_TSD, N_TL, N_TR_ALWAYS, N_BS_ST, N_BR, N_BL
// 已移至第934行之前定义，以便在wire声明和数组声明中使用

// -- ??????????????????????????????????????????????????--
integer st_minx, st_t;   // ????????????

// ========================= ????????????YD1C?? =========================
  // ?????????????10??6
reg signed [15:0] st_x     [0:N_ST-1];
reg signed [15:0] st_y     [0:N_ST-1];

wire ST_GREEN = (col_LR_S==C_GRN);

integer st_i;
// L→R 直行：无条件初始�??
initial begin
  for (st_i=0; st_i<N_ST; st_i=st_i+1) begin
    st_active[st_i] = 1'b1;
    st_x[st_i]      = -CAR_L - st_i*GAP_SPAWN;
    st_y[st_i]      = YD1C;
  end
end


integer st_prev_allow_x, st_nx;
always @(posedge vga_clk) begin
  if (move_tick) begin
    st_prev_allow_x = 32'sh7fffffff;
    for (st_i=0; st_i<N_ST; st_i=st_i+1) 
    if (st_active[st_i]&& (st_i < NUM_ST)) begin
      if (!ST_GREEN && (st_x[st_i] + (CAR_L>>1) >= STOP_L2R_X))
        st_nx = STOP_L2R_X - (CAR_L>>1);
      else
        st_nx = st_x[st_i] + $signed(CAR_SPD);

      if (st_prev_allow_x != 32'sh7fffffff && st_nx > st_prev_allow_x)
        st_nx = st_prev_allow_x;

      st_x[st_i] <= st_nx;
      st_y[st_i] <= YD1C;

      st_prev_allow_x = st_nx - GAP_QUEUE;

      if (is_out_right_x(st_nx)) begin
        st_minx = st_nx;
        for (st_t=0; st_t<N_ST; st_t=st_t+1)
          if (st_active[st_t] && st_x[st_t] < st_minx) st_minx = st_x[st_t];
        st_x[st_i] <= st_minx - GAP_SPAWN;
        st_y[st_i] <= YD1C;
      end
    end
  end
end

// ?????????????????????
wire car_hit_straight_any;
// L→R 直行命中
reg [N_ST-1:0] st_hit_array;
integer st_hit_i;

always @(*) begin
  st_hit_array = {N_ST{1'b0}};  // 自�?�应清零，避�?? N_ST�??10 的宽度问�??
  for (st_hit_i = 0; st_hit_i < N_ST; st_hit_i = st_hit_i + 1) begin
    st_hit_array[st_hit_i] =
      (st_hit_i < NUM_ST) && st_active[st_hit_i] && car_rect_xy(st_x[st_hit_i], st_y[st_hit_i]);
  end
end

assign car_hit_straight_any = |st_hit_array;


// ====================== ?????YD0C ?? XL_1???? FSM?? ======================
// ====================== ?????YD0C ?? XL_1???? FSM??????? ======================

// ????????? / ?????
localparam [1:0] RT_STRAIGHT=2'd0, RT_VERT=2'd1;

// ???????
localparam integer RT_TURN_X0 = XL - (CAR_L>>1);  // ???? XL ???????
localparam integer RT_X_TGT   = XL_1;             // ????????1???????

// ??????? FSM ?????
// reg        rt_active [0:N_RT-1];  // 已在统计代码之前声明
reg  [1:0] rt_state  [0:N_RT-1];
reg signed [15:0] rt_x [0:N_RT-1], rt_y [0:N_RT-1];

// ?????????????? GAP_SPAWN ???
integer k_rt;
initial begin
  for (k_rt=0; k_rt<N_RT; k_rt=k_rt+1) begin
    rt_active[k_rt] = 1'b1;
    rt_state[k_rt]  = RT_STRAIGHT;
    rt_x[k_rt]      = -CAR_L - k_rt*GAP_SPAWN;
    rt_y[k_rt]      = YD0C;
  end
end

// rt系锟斤拷锟狡讹拷锟竭硷拷
integer i_rt, prev_allow_x_rt, nx_rt, minx_rt, t_rt;

always @(posedge vga_clk) begin
  if (move_tick) begin
    // -- rt系锟叫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷水平锟轿ｏ�??
    prev_allow_x_rt = 32'sh7fffffff; // 锟斤拷始锟斤拷为锟斤拷锟�???
    for (i_rt=0; i_rt<N_RT; i_rt=i_rt+1) 
      if (rt_active[i_rt] && (i_rt < NUM_RT)) begin
      if (rt_state[i_rt]==RT_STRAIGHT) begin
        // 锟斤拷转锟斤拷锟教灯匡拷锟狡ｏ拷锟斤拷锟绞蓖ｏ拷锟酵Ｖ癸拷锟?
        if (col_LR_R == 2'd0 && (rt_x[i_rt] + (CAR_L>>1) >= STOP_L2R_X)) begin
          // 停锟斤拷停止锟竭ｏ拷锟斤拷头锟斤拷锟斤拷停止锟竭ｏ拷
          nx_rt = STOP_L2R_X - (CAR_L>>1);  // 停止
        end else if (rt_x[i_rt] < RT_TURN_X0) begin
          nx_rt = rt_x[i_rt] + $signed(CAR_SPD);   // 锟斤拷锟斤拷前锟斤拷
        end else begin
          nx_rt          = RT_X_TGT;               // 锟斤拷锟斤拷转锟斤拷悖拷锟斤拷锟侥匡拷锟絏
          rt_state[i_rt] <= RT_VERT;               // 锟斤拷锟诫垂直锟斤拷
        end

        // 锟斤拷锟斤拷锟竭硷拷锟斤拷锟斤拷锟杰筹拷锟斤拷前锟斤�??
        if (prev_allow_x_rt != 32'sh7fffffff && nx_rt > prev_allow_x_rt)
          nx_rt = prev_allow_x_rt;

        rt_x[i_rt] <= nx_rt;
        rt_y[i_rt] <= YD0C;                        // 锟斤拷锟斤拷锟斤拷y锟斤拷锟疥（锟铰凤拷锟斤拷锟斤拷锟斤�??
        prev_allow_x_rt = nx_rt - GAP_QUEUE;
      end
    end

    // -- rt系锟叫达拷直锟轿ｏ拷锟斤拷锟斤拷锟狡讹拷锟斤�??--
    for (i_rt=0; i_rt<N_RT; i_rt=i_rt+1) 
      if (rt_active[i_rt] && (i_rt < NUM_RT) && rt_state[i_rt]==RT_VERT) begin
      rt_x[i_rt] <= RT_X_TGT;                      // 锟斤拷锟斤拷锟斤拷X转锟斤拷�???
      rt_y[i_rt] <= rt_y[i_rt] + $signed(CAR_SPD); // 锟斤拷锟斤拷锟狡讹拷
      if (rt_y[i_rt] >= $signed(V_VALID + (CAR_L>>1))) begin
        // 锟斤拷锟斤拷锟斤拷幕锟阶诧拷锟斤拷循锟斤拷锟斤拷锟斤拷悖瑇锟斤拷锟斤拷锟斤拷为锟斤拷小x锟斤拷GAP_SPAWN
        minx_rt = rt_x[i_rt];
        for (t_rt=0; t_rt<N_RT; t_rt=t_rt+1)
          if (rt_active[t_rt] && rt_x[t_rt] < minx_rt) minx_rt = rt_x[t_rt];
        rt_x[i_rt]     <= minx_rt - GAP_SPAWN;
        rt_y[i_rt]     <= YD0C;
        rt_state[i_rt] <= RT_STRAIGHT;
      end
    end
  end
end

wire [5:0] rt_idx_en = { (NUM_RT>5), (NUM_RT>4), (NUM_RT>3), (NUM_RT>2), (NUM_RT>1), (NUM_RT>0) };

wire car_hit_rturn_arr =
  (rt_idx_en[0] & rt_active[0] && ((rt_state[0]==RT_STRAIGHT && car_rect_h(rt_x[0], rt_y[0])) ||
                                   (rt_state[0]==RT_VERT     && car_rect_v(rt_x[0], rt_y[0])))) |
  (rt_idx_en[1] & rt_active[1] && ((rt_state[1]==RT_STRAIGHT && car_rect_h(rt_x[1], rt_y[1])) ||
                                   (rt_state[1]==RT_VERT     && car_rect_v(rt_x[1], rt_y[1])))) |
  (rt_idx_en[2] & rt_active[2] && ((rt_state[2]==RT_STRAIGHT && car_rect_h(rt_x[2], rt_y[2])) ||
                                   (rt_state[2]==RT_VERT     && car_rect_v(rt_x[2], rt_y[2])))) |
  (rt_idx_en[3] & rt_active[3] && ((rt_state[3]==RT_STRAIGHT && car_rect_h(rt_x[3], rt_y[3])) ||
                                   (rt_state[3]==RT_VERT     && car_rect_v(rt_x[3], rt_y[3])))) |
  (rt_idx_en[4] & rt_active[4] && ((rt_state[4]==RT_STRAIGHT && car_rect_h(rt_x[4], rt_y[4])) ||
                                   (rt_state[4]==RT_VERT     && car_rect_v(rt_x[4], rt_y[4])))) |
  (rt_idx_en[5] & rt_active[5] && ((rt_state[5]==RT_STRAIGHT && car_rect_h(rt_x[5], rt_y[5])) ||
                                   (rt_state[5]==RT_VERT     && car_rect_v(rt_x[5], rt_y[5]))));

                    
//======================================================================
// LEFT TURN ARC (EAST -> NORTH, CCW)  ????????????????????????????
// ???????? 1/4 ?????? (X_TURN, YD2C) ????? (XR_1, YT)
localparam [1:0]   LT_STRAIGHT=2'd0, LT_ARC=2'd1, LT_VERT=2'd2;
wire [N_LT-1:0] lt_hit_rot;
wire [N_LT-1:0] lt_hit_valid;

// ---------- ?????????????????? XR_1 & YT?? ----------
// ????R = YD2C - YT?????LT_X_TURN = XR_1 - R??????C = (LT_X_TURN, YD2C - R)
localparam integer ARC_MGN     = 0;
localparam integer LT_R_BASE   = (YD2C - YT) - ARC_MGN;
localparam integer LT_R        = (LT_R_BASE>8) ? LT_R_BASE : 8;

localparam integer LT_X_TURN   = XR_1 - LT_R;
localparam integer LT_CX       = LT_X_TURN;
localparam integer LT_CY       = YD2C - LT_R;

// ---------- ??/???? ----------
// reg        lt_active [0:N_LT-1];  // 已在统计代码之前声明
reg  [1:0] lt_state  [0:N_LT-1];
reg signed [15:0] lt_x [0:N_LT-1], lt_y [0:N_LT-1];

// ---------- ?????????0..ANG_STEPS ??? 0??..90?? ????? ----------
reg [7:0]  lt_ang_idx   [0:N_LT-1];
reg [15:0] lt_tick_ps   [0:N_LT-1];
reg [15:0] lt_tick_cnt  [0:N_LT-1];
reg [7:0]  lt_ang_idx_d [0:N_LT-1];  // ???????????????????????

// ---------- ?????????????? ----------
integer idx_lt;
initial begin
  for (idx_lt=0; idx_lt<N_LT; idx_lt=idx_lt+1) begin
    lt_active[idx_lt]   = 1'b1;
    lt_state[idx_lt]    = LT_STRAIGHT;
    lt_x[idx_lt]        = -CAR_L - idx_lt*(CAR_L + 60);
    lt_y[idx_lt]        = YD2C;
    lt_ang_idx[idx_lt]  = 0;
    lt_tick_ps[idx_lt]  = 1;
    lt_tick_cnt[idx_lt] = 0;
    lt_ang_idx_d[idx_lt]= 0;
  end
end

// ---------- ??????????? ROM ?????????????? move_tick ??? ----------
integer ii_d_lt;
always @(posedge vga_clk) begin : LT_SHADOW_ANG
  integer j;
  for (j=0; j<N_LT; j=j+1) lt_ang_idx_d[j] <= lt_ang_idx[j];
end

// ---------- ??"???????"???? sin/cos??????=????? ----------
wire signed [15:0] lt_cos_pos_q [0:N_LT-1];
wire signed [15:0] lt_sin_pos_q [0:N_LT-1];

genvar g_lt_pos;
generate
for (g_lt_pos=0; g_lt_pos<N_LT; g_lt_pos=g_lt_pos+1) begin: G_LT_POS_TRIG
  wire signed [15:0] COS_Q_POS, SIN_Q_POS;
  sincos90_q88 #(.ANG_STEPS(ANG_STEPS)) u_sc_pos (
    .clk  (vga_clk),
    .idx  (lt_ang_idx_d[g_lt_pos]),   // ???? ?? ???? ROM ???
    .cos_q(COS_Q_POS),
    .sin_q(SIN_Q_POS)
  );
  // EAST??NORTH ???????????x = Cx + R*sin(a), y = Cy + R*cos(a)
  assign lt_cos_pos_q[g_lt_pos] = COS_Q_POS;   // +cos(a)
  assign lt_sin_pos_q[g_lt_pos] = SIN_Q_POS;   // +sin(a)
end
endgenerate

// ---------- ?????? ----------
integer lt_i, lt_prev_allow_x, lt_nx, lt_minx, lt_t;
integer lt_steps_on_arc;
integer tmp_ps;   // ???? lt_tick_ps ??
wire L_L_GREEN = (col_LR_L == C_GRN);

always @(posedge vga_clk) begin
  if (move_tick) begin
    // ===== ????????? + ???? + ???? + ?? =====
    lt_prev_allow_x = 32'sh7fffffff; // ??????????
    for (lt_i=0; lt_i<N_LT; lt_i=lt_i+1) 
      if (lt_active[lt_i] && (lt_i < NUM_LT)) begin
      if (lt_state[lt_i]==LT_STRAIGHT) begin
        // ??/????????????????????
        if (!L_L_GREEN && (lt_x[lt_i] + (CAR_L>>1) >= STOP_L2R_X))
          lt_nx = STOP_L2R_X - (CAR_L>>1);
        else if (lt_x[lt_i] < LT_X_TURN)
          lt_nx = lt_x[lt_i] + $signed(CAR_SPD);   // ???????
        else begin
          // ?????? a=0????????????????? ROM ?????
          lt_ang_idx[lt_i] <= 0;
          tmp_ps = ((LT_R + (CAR_SPD-1)) / CAR_SPD) / ANG_STEPS;
          if (tmp_ps <= 0) tmp_ps = 1;
          lt_tick_ps[lt_i] <= tmp_ps;
          lt_tick_cnt[lt_i]<= 0;

          lt_nx            = LT_X_TURN;   // ?????
          lt_state[lt_i]   <= LT_ARC;
        end

        // ?????????????????????????????????????? - GAP_QUEUE??
        if (lt_prev_allow_x != 32'sh7fffffff && lt_nx > lt_prev_allow_x)
          lt_nx = lt_prev_allow_x;

        lt_x[lt_i] <= lt_nx;
        lt_y[lt_i] <= YD2C;
        lt_prev_allow_x = lt_nx - (CAR_L + 20);
      end
    end

    // ===== ?????????????????? + ????????????=???? =====
    for (lt_i=0; lt_i<N_LT; lt_i=lt_i+1) 
    if (lt_active[lt_i] && (lt_i < NUM_LT) && lt_state[lt_i]==LT_ARC)  begin
      // ???????????????????"???????"???? lt_ang_idx??????
      if (lt_ang_idx[lt_i] < ANG_STEPS) begin
        lt_tick_cnt[lt_i] <= lt_tick_cnt[lt_i] + 1'b1;
        if (lt_tick_cnt[lt_i] >= lt_tick_ps[lt_i]) begin
          lt_tick_cnt[lt_i] <= 0;
          lt_ang_idx[lt_i]  <= lt_ang_idx[lt_i] + 1'b1; // ?????????????? ROM ????
        end
      end

      // ????????x = Cx + R*sin(a) ; y = Cy + R*cos(a)??Q8.8 ?? >>8??
      for (lt_steps_on_arc=0; lt_steps_on_arc<CAR_SPD; lt_steps_on_arc=lt_steps_on_arc+1) begin
        lt_x[lt_i] <= LT_CX + ( ($signed(LT_R) * $signed(lt_sin_pos_q[lt_i])) >>> 8 );
        lt_y[lt_i] <= LT_CY + ( ($signed(LT_R) * $signed(lt_cos_pos_q[lt_i])) >>> 8 );
      end

      // ???????? 90?? ??
      if (lt_ang_idx[lt_i] >= ANG_STEPS) lt_state[lt_i] <= LT_VERT;
    end

    // ===== ????????????????????? =====
    for (lt_i=0; lt_i<N_LT; lt_i=lt_i+1) 
    if (lt_active[lt_i] && (lt_i < NUM_LT) && lt_state[lt_i]==LT_VERT) begin
      lt_x[lt_i] <= XR_1;                         // ????? x ???
      lt_y[lt_i] <= lt_y[lt_i] - $signed(CAR_SPD);
      if ( (lt_y[lt_i] + $signed(CAR_L>>1)) < 0 ) begin
        // ???????????????????? x??
        lt_minx = lt_x[lt_i];
        for (lt_t=0; lt_t<N_LT; lt_t=lt_t+1)
          if (lt_active[lt_t] && lt_x[lt_t] < lt_minx) lt_minx = lt_x[lt_t];
        lt_x[lt_i]     <= lt_minx - (CAR_L + 60);
        lt_y[lt_i]     <= YD2C;
        lt_state[lt_i] <= LT_STRAIGHT;
        lt_ang_idx[lt_i]   <= 0;
        lt_tick_cnt[lt_i]  <= 0;
      end
    end
  end // move_tick
end

genvar g_lt;
generate
for (g_lt=0; g_lt<N_LT; g_lt=g_lt+1) begin: G_LT_ROT_HIT
  // ???????? trig??lt_cos_pos_q / lt_sin_pos_q
  wire signed [15:0] COS_Q = lt_cos_pos_q[g_lt]; // cos(a_d)
  wire signed [15:0] SIN_Q = lt_sin_pos_q[g_lt]; // sin(a_d)

  // EAST->NORTH?????????cos' = +cos(a)??sin' = -sin(a)
  wire signed [15:0] COS2_Q =  COS_Q;
  wire signed [15:0] SIN2_Q = -SIN_Q;

  car_rect_rot_ip #(.Q(8)) u_rot (
    .clk      (vga_clk),
    .valid_in (1'b1),
    .pix_x    (pix_x),
    .pix_y    (pix_y),
    .cx       (lt_x[g_lt]),
    .cy       (lt_y[g_lt]),
    .halfL    ({ {8{1'b0}}, (CAR_L>>1) }),
    .halfW    ({ {8{1'b0}}, (CAR_W>>1) }),
    .cos_q    (COS2_Q),
    .sin_q    (SIN2_Q),
    .hit      (lt_hit_rot[g_lt]),
    .valid_out(lt_hit_valid[g_lt])
  );
end
endgenerate

// 索引使能：与 NUM_LT 绑定；car_sw_2=1 时只�?? 0�??1
wire [3:0] lt_idx_en = { (NUM_LT>3), (NUM_LT>2), (NUM_LT>1), (NUM_LT>0) };

wire car_hit_lturn_arr =
  (lt_idx_en[0] & lt_active[0] && ((lt_state[0]==LT_STRAIGHT && car_rect_h(lt_x[0], lt_y[0])) ||
                                   (lt_state[0]==LT_ARC      && lt_hit_valid[0] && lt_hit_rot[0]) ||
                                   (lt_state[0]==LT_VERT     && car_rect_v(lt_x[0], lt_y[0])))) |
  (lt_idx_en[1] & lt_active[1] && ((lt_state[1]==LT_STRAIGHT && car_rect_h(lt_x[1], lt_y[1])) ||
                                   (lt_state[1]==LT_ARC      && lt_hit_valid[1] && lt_hit_rot[1]) ||
                                   (lt_state[1]==LT_VERT     && car_rect_v(lt_x[1], lt_y[1])))) |
  (lt_idx_en[2] & lt_active[2] && ((lt_state[2]==LT_STRAIGHT && car_rect_h(lt_x[2], lt_y[2])) ||
                                   (lt_state[2]==LT_ARC      && lt_hit_valid[2] && lt_hit_rot[2]) ||
                                   (lt_state[2]==LT_VERT     && car_rect_v(lt_x[2], lt_y[2])))) |
  (lt_idx_en[3] & lt_active[3] && ((lt_state[3]==LT_STRAIGHT && car_rect_h(lt_x[3], lt_y[3])) ||
                                   (lt_state[3]==LT_ARC      && lt_hit_valid[3] && lt_hit_rot[3]) ||
                                   (lt_state[3]==LT_VERT     && car_rect_v(lt_x[3], lt_y[3]))));

wire car_hit_r2l = 1'b0;
// ????????R_S / R_L ??????????????/?????????????????
wire RS_GREEN = (col_LR_S==C_GRN);  // ?????
wire RL_GREEN = (col_LR_L==C_GRN);  // ?????

// reg        stR_active [0:N_ST_R-1];  // 已在统计代码之前声明
reg signed [15:0] stR_x [0:N_ST_R-1];
reg signed [15:0] stR_y [0:N_ST_R-1];

integer i_stR, stR_prev_allow_x, stR_nx, stR_minx, stR_t;

// ???????????????????????
initial begin
  for (i_stR=0; i_stR<N_ST_R; i_stR=i_stR+1) begin
    stR_active[i_stR] = 1'b1;
    stR_x[i_stR]      = H_VALID + CAR_L + i_stR*GAP_SPAWN;  // ???????
    stR_y[i_stR]      = YU1C;
  end
end

always @(posedge vga_clk) begin
  if (move_tick) begin
    stR_prev_allow_x = -32'sh7fffffff; // ???????????????????????????????+???
    for (i_stR=0; i_stR<N_ST_R; i_stR=i_stR+1) 
    if (stR_active[i_stR] && (i_stR < NUM_ST_R)) begin
      // ??/?????????????????????????????? x-(L/2) ???? <= STOP_R2L_X??
      if (!RS_GREEN && (stR_x[i_stR] - (CAR_L>>1) <= STOP_R2L_X))
        stR_nx = STOP_R2L_X + (CAR_L>>1);
      else
        stR_nx = stR_x[i_stR] - $signed(CAR_SPD);

      // ??????????????????????? < ????????? + GAP_QUEUE??
      if (stR_prev_allow_x != -32'sh7fffffff && stR_nx < stR_prev_allow_x)
        stR_nx = stR_prev_allow_x;

      stR_x[i_stR] <= stR_nx;
      stR_y[i_stR] <= YU1C;

      stR_prev_allow_x = stR_nx + GAP_QUEUE;

      // ??????????????????
      if ((stR_nx + (CAR_L>>1)) < 0) begin
        stR_minx = stR_nx;
        for (stR_t=0; stR_t<N_ST_R; stR_t=stR_t+1)
          if (stR_active[stR_t] && stR_x[stR_t] > stR_minx) stR_minx = stR_x[stR_t]; // ??????
        stR_x[i_stR] <= stR_minx + GAP_SPAWN;
        stR_y[i_stR] <= YU1C;
      end
    end
  end
end

// ?????????????????????
wire car_hit_r2l_straight_arr;
// R→L 直行命中
reg [N_ST_R-1:0] stR_hit_array;
integer stR_hit_i;

always @(*) begin
  stR_hit_array = {N_ST_R{1'b0}};  // ? 自�?�应宽度清零
  for (stR_hit_i = 0; stR_hit_i < N_ST_R; stR_hit_i = stR_hit_i + 1) begin
    stR_hit_array[stR_hit_i] =
      (stR_hit_i < NUM_ST_R) &&
      stR_active[stR_hit_i] &&
      car_rect_h(stR_x[stR_hit_i], stR_y[stR_hit_i]);
  end
end

assign car_hit_r2l_straight_arr = |stR_hit_array;

  
localparam [1:0]   RT2_STRAIGHT=2'd0, RT2_VERT=2'd1;
localparam integer RT2_TURN_X0 = XR + (CAR_L>>1);  // ??????????
localparam integer RT2_X_TGT   = XR_3;             // ????????3???????

// reg        rt2_active [0:N_RT_R-1];  // 已在统计代码之前声明
reg  [1:0] rt2_state  [0:N_RT_R-1];
reg signed [15:0] rt2_x [0:N_RT_R-1], rt2_y [0:N_RT_R-1];

integer i_rt2, prev_allow_x_rt2, nx_rt2, maxx_rt2, t_rt2;

// ????????????????? YU0C ????
initial begin
  for (i_rt2=0; i_rt2<N_RT_R; i_rt2=i_rt2+1) begin
    rt2_active[i_rt2] = 1'b1;
    rt2_state[i_rt2]  = RT2_STRAIGHT;
    rt2_x[i_rt2]      = H_VALID + CAR_L + i_rt2*GAP_SPAWN;
    rt2_y[i_rt2]      = YU0C;
  end
end

always @(posedge vga_clk) begin
  if (move_tick) begin
    // rt2系锟叫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷水平锟轿ｏ�??
    prev_allow_x_rt2 = -32'sh7fffffff;
    for (i_rt2=0; i_rt2<N_RT_R; i_rt2=i_rt2+1) 
    if (rt2_active[i_rt2] && (i_rt2 < NUM_RT_R)) begin
      if (rt2_state[i_rt2]==RT2_STRAIGHT) begin
        // 锟斤拷转锟斤拷锟教灯匡拷锟狡ｏ拷锟斤拷锟绞蓖ｏ拷锟酵Ｖ癸拷锟?
        if (col_LR_R == 2'd0 && (rt2_x[i_rt2] - (CAR_L>>1) <= STOP_R2L_X)) begin
          // 停锟斤拷停止锟竭ｏ拷锟斤拷头锟斤拷锟斤拷停止锟竭ｏ拷
          nx_rt2 = STOP_R2L_X + (CAR_L>>1);  // 停止
        end else if (rt2_x[i_rt2] > RT2_TURN_X0) begin
          nx_rt2 = rt2_x[i_rt2] - $signed(CAR_SPD);
        end else begin
          nx_rt2           = RT2_X_TGT;    // 锟斤拷锟斤拷转锟斤拷悖拷锟斤拷锟侥匡拷锟絏
          rt2_state[i_rt2] <= RT2_VERT;
        end
        if (prev_allow_x_rt2 != -32'sh7fffffff && nx_rt2 < prev_allow_x_rt2)
          nx_rt2 = prev_allow_x_rt2;

        rt2_x[i_rt2] <= nx_rt2;  rt2_y[i_rt2] <= YU0C;
        prev_allow_x_rt2 = nx_rt2 + GAP_QUEUE;
      end
    end

// rt2系锟叫达拷直锟轿ｏ拷锟斤拷锟斤拷锟狡讹拷锟斤�??
for (i_rt2=0; i_rt2<N_RT_R; i_rt2=i_rt2+1) 
if (rt2_active[i_rt2] && (i_rt2 < NUM_RT_R) && rt2_state[i_rt2]==RT2_VERT) begin
  rt2_x[i_rt2] <= RT2_X_TGT;
  rt2_y[i_rt2] <= rt2_y[i_rt2] - $signed(CAR_SPD);  // 锟斤拷锟斤拷锟狡讹拷
  if ( (rt2_y[i_rt2] + $signed(CAR_L>>1)) < 0 ) begin  // 锟斤拷锟斤拷锟斤拷幕锟斤拷锟斤拷
    // 循锟斤拷锟斤�??"锟斤拷锟?"锟斤拷x锟斤拷锟斤拷锟斤拷为锟斤拷锟絰锟斤拷GAP_SPAWN
    maxx_rt2 = rt2_x[i_rt2];
    for (t_rt2=0; t_rt2<N_RT_R; t_rt2=t_rt2+1)
      if (rt2_active[t_rt2] && rt2_x[t_rt2] > maxx_rt2) maxx_rt2 = rt2_x[t_rt2];
    rt2_x[i_rt2]     <= maxx_rt2 + GAP_SPAWN;
    rt2_y[i_rt2]     <= YU0C;
    rt2_state[i_rt2] <= RT2_STRAIGHT;
  end
end

  end
end
wire [5:0] rt2_idx_en = { (NUM_RT_R>5), (NUM_RT_R>4), (NUM_RT_R>3), (NUM_RT_R>2), (NUM_RT_R>1), (NUM_RT_R>0) };
wire car_hit_rturn_R_arr =
  (rt2_idx_en[0] & rt2_active[0] && ((rt2_state[0]==RT2_STRAIGHT && car_rect_h(rt2_x[0], rt2_y[0])) ||
                                     (rt2_state[0]==RT2_VERT     && car_rect_v(rt2_x[0], rt2_y[0])))) |
  (rt2_idx_en[1] & rt2_active[1] && ((rt2_state[1]==RT2_STRAIGHT && car_rect_h(rt2_x[1], rt2_y[1])) ||
                                     (rt2_state[1]==RT2_VERT     && car_rect_v(rt2_x[1], rt2_y[1])))) |
  (rt2_idx_en[2] & rt2_active[2] && ((rt2_state[2]==RT2_STRAIGHT && car_rect_h(rt2_x[2], rt2_y[2])) ||
                                     (rt2_state[2]==RT2_VERT     && car_rect_v(rt2_x[2], rt2_y[2])))) |
  (rt2_idx_en[3] & rt2_active[3] && ((rt2_state[3]==RT2_STRAIGHT && car_rect_h(rt2_x[3], rt2_y[3])) ||
                                     (rt2_state[3]==RT2_VERT     && car_rect_v(rt2_x[3], rt2_y[3])))) |
  (rt2_idx_en[4] & rt2_active[4] && ((rt2_state[4]==RT2_STRAIGHT && car_rect_h(rt2_x[4], rt2_y[4])) ||
                                     (rt2_state[4]==RT2_VERT     && car_rect_v(rt2_x[4], rt2_y[4])))) |
  (rt2_idx_en[5] & rt2_active[5] && ((rt2_state[5]==RT2_STRAIGHT && car_rect_h(rt2_x[5], rt2_y[5])) ||
                                     (rt2_state[5]==RT2_VERT     && car_rect_v(rt2_x[5], rt2_y[5]))));


//======================================================================
// RIGHT-TOP LEFT TURN (WEST -> SOUTH, CCW 90??)
// ????????????????????????? 1/4 ???????????
// ?????????? (XL_3, YB)
//======================================================================
localparam [1:0]   LT2_STRAIGHT=2'd0, LT2_ARC=2'd1, LT2_VERT=2'd2;

// ---------- ???????????"???????????"???? ----------
localparam integer ARC_MGN2   = 0;
localparam integer LT2_R_BASE = (YB - YU2C) - ARC_MGN2;
localparam integer LT2_R      = (LT2_R_BASE>8) ? LT2_R_BASE : 8;

localparam integer LT2_CX     = XL_3 + LT2_R;
localparam integer LT2_CY     = YB;
localparam integer LT2_X_TURN = LT2_CX;  // ????? x ????

// ---------- ??/???? ----------
// reg        lt2_active [0:N_LT_R-1];  // 已在统计代码之前声明
reg  [1:0] lt2_state  [0:N_LT_R-1];
reg signed [15:0] lt2_x [0:N_LT_R-1], lt2_y [0:N_LT_R-1];

// ---------- ?????????0..ANG_STEPS ??? 0??..90?? ----------
reg [7:0]  lt2_ang_idx   [0:N_LT_R-1];
reg [15:0] lt2_tick_ps   [0:N_LT_R-1];
reg [15:0] lt2_tick_cnt  [0:N_LT_R-1];

// ---------- ?????????????"???????"?? ROM ??????? ----------
reg [7:0]  lt2_ang_idx_d [0:N_LT_R-1];
always @(posedge vga_clk) begin : LT2_SHADOW_ANG
  integer j;
  for (j=0; j<N_LT_R; j=j+1)
    lt2_ang_idx_d[j] <= lt2_ang_idx[j];
end

// ---------- ????????? trig???????????=???????????? ----------
wire signed [15:0] lt2_cos_pos_q [0:N_LT_R-1];
wire signed [15:0] lt2_sin_pos_q [0:N_LT_R-1];

genvar g_lt2_pos;
generate
for (g_lt2_pos=0; g_lt2_pos<N_LT_R; g_lt2_pos=g_lt2_pos+1) begin: G_LT2_POS_TRIG
  wire signed [15:0] COS_Q_POS, SIN_Q_POS;
  sincos90_q88 #(.ANG_STEPS(ANG_STEPS)) u_sc2_pos (
    .clk  (vga_clk),
    .idx  (lt2_ang_idx_d[g_lt2_pos]),   // ???? ?? ???? ROM ???
    .cos_q(COS_Q_POS),
    .sin_q(SIN_Q_POS)
  );
  // ????"???????????"??????????? sin(a), cos(a)
  assign lt2_cos_pos_q[g_lt2_pos] = COS_Q_POS;
  assign lt2_sin_pos_q[g_lt2_pos] = SIN_Q_POS;
end
endgenerate

// ---------- ??????? ----------
integer idx_lt2;
initial begin
  for (idx_lt2=0; idx_lt2<N_LT_R; idx_lt2=idx_lt2+1) begin
    lt2_active[idx_lt2]   = 1'b1;
    lt2_state[idx_lt2]    = LT2_STRAIGHT;
    lt2_x[idx_lt2]        = H_VALID + CAR_L + idx_lt2*(CAR_L + 60);
    lt2_y[idx_lt2]        = YU2C;
    lt2_ang_idx[idx_lt2]  = 0;
    lt2_tick_ps[idx_lt2]  = 1;
    lt2_tick_cnt[idx_lt2] = 0;
  end
end

// ---------- ?????? ----------
integer i_lt2, prev_allow_x_lt2, nx_lt2, maxx_lt2, t_lt2;
integer steps_on_arc_lt2;
integer tmp_ps2;

always @(posedge vga_clk) begin
  if (move_tick) begin
    // ===== ????????? + ????????????=====
    prev_allow_x_lt2 = -32'sh7fffffff;
    for (i_lt2=0; i_lt2<N_LT_R; i_lt2=i_lt2+1) 
    if (lt2_active[i_lt2] && (i_lt2 < NUM_LT_R)) begin
      if (lt2_state[i_lt2]==LT2_STRAIGHT) begin
        if (!RL_GREEN && (lt2_x[i_lt2] - (CAR_L>>1) <= STOP_R2L_X))
          nx_lt2 = STOP_R2L_X + (CAR_L>>1);
        else if (lt2_x[i_lt2] > LT2_X_TURN)
          nx_lt2 = lt2_x[i_lt2] - $signed(CAR_SPD);
        else begin
          // ?????? a=0???????????????? ROM ?? trig??
          lt2_ang_idx[i_lt2] <= 0;
          tmp_ps2 = ((LT2_R + (CAR_SPD-1)) / CAR_SPD) / ANG_STEPS;
          if (tmp_ps2 <= 0) tmp_ps2 = 1;
          lt2_tick_ps[i_lt2]  <= tmp_ps2;
          lt2_tick_cnt[i_lt2] <= 0;

          nx_lt2              = LT2_X_TURN;
          lt2_state[i_lt2]    <= LT2_ARC;
        end

        // ????????????
        if (prev_allow_x_lt2 != -32'sh7fffffff && nx_lt2 < prev_allow_x_lt2)
          nx_lt2 = prev_allow_x_lt2;

        lt2_x[i_lt2] <= nx_lt2;
        lt2_y[i_lt2] <= YU2C;
        prev_allow_x_lt2 = nx_lt2 + (CAR_L + 20);
      end
    end

    // ===== ??????WEST ?? SOUTH??????? 0..90??=====
    for (i_lt2=0; i_lt2<N_LT_R; i_lt2=i_lt2+1) 
    if (lt2_active[i_lt2] && (i_lt2 < NUM_LT_R) && lt2_state[i_lt2]==LT2_ARC)  begin
      // ???????????
      if (lt2_ang_idx[i_lt2] < ANG_STEPS) begin
        lt2_tick_cnt[i_lt2] <= lt2_tick_cnt[i_lt2] + 1'b1;
        if (lt2_tick_cnt[i_lt2] >= lt2_tick_ps[i_lt2]) begin
          lt2_tick_cnt[i_lt2] <= 0;
          lt2_ang_idx[i_lt2]  <= lt2_ang_idx[i_lt2] + 1'b1; // ?????????????? ROM ????
        end
      end

      // ??????????"???????????"????
      // x = Cx - R*sin(a) ; y = Cy - R*cos(a)  ??Q8.8 ?? >>8??
      for (steps_on_arc_lt2=0; steps_on_arc_lt2<CAR_SPD; steps_on_arc_lt2=steps_on_arc_lt2+1) begin
        lt2_x[i_lt2] <= LT2_CX - ( ($signed(LT2_R) * $signed(lt2_sin_pos_q[i_lt2])) >>> 8 );
        lt2_y[i_lt2] <= LT2_CY - ( ($signed(LT2_R) * $signed(lt2_cos_pos_q[i_lt2])) >>> 8 );
      end

      // ???????????????????????
      if (lt2_ang_idx_d[i_lt2] >= ANG_STEPS) begin
        lt2_state[i_lt2] <= LT2_VERT;
      end
    end

    // ===== ????????x ??? XL_3??=====
    for (i_lt2=0; i_lt2<N_LT_R; i_lt2=i_lt2+1) 
    if (lt2_active[i_lt2] && (i_lt2 < NUM_LT_R) && lt2_state[i_lt2]==LT2_VERT) begin
      lt2_x[i_lt2] <= XL_3;
      lt2_y[i_lt2] <= lt2_y[i_lt2] + $signed(CAR_SPD);
      if (lt2_y[i_lt2] >= $signed(V_VALID + (CAR_L>>1))) begin
        // ???????????"????"?? x??
        maxx_lt2 = lt2_x[i_lt2];
        for (t_lt2=0; t_lt2<N_LT_R; t_lt2=t_lt2+1)
          if (lt2_active[t_lt2] && lt2_x[t_lt2] > maxx_lt2) maxx_lt2 = lt2_x[t_lt2];
        lt2_x[i_lt2]     <= maxx_lt2 + (CAR_L + 60);
        lt2_y[i_lt2]     <= YU2C;
        lt2_state[i_lt2] <= LT2_STRAIGHT;
        lt2_ang_idx[i_lt2]<= 0;
        lt2_tick_cnt[i_lt2]<= 0;
      end
    end
  end
end
// ================= ??????????WEST -> SOUTH?? =================
wire [N_LT_R-1:0] lt2_hit_rot;
wire [N_LT_R-1:0] lt2_hit_valid;

genvar g_lt2;
generate
for (g_lt2=0; g_lt2<N_LT_R; g_lt2=g_lt2+1) begin: G_LT2_ROT_HIT
  // ???????? trig??lt2_cos_pos_q / lt2_sin_pos_q
  wire signed [15:0] COS_Q = lt2_cos_pos_q[g_lt2]; // cos(a_d)
  wire signed [15:0] SIN_Q = lt2_sin_pos_q[g_lt2]; // sin(a_d)

  // WEST -> SOUTH????????cos' = -cos(a)??sin' = +sin(a)
  wire signed [15:0] COS2_Q = -COS_Q;
  wire signed [15:0] SIN2_Q =  SIN_Q;

  car_rect_rot_ip #(.Q(8)) u_rot2 (
    .clk      (vga_clk),
    .valid_in (1'b1),
    .pix_x    (pix_x),
    .pix_y    (pix_y),
    .cx       (lt2_x[g_lt2]),
    .cy       (lt2_y[g_lt2]),
    .halfL    ({ {8{1'b0}}, (CAR_L>>1) }),
    .halfW    ({ {8{1'b0}}, (CAR_W>>1) }),
    .cos_q    (COS2_Q),
    .sin_q    (SIN2_Q),
    .hit      (lt2_hit_rot[g_lt2]),
    .valid_out(lt2_hit_valid[g_lt2])
  );
end
endgenerate

// 索引使能：当 car_sw_2=1 �?? NUM_LT_R=2，只放行下标 0�??1
wire [3:0] lt2_idx_en = { (NUM_LT_R>3), (NUM_LT_R>2), (NUM_LT_R>1), (NUM_LT_R>0) };

wire car_hit_lturn_R_arr =
  (lt2_idx_en[0] & lt2_active[0] && ((lt2_state[0]==LT2_STRAIGHT && car_rect_h(lt2_x[0], lt2_y[0])) ||
                                     (lt2_state[0]==LT2_ARC      && lt2_hit_valid[0] && lt2_hit_rot[0]) ||
                                     (lt2_state[0]==LT2_VERT     && car_rect_v(lt2_x[0], lt2_y[0])))) |
  (lt2_idx_en[1] & lt2_active[1] && ((lt2_state[1]==LT2_STRAIGHT && car_rect_h(lt2_x[1], lt2_y[1])) ||
                                     (lt2_state[1]==LT2_ARC      && lt2_hit_valid[1] && lt2_hit_rot[1]) ||
                                     (lt2_state[1]==LT2_VERT     && car_rect_v(lt2_x[1], lt2_y[1])))) |
  (lt2_idx_en[2] & lt2_active[2] && ((lt2_state[2]==LT2_STRAIGHT && car_rect_h(lt2_x[2], lt2_y[2])) ||
                                     (lt2_state[2]==LT2_ARC      && lt2_hit_valid[2] && lt2_hit_rot[2]) ||
                                     (lt2_state[2]==LT2_VERT     && car_rect_v(lt2_x[2], lt2_y[2])))) |
  (lt2_idx_en[3] & lt2_active[3] && ((lt2_state[3]==LT2_STRAIGHT && car_rect_h(lt2_x[3], lt2_y[3])) ||
                                     (lt2_state[3]==LT2_ARC      && lt2_hit_valid[3] && lt2_hit_rot[3]) ||
                                     (lt2_state[3]==LT2_VERT     && car_rect_v(lt2_x[3], lt2_y[3]))));


//======================================================================
// ????????????XL_3 / XL_2 / XL_1???????????????????
// 1) XL_3 ?????NORTH -> EAST??CW 90???? ???? YD2C
// 2) XL_2 ????????NORTH -> SOUTH??
// 3) XL_1 ???????NORTH -> WEST??CCW 90?? ??"?????"??????????????? ???? YU0C
//   - TL: ??????????????? col_TB_L??"???????"??
//   - TS: ??????????????? col_TB_S??"???????+???"??
//======================================================================

// ---------- ??????????????? ----------
localparam integer STOP_T2B_Y = YT - ZB_OFF -XW ;      // ????????????

// ---------- ?????? ----------
wire TL_GREEN = (col_TB_L==C_GRN);
wire TS_GREEN = (col_TB_S==C_GRN);

//======================================================================
// A) XL_2 ?????????? TS ?????????????? + ???? + ??????
//======================================================================
// ===== A) XL_2 ?????????? TS ????? =====
// reg        tsd_active [0:N_TSD-1];  // 已在统计代码之前声明
reg signed [15:0] tsd_x [0:N_TSD-1];
reg signed [15:0] tsd_y [0:N_TSD-1];

integer i_tsd, tsd_prev_allow_y, ny_tsd, topy_tsd, t_tsd;

initial begin
  for (i_tsd=0; i_tsd<N_TSD; i_tsd=i_tsd+1) begin
    tsd_active[i_tsd] = 1'b1;
    tsd_x[i_tsd]      = XL_2;
    tsd_y[i_tsd]      = -CAR_L - i_tsd*GAP_SPAWN; // ???????????????
  end
end

always @(posedge vga_clk) begin
  if (move_tick) begin
    // ????????????"?????? GAP"--??? +?? / ?????
    tsd_prev_allow_y = 32'sh7fffffff;
    for (i_tsd=0; i_tsd<N_TSD; i_tsd=i_tsd+1) 
    if (tsd_active[i_tsd]&& (i_tsd < NUM_TSD)) begin
      // ??/?????????? STOP_T2B_Y
      if (!TS_GREEN && (tsd_y[i_tsd] + (CAR_L>>1) >= STOP_T2B_Y))
        ny_tsd = STOP_T2B_Y - (CAR_L>>1);
      else
        ny_tsd = tsd_y[i_tsd] + $signed(CAR_SPD);

      // ?????????????ny ??????????????
      if (ny_tsd > tsd_prev_allow_y) ny_tsd = tsd_prev_allow_y;

      tsd_x[i_tsd] <= XL_2;
      tsd_y[i_tsd] <= ny_tsd;

      // ?????????????? y - GAP_QUEUE
      tsd_prev_allow_y = ny_tsd - GAP_QUEUE;

      // ????????????????????
      if (ny_tsd - (CAR_L>>1) > V_VALID) begin
        topy_tsd = tsd_y[i_tsd];
        for (t_tsd=0; t_tsd<N_TSD; t_tsd=t_tsd+1)
          if (tsd_active[t_tsd] && tsd_y[t_tsd] < topy_tsd)
            topy_tsd = tsd_y[t_tsd];
        tsd_x[i_tsd] <= XL_2;
        tsd_y[i_tsd] <= topy_tsd - GAP_SPAWN;
      end
    end
  end
end


// ?????????????????????
wire car_hit_TS_down_arr;
reg [N_TSD-1:0] tsd_hit_array;  // 不写�?? 10
integer tsd_hit_i;
always @(*) begin
  tsd_hit_array = {N_TSD{1'b0}};
  for (tsd_hit_i = 0; tsd_hit_i < N_TSD; tsd_hit_i = tsd_hit_i + 1) begin
    if ((tsd_hit_i < NUM_TSD) && tsd_active[tsd_hit_i] &&
        car_rect_v(tsd_x[tsd_hit_i], tsd_y[tsd_hit_i])) begin
      tsd_hit_array[tsd_hit_i] = 1'b1;
    end
  end
end
assign car_hit_TS_down_arr = |tsd_hit_array;

//======================================================================
// TOP-LEFT TURN @ XL_3  (SOUTH -> EAST, CCW ~90?? in 180??..270?? quadrant)
// ???? XL_3 ???????????? TL_GREEN=col_TB_L ?????????????? YT ????
// ?????????????? 180???270?????????? y??YD2C????????????
// ?????CCW???? x = Cx - R*cos(a_d) ; y = Cy + R*sin(a_d)
// ????o??????????????cos' = +sin(a_d) ; sin' = +cos(a_d)
//======================================================================


localparam [1:0]   TL_STRAIGHT=2'd0, TL_ARC=2'd1, TL_HORZ=2'd2;

// ---- ???????????"?????????"? a=0 ?????????????????????? ----
localparam integer ARC_MGN_TL = 2;                           // ??????????(??0..3)
localparam integer TL_R_BASE  = (YD2C - YT) - ARC_MGN_TL;    // ??????YD2C-YT
localparam integer TL_R       = (TL_R_BASE > 8) ? TL_R_BASE : 8;
localparam integer TL_CX      = XL_3 + TL_R;                 // ??? x
localparam integer TL_CY      = YT;                          // ??? y?????????
localparam integer TL_Y_TURN  = YT;                          // ?????????????
localparam integer TL_Y_END   = YD2C;                        // ??? y ???

// ---------- ??/???? ----------
// reg        tl_active [0:N_TL-1];  // 已在统计代码之前声明
reg  [1:0] tl_state  [0:N_TL-1];
reg signed [15:0] tl_x [0:N_TL-1], tl_y [0:N_TL-1];

// ---------- ???????0..ANG_STEPS ?? 0??..90??????? ----------
reg  [7:0] tl_ang_idx   [0:N_TL-1];   // ??????????????????
reg  [7:0] tl_ang_idx_d [0:N_TL-1];   // ???????????/????????
reg [15:0] tl_tick_ps   [0:N_TL-1];
reg [15:0] tl_tick_cnt  [0:N_TL-1];

// ---------- ??????? ----------
integer k_tl_init;
initial begin
  for (k_tl_init=0; k_tl_init<N_TL; k_tl_init=k_tl_init+1) begin
    tl_active[k_tl_init]   = 1'b1;
    tl_state[k_tl_init]    = TL_STRAIGHT;
    tl_x[k_tl_init]        = XL_3;
    tl_y[k_tl_init]        = -CAR_L - k_tl_init*(CAR_L + 60);  // ???????????????
    tl_ang_idx[k_tl_init]  = 0;
    tl_ang_idx_d[k_tl_init]= 0;
    tl_tick_ps[k_tl_init]  = 1;
    tl_tick_cnt[k_tl_init] = 0;
  end
end

// ---------- ??????????? ROM ?????????????? move_tick ??? ----------
always @(posedge vga_clk) begin : TL_SHADOW_ANG
  integer j;
  for (j=0; j<N_TL; j=j+1) tl_ang_idx_d[j] <= tl_ang_idx[j];
end

// ---------- ?????? trig??????=???? ?? ??????????CCW ????????? ----------
// x = Cx - R*cos(a_d) ; y = Cy + R*sin(a_d)
wire signed [15:0] tl_pos_cos_q [0:N_TL-1];
wire signed [15:0] tl_pos_sin_q [0:N_TL-1];
genvar g_tl_pos;
generate
for (g_tl_pos=0; g_tl_pos<N_TL; g_tl_pos=g_tl_pos+1) begin: G_TL_POS_TRIG
  wire signed [15:0] COS_Q_POS, SIN_Q_POS;
  sincos90_q88 #(.ANG_STEPS(ANG_STEPS)) u_sc_pos (
    .clk  (vga_clk),
    .idx  (tl_ang_idx_d[g_tl_pos]),
    .cos_q(COS_Q_POS),
    .sin_q(SIN_Q_POS)
  );
  assign tl_pos_cos_q[g_tl_pos] = COS_Q_POS; // cos(a_d)
  assign tl_pos_sin_q[g_tl_pos] = SIN_Q_POS; // sin(a_d)
end
endgenerate

// ---------- ?????? ----------
integer i_tl, prev_allow_y_tl, ny_tl, steps_on_arc_tl, tmp_ps_tl;

always @(posedge vga_clk) begin
  if (move_tick) begin
    // === ???????? + ???? + ?????????????????????? ===
    prev_allow_y_tl = 32'sh7fffffff; // ??????? ny ????????????????
    for (i_tl=0; i_tl<N_TL; i_tl=i_tl+1) 
    if (tl_active[i_tl]&& (i_tl < NUM_TL)) begin
      if (tl_state[i_tl]==TL_STRAIGHT) begin
        if (!TL_GREEN && (tl_y[i_tl] + (CAR_L>>1) >= STOP_T2B_Y))
          ny_tl = STOP_T2B_Y - (CAR_L>>1);
        else if ( (tl_y[i_tl] + (CAR_L>>1)) < TL_Y_TURN )
          ny_tl = tl_y[i_tl] + $signed(CAR_SPD);              // ???????????
        else begin
          ny_tl             = TL_Y_TURN - (CAR_L>>1);         // ?????????(?????????)
          tl_ang_idx[i_tl]  <= 0;                             // ?? a=0
          tmp_ps_tl         = ((TL_R + (CAR_SPD-1)) / CAR_SPD) / ANG_STEPS;
          if (tmp_ps_tl <= 0) tmp_ps_tl = 1;
          tl_tick_ps[i_tl]  <= tmp_ps_tl;
          tl_tick_cnt[i_tl] <= 0;
          tl_state[i_tl]    <= TL_ARC;
        end
        if (ny_tl > prev_allow_y_tl) ny_tl = prev_allow_y_tl; // ???????
        tl_x[i_tl] <= XL_3;
        tl_y[i_tl] <= ny_tl;
        prev_allow_y_tl = ny_tl - (CAR_L + 20);
      end
    end

    // === ????????????????? + ????????????????????? CCW?? ===
    for (i_tl=0; i_tl<N_TL; i_tl=i_tl+1) 
    if (tl_active[i_tl] && tl_state[i_tl]==TL_ARC && (i_tl < NUM_TL)) begin
      if (tl_ang_idx[i_tl] < ANG_STEPS) begin
        tl_tick_cnt[i_tl] <= tl_tick_cnt[i_tl] + 1'b1;
        if (tl_tick_cnt[i_tl] >= tl_tick_ps[i_tl]) begin
          tl_tick_cnt[i_tl] <= 0;
          tl_ang_idx[i_tl]  <= tl_ang_idx[i_tl] + 1'b1;       // ?????????????
        end
      end
      // ???????????????
      // x = Cx - R*cos(a_d) ; y = Cy + R*sin(a_d)
      for (steps_on_arc_tl=0; steps_on_arc_tl<CAR_SPD; steps_on_arc_tl=steps_on_arc_tl+1) begin
        tl_x[i_tl] <= TL_CX - ( ($signed(TL_R) * $signed(tl_pos_cos_q[i_tl])) >>> 8 );
        tl_y[i_tl] <= TL_CY + ( ($signed(TL_R) * $signed(tl_pos_sin_q[i_tl])) >>> 8 );
      end

      // ??????????? 90????270??????????? ??????y ???? YD2C
      if (tl_ang_idx_d[i_tl] >= ANG_STEPS) begin
        tl_state[i_tl] <= TL_HORZ;
        tl_y[i_tl]     <= TL_Y_END;
      end
    end

    // === ???????????????????? ===
    for (i_tl=0; i_tl<N_TL; i_tl=i_tl+1) 
    if (tl_active[i_tl] && (i_tl < NUM_TL) && tl_state[i_tl]==TL_HORZ) begin
      tl_x[i_tl] <= tl_x[i_tl] + $signed(CAR_SPD);
      tl_y[i_tl] <= TL_Y_END;
      if ( (tl_x[i_tl] - $signed(CAR_L>>1)) > $signed(H_VALID) ) begin
        tl_x[i_tl]     <= XL_3;
        tl_y[i_tl]     <= -CAR_L - (i_tl*(CAR_L + 60));
        tl_state[i_tl] <= TL_STRAIGHT;
        tl_ang_idx[i_tl]  <= 0;
        tl_tick_cnt[i_tl] <= 0;
      end
    end
  end
end

// ---------- ???????????? TL_ARC ??? rot_ip???????????????(????) ----------
// ???????CCW??S->E????cos' = +sin(a_d)??sin' = +cos(a_d)  ?? ??"?????"
wire [N_TL-1:0] tl_hit_rot, tl_hit_valid;
genvar g_tl_hit;
generate
for (g_tl_hit=0; g_tl_hit<N_TL; g_tl_hit=g_tl_hit+1) begin: G_TL_ROT_HIT
  // ?????????????????????tl_pos_cos_q / tl_pos_sin_q
  wire signed [15:0] COS_Q_HIT = tl_pos_cos_q[g_tl_hit]; // cos(a_d)
  wire signed [15:0] SIN_Q_HIT = tl_pos_sin_q[g_tl_hit]; // sin(a_d)

  // S->E??????????????????????????cos' = +sin(a_d), sin' = +cos(a_d)
  wire signed [15:0] COS2_Q =  SIN_Q_HIT;
  wire signed [15:0] SIN2_Q =  COS_Q_HIT;

  car_rect_rot_ip #(.Q(8)) u_rot (
    .clk      (vga_clk),
    .valid_in (1'b1),
    .pix_x    (pix_x),
    .pix_y    (pix_y),
    .cx       (tl_x[g_tl_hit]),
    .cy       (tl_y[g_tl_hit]),
    .halfL    ({ {8{1'b0}}, (CAR_L>>1) }),
    .halfW    ({ {8{1'b0}}, (CAR_W>>1) }),
    .cos_q    (COS2_Q),
    .sin_q    (SIN2_Q),
    .hit      (tl_hit_rot[g_tl_hit]),
    .valid_out(tl_hit_valid[g_tl_hit])
  );
end
endgenerate

// 替换你现在的 car_hit_TL_leftturn_arr 手写 0..5 版本�??
reg [N_TL-1:0] tl_hit_array;
integer tl_k;
always @(*) begin
  tl_hit_array = {N_TL{1'b0}};
  for (tl_k = 0; tl_k < N_TL; tl_k = tl_k + 1) begin
    if (tl_k < NUM_TL && tl_active[tl_k]) begin
      tl_hit_array[tl_k] =
          (tl_state[tl_k]==TL_STRAIGHT && car_rect_v(tl_x[tl_k], tl_y[tl_k])) ||
          (tl_state[tl_k]==TL_HORZ     && car_rect_h(tl_x[tl_k], tl_y[tl_k])) ||
          (tl_state[tl_k]==TL_ARC      && tl_hit_valid[tl_k] && tl_hit_rot[tl_k]);
    end
  end
end
wire car_hit_TL_leftturn_arr = |tl_hit_array;

// ========================== TR：顶部→左侧（NORTH -> WEST）右转车�?? ==========================
// 依据人行横道（Zebra）外缘来确定真正的停止线，避免误停在 YT 本体线上

// 斑马线（顶边）内外缘（已存在全局参数：YT / ZB_OFF / XW�??
localparam integer ZEBRA_TOP_OUT = YT - ZB_OFF - XW; // 远离路口�??侧（外缘�??
localparam integer ZEBRA_TOP_IN  = YT - ZB_OFF;      // 靠路口一侧（内缘�??

// 停止线微调：把停止线放在斑马线外缘再向外偏移若干像素（视觉余量，0~4 都可�??
localparam integer STOP_TR_PAD   = 2;

// TR �??"真正停止�??"：靠路口外侧、略外移
localparam integer STOP_TR_Y     = ZEBRA_TOP_OUT - STOP_TR_PAD;

// 直行段到达此 y 触发进入水平右转；右转后保持的目�?? y
localparam integer TR_TURN_Y0    = YT + (CAR_L>>1);
localparam integer TR_Y_TGT      = YU0C;

// 车流规模与状态编�??
localparam [1:0]   TR_STRAIGHT   = 2'd0,   // 直行向下（自屏外进入�??
                   TR_HORZ       = 2'd1;   // 右转向左（水平出屏）

// 车队数组
// reg        tr_active [0:N_TR_ALWAYS-1];  // 已在统计代码之前声明
reg  [1:0] tr_state  [0:N_TR_ALWAYS-1];
reg signed [15:0] tr_x [0:N_TR_ALWAYS-1], tr_y [0:N_TR_ALWAYS-1];

// 辅助变量
integer i_tr, prev_allow_y_tr, ny_tr, miny_tr, t_tr;
integer head_y_now, head_y_next;

// 初始化：车辆�?? XL_1 竖直车道上方依次进入
initial begin : INIT_TR_CARS
  integer k;
  for (k=0; k<N_TR_ALWAYS; k=k+1) begin
    tr_active[k] = 1'b1;
    tr_state[k]  = TR_STRAIGHT;
    tr_x[k]      = XL_1;
    tr_y[k]      = -CAR_L - k*GAP_SPAWN;
  end
end

// 运动与灯控：与其它右转段（如 rt2）一致的"下一拍车头跨线预�??"
always @(posedge vga_clk) begin
  if (move_tick) begin
    // ---------------- 竖直直行阶段（向下） ----------------
    // ---------------- 竖直直行阶段（向下） ----------------
prev_allow_y_tr = 32'sh7fffffff;  // 允许�??"�??�?? y"先设很大
for (i_tr=0; i_tr<N_TR_ALWAYS; i_tr=i_tr+1) 
if (tr_active[i_tr]&& (i_tr < NUM_TR)) begin
  if (tr_state[i_tr]==TR_STRAIGHT) begin
    head_y_now  = tr_y[i_tr] + (CAR_L>>1);
    head_y_next = tr_y[i_tr] + $signed(CAR_SPD) + (CAR_L>>1);

    // �??/黄：下一拍车头跨线就刹停�?? STOP_TR_Y 外侧
    if ((col_TB_R==C_RED || col_TB_R==C_YEL) && head_y_next >= STOP_TR_Y) begin
      ny_tr = STOP_TR_Y - (CAR_L>>1);
    end
    // 未到转弯触发�?? �?? 继续向下
    else if (tr_y[i_tr] < TR_TURN_Y0) begin
      ny_tr = tr_y[i_tr] + $signed(CAR_SPD);
    end
    // 到达转弯触发�?? �?? 切水平右�??
    else begin
      ny_tr          = TR_Y_TGT;
      tr_state[i_tr] <= TR_HORZ;
    end

    // 队列防追尾：不得超过"允许的最�?? y"
    if (ny_tr > prev_allow_y_tr)
      ny_tr = prev_allow_y_tr;

    tr_x[i_tr] <= XL_1;
    tr_y[i_tr] <= ny_tr;

    // 给下�??辆车的最�?? y：当前车�?? - 间距（向上留空）
    prev_allow_y_tr = ny_tr - GAP_QUEUE;
  end
end


    // ---------------- 水平右转阶段（向左） ----------------
    for (i_tr=0; i_tr<N_TR_ALWAYS; i_tr=i_tr+1) 
    if (tr_active[i_tr] && (i_tr < NUM_TR) && tr_state[i_tr]==TR_HORZ) begin
      tr_x[i_tr] <= tr_x[i_tr] - $signed(CAR_SPD);
      tr_y[i_tr] <= TR_Y_TGT;

      // 出屏左侧后重置到竖直段起点（与其它段�??致的回收逻辑�??
      if ( (tr_x[i_tr] + $signed(CAR_L>>1)) < 0 ) begin
        // 取当前队列中�??小的 y（最靠上的车），在其上方按固定间隔生�??
        miny_tr = tr_y[i_tr];
        for (t_tr=0; t_tr<N_TR_ALWAYS; t_tr=t_tr+1)
          if (tr_active[t_tr] && (t_tr < NUM_TR)&& tr_y[t_tr] < miny_tr) miny_tr = tr_y[t_tr];
        tr_x[i_tr]     <= XL_1;
        tr_y[i_tr]     <= -CAR_L - (i_tr*(CAR_L + 60));
        tr_state[i_tr] <= TR_STRAIGHT;
      end
    end
  end
end

// （可选调试：画线便于肉眼校准停止�??/斑马线位置�?�上线前注释掉）
// if (PY>=ZEBRA_TOP_OUT-1 && PY<=ZEBRA_TOP_OUT+1) pix_cross <= 16'hFFFF; // 斑马外缘（白�??
// if (PY>=ZEBRA_TOP_IN-1  && PY<=ZEBRA_TOP_IN+1 ) pix_cross <= 16'hFFFF; // 斑马内缘（白�??
// if (PY>=STOP_TR_Y-1     && PY<=STOP_TR_Y+1    ) pix_cross <= 16'h07FF; // TR 停止线（青）

reg [N_TR_ALWAYS-1:0] tr_hit_array;
integer tr_k;
always @(*) begin
  tr_hit_array = {N_TR_ALWAYS{1'b0}};
  for (tr_k=0; tr_k<N_TR_ALWAYS; tr_k=tr_k+1) begin
    if (tr_k < NUM_TR && tr_active[tr_k]) begin
      tr_hit_array[tr_k] =
        (tr_state[tr_k]==TR_STRAIGHT && car_rect_v(tr_x[tr_k], tr_y[tr_k])) ||
        (tr_state[tr_k]==TR_HORZ     && car_rect_h(tr_x[tr_k], tr_y[tr_k]));
    end
  end
end
wire car_hit_TR_always_arr = |tr_hit_array;

                    
                    //======================================================================
// BOTTOM -> TOP three flows (XR_2 / XR_3 / XR_1)
// 1) XR_2 ????????BS=????? STOP_B2T_Y??
// 2) XR_3 ??? ?? ???? YD0C??BS=?????????????
// 3) XR_1 ?????BL=?????? 0..90??SOUTH->WEST?????? YU2C ????????
//======================================================================

localparam integer STOP_B2T_Y = YB + ZB_OFF + XW;  // ????????????????????
wire BS_GREEN = (col_TB_S==C_GRN); // ???????+?????Bottom Straight ??? TB_S??
wire BL_GREEN = (col_TB_L==C_GRN); // ?????????Bottom Left ??? TB_L)

//========================= 1) XR_2 ??????? =========================
// reg        bs_st_active [0:N_BS_ST-1];  // 已在统计代码之前声明
reg signed [15:0] bs_st_x [0:N_BS_ST-1];
reg signed [15:0] bs_st_y [0:N_BS_ST-1];

integer i_bs_st, bs_prev_allow_y, bs_ny, bs_maxy, t_bs_st;

initial begin
  for (i_bs_st=0; i_bs_st<N_BS_ST; i_bs_st=i_bs_st+1) begin
    bs_st_active[i_bs_st] = 1'b1;
    bs_st_x[i_bs_st]      = XR_2;
    bs_st_y[i_bs_st]      = V_VALID + CAR_L + i_bs_st*GAP_SPAWN; // ??????????????
  end
end

always @(posedge vga_clk) begin
  if (move_tick) begin
    // ?????y ??????????? y ????????"???????? y"???????????
    bs_prev_allow_y = -32'sh7fffffff; // ???? = -???????????????
    for (i_bs_st=0; i_bs_st<N_BS_ST; i_bs_st=i_bs_st+1) 
    if (bs_st_active[i_bs_st] && (i_bs_st < NUM_BS_ST)) begin
      // ??/????????????? STOP_B2T_Y???????y-(L/2) <= STOP_B2T_Y??
      if (!BS_GREEN && (bs_st_y[i_bs_st] - (CAR_L>>1) <= STOP_B2T_Y))
        bs_ny = STOP_B2T_Y + (CAR_L>>1);
      else
        bs_ny = bs_st_y[i_bs_st] - $signed(CAR_SPD);

      // ?????????????ny ???? < ???????? y??
      if (bs_prev_allow_y != -32'sh7fffffff && bs_ny < bs_prev_allow_y)
        bs_ny = bs_prev_allow_y;

      bs_st_x[i_bs_st] <= XR_2;
      bs_st_y[i_bs_st] <= bs_ny;

      // ??????????????? y = ???? y + GAP_QUEUE
      bs_prev_allow_y = bs_ny + GAP_QUEUE;

      // ????????????????????????"????"?? y??
      if ( (bs_ny + (CAR_L>>1)) < 0 ) begin
        bs_maxy = bs_st_y[i_bs_st];
        for (t_bs_st=0; t_bs_st<N_BS_ST; t_bs_st=t_bs_st+1)
          if (bs_st_active[t_bs_st]&& (t_bs_st < NUM_BS_ST) && bs_st_y[t_bs_st] > bs_maxy) bs_maxy = bs_st_y[t_bs_st];
        bs_st_x[i_bs_st] <= XR_2;
        bs_st_y[i_bs_st] <= bs_maxy + GAP_SPAWN;
      end
    end
  end
end

reg [N_BS_ST-1:0] bs_hit_array;
integer bs_hit_i;
always @(*) begin
  bs_hit_array = {N_BS_ST{1'b0}};
  for (bs_hit_i = 0; bs_hit_i < N_BS_ST; bs_hit_i = bs_hit_i + 1) begin
    if (bs_st_active[bs_hit_i] && (bs_hit_i < NUM_BS_ST) &&
        car_rect_v(bs_st_x[bs_hit_i], bs_st_y[bs_hit_i]))
      bs_hit_array[bs_hit_i] = 1'b1;
  end
end
wire car_hit_BS_up_arr = |bs_hit_array;

//========================= 2) XR_3 ??????????????? =========================
localparam [1:0]   BR_VERT=2'd0, BR_HORZ=2'd1;

// reg        br_active [0:N_BR-1];  // 已在统计代码之前声明
reg  [1:0] br_state  [0:N_BR-1];
reg signed [15:0] br_x [0:N_BR-1], br_y [0:N_BR-1];

integer i_br, br_prev_allow_y, br_ny, br_maxy, t_br;

localparam integer BR_TURN_Y0 = YB - (CAR_L>>1); // ????????????
localparam integer BR_Y_TGT   = YD0C;            // ??????????????????

initial begin
  for (i_br=0; i_br<N_BR; i_br=i_br+1) begin
    br_active[i_br] = 1'b1;
    br_state[i_br]  = BR_VERT;
    br_x[i_br]      = XR_3;
    br_y[i_br]      = V_VALID + CAR_L + i_br*GAP_SPAWN; // ???????
  end
end

always @(posedge vga_clk) begin
  if (move_tick) begin
    // br系锟叫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷锟斤拷直锟轿ｏ�??
    br_prev_allow_y = -32'sh7fffffff;
    for (i_br=0; i_br<N_BR; i_br=i_br+1) 
    if (br_active[i_br]&& (i_br < NUM_BR)) begin
      if (br_state[i_br]==BR_VERT) begin
        // 锟斤拷转锟斤拷锟教灯匡拷锟狡ｏ拷锟斤拷锟绞蓖ｏ拷锟酵Ｖ癸拷锟?
        if (col_TB_R == 2'd0 && (br_y[i_br] - (CAR_L>>1) <= STOP_B2T_Y)) begin
          // 停锟斤拷停止锟竭ｏ拷锟斤拷头锟斤拷锟斤拷停止锟竭ｏ拷
          br_ny = STOP_B2T_Y + (CAR_L>>1);  // 停止
        end else if (br_y[i_br] > BR_TURN_Y0) begin
          br_ny = br_y[i_br] - $signed(CAR_SPD);
        end else begin
          br_ny          = BR_Y_TGT;   // 锟斤拷锟斤拷转锟斤拷悖拷锟斤拷锟侥匡拷锟結
          br_state[i_br] <= BR_HORZ;
        end
        if (br_prev_allow_y != -32'sh7fffffff && br_ny < br_prev_allow_y)
          br_ny = br_prev_allow_y;

        br_x[i_br] <= XR_3;
        br_y[i_br] <= br_ny;
        br_prev_allow_y = br_ny + GAP_QUEUE;
      end
    end

    // ????????????????
    for (i_br=0; i_br<N_BR; i_br=i_br+1) 
    if (br_active[i_br] && (i_br < NUM_BR)&& br_state[i_br]==BR_HORZ) begin
      br_x[i_br] <= br_x[i_br] + $signed(CAR_SPD);
      br_y[i_br] <= BR_Y_TGT;
      if ( (br_x[i_br] - $signed(CAR_L>>1)) > $signed(H_VALID) ) begin
        // ???????????
        br_maxy = br_y[i_br];
        for (t_br=0; t_br<N_BR; t_br=t_br+1)
          if (br_active[t_br]&& (t_br < NUM_BR) && br_y[t_br] > br_maxy) br_maxy = br_y[t_br];
        br_x[i_br]     <= XR_3;
        br_y[i_br]     <= br_maxy + GAP_SPAWN;
        br_state[i_br] <= BR_VERT;
      end
    end
  end
end

reg [N_BR-1:0] br_hit_array;
integer br_k;
always @(*) begin
  br_hit_array = {N_BR{1'b0}};
  for (br_k=0; br_k<N_BR; br_k=br_k+1) begin
    if (br_k < NUM_BR && br_active[br_k]) begin
      br_hit_array[br_k] =
        (br_state[br_k]==BR_VERT && car_rect_v(br_x[br_k], br_y[br_k])) ||
        (br_state[br_k]==BR_HORZ && car_rect_h(br_x[br_k], br_y[br_k]));
    end
  end
end
wire car_hit_BR_turn_arr = |br_hit_array;


//========================= 3) XR_1 ???????? 0..90?? =========================
localparam [1:0]   BL_VERT=2'd0, BL_ARC=2'd1, BL_HORZ=2'd2;

// ?????????a=0..90??: ????? y=YB????? y=YU2C??SOUTH->WEST??CCW??
// ???????????????????
localparam integer BL_R_BASE = (YB - YU2C);
localparam integer BL_R      = (BL_R_BASE>8) ? BL_R_BASE : 8;
localparam integer BL_CX     = XR_1 - BL_R;       // ???X????????  
localparam integer BL_CY     = YB;                // ???Y = ???????????????
localparam integer BL_Y_TURN = YB;                // ???????????
localparam integer BL_Y_END  = YU2C;              // ??? y

// reg        bl_active [0:N_BL-1];  // 已在统计代码之前声明
reg  [1:0] bl_state  [0:N_BL-1];
reg signed [15:0] bl_x [0:N_BL-1], bl_y [0:N_BL-1];

// ???????0..ANG_STEPS ??? 0??..90??
reg  [7:0] bl_ang_idx   [0:N_BL-1];
reg  [7:0] bl_ang_idx_d [0:N_BL-1];
reg [15:0] bl_tick_ps   [0:N_BL-1];
reg [15:0] bl_tick_cnt  [0:N_BL-1];

integer k_bl;
initial begin
  for (k_bl=0; k_bl<N_BL; k_bl=k_bl+1) begin
    bl_active[k_bl]   = 1'b1;
    bl_state[k_bl]    = BL_VERT;
    bl_x[k_bl]        = XR_1;
    bl_y[k_bl]        = V_VALID + CAR_L + k_bl*(CAR_L + 60);
    bl_ang_idx[k_bl]  = 0;
    bl_ang_idx_d[k_bl]= 0;
    bl_tick_ps[k_bl]  = 1;
    bl_tick_cnt[k_bl] = 0;
  end
end

// ????????
always @(posedge vga_clk) begin : BL_SHADOW_ANG
  integer j;
  for (j=0; j<N_BL; j=j+1) bl_ang_idx_d[j] <= bl_ang_idx[j];
end

// ?????? trig??????=???? ?? ??????????
wire signed [15:0] bl_pos_cos_q [0:N_BL-1];
wire signed [15:0] bl_pos_sin_q [0:N_BL-1];
genvar g_bl_pos;
generate
for (g_bl_pos=0; g_bl_pos<N_BL; g_bl_pos=g_bl_pos+1) begin: G_BL_POS_TRIG
  wire signed [15:0] COS_Q_POS, SIN_Q_POS;
  sincos90_q88 #(.ANG_STEPS(ANG_STEPS)) u_sc_bl_pos (
    .clk  (vga_clk),
    .idx  (bl_ang_idx_d[g_bl_pos]),
    .cos_q(COS_Q_POS),
    .sin_q(SIN_Q_POS)
  );
  assign bl_pos_cos_q[g_bl_pos] = COS_Q_POS; // cos(a_d)
  assign bl_pos_sin_q[g_bl_pos] = SIN_Q_POS; // sin(a_d)
end
endgenerate

// ??????
integer i_bl, bl_prev_allow_y, bl_ny, steps_on_arc_bl, tmp_ps_bl;

always @(posedge vga_clk) begin
  if (move_tick) begin
    // === ??????????(BL) + ???? + ????????????????????===
    bl_prev_allow_y = -32'sh7fffffff;
    for (i_bl=0; i_bl<N_BL; i_bl=i_bl+1) 
    if (bl_active[i_bl]&& (i_bl < NUM_BL)) begin
      if (bl_state[i_bl]==BL_VERT) begin
        if (!BL_GREEN && (bl_y[i_bl] - (CAR_L>>1) <= STOP_B2T_Y))
          bl_ny = STOP_B2T_Y + (CAR_L>>1);
        else if ( (bl_y[i_bl] - (CAR_L>>1)) > BL_Y_TURN )
          bl_ny = bl_y[i_bl] - $signed(CAR_SPD);  // ????
        else begin
          bl_ny            = BL_Y_TURN + (CAR_L>>1); // ?????????????? YB??
          bl_ang_idx[i_bl] <= 0;                     // ?? a=0
          tmp_ps_bl        = ((BL_R + (CAR_SPD-1)) / CAR_SPD) / ANG_STEPS;
          if (tmp_ps_bl <= 0) tmp_ps_bl = 1;
          bl_tick_ps[i_bl] <= tmp_ps_bl;
          bl_tick_cnt[i_bl]<= 0;
          bl_state[i_bl]   <= BL_ARC;
        end
        if (bl_prev_allow_y != -32'sh7fffffff && bl_ny < bl_prev_allow_y)
          bl_ny = bl_prev_allow_y;

        bl_x[i_bl] <= XR_1;
        bl_y[i_bl] <= bl_ny;
        bl_prev_allow_y = bl_ny + (CAR_L + 20);
      end
    end

    // === ??????????? + ????????0..90??S->W??CCW??===
    for (i_bl=0; i_bl<N_BL; i_bl=i_bl+1) 
    if (bl_active[i_bl] && (i_bl < NUM_BL)&& bl_state[i_bl]==BL_ARC) begin
      if (bl_ang_idx[i_bl] < ANG_STEPS) begin
        bl_tick_cnt[i_bl] <= bl_tick_cnt[i_bl] + 1'b1;
        if (bl_tick_cnt[i_bl] >= bl_tick_ps[i_bl]) begin
          bl_tick_cnt[i_bl] <= 0;
          bl_ang_idx[i_bl]  <= bl_ang_idx[i_bl] + 1'b1;
        end
      end
      // x = Cx + R*cos(a) ; y = Cy - R*sin(a)
      for (steps_on_arc_bl=0; steps_on_arc_bl<CAR_SPD; steps_on_arc_bl=steps_on_arc_bl+1) begin
        bl_x[i_bl] <= BL_CX + ( ($signed(BL_R) * $signed(bl_pos_cos_q[i_bl])) >>> 8 );
        bl_y[i_bl] <= BL_CY - ( ($signed(BL_R) * $signed(bl_pos_sin_q[i_bl])) >>> 8 );
      end

      if (bl_ang_idx_d[i_bl] >= ANG_STEPS) begin
        bl_state[i_bl] <= BL_HORZ;
        bl_x[i_bl]     <= BL_CX;              // ???????x = XR_1 - BL_R
        bl_y[i_bl]     <= BL_Y_END;           // = YU2C
      end
    end

    // === ?????????????????? ===
    for (i_bl=0; i_bl<N_BL; i_bl=i_bl+1) 
    if (bl_active[i_bl] && (i_bl < NUM_BL)&& bl_state[i_bl]==BL_HORZ) begin
      bl_x[i_bl] <= bl_x[i_bl] - $signed(CAR_SPD);
      bl_y[i_bl] <= BL_Y_END;
      if ( (bl_x[i_bl] + $signed(CAR_L>>1)) < 0 ) begin
        // ???????? XR_1 ????
        bl_x[i_bl]     <= XR_1;
        bl_y[i_bl]     <= V_VALID + CAR_L + (i_bl*(CAR_L + 60));
        bl_state[i_bl] <= BL_VERT;
        bl_ang_idx[i_bl]  <= 0;
        bl_tick_cnt[i_bl] <= 0;
      end
    end
  end
end

// ????????????????????
// ????????????????????? bl_pos_cos_q/bl_pos_sin_q??
wire [N_BL-1:0] bl_hit_rot, bl_hit_valid;
genvar g_bl_hit;
generate
for (g_bl_hit=0; g_bl_hit<N_BL; g_bl_hit=g_bl_hit+1) begin: G_BL_ROT_HIT
  // SOUTH->WEST?????0????90????1/4???
  wire signed [15:0] COS_Q_HIT = bl_pos_cos_q[g_bl_hit];
  wire signed [15:0] SIN_Q_HIT = bl_pos_sin_q[g_bl_hit];

  // ????????????????????????(-sin(a), -cos(a))
  // cos(theta) = -sin(a), sin(theta) = -cos(a)
  wire signed [15:0] COS2_Q = -SIN_Q_HIT;
  wire signed [15:0] SIN2_Q = -COS_Q_HIT;

  car_rect_rot_ip #(.Q(8)) u_rot_bl (
    .clk      (vga_clk),
    .valid_in (1'b1),
    .pix_x    (pix_x),
    .pix_y    (pix_y),
    .cx       (bl_x[g_bl_hit]),
    .cy       (bl_y[g_bl_hit]),
    .halfL    ({ {8{1'b0}}, (CAR_L>>1) }),
    .halfW    ({ {8{1'b0}}, (CAR_W>>1) }),
    .cos_q    (COS2_Q),
    .sin_q    (SIN2_Q),
    .hit      (bl_hit_rot[g_bl_hit]),
    .valid_out(bl_hit_valid[g_bl_hit])
  );
end
endgenerate

reg [N_BL-1:0] bl_hit_array;
integer bl_k;
always @(*) begin
  bl_hit_array = {N_BL{1'b0}};
  for (bl_k=0; bl_k<N_BL; bl_k=bl_k+1) begin
    if (bl_k < NUM_BL && bl_active[bl_k]) begin
      bl_hit_array[bl_k] =
        (bl_state[bl_k]==BL_VERT && car_rect_v(bl_x[bl_k], bl_y[bl_k])) ||
        (bl_state[bl_k]==BL_HORZ && car_rect_h(bl_x[bl_k], bl_y[bl_k])) ||
        (bl_state[bl_k]==BL_ARC  && bl_hit_valid[bl_k] && bl_hit_rot[bl_k]);
    end
  end
end
wire car_hit_BL_left_arr = |bl_hit_array;


//======================================================================
// ??????????????????????????????????????????????????
// ?????????"?????/?????????????????????"?????
//======================================================================
localparam [15:0] PURPLE = 16'hF81F; // ???
localparam [15:0] GREY  = 16'h7BEF;  // ???????????
localparam [15:0] ORANGE = 16'hFD80; // 浅橙�? (R=31, G=44, B=0)

// ======= 聚合：各方向斑马线上是否"当前有人" =======
reg ped_on_top, ped_on_bottom, ped_on_left, ped_on_right;
integer p_i;

always @* begin
  ped_on_top    = 1'b0;
  ped_on_bottom = 1'b0;
  ped_on_left   = 1'b0;
  ped_on_right  = 1'b0;

  for (p_i=0; p_i<MAX_PED; p_i=p_i+1) begin
    if (p_i < PED_N) begin
      if (hit_top_idx(p_i))    ped_on_top    = 1'b1;
      if (hit_bot_idx(p_i))    ped_on_bottom = 1'b1;
      if (hit_left_idx(p_i))   ped_on_left   = 1'b1;
      if (hit_right_idx(p_i))  ped_on_right  = 1'b1;
    end
  end
end

// "行人总数<5"的条件：你现�?? PED_N=4 �?? 10（二选一�??
wire ped_small_total = (PED_N < 5);

    // ?????????????????????
    always @(posedge vga_clk or negedge sys_rst_n) begin
      if(!sys_rst_n) begin
        pix_cross <= 16'h0000;
      end else begin
        // ???????????? ROM ???????????????????????????????
        pix_cross <= (USE_BG) ? BG_RGB : 16'h0000;
    // =====================================================
    // 2) ????????????/????/???/???/???????
    //    ?? USE_BG=1????ROM????????????????????????????
    // =====================================================
    if (!USE_BG) begin
    
      // -- ???????????????????????????--
      if (h_dash_on_core) pix_cross <= YEL;
      if (v_dash_on_core) pix_cross <= YEL;

      // -- ??????????????????????????????--
      if (lane_h_u1 | lane_h_u2 | lane_h_d1 | lane_h_d2
       |  lane_v_lsep1 | lane_v_lsep2 | lane_v_rsep1 | lane_v_rsep2)
        pix_cross <= WHITE;

      // -- ???????? -- 
      if (edge_top_L | edge_top_R | edge_bot_L | edge_bot_R
       |  edge_left_T | edge_left_B | edge_right_T | edge_right_B)
        pix_cross <= WHITE;

      // -- ??????? -- 
      if (c_tl_arc | c_tr_arc | c_bl_arc | c_br_arc)
        pix_cross <= WHITE;

      // -- ???????????????????????/???????--
      if (zb_top | zb_bot | zb_left | zb_right)
        pix_cross <= WHITE;
    end

    // 车辆绘制（无论是否USE_BG都绘制）
    if ( car_hit_TS_down_arr
       | car_hit_TL_leftturn_arr
       | car_hit_TR_always_arr )
      pix_cross <= PURPLE;
    
    // ==== R→L方向车辆 ====
    if ( car_hit_r2l_straight_arr
       | car_hit_rturn_R_arr
       | car_hit_lturn_R_arr )
      pix_cross <= BLUE;
      
    // ==== XR方向车辆 ====
    if ( car_hit_BS_up_arr
       | car_hit_BR_turn_arr
       | car_hit_BL_left_arr )
      pix_cross <= GREY;
    
    // ==== 主要车辆（直�?/转向�?====
    if ( car_hit_rturn_arr
       |  car_hit_lturn_arr
       |  car_hit_straight_any )
      pix_cross <= ORANGE;   // 橙色车辆

    // ==== 信号灯边�? ====
    if (lightL_border) pix_cross <= WHITE;
    if (lightR_border) pix_cross <= WHITE;
    if (lightT_border) pix_cross <= WHITE;
    if (lightB_border) pix_cross <= WHITE;
    
    // ==== 信号灯显�? ====
    // L方向信号�?
    if (L_S_rect && col_LR_S==C_GRN) pix_cross <= COL_GRN;
    if (L_S_rect && col_LR_S==C_YEL) pix_cross <= COL_YEL;
    if (L_S_rect && col_LR_S==C_RED) pix_cross <= COL_RED;
    
    if (L_L_rect && col_LR_L==C_GRN) pix_cross <= COL_GRN;
    if (L_L_rect && col_LR_L==C_YEL) pix_cross <= COL_YEL;
    if (L_L_rect && col_LR_L==C_RED) pix_cross <= COL_RED;
    
    if (L_R_rect && col_LR_R==C_GRN) pix_cross <= COL_GRN;
    if (L_R_rect && col_LR_R==C_YEL) pix_cross <= COL_YEL;
    if (L_R_rect && col_LR_R==C_RED) pix_cross <= COL_RED;
    
    // R方向信号�?
    if (R_R_rect && col_LR_R==C_GRN) pix_cross <= COL_GRN;
    if (R_R_rect && col_LR_R==C_YEL) pix_cross <= COL_YEL;
    if (R_R_rect && col_LR_R==C_RED) pix_cross <= COL_RED;
    
    if (R_L_rect && col_LR_S==C_GRN) pix_cross <= COL_GRN;
    if (R_L_rect && col_LR_S==C_YEL) pix_cross <= COL_YEL;
    if (R_L_rect && col_LR_S==C_RED) pix_cross <= COL_RED;
    
    if (R_S_rect && col_LR_L==C_GRN) pix_cross <= COL_GRN;
    if (R_S_rect && col_LR_L==C_YEL) pix_cross <= COL_YEL;
    if (R_S_rect && col_LR_L==C_RED) pix_cross <= COL_RED;
    
    // T方向信号�?
    if (T_R_rect && col_TB_R==C_GRN) pix_cross <= COL_GRN;
    if (T_R_rect && col_TB_R==C_YEL) pix_cross <= COL_YEL;
    if (T_R_rect && col_TB_R==C_RED) pix_cross <= COL_RED;
    
    if (T_L_rect && col_TB_S==C_GRN) pix_cross <= COL_GRN;
    if (T_L_rect && col_TB_S==C_YEL) pix_cross <= COL_YEL;
    if (T_L_rect && col_TB_S==C_RED) pix_cross <= COL_RED;
    
    if (T_S_rect && col_TB_L==C_GRN) pix_cross <= COL_GRN;
    if (T_S_rect && col_TB_L==C_YEL) pix_cross <= COL_YEL;
    if (T_S_rect && col_TB_L==C_RED) pix_cross <= COL_RED;
    
    // B方向信号�?
    if (B_S_rect && col_TB_L==C_GRN) pix_cross <= COL_GRN;
    if (B_S_rect && col_TB_L==C_YEL) pix_cross <= COL_YEL;
    if (B_S_rect && col_TB_L==C_RED) pix_cross <= COL_RED;
    
    if (B_L_rect && col_TB_S==C_GRN) pix_cross <= COL_GRN;
    if (B_L_rect && col_TB_S==C_YEL) pix_cross <= COL_YEL;
    if (B_L_rect && col_TB_S==C_RED) pix_cross <= COL_RED;
    
    if (B_R_rect && col_TB_R==C_GRN) pix_cross <= COL_GRN;
    if (B_R_rect && col_TB_R==C_YEL) pix_cross <= COL_YEL;
    if (B_R_rect && col_TB_R==C_RED) pix_cross <= COL_RED;
    
    // ==== 路口大型信号�? ====
    // ZT
    if (col_LR_S==C_GRN && is_rect(ZT_G_x0, ZT_G_x1, ZT_G_y0, ZT_G_y1)) pix_cross <= COL_GRN;
    if (col_LR_S==C_YEL && is_rect(ZT_Y_x0, ZT_Y_x1, ZT_Y_y0, ZT_Y_y1)) pix_cross <= COL_YEL;
    if (col_LR_S==C_RED && is_rect(ZT_R_x0, ZT_R_x1, ZT_R_y0, ZT_R_y1)) pix_cross <= COL_RED;
    
    // ZB
    if (col_LR_S==C_GRN && is_rect(ZB_G_x0, ZB_G_x1, ZB_G_y0, ZB_G_y1)) pix_cross <= COL_GRN;
    if (col_LR_S==C_YEL && is_rect(ZB_Y_x0, ZB_Y_x1, ZB_Y_y0, ZB_Y_y1)) pix_cross <= COL_YEL;
    if (col_LR_S==C_RED && is_rect(ZB_R_x0, ZB_R_x1, ZB_R_y0, ZB_R_y1)) pix_cross <= COL_RED;
    
    // ZL
    if (col_TB_S==C_GRN && is_rect(ZL_G_x0, ZL_G_x1, ZL_G_y0, ZL_G_y1)) pix_cross <= COL_GRN;
    if (col_TB_S==C_YEL && is_rect(ZL_Y_x0, ZL_Y_x1, ZL_Y_y0, ZL_Y_y1)) pix_cross <= COL_YEL;
    if (col_TB_S==C_RED && is_rect(ZL_R_x0, ZL_R_x1, ZL_R_y0, ZL_R_y1)) pix_cross <= COL_RED;
    
    // ZR
    if (col_TB_S==C_GRN && is_rect(ZR_G_x0, ZR_G_x1, ZR_G_y0, ZR_G_y1)) pix_cross <= COL_GRN;
    if (col_TB_S==C_YEL && is_rect(ZR_Y_x0, ZR_Y_x1, ZR_Y_y0, ZR_Y_y1)) pix_cross <= COL_YEL;
    if (col_TB_S==C_RED && is_rect(ZR_R_x0, ZR_R_x1, ZR_R_y0, ZR_R_y1)) pix_cross <= COL_RED;
    
    // ==== 信号灯盒边框 ====
    if (is_rect(ZT_x0, ZT_x1, ZT_y0, ZT_y0 + ZBOX_BORDER_T)) pix_cross <= WHITE;
    if (is_rect(ZT_x0, ZT_x1, ZT_y1 - ZBOX_BORDER_T, ZT_y1)) pix_cross <= WHITE;
    if (is_rect(ZT_x0, ZT_x0 + ZBOX_BORDER_T, ZT_y0, ZT_y1)) pix_cross <= WHITE;
    if (is_rect(ZT_x1 - ZBOX_BORDER_T, ZT_x1, ZT_y0, ZT_y1)) pix_cross <= WHITE;
    
    if (is_rect(ZB_x0, ZB_x1, ZB_y0, ZB_y0 + ZBOX_BORDER_T)) pix_cross <= WHITE;
    if (is_rect(ZB_x0, ZB_x1, ZB_y1 - ZBOX_BORDER_T, ZB_y1)) pix_cross <= WHITE;
    if (is_rect(ZB_x0, ZB_x0 + ZBOX_BORDER_T, ZB_y0, ZB_y1)) pix_cross <= WHITE;
    if (is_rect(ZB_x1 - ZBOX_BORDER_T, ZB_x1, ZB_y0, ZB_y1)) pix_cross <= WHITE;
    
    if (is_rect(ZL_x0, ZL_x1, ZL_y0, ZL_y0 + ZBOX_BORDER_T)) pix_cross <= WHITE;
    if (is_rect(ZL_x0, ZL_x1, ZL_y1 - ZBOX_BORDER_T, ZL_y1)) pix_cross <= WHITE;
    if (is_rect(ZL_x0, ZL_x0 + ZBOX_BORDER_T, ZL_y0, ZL_y1)) pix_cross <= WHITE;
    if (is_rect(ZL_x1 - ZBOX_BORDER_T, ZL_x1, ZL_y0, ZL_y1)) pix_cross <= WHITE;
    
    if (is_rect(ZR_x0, ZR_x1, ZR_y0, ZR_y0 + ZBOX_BORDER_T)) pix_cross <= WHITE;
    if (is_rect(ZR_x0, ZR_x1, ZR_y1 - ZBOX_BORDER_T, ZR_y1)) pix_cross <= WHITE;
    if (is_rect(ZR_x0, ZR_x0 + ZBOX_BORDER_T, ZR_y0, ZR_y1)) pix_cross <= WHITE;
    if (is_rect(ZR_x1 - ZBOX_BORDER_T, ZR_x1, ZR_y0, ZR_y1)) pix_cross <= WHITE;
    
    // ==== 文字信息显示 ====
    if (mode_label_on) pix_cross <= WHITE;
    if (people_label_on) pix_cross <= WHITE;
    if (people_num_on) pix_cross <= WHITE;
    if (lrcar_label_on) pix_cross <= WHITE;
    if (lrcar_num_on) pix_cross <= WHITE;
    if (tbcar_label_on) pix_cross <= WHITE;
    if (tbcar_num_on) pix_cross <= WHITE;
    if (speed_label_on) pix_cross <= WHITE;
    if (speed_num_on) pix_cross <= WHITE;
    
    // ==== 绿灯时长显示 ====
    if (tb_green_label_on) pix_cross <= WHITE;
    if (tb_s_label_on) pix_cross <= WHITE;
    if (tb_s_num_on) pix_cross <= WHITE;
    if (tb_l_label_on) pix_cross <= WHITE;
    if (tb_l_num_on) pix_cross <= WHITE;
    if (tb_r_label_on) pix_cross <= WHITE;
    if (tb_r_num_on) pix_cross <= WHITE;
    if (lr_green_label_on) pix_cross <= WHITE;
    if (lr_s_label_on) pix_cross <= WHITE;
    if (lr_s_num_on) pix_cross <= WHITE;
    if (lr_l_label_on) pix_cross <= WHITE;
    if (lr_l_num_on) pix_cross <= WHITE;
    if (lr_r_label_on) pix_cross <= WHITE;
    if (lr_r_num_on) pix_cross <= WHITE;
    
    // ==== 倒计时已删除 ====
    
    // ==== 行人显示 ====
    if (ped_hit) pix_cross <= 16'hFFFF;
        end
    end
endmodule                                     