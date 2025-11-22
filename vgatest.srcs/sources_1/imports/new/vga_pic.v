// vga_pic.v - 锟斤拷锟斤拷位锟斤拷色锟斤拷锟斤拷锟斤拷位锟斤拷+ 锟斤拷锟斤拷锟斤拷锟斤拷 + 锟洁车锟斤拷锟斤拷锟斤拷 + 锟斤拷瓶锟酵Ｖ癸拷锟酵ｏ拷锟?? + 状态色锟斤拷 + 模式色锟斤拷 + 锟斤拷锟斤拷锟斤拷(people_count)
`timescale 1ns/1ps
module vga_pic(
    input  wire        vga_clk,
    input  wire        sys_rst_n,
    input  wire [9:0]  pix_x,
    input  wire [9:0]  pix_y,
    // 锟斤拷锟斤拷锟侥碉拷 = 锟斤拷路锟斤拷
    input  wire        main_R,
    input  wire        main_Y,
    input  wire        main_G,
    // 锟斤拷锟剿匡拷锟侥碉拷 = 锟斤拷路锟斤拷
    input  wire        side_R,
    input  wire        side_Y,
    input  wire        side_G,
    // 锟斤拷锟斤拷锟斤拷位/锟斤拷锟斤拷
    input  wire [9:0]  car_x,
    input  wire [9:0]  ped_y,
    // 锟狡匡拷状态锟斤拷锟斤拷锟较斤拷16x16小色锟介）
    input  wire [2:0]  tl_state,

    // --- 锟斤拷锟斤拷锟斤拷锟狡ｏ拷锟斤拷锟皆讹拷锟斤拷/模式映锟戒） ---
    input  wire [7:0]  car_count_up,      // 锟斤拷锟叫筹拷锟斤拷锟斤拷 (0..N_UP)
    input  wire [7:0]  car_count_down,    // 锟斤拷锟叫筹拷锟斤拷锟斤拷 (0..N_DN)
    input  wire [7:0]  people_count,      // 锟斤拷锟斤拷锟斤拷 (0..MAX_PED)
    input  wire [7:0]  walk_time_sec,     // 锟斤拷前杩囪璁℃椂锛堢锟??
    input  wire [3:0]  eff_tens,          // 效锟斤拷十位 (0~9)
    input  wire [3:0]  eff_ones,          // 效锟斤拷锟斤拷位 (0~9)
    input  wire [3:0]  eff_frac,          // 效锟斤拷小位 (0~9)

    // 模式锟脚ｏ拷锟斤拷锟斤拷锟斤拷锟较斤拷16x16色锟斤拷锟斤拷示锟斤拷0=LOW 1=NORMAL 2=MANY锟斤拷
    input  wire [1:0]  modenum,

    // 锟斤拷选锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷"时锟斤拷锟斤拷位"锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷也锟斤拷锟节憋拷锟侥硷拷锟斤拷锟斤拷? localparam锟斤拷
    input  wire [9:0]  ped_phase_step,
        // 高峰模式固定绿灯时间（单位：秒），由 traffic_ctrl_adaptive 传进来
    input  wire [7:0]  peak_main_green_s, // 当前模式下主路绿灯固定时长
    input  wire [7:0]  peak_side_green_s, // 当前模式下人行道/支路绿灯固定时长
    output reg  [15:0] pix_data
);
    // 锟斤拷锟斤拷色
    localparam BLACK = 16'h0000, WHITE=16'hFFFF, YELLOW=16'hFFE0, RED=16'hF800, GREEN=16'h07E0;
    
    // 模式编码：0=早高峰 1=正常期间 2=晚高峰
    localparam [1:0] MORNING_PEAK = 2'd0;
    localparam [1:0] NORMAL       = 2'd1;
    localparam [1:0] EVENING_PEAK = 2'd2;

    // 锟街憋拷锟斤拷
    localparam H_VALID=10'd640, V_VALID=10'd480;

    // 锟斤拷路锟斤拷锟斤拷锟斤拷
    localparam Y_TOP=10'd190, Y_BOTTOM=10'd310;
    localparam SIDE_H=10'd3, MID_H=10'd2;

    // ---- 4 车道建模：把 (Y_TOP~Y_BOTTOM) 均分成 4 份 ----
    localparam [9:0] ROAD_H    = Y_BOTTOM - Y_TOP;           // 总路面高度 120
    localparam [9:0] LANE_STEP = ROAD_H >> 2;                // 每个车道高度 = 120/4 = 30
    localparam [9:0] Y_MID     = (Y_TOP + Y_BOTTOM) >> 1;    // 中心线（2/4 位置）

    // 3 条车道分界线（从上到下）：
    // 顶边 Y_TOP
    localparam [9:0] Y_LANE1   = Y_TOP + LANE_STEP;          // 1/4 位置
    localparam [9:0] Y_LANE2   = Y_TOP + (LANE_STEP << 1);   // 2/4 位置 = Y_MID
    localparam [9:0] Y_LANE3   = Y_TOP + (LANE_STEP * 3);    // 3/4 位置
    // 底边 Y_BOTTOM

    localparam H_DASH_PERIOD=10'd64, H_DASH_ON=10'd24;


    // 锟斤拷路锟斤拷锟斤拷直锟斤拷
    localparam X_LEFT = 10'd300, X_RIGHT=10'd340;
    localparam V_SIDE_W = 10'd3;
    localparam X_MID  = (X_LEFT + X_RIGHT) >> 1;
    localparam V_DASH_PERIOD = 10'd64, V_DASH_ON = 10'd24;
    // 斑马线参数：在人行道内部画一条条横向白条
    localparam [9:0] ZEBRA_X_L    = X_LEFT  + 10'd4;  // 内缩一点，别贴边
    localparam [9:0] ZEBRA_X_R    = X_RIGHT - 10'd4;
    localparam [9:0] ZEBRA_Y_TOP  = Y_TOP;
    localparam [9:0] ZEBRA_Y_BOT  = Y_BOTTOM;
    localparam [9:0] ZEBRA_STRIPE_H = 10'd4;   // 每条白条高度
    localparam [9:0] ZEBRA_PERIOD   = 10'd12;  // 条之间间隔周期

    // 锟洁车锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟诫）
    localparam integer N_UP = 15, N_DN = 15;
    localparam integer SP_BASE = 96;
    localparam [9:0]  CAR_W=10'd20, CAR_H=10'd12;
    localparam [9:0]  UP_Y_MIN = Y_TOP + 10'd6, UP_Y_MAX = Y_MID - 10'd10;
    localparam [9:0]  DN_Y_MIN = Y_MID + 10'd4, DN_Y_MAX = Y_BOTTOM - 10'd8;

    // ==== 四条车道的中心线（4 个车道真正均分 + 居中）====
    // 整个路面高度：ROAD_H = Y_BOTTOM - Y_TOP
    // 每条车道高度：LANE_STEP = ROAD_H / 4
    // 四个车道区间：
    //   1: [Y_TOP   .. Y_LANE1]
    //   2: [Y_LANE1 .. Y_LANE2]
    //   3: [Y_LANE2 .. Y_LANE3]
    //   4: [Y_LANE3 .. Y_BOTTOM]
    // 各自中心 = 区间起点 + LANE_STEP/2

    // 上半部分两条车道（作为上行车道的中心线）
    localparam [9:0] UP_LANE0_Y = Y_TOP   + (LANE_STEP >> 1);   // 第一条（最上）
    localparam [9:0] UP_LANE1_Y = Y_LANE1 + (LANE_STEP >> 1);   // 第二条

    // 下半部分两条车道（作为下行车道的中心线）
    localparam [9:0] DN_LANE0_Y = Y_LANE2 + (LANE_STEP >> 1);   // 第三条
    localparam [9:0] DN_LANE1_Y = Y_LANE3 + (LANE_STEP >> 1);   // 第四条（最下）


    // 停止线提前一点
    localparam [9:0] STOP_X_L = 10'd275; // 原 280，向"上游"提前 16 像素
    localparam [9:0] STOP_X_R = 10'd365; // 原 360，对称移动
    localparam [9:0] NEAR_WIN = 10'd10;  // 原 28，窗口缩小
    // 车排队时，车头之间的水平间隔
    localparam [9:0] QUEUE_GAP = 10'd4;  // 可以之后再微调


    // 锟斤拷锟斤拷锟叫讹拷
    wire in_area   = (pix_x < H_VALID) && (pix_y < V_VALID);
    wire top_side  = (pix_y >= (Y_TOP    - SIDE_H)) && (pix_y <= (Y_TOP    + SIDE_H));
    wire bot_side  = (pix_y >= (Y_BOTTOM - SIDE_H)) && (pix_y <= (Y_BOTTOM + SIDE_H));
    wire left_side = (pix_x >= (X_LEFT   - V_SIDE_W)) && (pix_x <= (X_LEFT   + V_SIDE_W));
    wire right_side= (pix_x >= (X_RIGHT  - V_SIDE_W)) && (pix_x <= (X_RIGHT  + V_SIDE_W));
    // 三条车道分界带：上/中/下
    wire h_lane1_band = (pix_y >= (Y_LANE1 - MID_H)) && (pix_y <= (Y_LANE1 + MID_H));
    wire h_mid_band   = (pix_y >= (Y_LANE2 - MID_H)) && (pix_y <= (Y_LANE2 + MID_H)); // 中间那条
    wire h_lane3_band = (pix_y >= (Y_LANE3 - MID_H)) && (pix_y <= (Y_LANE3 + MID_H));
    wire h_dash_on    = ((pix_x % H_DASH_PERIOD) < H_DASH_ON);
    
    wire in_zebra_zone =
        (pix_x >= ZEBRA_X_L) && (pix_x <= ZEBRA_X_R) &&
        (pix_y >= ZEBRA_Y_TOP) && (pix_y <= ZEBRA_Y_BOT);

    // 相对高度（只在 Y_TOP~Y_BOTTOM 有意义，外面 in_zebra_zone=0）
    wire [9:0] zebra_rel_y = pix_y - ZEBRA_Y_TOP;
    
    // 每 4 行算一个"band"：右移 2 位，相当于除以 4
    wire [4:0] zebra_band  = zebra_rel_y[6:2];
    
    // 让偶数 band 画白，奇数 band 不画（4 行白、4 行黑交替）
    wire zebra_on =
        in_zebra_zone &&
        (zebra_band[0] == 1'b0);

    
    wire v_mid_band= 1'b0;
    wire v_dash_on = 1'b0; 

    // 锟斤拷锟剿ｏ拷小圆锟斤拷
    localparam PED_R = 10'd5;

    // 锟斤拷直锟斤拷路锟节诧拷锟侥猴拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟??+2锟斤拷锟截帮拷全锟竭ｏ拷
    localparam [9:0] PED_MARGIN = PED_R + 10'd2;
    localparam [9:0] INNER_L = X_LEFT  + PED_MARGIN;
    localparam [9:0] INNER_R = X_RIGHT - PED_MARGIN;
    localparam [9:0] INNER_W = INNER_R - INNER_L;
    // 锟斤拷锟斤拷"锟斤拷锟揭摆讹拷"锟侥猴拷锟斤拷锟斤拷锟角诧拷锟斤拷锟斤拷锟斤拷锟皆革拷锟矫ｏ拷
    wire [5:0] s      = ped_y[5:0];                 // 0..63
    wire [4:0] tri_v  = s[5] ? (5'd31 - s[4:0])     // 0..31..0
                             :  s[4:0];
    wire [10:0] mult  = tri_v * INNER_W;            // 锟斤拷 31 * (INNER_W)
    wire [9:0]  offset= (mult + 5'd15) >> 5;        // 约锟斤拷锟斤拷 /32
    wire [9:0]  PED_X_dyn = INNER_L + offset;       // 圆锟斤拷X锟斤拷锟斤拷锟剿参匡拷锟斤拷

    // 锟斤拷锟斤拷圆锟叫讹拷锟斤拷锟轿匡拷锟芥）
    wire signed [11:0] dx_ref = $signed({1'b0,pix_x}) - $signed({1'b0,PED_X_dyn});
    wire signed [11:0] dy_ref = $signed({1'b0,pix_y})  - $signed({1'b0,ped_y});

    // ===== 锟斤拷锟斤拷位锟斤拷锟斤拷挪位锟矫ｏ拷=====
    localparam LAMP_W = 10'd14, LAMP_H = 10'd14;

    // 锟斤拷锟斤拷锟侥灯ｏ拷锟斤拷锟狡ｏ拷锟斤拷路锟叫硷拷锟斤拷锟斤拷锟??
    localparam CAR_LAMP_X0 = (H_VALID>>1) - (LAMP_W>>1);  // 水平锟斤拷锟斤拷
    localparam CAR_LAMP_X1 = CAR_LAMP_X0 + LAMP_W;
    localparam CAR_LAMP_Y0 = Y_MID - (LAMP_H>>1);  // 锟斤拷路锟叫硷拷?
    localparam CAR_LAMP_Y1 = CAR_LAMP_Y0 + LAMP_H;

    // 锟斤拷锟剿匡拷锟侥灯ｏ拷锟斤拷疲锟斤拷锟斤拷锟斤拷锟斤拷械锟斤拷也啵伙拷霉潭锟??
    localparam PED_LAMP_X0 = X_RIGHT + 10'd6;
    localparam PED_LAMP_X1 = PED_LAMP_X0 + LAMP_W;
    localparam PED_LAMP_Y0 = Y_TOP - 10'd20;
    localparam PED_LAMP_Y1 = PED_LAMP_Y0 + LAMP_H;

    wire inLampCar = (pix_x>=CAR_LAMP_X0 && pix_x<CAR_LAMP_X1) && (pix_y>=CAR_LAMP_Y0 && pix_y<CAR_LAMP_Y1);
    wire inLampPed = (pix_x>=PED_LAMP_X0 && pix_x<PED_LAMP_X1) && (pix_y>=PED_LAMP_Y0 && pix_y<PED_LAMP_Y1);

    // 锟斤拷色锟斤拷锟斤拷锟斤拷锟斤拷锟饺硷拷 G>Y>R锟斤拷
    function [15:0] color_from_main; input mr,my,mg;
        begin
            if(mg)      color_from_main = GREEN;
            else if(my) color_from_main = YELLOW;
            else        color_from_main = RED;
        end
    endfunction
    function [15:0] color_from_side; input sr,sy,sg;
        begin
            if(sg)      color_from_side = GREEN;
            else if(sy) color_from_side = YELLOW;
            else        color_from_side = RED;
        end
    endfunction

    // 锟洁车位锟矫ｏ拷锟斤拷锟斤拷锟斤拷 + 锟斤拷瓶锟酵Ｖ癸拷锟酵ｏ拷锟斤拷锟??
    function [9:0] wrap640; input [10:0] v; begin wrap640 = (v>=11'd640)? (v-11'd640) : v[9:0]; end endfunction
    function [9:0] wrap480; input [10:0] v; begin wrap480 = (v>=11'd480)? (v-11'd480) : v[9:0]; end endfunction

    reg [9:0] car_up_x [0:N_UP-1], car_up_y [0:N_UP-1];
    reg [9:0] car_dn_x [0:N_DN-1], car_dn_y [0:N_DN-1];
    
    reg [9:0] base_x_dn, base_x_up;  // 当前这辆车的"原始"x（未排队前）
    reg [1:0] lane_dn, lane_up;      // 这辆车在第几条车道(0..3)
    
    reg [10:0] acc_dn, acc_up;
    
    integer i, k;
    
    // 每条车道各自的排队计数（下行/上行各 4 条）
    integer q_dn0, q_dn1, q_dn2, q_dn3;
    integer q_up0, q_up1, q_up2, q_up3;

    function [10:0] dn_spacing; input integer idx; begin dn_spacing = (SP_BASE + ((idx*37 + 11) % 29)); end endfunction
    function [10:0] up_spacing; input integer idx; begin up_spacing = (SP_BASE + ((idx*31 +  7) % 27)); end endfunction
    // ==== 新版：把车分配到四条"离散车道"，再在车道里做一点小抖动 ====
    // idx 奇偶控制车道：0/2/4/... 用 lane0，1/3/5/... 用 lane1
    // ==== 统一当成 4 条车道（从上到下 0/1/2/3）====
    // lane_idx = 0 -> 1 号车道（最上面，原 UP_LANE0_Y）
    // lane_idx = 1 -> 2 号车道（原 UP_LANE1_Y）
    // lane_idx = 2 -> 3 号车道（原 DN_LANE0_Y）
    // lane_idx = 3 -> 4 号车道（最下面，原 DN_LANE1_Y）

    // 根据 lane_idx 取得中心 Y 并加一点抖动
    // 根据 lane_idx 取得该车道的中心 Y，把车矩形竖直居中到这条线（不抖动版）
    function [9:0] lane_yoff;
        input integer idx;       // 第几辆车（现在不用抖动，可以不关心）
        input [1:0] lane_idx;    // 0~3 -> 第几条车道
        reg   [9:0] base_y;
    begin
        case(lane_idx)
            2'd0: base_y = UP_LANE0_Y;   // 1 号车道中心
            2'd1: base_y = UP_LANE1_Y;   // 2 号车道中心
            2'd2: base_y = DN_LANE0_Y;   // 3 号车道中心
            default: base_y = DN_LANE1_Y; // 4 号车道中心
        endcase
    
        // 车高 CAR_H=12，把车矩形顶边放在 base_y - CAR_H/2，这样车中心就在车道中心线上
        lane_yoff = base_y - (CAR_H >> 1);  // 没有 jitter，完全不抖
    end
    endfunction


    // 根据模式，决定"右->左 (up) 的车"用哪几条车道
    function [1:0] up_lane_idx;
        input integer idx;     // 第几辆 up 车
        input [1:0] mode;      // 当前模式
    begin
        case (mode)
            MORNING_PEAK: begin
                // 早高峰：1 号车道右->左，其它 2/3/4 号车道都是左->右
                up_lane_idx = 2'd0;   // 全部 up 车都放在 lane0（1 号车道）
            end
            NORMAL: begin
                // 正常：1、2 号车道右->左
                up_lane_idx = (idx[0] == 1'b0) ? 2'd0 : 2'd1;  // 偶数车 lane1，上面；奇数车 lane2
            end
            default: begin // EVENING_PEAK
                // 晚高峰：1、2、3 号车道右->左
                case (idx % 3)
                    0: up_lane_idx = 2'd0; // 1 号车道
                    1: up_lane_idx = 2'd1; // 2 号车道
                    default: up_lane_idx = 2'd2; // 3 号车道
                endcase
            end
        endcase
    end
    endfunction

    // 根据模式，决定"左->右 (down) 的车"用哪几条车道
    function [1:0] dn_lane_idx;
        input integer idx;     // 第几辆 down 车
        input [1:0] mode;
    begin
        case (mode)
            MORNING_PEAK: begin
                // 早高峰：2、3、4 号车道左->右
                case (idx % 3)
                    0: dn_lane_idx = 2'd1; // 2 号车道
                    1: dn_lane_idx = 2'd2; // 3 号车道
                    default: dn_lane_idx = 2'd3; // 4 号车道
                endcase
            end
            NORMAL: begin
                // 正常：3、4 号车道左->右
                dn_lane_idx = (idx[0] == 1'b0) ? 2'd2 : 2'd3;  // 偶数车 lane3，奇数车 lane4
            end
            default: begin // EVENING_PEAK
                // 晚高峰：只有 4 号车道左->右
                dn_lane_idx = 2'd3;
            end
        endcase
    end
    endfunction
    
// *********** 修改后：每条车道自己排队、不重叠 ***********
always @* begin
    // ========== 下行车：左 -> 右，在 STOP_X_L 前排队 ==========
    acc_dn = 11'd0;
    q_dn0  = 0;
    q_dn1  = 0;
    q_dn2  = 0;
    q_dn3  = 0;

    for(i=0;i<N_DN;i=i+1) begin
        // 1) 原始流动位置（没排队前）
        base_x_dn = wrap640({1'b0,car_x} + acc_dn);
        lane_dn   = dn_lane_idx(i, modenum);   // 这辆车在第几条车道(0..3)
        car_dn_y[i] = lane_yoff(i, lane_dn);   // 对应车道的 Y

        if (main_R) begin
            // 2) 只有接近停止线这一小段范围的车，才参与排队
            if ( (base_x_dn + CAR_W >  STOP_X_L - NEAR_WIN) &&
                 (base_x_dn + CAR_W <= STOP_X_L + NEAR_WIN) ) begin
                // 同一车道上的车各自排一条队，不互相盖住
                case(lane_dn)
                    2'd0: begin
                        car_dn_x[i] = STOP_X_L - CAR_W - (CAR_W + QUEUE_GAP)*q_dn0;
                        q_dn0 = q_dn0 + 1;
                    end
                    2'd1: begin
                        car_dn_x[i] = STOP_X_L - CAR_W - (CAR_W + QUEUE_GAP)*q_dn1;
                        q_dn1 = q_dn1 + 1;
                    end
                    2'd2: begin
                        car_dn_x[i] = STOP_X_L - CAR_W - (CAR_W + QUEUE_GAP)*q_dn2;
                        q_dn2 = q_dn2 + 1;
                    end
                    default: begin  // 2'd3
                        car_dn_x[i] = STOP_X_L - CAR_W - (CAR_W + QUEUE_GAP)*q_dn3;
                        q_dn3 = q_dn3 + 1;
                    end
                endcase
            end else begin
                // 还在离停止线比较远的地方，正常流动
                car_dn_x[i] = base_x_dn;
            end
        end else begin
            // 绿/黄灯：完全按正常流动
            car_dn_x[i] = base_x_dn;
        end

        acc_dn = acc_dn + dn_spacing(i);
    end

    // ========== 上行车：右 -> 左，在 STOP_X_R 前排队 ==========
    acc_up = 11'd40;
    q_up0  = 0;
    q_up1  = 0;
    q_up2  = 0;
    q_up3  = 0;

    for(i=0;i<N_UP;i=i+1) begin
        // 1) 原始流动位置
        base_x_up = 10'd639 - wrap640({1'b0,car_x} + acc_up);
        lane_up   = up_lane_idx(i, modenum);
        car_up_y[i] = lane_yoff(i, lane_up);

        if (main_R) begin
            // 2) 靠近右侧停止线的一小段范围参与排队
            if ( (base_x_up <  STOP_X_R + NEAR_WIN) &&
                 (base_x_up >= STOP_X_R - NEAR_WIN) ) begin
                case(lane_up)
                    2'd0: begin
                        car_up_x[i] = STOP_X_R + (CAR_W + QUEUE_GAP)*q_up0;
                        q_up0 = q_up0 + 1;
                    end
                    2'd1: begin
                        car_up_x[i] = STOP_X_R + (CAR_W + QUEUE_GAP)*q_up1;
                        q_up1 = q_up1 + 1;
                    end
                    2'd2: begin
                        car_up_x[i] = STOP_X_R + (CAR_W + QUEUE_GAP)*q_up2;
                        q_up2 = q_up2 + 1;
                    end
                    default: begin  // 2'd3
                        car_up_x[i] = STOP_X_R + (CAR_W + QUEUE_GAP)*q_up3;
                        q_up3 = q_up3 + 1;
                    end
                endcase
            end else begin
                // 还没到停止线附近，正常跑
                car_up_x[i] = base_x_up;
            end
        end else begin
            // 绿/黄灯：正常流
            car_up_x[i] = base_x_up;
        end

        acc_up = acc_up + up_spacing(i);
    end
end





    // -- 锟斤拷锟较角ｏ拷锟斤拷通锟斤拷状态小色锟介（16x16锟斤拷 --
    // 璋冭瘯灏忔柟鍧楀凡绉婚櫎

    // ================= 屑锟?? =================
function [0:0] diamond_hit_xy;
  input [9:0] cx, cy; reg [10:0] dx, dy, md;
  begin
    dx = (pix_x > cx) ? (pix_x - cx) : (cx - pix_x);
    dy = (pix_y > cy) ? (pix_y - cy) : (cy - pix_y);
    md = dx + dy;
    // PED_R 锟斤拷 10 位锟斤拷锟斤拷锟斤拷md 11 位锟斤拷锟斤拷锟斤拷冉锟?? OK锟斤拷锟桔猴拷为锟斤拷锟斤拷展锟斤拷
    diamond_hit_xy = (md <= PED_R);
  end
endfunction
    // -- 锟斤拷锟斤拷锟剿ｏ拷锟斤拷"锟斤拷锟斤拷锟竭凤拷"锟斤拷珊锟斤拷锟?? + for 锟桔猴拷 --
    localparam integer MAX_PED = 32; // 锟斤拷源锟斤拷锟睫ｏ拷锟缴帮拷锟斤拷锟斤拷锟??

// === 锟斤拷锟斤拷锟剿ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟叫ｏ拷锟芥换原 is_ped_circle_for_phase锟斤拷===
function [0:0] is_ped_circle_for_phase;   // 锟斤拷锟街可憋拷锟斤拷锟斤拷锟侥讹拷锟斤拷锟矫达拷
    input [9:0] phase;               // 锟斤拷锟斤拷说锟绞憋拷锟斤拷锟轿黄拷锟??
    reg  [9:0] ped_y_i;
    reg  [5:0] s_i;
    reg  [4:0] tri_i;
    reg  [10:0] mult_i;
    reg  [9:0]  off_i;
    reg  [9:0]  ped_x_i;
begin
    // 1) y = 原始 ped_y + 锟斤拷位锟斤拷锟斤拷锟斤拷 0..479锟斤拷
    ped_y_i = wrap480({1'b0,ped_y} + phase);

    // 2) x = 锟斤拷锟角诧拷锟斤拷锟?? 锟斤拷 [INNER_L..INNER_R]
    s_i     = ped_y_i[5:0];
    tri_i   = s_i[5] ? (5'd31 - s_i[4:0]) : s_i[4:0];
    mult_i  = tri_i * INNER_W;
    off_i   = (mult_i + 5'd15) >> 5;
    ped_x_i = INNER_L + off_i;

    // 3) 锟斤拷锟斤拷锟斤拷锟叫ｏ拷|dx|+|dy| <= PED_R
    is_ped_circle_for_phase = diamond_hit_xy(ped_x_i, ped_y_i);
end
endfunction

    reg inPedAny;
    integer pi;
    always @* begin
        inPedAny = 1'b0;
        for (pi = 0; pi < MAX_PED; pi = pi + 1) begin
            if (pi < people_count) begin
                if (is_ped_circle_for_phase(pi * ped_phase_step))
                    inPedAny = 1'b1;
            end
        end
    end

    // ===================== 字符显示：行人数、车数、walktime、eff、模式 =====================
    // 显示位置（左上角为主）
    localparam CHAR_W       = 8;
    localparam CHAR_H       = 8;
    localparam SPACE_W      = 1;
    localparam TEXT_X       = 10'd8;
    localparam TEXT_PEOPLE_Y= 10'd32;
    localparam TEXT_CAR_Y   = TEXT_PEOPLE_Y + CHAR_H + 2;
    localparam TEXT_WALK_Y  = TEXT_CAR_Y    + CHAR_H + 2;
    localparam TEXT_EFF_Y   = TEXT_WALK_Y   + CHAR_H + 2;
    
    // 计算车辆总数
    wire [7:0] car_total = car_count_up + car_count_down;

    // -------------------- "people:" 标签 --------------------
    localparam TEXT_PEOPLE_LEN = 7;
    wire in_people_label =
        (pix_x >= TEXT_X) &&
        (pix_x <  TEXT_X + TEXT_PEOPLE_LEN*(CHAR_W+SPACE_W)) &&
        (pix_y >= TEXT_PEOPLE_Y) &&
        (pix_y <  TEXT_PEOPLE_Y + CHAR_H);

    wire [9:0] people_label_col      = pix_x - TEXT_X;
    wire [9:0] people_label_dy       = pix_y - TEXT_PEOPLE_Y;
    wire [2:0] people_label_row      = people_label_dy[2:0];
    wire [2:0] people_label_ch_idx   = people_label_col / (CHAR_W + SPACE_W);
    wire [3:0] people_label_col_mod  = people_label_col % (CHAR_W + SPACE_W);
    wire       people_label_in_space = (people_label_col_mod >= CHAR_W);
    wire [2:0] people_label_col_in_char = CHAR_W - 1 - people_label_col_mod[2:0];

    wire [8*7-1:0] str_people_pack = "people:";
    wire [7:0] people_label_ch_ascii =
        str_people_pack[8*(6-people_label_ch_idx) +: 8];

    wire [7:0] people_label_row_bits;
    font u_font_people_label(
        .ascii(people_label_ch_ascii),
        .row  (people_label_row),
        .bits (people_label_row_bits)
    );
    wire people_label_on =
        in_people_label && !people_label_in_space ?
        people_label_row_bits[people_label_col_in_char] : 1'b0;

    // people 数字（两位）
    localparam PEOPLE_NUM_X = TEXT_X + TEXT_PEOPLE_LEN*(CHAR_W+SPACE_W);
    wire [3:0] people_tens = people_count / 10;
    wire [3:0] people_ones = people_count % 10;

    wire in_people_num =
        (pix_x >= PEOPLE_NUM_X) &&
        (pix_x <  PEOPLE_NUM_X + 2*(CHAR_W+SPACE_W)) &&
        (pix_y >= TEXT_PEOPLE_Y) &&
        (pix_y <  TEXT_PEOPLE_Y + CHAR_H);

    wire [9:0] people_num_col      = pix_x - PEOPLE_NUM_X;
    wire [9:0] people_num_dy       = pix_y - TEXT_PEOPLE_Y;
    wire [2:0] people_num_row      = people_num_dy[2:0];
    wire [1:0] people_num_ch_idx   = people_num_col / (CHAR_W + SPACE_W);
    wire [3:0] people_num_col_mod  = people_num_col % (CHAR_W + SPACE_W);
    wire       people_num_in_space = (people_num_col_mod >= CHAR_W);
    wire [2:0] people_num_col_in_char = CHAR_W - 1 - people_num_col_mod[2:0];

    wire [7:0] people_num_ch_ascii =
        (people_num_ch_idx == 2'd0) ? (8'h30 + {1'b0, people_tens}) :
                                      (8'h30 + {1'b0, people_ones});

    wire [7:0] people_num_row_bits;
    font u_font_people_num(
        .ascii(people_num_ch_ascii),
        .row  (people_num_row),
        .bits (people_num_row_bits)
    );
    wire people_num_on =
        in_people_num && !people_num_in_space ?
        people_num_row_bits[people_num_col_in_char] : 1'b0;

    // -------------------- "car:" 标签 --------------------
    localparam TEXT_CAR_LEN = 4;
    wire in_car_label =
        (pix_x >= TEXT_X) &&
        (pix_x <  TEXT_X + TEXT_CAR_LEN*(CHAR_W+SPACE_W)) &&
        (pix_y >= TEXT_CAR_Y) &&
        (pix_y <  TEXT_CAR_Y + CHAR_H);

    wire [9:0] car_label_col      = pix_x - TEXT_X;
    wire [9:0] car_label_dy       = pix_y - TEXT_CAR_Y;
    wire [2:0] car_label_row      = car_label_dy[2:0];
    wire [2:0] car_label_ch_idx   = car_label_col / (CHAR_W + SPACE_W);
    wire [3:0] car_label_col_mod  = car_label_col % (CHAR_W + SPACE_W);
    wire       car_label_in_space = (car_label_col_mod >= CHAR_W);
    wire [2:0] car_label_col_in_char = CHAR_W - 1 - car_label_col_mod[2:0];

    wire [8*4-1:0] str_car_pack = "car:";
    wire [7:0] car_label_ch_ascii =
        str_car_pack[8*(3-car_label_ch_idx) +: 8];

    wire [7:0] car_label_row_bits;
    font u_font_car_label(
        .ascii(car_label_ch_ascii),
        .row  (car_label_row),
        .bits (car_label_row_bits)
    );
    wire car_label_on =
        in_car_label && !car_label_in_space ?
        car_label_row_bits[car_label_col_in_char] : 1'b0;

    // car 数字（两位）
    localparam CAR_NUM_X = TEXT_X + TEXT_CAR_LEN*(CHAR_W+SPACE_W);
    wire [3:0] car_tens = car_total / 10;
    wire [3:0] car_ones = car_total % 10;

    wire in_car_num =
        (pix_x >= CAR_NUM_X) &&
        (pix_x <  CAR_NUM_X + 2*(CHAR_W+SPACE_W)) &&
        (pix_y >= TEXT_CAR_Y) &&
        (pix_y <  TEXT_CAR_Y + CHAR_H);

    wire [9:0] car_num_col      = pix_x - CAR_NUM_X;
    wire [9:0] car_num_dy       = pix_y - TEXT_CAR_Y;
    wire [2:0] car_num_row      = car_num_dy[2:0];
    wire [1:0] car_num_ch_idx   = car_num_col / (CHAR_W + SPACE_W);
    wire [3:0] car_num_col_mod  = car_num_col % (CHAR_W + SPACE_W);
    wire       car_num_in_space = (car_num_col_mod >= CHAR_W);
    wire [2:0] car_num_col_in_char = CHAR_W - 1 - car_num_col_mod[2:0];

    wire [7:0] car_num_ch_ascii =
        (car_num_ch_idx == 2'd0) ? (8'h30 + {1'b0, car_tens}) :
                                   (8'h30 + {1'b0, car_ones});

    wire [7:0] car_num_row_bits;
    font u_font_car_num(
        .ascii(car_num_ch_ascii),
        .row  (car_num_row),
        .bits (car_num_row_bits)
    );
    wire car_num_on =
        in_car_num && !car_num_in_space ?
        car_num_row_bits[car_num_col_in_char] : 1'b0;

    // -------------------- "walktime:" 标签 --------------------
    localparam TEXT_WALK_LEN   = 9;
    localparam WALK_NUM_CHARS  = 3;
    localparam WALK_NUM_X      = TEXT_X + TEXT_WALK_LEN*(CHAR_W+SPACE_W);

    wire in_walk_label =
        (pix_x >= TEXT_X) &&
        (pix_x <  TEXT_X + TEXT_WALK_LEN*(CHAR_W+SPACE_W)) &&
        (pix_y >= TEXT_WALK_Y) &&
        (pix_y <  TEXT_WALK_Y + CHAR_H);

    wire [9:0] walk_label_col      = pix_x - TEXT_X;
    wire [9:0] walk_label_dy       = pix_y - TEXT_WALK_Y;
    wire [2:0] walk_label_row      = walk_label_dy[2:0];
    wire [3:0] walk_label_ch_idx   = walk_label_col / (CHAR_W + SPACE_W);
    wire [3:0] walk_label_col_mod  = walk_label_col % (CHAR_W + SPACE_W);
    wire       walk_label_in_space = (walk_label_col_mod >= CHAR_W);
    wire [2:0] walk_label_col_in_char = CHAR_W - 1 - walk_label_col_mod[2:0];

    wire [8*9-1:0] str_walk_pack = "walktime:";
    wire [7:0] walk_label_ch_ascii =
        str_walk_pack[8*(8-walk_label_ch_idx) +: 8];

    wire [7:0] walk_label_row_bits;
    font u_font_walk_label(
        .ascii(walk_label_ch_ascii),
        .row  (walk_label_row),
        .bits (walk_label_row_bits)
    );
    wire walk_label_on =
        in_walk_label && !walk_label_in_space ?
        walk_label_row_bits[walk_label_col_in_char] : 1'b0;

    // walktime 数字（0~99）
    wire [7:0] walk_time_clamped = (walk_time_sec > 99) ? 8'd99 : walk_time_sec;
    wire [7:0] walk_tens_val     = walk_time_clamped / 8'd10;
    wire [7:0] walk_ones_val     = walk_time_clamped % 8'd10;
    wire [3:0] walk_tens_digit   = walk_tens_val[3:0];
    wire [3:0] walk_ones_digit   = walk_ones_val[3:0];

    wire in_walk_num =
        (pix_x >= WALK_NUM_X) &&
        (pix_x <  WALK_NUM_X + WALK_NUM_CHARS*(CHAR_W+SPACE_W)) &&
        (pix_y >= TEXT_WALK_Y) &&
        (pix_y <  TEXT_WALK_Y + CHAR_H);

    wire [9:0] walk_num_col      = pix_x - WALK_NUM_X;
    wire [9:0] walk_num_dy       = pix_y - TEXT_WALK_Y;
    wire [2:0] walk_num_row      = walk_num_dy[2:0];
    wire [1:0] walk_num_ch_idx   = walk_num_col / (CHAR_W + SPACE_W);
    wire [3:0] walk_num_col_mod  = walk_num_col % (CHAR_W + SPACE_W);
    wire       walk_num_in_space = (walk_num_col_mod >= CHAR_W);
    wire [2:0] walk_num_col_in_char = CHAR_W - 1 - walk_num_col_mod[2:0];

    wire [7:0] walk_num_ch_ascii =
        (walk_num_ch_idx == 2'd0) ?
            ((walk_tens_digit == 4'd0) ? 8'h20 :
                                        (8'h30 + {4'b0000, walk_tens_digit})) :
        (walk_num_ch_idx == 2'd1) ?
            (8'h30 + {4'b0000, walk_ones_digit}) :
            8'h73; // 's'

    wire [7:0] walk_num_row_bits;
    font u_font_walk_num(
        .ascii(walk_num_ch_ascii),
        .row  (walk_num_row),
        .bits (walk_num_row_bits)
    );
    wire walk_num_on =
        in_walk_num && !walk_num_in_space ?
        walk_num_row_bits[walk_num_col_in_char] : 1'b0;

    // -------------------- "efficiency:" 标签 --------------------
    localparam TEXT_EFF_LEN   = 11;
    localparam EFF_NUM_CHARS  = 4;
    localparam EFF_NUM_X      = TEXT_X + TEXT_EFF_LEN*(CHAR_W+SPACE_W);

    wire in_eff_label =
        (pix_x >= TEXT_X) &&
        (pix_x <  TEXT_X + TEXT_EFF_LEN*(CHAR_W+SPACE_W)) &&
        (pix_y >= TEXT_EFF_Y) &&
        (pix_y <  TEXT_EFF_Y + CHAR_H);

    wire [9:0] eff_label_col      = pix_x - TEXT_X;
    wire [9:0] eff_label_dy       = pix_y - TEXT_EFF_Y;
    wire [2:0] eff_label_row      = eff_label_dy[2:0];
    wire [3:0] eff_label_ch_idx   = eff_label_col / (CHAR_W + SPACE_W);
    wire [3:0] eff_label_col_mod  = eff_label_col % (CHAR_W + SPACE_W);
    wire       eff_label_in_space = (eff_label_col_mod >= CHAR_W);
    wire [2:0] eff_label_col_in_char = CHAR_W - 1 - eff_label_col_mod[2:0];

    wire [8*11-1:0] str_eff_pack = "efficiency:";
    wire [7:0] eff_label_ch_ascii =
        str_eff_pack[8*(10-eff_label_ch_idx) +: 8];

    wire [7:0] eff_label_row_bits;
    font u_font_eff_label(
        .ascii(eff_label_ch_ascii),
        .row  (eff_label_row),
        .bits (eff_label_row_bits)
    );
    wire eff_label_on =
        in_eff_label && !eff_label_in_space ?
        eff_label_row_bits[eff_label_col_in_char] : 1'b0;

    // efficiency 数字（xx.x）
    wire [3:0] eff_int_tens   = eff_tens;
    wire [3:0] eff_int_ones   = eff_ones;
    wire [3:0] eff_frac_digit = eff_frac;

    wire in_eff_num =
        (pix_x >= EFF_NUM_X) &&
        (pix_x <  EFF_NUM_X + EFF_NUM_CHARS*(CHAR_W+SPACE_W)) &&
        (pix_y >= TEXT_EFF_Y) &&
        (pix_y <  TEXT_EFF_Y + CHAR_H);

    wire [9:0] eff_num_col      = pix_x - EFF_NUM_X;
    wire [9:0] eff_num_dy       = pix_y - TEXT_EFF_Y;
    wire [2:0] eff_num_row      = eff_num_dy[2:0];
    wire [1:0] eff_num_ch_idx   = eff_num_col / (CHAR_W + SPACE_W);
    wire [3:0] eff_num_col_mod  = eff_num_col % (CHAR_W + SPACE_W);
    wire       eff_num_in_space = (eff_num_col_mod >= CHAR_W);
    wire [2:0] eff_num_col_in_char = CHAR_W - 1 - eff_num_col_mod[2:0];

    wire [7:0] eff_num_ch_ascii =
        (eff_num_ch_idx == 2'd0) ?
            ((eff_int_tens == 4'd0) ? 8'h20 :
                                      (8'h30 + {4'b0000, eff_int_tens})) :
        (eff_num_ch_idx == 2'd1) ?
            (8'h30 + {4'b0000, eff_int_ones}) :
        (eff_num_ch_idx == 2'd2) ?
            8'h2E :
            (8'h30 + {4'b0000, eff_frac_digit});

    wire [7:0] eff_num_row_bits;
    font u_font_eff_num(
        .ascii(eff_num_ch_ascii),
        .row  (eff_num_row),
        .bits (eff_num_row_bits)
    );
    wire eff_num_on =
        in_eff_num && !eff_num_in_space ?
        eff_num_row_bits[eff_num_col_in_char] : 1'b0;

    // ---------- 只在 NORMAL 模式显示 walktime / efficiency ----------
    wire is_normal_mode = (modenum == NORMAL);
    wire walk_text_on   = is_normal_mode && (walk_label_on || walk_num_on);
    wire eff_text_on    = is_normal_mode && (eff_label_on  || eff_num_on);

    // ---------- 模式文字和高峰期固定绿灯时长 ----------
    wire is_peak_mode = (modenum == MORNING_PEAK) || (modenum == EVENING_PEAK);

    // 将高峰固定绿灯时长截断到 0~99，并拆成十位个位
    wire [7:0] main_peak_clamped = (peak_main_green_s > 8'd99) ? 8'd99 : peak_main_green_s;
    wire [7:0] side_peak_clamped = (peak_side_green_s > 8'd99) ? 8'd99 : peak_side_green_s;

    wire [3:0] main_peak_tens = main_peak_clamped / 10;
    wire [3:0] main_peak_ones = main_peak_clamped % 10;
    wire [3:0] side_peak_tens = side_peak_clamped / 10;
    wire [3:0] side_peak_ones = side_peak_clamped % 10;

    // ---- 第 1 行：左下角模式单词：morning / normal / evening ----
    localparam TEXT_MODE_Y   = V_VALID - 10'd32;
    localparam TEXT_MODE_LEN = 7;  // 最长 "morning"/"evening"

    wire in_mode_text =
        (pix_x >= TEXT_X) &&
        (pix_x <  TEXT_X + TEXT_MODE_LEN*(CHAR_W+SPACE_W)) &&
        (pix_y >= TEXT_MODE_Y) &&
        (pix_y <  TEXT_MODE_Y + CHAR_H);

    wire [9:0] mode_col         = pix_x - TEXT_X;
    wire [9:0] mode_dy          = pix_y - TEXT_MODE_Y;
    wire [2:0] mode_row         = mode_dy[2:0];
    wire [2:0] mode_ch_idx      = mode_col / (CHAR_W + SPACE_W);
    wire [3:0] mode_col_mod     = mode_col % (CHAR_W + SPACE_W);
    wire       mode_in_space    = (mode_col_mod >= CHAR_W);
    wire [2:0] mode_col_in_char = CHAR_W - 1 - mode_col_mod[2:0];

    function [7:0] mode_word_ascii;
        input [1:0] mode;
        input [2:0] idx;
        begin
            case (mode)
                MORNING_PEAK: begin // "morning"
                    case (idx)
                        3'd0: mode_word_ascii = "m";
                        3'd1: mode_word_ascii = "o";
                        3'd2: mode_word_ascii = "r";
                        3'd3: mode_word_ascii = "n";
                        3'd4: mode_word_ascii = "i";
                        3'd5: mode_word_ascii = "n";
                        3'd6: mode_word_ascii = "g";
                        default: mode_word_ascii = " ";
                    endcase
                end
                NORMAL: begin // "normal"
                    case (idx)
                        3'd0: mode_word_ascii = "n";
                        3'd1: mode_word_ascii = "o";
                        3'd2: mode_word_ascii = "r";
                        3'd3: mode_word_ascii = "m";
                        3'd4: mode_word_ascii = "a";
                        3'd5: mode_word_ascii = "l";
                        default: mode_word_ascii = " ";
                    endcase
                end
                default: begin // EVENING_PEAK -> "evening"
                    case (idx)
                        3'd0: mode_word_ascii = "e";
                        3'd1: mode_word_ascii = "v";
                        3'd2: mode_word_ascii = "e";
                        3'd3: mode_word_ascii = "n";
                        3'd4: mode_word_ascii = "i";
                        3'd5: mode_word_ascii = "n";
                        3'd6: mode_word_ascii = "g";
                        default: mode_word_ascii = " ";
                    endcase
                end
            endcase
        end
    endfunction

    wire [7:0] mode_ch_ascii = mode_word_ascii(modenum, mode_ch_idx);
    wire [7:0] mode_row_bits;
    font u_font_mode(
        .ascii(mode_ch_ascii),
        .row  (mode_row),
        .bits (mode_row_bits)
    );
    wire mode_text_on =
        in_mode_text && !mode_in_space ?
        mode_row_bits[mode_col_in_char] : 1'b0;

    // ---- 第 2 行：主路绿灯时长 "main:xxs"（只在高峰模式下显示）----
    localparam TEXT_MAIN_Y   = TEXT_MODE_Y + CHAR_H + 2;
    localparam TEXT_MAIN_LEN = 8;  // "main:xxs"

    // ---- 第 3 行：支路绿灯时长 "side:yys"（只在高峰模式下显示）----
    localparam TEXT_SIDE_Y   = TEXT_MAIN_Y + CHAR_H + 2;
    localparam TEXT_SIDE_LEN = 8;  // "side:yys"
    // ---------------- 主路 "main:xxs" ----------------
    wire in_main_text =
        (pix_x >= TEXT_X) &&
        (pix_x <  TEXT_X + TEXT_MAIN_LEN*(CHAR_W+SPACE_W)) &&
        (pix_y >= TEXT_MAIN_Y) &&
        (pix_y <  TEXT_MAIN_Y + CHAR_H);

    wire [9:0] main_col         = pix_x - TEXT_X;
    wire [9:0] main_dy          = pix_y - TEXT_MAIN_Y;
    wire [2:0] main_row         = main_dy[2:0];
    wire [2:0] main_ch_idx      = main_col / (CHAR_W + SPACE_W);
    wire [3:0] main_col_mod     = main_col % (CHAR_W + SPACE_W);
    wire       main_in_space    = (main_col_mod >= CHAR_W);
    wire [2:0] main_col_in_char = CHAR_W - 1 - main_col_mod[2:0];

    // 生成 "m a i n : 数 字 s"
    function [7:0] main_word_ascii;
        input [2:0] idx;
        input [3:0] m_tens, m_ones;
        reg   [7:0] ch;
        begin
            ch = 8'h20; // 默认空格
            case (idx)
                3'd0: ch = "m";
                3'd1: ch = "a";
                3'd2: ch = "i";
                3'd3: ch = "n";
                3'd4: ch = ":";
                3'd5: ch = (m_tens == 4'd0) ? 8'h20 : (8'h30 + {4'b0000, m_tens});
                3'd6: ch =  8'h30 + {4'b0000, m_ones};
                3'd7: ch = "s";
                default: ch = 8'h20;
            endcase
            main_word_ascii = ch;
        end
    endfunction

    wire [7:0] main_ch_ascii =
        main_word_ascii(main_ch_idx, main_peak_tens, main_peak_ones);

    wire [7:0] main_row_bits;
    font u_font_main(
        .ascii(main_ch_ascii),
        .row  (main_row),
        .bits (main_row_bits)
    );

    wire main_text_on =
        is_peak_mode && in_main_text && !main_in_space ?
        main_row_bits[main_col_in_char] : 1'b0;
    // ---------------- 支路 "side:yys" ----------------
    wire in_side_text =
        (pix_x >= TEXT_X) &&
        (pix_x <  TEXT_X + TEXT_SIDE_LEN*(CHAR_W+SPACE_W)) &&
        (pix_y >= TEXT_SIDE_Y) &&
        (pix_y <  TEXT_SIDE_Y + CHAR_H);

    wire [9:0] side_col         = pix_x - TEXT_X;
    wire [9:0] side_dy          = pix_y - TEXT_SIDE_Y;
    wire [2:0] side_row         = side_dy[2:0];
    wire [2:0] side_ch_idx      = side_col / (CHAR_W + SPACE_W);
    wire [3:0] side_col_mod     = side_col % (CHAR_W + SPACE_W);
    wire       side_in_space    = (side_col_mod >= CHAR_W);
    wire [2:0] side_col_in_char = CHAR_W - 1 - side_col_mod[2:0];

    function [7:0] side_word_ascii;
        input [2:0] idx;
        input [3:0] s_tens, s_ones;
        reg   [7:0] ch;
        begin
            ch = 8'h20;
            case (idx)
                3'd0: ch = "s";
                3'd1: ch = "i";
                3'd2: ch = "d";
                3'd3: ch = "e";
                3'd4: ch = ":";
                3'd5: ch = (s_tens == 4'd0) ? 8'h20 : (8'h30 + {4'b0000, s_tens});
                3'd6: ch =  8'h30 + {4'b0000, s_ones};
                3'd7: ch = "s";
                default: ch = 8'h20;
            endcase
            side_word_ascii = ch;
        end
    endfunction

    wire [7:0] side_ch_ascii =
        side_word_ascii(side_ch_idx, side_peak_tens, side_peak_ones);

    wire [7:0] side_row_bits;
    font u_font_side(
        .ascii(side_ch_ascii),
        .row  (side_row),
        .bits (side_row_bits)
    );

    wire side_text_on =
        is_peak_mode && in_side_text && !side_in_space ?
        side_row_bits[side_col_in_char] : 1'b0;


    // ===================== 字符显示结束 =====================

    
    // 缁樺埗

    // 
    always @(posedge vga_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            pix_data <= BLACK;
        end else if(!in_area) begin
            pix_data <= BLACK;
        end else begin
            // 锟斤拷锟斤拷
            pix_data <= BLACK;

            // 锟斤拷路锟阶边ｏ拷锟斤拷锟斤拷锟斤拷位锟斤拷锟斤拷锟斤拷十锟街碉拷锟竭ｏ拷
            if ( (top_side || bot_side) && !(pix_x >= X_LEFT && pix_x <= X_RIGHT) )
                pix_data <= WHITE;
            if ( (left_side || right_side) && !(pix_y >= Y_TOP && pix_y <= Y_BOTTOM) )
                pix_data <= WHITE;

            // 车道内部虚线：
            // 上/下两条车道分界：白色虚线
            if ((h_lane1_band || h_lane3_band) && h_dash_on && !(top_side || bot_side))
                pix_data <= WHITE;

            // 中心分界线：双向之间的黄色虚线（保留你原来的效果）
            if (h_mid_band && h_dash_on && !(top_side || bot_side))
                pix_data <= YELLOW;
                
            // 斑马线：在人行道区域画横向白条（让行人和车可以覆盖在上面）
            if (zebra_on)
                pix_data <= WHITE;

            // 信号灯位置：主路灯、行人灯
            if(inLampCar) pix_data <= color_from_main(main_R, main_Y, main_G);
            if(inLampPed) pix_data <= color_from_side(side_R, side_Y, side_G);

            // 锟洁车锟斤拷锟较ｏ拷锟斤拷锟斤拷- 锟斤拷锟斤拷锟脚匡拷
            for(k=0;k<N_UP;k=k+1) begin
                if (k < car_count_up) begin
                    if ( (pix_x>=car_up_x[k]) && (pix_x<car_up_x[k]+CAR_W) &&
                         (pix_y>=car_up_y[k]) && (pix_y<car_up_y[k]+CAR_H) )
                        pix_data <= WHITE;
                end
            end

            // 锟洁车锟斤拷锟铰ｏ拷锟斤拷锟斤拷- 锟斤拷锟斤拷锟脚控ｏ拷锟斤拷锟诫）
            for(k=0;k<N_DN;k=k+1) begin
                if (k < car_count_down) begin
                    if ( (pix_x>=car_dn_x[k]) && (pix_x<car_dn_x[k]+CAR_W) &&
                         (pix_y>=car_dn_y[k]) && (pix_y<car_dn_y[k]+CAR_H) )
                        pix_data <= WHITE;
                end
            end

            // 锟斤拷锟斤拷锟剿ｏ拷锟斤拷锟斤拷 people_count锟斤拷锟斤拷位锟斤拷锟?? ped_phase_step锟斤拷
            if(inPedAny) begin
               pix_data <= WHITE;
            end

            // 字符显示：行人数、车数，NORMAL 模式下的 walk/eff，
            // 以及底部模式和高峰固定绿灯时长（白色）
            if(people_label_on || people_num_on || car_label_on || car_num_on ||
               walk_text_on    || eff_text_on    ||
               mode_text_on    || main_text_on   || side_text_on) begin
                pix_data <= WHITE;
            end
        end
    end
endmodule 