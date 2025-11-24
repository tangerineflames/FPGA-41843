`timescale 1ns/1ps

// ========================= TOP: traffic_adapt2 (no UART) =========================
module traffic_adapt2 #(
  // ï¿½ï¿½Ð©ï¿½ï¿½ï¿½ï¿½Ö»ï¿½ï¿½Îªï¿½Ïµï¿½Ä¬ï¿½ï¿½Öµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½Ï²ï¿½ cfg_* ï¿½ï¿½ï¿½ï¿½
  parameter BASE_S  = 40,
  parameter BASE_L  = 30,
  parameter BASE_R  = 20,
  parameter T_YEL   = 12,
  // ï¿½ï¿½ï¿½ï¿½ï¿½Ó³É£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¦Ä£Ê½ï¿½ï¿½Ð§ï¿½ï¿½
  parameter EXT_BIG   = 10,
  parameter EXT_SMALL = 5,
  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½many / small ï¿½Öµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½
  parameter PED_RT_TH = 5,
  // ï¿½ï¿½ï¿½ï¿½Ó°ï¿½ì£¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½
  parameter PED_EXT_S  = 8,
  parameter PED_RT_CUT = 6,
  parameter MIN_RIGHT  = 8,  // ï¿½ï¿½×ªï¿½ï¿½Ð¡Ê±ï¿½ï¿½ï¿½ï¿½ï¿½Þ£ï¿½clampï¿½ï¿½

  // ï¿½ï¿½×ª mini-FSM
  parameter RTY_S          = 1,
  parameter PED_ON_DB_S    = 2,  // È¥ï¿½ï¿½ï¿½ï¿½È·ï¿½ï¿½"ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½"ï¿½ï¿½ï¿½Ð»ï¿½
  parameter PED_OFF_DB_S   = 2,  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Í·Å£ï¿½
  parameter MIN_RIGHT_FREE = 2   // ï¿½ï¿½×ªï¿½ï¿½Ð¡ï¿½ï¿½ï¿½Ð±ï¿½ï¿½ï¿½
)(
  input  wire        clk,
  input  wire        rst_n,
  input  wire        sec_tick,      // 1Hz

  // ï¿½ï¿½ï¿½ï¿½Æ«ï¿½ï¿½ & ï¿½ï¿½ï¿½ï¿½Ì¬ï¿½Æ£ï¿½ï¿½ï¿½ï¿½ï¿½ scene_crossï¿½ï¿½
  input  wire [1:0]  car_bias,      // 00=ï¿½ï¿½ï¿½ï¿½ 01=LRï¿½ï¿½ 10=TBï¿½ï¿½
  input  wire [4:0]  ped_n_lr,
  input  wire [4:0]  ped_n_tb,
  input  wire        ped_has_lr,
  input  wire        ped_has_tb,
  input  wire        ped_active_lr,  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú²ï¿½ÊµÊ±Õ¼ï¿½ï¿½
  input  wire        ped_active_tb,
  input  wire        mode_adapt_sw,  // 1=ï¿½ï¿½ï¿½ï¿½Ó¦

  // ï¿½Ï²ã£¨scene_crossï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö±Í¨ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½Ò»ï¿½Â£ï¿½Ã¿ï¿½Ä¸ï¿½ï¿½ï¿??
  input  wire [7:0]  cfg_LRS,  // LR Ö±ï¿½ï¿½
  input  wire [7:0]  cfg_LRL,  // LR ï¿½ï¿½×ª
  input  wire [7:0]  cfg_LRR,  // LR ï¿½ï¿½×ª
  input  wire [7:0]  cfg_TBS,  // TB Ö±ï¿½ï¿½
  input  wire [7:0]  cfg_TBL,  // TB ï¿½ï¿½×ª
  input  wire [7:0]  cfg_TBR,  // TB ï¿½ï¿½×ª

  output reg  [1:0]  col_LR_S,
  output reg  [1:0]  col_LR_L,
  output reg  [1:0]  col_LR_R,
  output reg  [1:0]  col_TB_S,
  output reg  [1:0]  col_TB_L,
  output reg  [1:0]  col_TB_R,

  // ï¿½Ìµï¿½Ê±ï¿½ï¿½Ö¸Ê¾ï¿½ï¿½ï¿½ë£©
  output wire [7:0]  green_sec_LR_S,
  output wire [7:0]  green_sec_LR_L,
  output wire [7:0]  green_sec_LR_R,
  output wire [7:0]  green_sec_TB_S,
  output wire [7:0]  green_sec_TB_L,
  output wire [7:0]  green_sec_TB_R,
    // ÐÂÔö£º¸øÊýÂë¹ÜÓÃµÄ"µ±Ç°ÂÌµÆÊ£ÓàÊ±¼ä"ºÍµ±Ç°ÏàÎ»ÀàÐÍ
  output reg  [7:0]  phase_left_s,   // 0~99£¬µ±Ç°ÏàÎ»»¹Ê£¶àÉÙ"ÐéÄâÃë"
  output reg  [2:0]  phase_id        // 0=ÎÞÂÌµÆ,1=LRÖ±,2=LR×ó,3=LRÓÒ,4=TBÖ±,5=TB×ó,6=TBÓÒ
);

  localparam [1:0] C_RED=2'd0, C_YEL=2'd1, C_GRN=2'd2;
  localparam [2:0] PH_NONE = 3'd0,
                   PH_LR_S = 3'd1,
                   PH_LR_L = 3'd2,
                   PH_LR_R = 3'd3,
                   PH_TB_S = 3'd4,
                   PH_TB_L = 3'd5,
                   PH_TB_R = 3'd6;  
  // ================= åŸºå‡†æ—¶é•¿ï¼šæ ¹æ®æ¨¡å¼é?‰æ‹©ä¸åŒæ¥æº =================
  // è‡ªé?‚åº”æ¨¡å¼ï¼šä½¿ç”¨å†…ç½®å‚æ•°BASE_*ï¼ˆä¸å—UARTå½±å“ï¼?
  // å›ºå®šæ¨¡å¼ï¼šä½¿ç”¨cfg_*ï¼ˆUARTé…ç½®ï¼?
  wire [7:0] base_LRS = mode_adapt_sw ? BASE_S : cfg_LRS;
  wire [7:0] base_LRL = mode_adapt_sw ? BASE_L : cfg_LRL;
  wire [7:0] base_LRR = mode_adapt_sw ? BASE_R : cfg_LRR;
  wire [7:0] base_TBS = mode_adapt_sw ? BASE_S : cfg_TBS;
  wire [7:0] base_TBL = mode_adapt_sw ? BASE_L : cfg_TBL;
  wire [7:0] base_TBR = mode_adapt_sw ? BASE_R : cfg_TBR;

  // ====== ï¿½ï¿½Ê±ï¿½Ä´ï¿½ï¿½ï¿½ï¿½ï¿½Ô­Ê¼/ï¿½ï¿½Ò»ï¿½Ä£ï¿½======  -- ï¿½ï¿½Ú¶ï¿½ï¿½Ý±ï¿½ï¿½ï¿½Í¬ï¿½ï¿½Í¬ï¿½ï¿½ï¿½ï¿??
  reg [7:0] add_S_LR, add_L_LR, add_R_LR;
  reg [7:0] add_S_TB, add_L_TB, add_R_TB;
  reg [7:0] n_add_S_LR, n_add_L_LR, n_add_R_LR;
  reg [7:0] n_add_S_TB, n_add_L_TB, n_add_R_TB;

  // === Ä£Ê½ï¿½Å¿Øºï¿½Ä¼ï¿½Ê±ï¿½ï¿½ï¿½Ì¶ï¿½Ä£Ê½ï¿½ï¿??=0ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½= n_add_*ï¿½ï¿½===
  wire [7:0] g_add_S_LR = mode_adapt_sw ? n_add_S_LR : 8'd0;
  wire [7:0] g_add_L_LR = mode_adapt_sw ? n_add_L_LR : 8'd0;
  wire [7:0] g_add_R_LR = mode_adapt_sw ? n_add_R_LR : 8'd0;
  wire [7:0] g_add_S_TB = mode_adapt_sw ? n_add_S_TB : 8'd0;
  wire [7:0] g_add_L_TB = mode_adapt_sw ? n_add_L_TB : 8'd0;
  wire [7:0] g_add_R_TB = mode_adapt_sw ? n_add_R_TB : 8'd0;

  // ====== Ê±ï¿½ï¿½ï¿½ï¿½ï¿½Ì¶ï¿½/ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½ ======
  reg [15:0] T0,T1,T2,T3,T4,T5,T6,T7,T8; // LR ï¿½Ì¶ï¿½ï¿½ï¿½
  reg [15:0] U0,U1,U2,U3,U4,U5,U6,U7,U8; // TB ï¿½Ì¶ï¿½ï¿½ï¿½
  reg [15:0] FULL_CYCLE;

  reg [15:0] A0,A1,A2,A3,A4,A5,A6,A7,A8; // LR ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½ A
  reg [15:0] B0,B1,B2,B3,B4,B5,B6,B7,B8; // TB ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½ B
  reg [15:0] FULL_CYCLE_ADAPT;

  reg [15:0] t_cnt;

  // ï¿½ï¿½Ä£ï¿½Ð¶ï¿½
  wire lr_many  = (mode_adapt_sw && (ped_n_lr >= PED_RT_TH));
  wire tb_many  = (mode_adapt_sw && (ped_n_tb >= PED_RT_TH));
  wire lr_small = (mode_adapt_sw && (ped_n_lr <  PED_RT_TH));
  wire tb_small = (mode_adapt_sw && (ped_n_tb <  PED_RT_TH));

  // ï¿½Ì¶ï¿½ï¿½ï¿½Î»ï¿½Ç·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ªï¿½Î£ï¿½ï¿½Ì¶ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½×¨ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½
  wire rt_phase_enable_LR = (!mode_adapt_sw);
  wire rt_phase_enable_TB = (!mode_adapt_sw);

  // ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½Ç·ï¿½ï¿½ï¿½ï¿½ï¿½ A/B ï¿½ï¿½ï¿½ï¿½
  wire use_any_AB = (mode_adapt_sw && (lr_small || tb_small));

  function in_range; input [15:0] s0, s1, x; begin in_range = (x>=s0) && (x<s1); end endfunction
  function [7:0] clamp_right; input [7:0] val; begin clamp_right = (val < MIN_RIGHT) ? MIN_RIGHT : val; end endfunction

  // ====== Ñ¡ï¿½ï¿½ ======
  wire useA_LR = (mode_adapt_sw && lr_small);
  wire useB_TB = (mode_adapt_sw && tb_small);

  wire lr_in_S = useA_LR ? in_range(A0,A1,t_cnt) : in_range(T0,T1,t_cnt);
  wire lr_in_L = useA_LR ? in_range(A3,A4,t_cnt) : in_range(T3,T4,t_cnt);
  wire tb_in_S = useB_TB ? in_range(B0,B1,t_cnt) : in_range(U0,U1,t_cnt);
  wire tb_in_L = useB_TB ? in_range(B3,B4,t_cnt) : in_range(U3,U4,t_cnt);

  // ====== Ê±ï¿½ï¿½/ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ã£ºÖ»ï¿½ï¿½ sec_tick ï¿½ï¿½ï¿½Â£ï¿½ï¿½ï¿½ï¿½ï¿½Ú¶ï¿½ï¿½ÝµÄ½ï¿½ï¿½ï¿?? ======
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      t_cnt <= 0;

      add_S_LR<=0; add_L_LR<=0; add_R_LR<=0;
      add_S_TB<=0; add_L_TB<=0; add_R_TB<=0;
      n_add_S_LR<=0; n_add_L_LR<=0; n_add_R_LR<=0;
      n_add_S_TB<=0; n_add_L_TB<=0; n_add_R_TB<=0;

      // ï¿½Ïµï¿½Ä¬ï¿½Ï£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½Ü¿ï¿½á±» base_* + g_add_* ï¿½ï¿½ï¿½ï¿½
      T0<=0; T1<=BASE_S; T2<=BASE_S+T_YEL;
      T3<=T2; T4<=T3+BASE_L; T5<=T4+T_YEL;
      T6<=T5; T7<=T6+BASE_R; T8<=T7+T_YEL;

      U0<=T8; U1<=U0+BASE_S; U2<=U1+T_YEL;
      U3<=U2; U4<=U3+BASE_L; U5<=U4+T_YEL;
      U6<=U5; U7<=U6+BASE_R; U8<=U7+T_YEL;

      FULL_CYCLE<=U8;

      A0<=0; A1<=BASE_S; A2<=BASE_S+T_YEL;
      A3<=A2; A4<=A3+BASE_L; A5<=A4+T_YEL;
      A6<=A5; A7<=A6+BASE_R; A8<=A7+T_YEL;

      B0<=A8; B1<=B0+BASE_S; B2<=B1+T_YEL;
      B3<=B2; B4<=B3+BASE_L; B5<=B4+T_YEL;
      B6<=B5; B7<=B6+BASE_R; B8<=B7+T_YEL;

      FULL_CYCLE_ADAPT<=B8;
    end
    else if (sec_tick) begin
      // ï¿½ï¿½ï¿½Ú¼ï¿½ï¿½ï¿½
      if (t_cnt >= ((use_any_AB ? FULL_CYCLE_ADAPT : FULL_CYCLE) - 1)) t_cnt <= 0;
      else t_cnt <= t_cnt + 16'd1;

      // 1) ï¿½ï¿½ï¿½ï¿½Æ«ï¿½ï¿½ + 2) ï¿½ï¿½ï¿½ï¿½Ó°ï¿½ì£ºï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¦Ä£Ê½ï¿½ï¿½Ð§ï¿½ï¿½ï¿½Ì¶ï¿½Ä£Ê½È«ï¿½ï¿½ï¿½ï¿½ 0
      if (mode_adapt_sw) begin
        // ï¿½ï¿½ï¿½ï¿½Æ«ï¿½ï¿½
        n_add_S_LR = (car_bias==2'b01) ? EXT_BIG   : (car_bias==2'b10 ? 8'd0 : EXT_SMALL);
        n_add_L_LR = (car_bias==2'b01) ? EXT_SMALL : 8'd0;
        n_add_R_LR = (car_bias==2'b01) ? EXT_SMALL : 8'd0;

        n_add_S_TB = (car_bias==2'b10) ? EXT_BIG   : (car_bias==2'b01 ? 8'd0 : EXT_SMALL);
        n_add_L_TB = (car_bias==2'b10) ? EXT_SMALL : 8'd0;
        n_add_R_TB = (car_bias==2'b10) ? EXT_SMALL : 8'd0;

        // ï¿½ï¿½ï¿½ï¿½Ó°ï¿½ï¿½
        if (ped_has_lr) begin
          n_add_S_TB = n_add_S_TB + PED_EXT_S;
          n_add_R_LR = (n_add_R_LR >= PED_RT_CUT) ? (n_add_R_LR - PED_RT_CUT) : 8'd0;
        end
        if (ped_has_tb) begin
          n_add_S_LR = n_add_S_LR + PED_EXT_S;
          n_add_R_TB = (n_add_R_TB >= PED_RT_CUT) ? (n_add_R_TB - PED_RT_CUT) : 8'd0;
        end

        if (ped_n_lr >= PED_RT_TH) n_add_R_LR = 8'd0;
        if (ped_n_tb >= PED_RT_TH) n_add_R_TB = 8'd0;
      end else begin
        n_add_S_LR = 8'd0; n_add_L_LR = 8'd0; n_add_R_LR = 8'd0;
        n_add_S_TB = 8'd0; n_add_L_TB = 8'd0; n_add_R_TB = 8'd0;
      end

      // 3) ï¿½ï¿½ï¿½ï¿½ -- ï¿½ï¿½Ú¶ï¿½ï¿½ï¿½Ò»ï¿½Â£ï¿½ï¿½ï¿½È»ï¿½ï¿½ï¿½ï¿½Ãµï¿½ï¿½ï¿½ g_add_*ï¿½ï¿½
      add_S_LR <= n_add_S_LR; add_L_LR <= n_add_L_LR; add_R_LR <= n_add_R_LR;
      add_S_TB <= n_add_S_TB; add_L_TB <= n_add_L_TB; add_R_TB <= n_add_R_TB;

      // 4) ï¿½Ì¶ï¿½ï¿½ï¿½ï¿½ï¿½ T/Uï¿½ï¿½Ê¹ï¿½ï¿½ base_* + g_add_*ï¿½ï¿½
      T0 <= 16'd0;
      T1 <= 16'd0 + ({8'd0,base_LRS} + {8'd0,g_add_S_LR});
      T2 <= T1 + T_YEL;

      T3 <= T2;
      T4 <= T3 + ({8'd0,base_LRL} + {8'd0,g_add_L_LR});
      T5 <= T4 + T_YEL;

      if (rt_phase_enable_LR) begin
        T6 <= T5;
        T7 <= T6 + (lr_small ? 16'd0 : {8'd0,clamp_right(base_LRR + g_add_R_LR)});
        T8 <= T7 + (lr_small ? 16'd0 : T_YEL);
      end else begin
        T6 <= T5; T7 <= T6; T8 <= T7;
      end

      U0 <= T8;
      U1 <= U0 + ({8'd0,base_TBS} + {8'd0,g_add_S_TB});
      U2 <= U1 + T_YEL;

      U3 <= U2;
      U4 <= U3 + ({8'd0,base_TBL} + {8'd0,g_add_L_TB});
      U5 <= U4 + T_YEL;

      if (rt_phase_enable_TB) begin
        U6 <= U5;
        U7 <= U6 + (tb_small ? 16'd0 : {8'd0,clamp_right(base_TBR + g_add_R_TB)});
        U8 <= U7 + (tb_small ? 16'd0 : T_YEL);
      end else begin
        U6 <= U5; U7 <= U6; U8 <= U7;
      end

      FULL_CYCLE <= U8;

      // 5) ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½ï¿½ï¿½ A/Bï¿½ï¿½Í¬ï¿½ï¿½ï¿½ï¿½ base_* + g_add_*ï¿½ï¿½
      A0 <= 16'd0;
      A1 <= 16'd0 + ({8'd0,base_LRS} + {8'd0,g_add_S_LR});
      A2 <= A1 + T_YEL;

      A3 <= A2;
      A4 <= A3 + ({8'd0,base_LRL} + {8'd0,g_add_L_LR});
      A5 <= A4 + T_YEL;

      if (lr_small) begin
        A6 <= A5; A7 <= A6; A8 <= A7;
      end else begin
        A6 <= A5; A7 <= A6 + {8'd0,clamp_right(base_LRR + g_add_R_LR)}; A8 <= A7 + T_YEL;
      end

      B0 <= A8;
      B1 <= B0 + ({8'd0,base_TBS} + {8'd0,g_add_S_TB});
      B2 <= B1 + T_YEL;

      B3 <= B2;
      B4 <= B3 + ({8'd0,base_TBL} + {8'd0,g_add_L_TB});
      B5 <= B4 + T_YEL;

      if (tb_small) begin
        B6 <= B5; B7 <= B6; B8 <= B7;
      end else begin
        B6 <= B5; B7 <= B6 + {8'd0,clamp_right(base_TBR + g_add_R_TB)}; B8 <= B7 + T_YEL;
      end

      FULL_CYCLE_ADAPT <= B8;
    end
  end

  // ====== ï¿½ï¿½×ª mini-FSMï¿½ï¿½ï¿½Â¼ï¿½Ê±ï¿½Ó£ï¿½sec_tick ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ë¿ªï¿½Â½ï¿½ï¿½Ø£ï¿½ ======
  localparam [1:0] RT_G=2'd0, RT_Y=2'd1, RT_R=2'd2;

  // LR
  reg [1:0] rtLR_state; reg [7:0] rtLR_sec, rtLR_g_sec;
  reg       ped_has_lr_d, ped_active_lr_d;
  reg [3:0] ped_on_cnt_lr;

  // TB
  reg [1:0] rtTB_state; reg [7:0] rtTB_sec, rtTB_g_sec;
  reg       ped_has_tb_d, ped_active_tb_d;
  reg [3:0] ped_on_cnt_tb;

  // ï¿½ï¿½ small ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ S/L ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ mini-FSMï¿½ï¿½ï¿½ï¿½Ú¶ï¿½ï¿½ï¿½Í¬ï¿½ï¿½ï¿½ï¿??
  wire lr_fsm_enable = (mode_adapt_sw && lr_small && (lr_in_S || lr_in_L));
  wire tb_fsm_enable = (mode_adapt_sw && tb_small && (tb_in_S || tb_in_L));

  // È¥ï¿½ï¿½/ï¿½ï¿½Ð¡ï¿½ï¿½ï¿½ï¿½
  wire ped_on_ok_lr  = (ped_on_cnt_lr >= PED_ON_DB_S);
  wire min_g_ok_lr   = (rtLR_g_sec    >= MIN_RIGHT_FREE);
  wire ped_on_ok_tb  = (ped_on_cnt_tb >= PED_ON_DB_S);
  wire min_g_ok_tb   = (rtTB_g_sec    >= MIN_RIGHT_FREE);

  // ï¿½Â½ï¿½ï¿½Ø£ï¿½ï¿½ï¿½ÊµÊ±Õ¼ï¿½Ã£ï¿½
  wire lr_fall_edge = (ped_active_lr_d==1'b1) && (ped_active_lr==1'b0);
  wire tb_fall_edge = (ped_active_tb_d==1'b1) && (ped_active_tb==1'b0);

  // ï¿½Â¼ï¿½Ê±ï¿½ï¿½
  wire lr_event_tick = sec_tick | lr_fall_edge;
  wire tb_event_tick = sec_tick | tb_fall_edge;

  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö»ï¿½ï¿½ sec_tickï¿½ï¿½×ªï¿½Æ£ï¿½ï¿½ï¿½ event_tickï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      rtLR_state<=RT_G; rtLR_sec<=0; rtLR_g_sec<=0; ped_on_cnt_lr<=0;
      rtTB_state<=RT_G; rtTB_sec<=0; rtTB_g_sec<=0; ped_on_cnt_tb<=0;
      ped_has_lr_d<=0; ped_active_lr_d<=0;
      ped_has_tb_d<=0; ped_active_tb_d<=0;
    end else begin
      // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ø¼ï¿½ï¿??
      ped_has_lr_d    <= ped_has_lr;
      ped_active_lr_d <= ped_active_lr;
      ped_has_tb_d    <= ped_has_tb;
      ped_active_tb_d <= ped_active_tb;

      // ï¿½ï¿½ï¿½ï¿½Ö»ï¿½ï¿½ sec_tick
      if (sec_tick) begin
        // LR
        rtLR_sec   <= (rtLR_sec==8'hFF) ? rtLR_sec : (rtLR_sec + 1'b1);
        rtLR_g_sec <= (rtLR_state==RT_G) ? ((rtLR_g_sec==8'hFF)?rtLR_g_sec:(rtLR_g_sec+1'b1)) : 0;
        if (lr_fsm_enable)
          ped_on_cnt_lr <= ped_active_lr ? ((ped_on_cnt_lr < PED_ON_DB_S)?(ped_on_cnt_lr+1'b1):ped_on_cnt_lr) : 0;
        else ped_on_cnt_lr <= 0;

        // TB
        rtTB_sec   <= (rtTB_sec==8'hFF) ? rtTB_sec : (rtTB_sec + 1'b1);
        rtTB_g_sec <= (rtTB_state==RT_G) ? ((rtTB_g_sec==8'hFF)?rtTB_g_sec:(rtTB_g_sec+1'b1)) : 0;
        if (tb_fsm_enable)
          ped_on_cnt_tb <= ped_active_tb ? ((ped_on_cnt_tb < PED_ON_DB_S)?(ped_on_cnt_tb+1'b1):ped_on_cnt_tb) : 0;
        else ped_on_cnt_tb <= 0;
      end

      // ===== LR ×´Ì¬ï¿½ï¿½ï¿½ï¿½ï¿½Â¼ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ =====
      if (lr_event_tick) begin
        if (lr_fsm_enable) begin
          case (rtLR_state)
            RT_G: begin
              if (lr_in_S && ped_on_ok_lr && min_g_ok_lr) begin
                rtLR_state <= RT_Y;
                rtLR_sec   <= 0; // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÆµÆ¼ï¿½ï¿½ï¿??
              end
            end
            RT_Y: begin
              if (sec_tick && (rtLR_sec >= RTY_S)) begin
                rtLR_state <= RT_R;
                rtLR_sec   <= 0;
              end
            end
            RT_R: begin
              // ï¿½Ø¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ë¿ªï¿½ï¿½ï¿½ï¿½Ö±ï¿½Ð´ï¿½ï¿½ï¿½ï¿½Ë£ï¿½ï¿½ï¿½ï¿½ï¿½×ªï¿½Ì£ï¿½ï¿½ï¿½ï¿½ï¿½ sec_tick
              if (lr_in_L || !ped_has_lr || !ped_active_lr || (lr_in_S && !ped_active_lr)) begin
                rtLR_state <= RT_G;
                rtLR_sec   <= 0;   // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ã£¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿??
                rtLR_g_sec <= 0;   // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ã£¬MIN ï¿½ï¿½ï¿½Â¼ï¿½
              end
            end
          endcase
        end else begin
          rtLR_state <= RT_G;
          rtLR_sec   <= 0;
        end
      end

      // ===== TB ×´Ì¬ï¿½ï¿½ï¿½ï¿½ï¿½Â¼ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ =====
      if (tb_event_tick) begin
        if (tb_fsm_enable) begin
          case (rtTB_state)
            RT_G: begin
              if (tb_in_S && ped_on_ok_tb && min_g_ok_tb) begin
                rtTB_state <= RT_Y;
                rtTB_sec   <= 0;
              end
            end
            RT_Y: begin
              if (sec_tick && (rtTB_sec >= RTY_S)) begin
                rtTB_state <= RT_R;
                rtTB_sec   <= 0;
              end
            end
            RT_R: begin
              if (tb_in_L || !ped_has_tb || !ped_active_tb || (tb_in_S && !ped_active_tb)) begin
                rtTB_state <= RT_G;
                rtTB_sec   <= 0;
                rtTB_g_sec <= 0;
              end
            end
          endcase
        end else begin
          rtTB_state <= RT_G;
          rtTB_sec   <= 0;
        end
      end
    end
  end

  wire [1:0] rtLR_col = (rtLR_state==RT_G)?C_GRN : (rtLR_state==RT_Y)?C_YEL : C_RED;
  wire [1:0] rtTB_col = (rtTB_state==RT_G)?C_GRN : (rtTB_state==RT_Y)?C_YEL : C_RED;

  // ====== ï¿½ï¿½É«ï¿½ï¿½Ï£ï¿½ï¿½ï¿½Ú¶ï¿½ï¿½ï¿½Ò»ï¿½Â£ï¿½ ======
  always @(*) begin
    // È±Ê¡È«ï¿½ï¿½
    col_LR_S = C_RED;  col_LR_L = C_RED;  col_LR_R = C_RED;
    col_TB_S = C_RED;  col_TB_L = C_RED;  col_TB_R = C_RED;

    // Ö±/ï¿½ï¿½
    if (mode_adapt_sw && lr_small) begin
      if      (in_range(A0, A1, t_cnt)) col_LR_S = C_GRN;
      else if (in_range(A1, A2, t_cnt)) col_LR_S = C_YEL;
      if      (in_range(A3, A4, t_cnt)) col_LR_L = C_GRN;
      else if (in_range(A4, A5, t_cnt)) col_LR_L = C_YEL;
    end else begin
      if      (in_range(T0, T1, t_cnt)) col_LR_S = C_GRN;
      else if (in_range(T1, T2, t_cnt)) col_LR_S = C_YEL;
      if      (in_range(T3, T4, t_cnt)) col_LR_L = C_GRN;
      else if (in_range(T4, T5, t_cnt)) col_LR_L = C_YEL;
    end

    if (mode_adapt_sw && tb_small) begin
      if      (in_range(B0, B1, t_cnt)) col_TB_S = C_GRN;
      else if (in_range(B1, B2, t_cnt)) col_TB_S = C_YEL;
      if      (in_range(B3, B4, t_cnt)) col_TB_L = C_GRN;
      else if (in_range(B4, B5, t_cnt)) col_TB_L = C_YEL;
    end else begin
      if      (in_range(U0, U1, t_cnt)) col_TB_S = C_GRN;
      else if (in_range(U1, U2, t_cnt)) col_TB_S = C_YEL;
      if      (in_range(U3, U4, t_cnt)) col_TB_L = C_GRN;
      else if (in_range(U4, U5, t_cnt)) col_TB_L = C_YEL;
    end

    // ï¿½ï¿½×ªï¿½ï¿½many ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½small ï¿½ï¿½Ö±ï¿½Ð´ï¿½ï¿½ï¿½ mini-FSM
    if (!mode_adapt_sw) begin
      if      (in_range(T6, T7, t_cnt)) col_LR_R = C_GRN;
      else if (in_range(T7, T8, t_cnt)) col_LR_R = C_YEL;

      if      (in_range(U6, U7, t_cnt)) col_TB_R = C_GRN;
      else if (in_range(U7, U8, t_cnt)) col_TB_R = C_YEL;
    end else begin
      if (lr_many) begin
        if      (in_range(T3, T4, t_cnt)) col_LR_R = C_GRN;
        else if (in_range(T4, T5, t_cnt)) col_LR_R = C_YEL;
        else                              col_LR_R = C_RED;
      end else begin
        if      (lr_in_L) col_LR_R = C_GRN;
        else if (lr_in_S) col_LR_R = rtLR_col;
        else              col_LR_R = C_RED;
      end

      if (tb_many) begin
        if      (in_range(U3, U4, t_cnt)) col_TB_R = C_GRN;
        else if (in_range(U4, U5, t_cnt)) col_TB_R = C_YEL;
        else                              col_TB_R = C_RED;
      end else begin
        if      (tb_in_L) col_TB_R = C_GRN;
        else if (tb_in_S) col_TB_R = rtTB_col;
        else              col_TB_R = C_RED;
      end
    end

    // ========== ï¿½ï¿½È«ï¿½ï¿½ï¿½ï¿½ ==========
    if ((col_LR_S==C_RED) && (col_LR_L==C_RED) && (col_LR_R==C_RED)
     && (col_TB_S==C_RED) && (col_TB_L==C_RED) && (col_TB_R==C_RED)) begin
      col_LR_S = C_GRN;
    end

    // ========== Ë«ï¿½ï¿½Ð¡Ð¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ë£ï¿½Ë«ï¿½ï¿½ï¿½ï¿½×ªÒ»ï¿½ï¿½ï¿?? ==========
    if (mode_adapt_sw && lr_small && tb_small && !ped_has_lr && !ped_has_tb) begin
      col_LR_R = C_GRN; col_TB_R = C_GRN;
    end
  end
  // ========== µ±Ç°ÂÌµÆÏàÎ»µ¹¼ÆÊ±£¨¸øÊýÂë¹ÜÓÃ£© ==========
  reg [15:0] left_tmp;

  always @(*) begin
    phase_id   = PH_NONE;
    left_tmp   = 16'd0;

    if (!mode_adapt_sw) begin
      // ============= ¹Ì¶¨ÅäÊ±Ä£Ê½£ºS/L/R ¶¼ÓÐµ¹¼ÆÊ± =============
      // 1) LR ·½Ïò£ºÖ±ÐÐ -> ×ó×ª -> ÓÒ×ª
      if (in_range(T0, T1, t_cnt)) begin
        phase_id = PH_LR_S;
        left_tmp = (T1 > t_cnt) ? (T1 - t_cnt) : 16'd0;
      end
      else if (in_range(T3, T4, t_cnt)) begin
        phase_id = PH_LR_L;
        left_tmp = (T4 > t_cnt) ? (T4 - t_cnt) : 16'd0;
      end
      else if (in_range(T6, T7, t_cnt)) begin
        phase_id = PH_LR_R;
        left_tmp = (T7 > t_cnt) ? (T7 - t_cnt) : 16'd0;
      end

      // 2) TB ·½Ïò£ºÖ±ÐÐ -> ×ó×ª -> ÓÒ×ª
      else if (in_range(U0, U1, t_cnt)) begin
        phase_id = PH_TB_S;
        left_tmp = (U1 > t_cnt) ? (U1 - t_cnt) : 16'd0;
      end
      else if (in_range(U3, U4, t_cnt)) begin
        phase_id = PH_TB_L;
        left_tmp = (U4 > t_cnt) ? (U4 - t_cnt) : 16'd0;
      end
      else if (in_range(U6, U7, t_cnt)) begin
        phase_id = PH_TB_R;
        left_tmp = (U7 > t_cnt) ? (U7 - t_cnt) : 16'd0;
      end
      // ÆäËûÏàÎ»£¨È«ºì¡¢»ÆµÆ£©±£³Ö PH_NONE / 0
    end
    else begin
      // ============= ×ÔÊÊÓ¦Ä£Ê½£ºÖ»¶ÔÖ±ÐÐ S ×öµ¹¼ÆÊ±£¬ÓÒ×ª¿ÉÒÔÃ»ÓÐ =============
      // LR Ö±ÐÐÂÌ
      if (lr_in_S && (col_LR_S == C_GRN)) begin
        phase_id = PH_LR_S;
        if (useA_LR)
          left_tmp = (A1 > t_cnt) ? (A1 - t_cnt) : 16'd0;
        else
          left_tmp = (T1 > t_cnt) ? (T1 - t_cnt) : 16'd0;
      end
      // TB Ö±ÐÐÂÌ
      else if (tb_in_S && (col_TB_S == C_GRN)) begin
        phase_id = PH_TB_S;
        if (useB_TB)
          left_tmp = (B1 > t_cnt) ? (B1 - t_cnt) : 16'd0;
        else
          left_tmp = (U1 > t_cnt) ? (U1 - t_cnt) : 16'd0;
      end
      else begin
        phase_id = PH_NONE;
        left_tmp = 16'd0;
      end
    end

    // ÏÞ·ùµ½ 0~255£¬¸øÊýÂë¹ÜÓÃ¹»ÁË
    if (left_tmp > 16'd255)
      phase_left_s = 8'd255;
    else
      phase_left_s = left_tmp[7:0];
  end

  // ====== ï¿½Ìµï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿?? ======
  // ×¢ï¿½ï¿½ï¿½Ì¶ï¿½Ä£Ê½ï¿½ï¿½ g_add_* = 0ï¿½ï¿½ï¿½ï¿½Ë´Ë´ï¿½ï¿½ï¿½Ê¾ï¿½Ä¾ï¿½ï¿½ï¿?? base_*ï¿½ï¿½
  //     ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½ï¿½ï¿½Ê¾ base_* + ï¿½Ñ¼ï¿½ï¿½ï¿½Ä¼Ó³É£ï¿½S/Lï¿½ï¿½ï¿½ï¿½R ï¿½Ú¹Ì¶ï¿½Ä£Ê½ï¿½Â²ï¿½ï¿½ï¿½Ð§ï¿½ï¿½ï¿½ï¿½
  assign green_sec_LR_S = base_LRS + g_add_S_LR;
  assign green_sec_LR_L = base_LRL + g_add_L_LR;
  assign green_sec_LR_R = (!mode_adapt_sw) ? (base_LRR + g_add_R_LR) : 8'd0;

  assign green_sec_TB_S = base_TBS + g_add_S_TB;
  assign green_sec_TB_L = base_TBL + g_add_L_TB;
  assign green_sec_TB_R = (!mode_adapt_sw) ? (base_TBR + g_add_R_TB) : 8'd0;

endmodule
