`timescale 1ns/1ps
module traffic_ctrl_adaptive #(
    parameter integer CLK_HZ      = 25_175_000,
    parameter integer SEC_SCALE   = 4,          // 仿真"秒"的加速倍数，=4 表示快 4 倍
        
    // NORMAL 模式下自适应用的参数
    parameter integer MIN_GREEN_S = 5,
    parameter integer MAX_GREEN_S = 12,
    parameter integer YELLOW_S    = 1,
    parameter integer ALL_RED_S   = 1,
    parameter integer GAP_S       = 1,
    parameter integer FAST_RELEASE= 1,
    // ] 早 / 晚高峰固定绿灯时间（秒）：主 12，辅 5
    parameter integer MORN_MAIN_S = 12,  // 早高峰主路绿 12s
    parameter integer MORN_SIDE_S = 5,   // 早高峰支路绿 5s
    parameter integer EVEN_MAIN_S = 12,  // 晚高峰主路绿 12s
    parameter integer EVEN_SIDE_S = 5    // 晚高峰支路绿 5s

)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  mode,        // 0=早高峰,1=正常,2=晚高峰
    input  wire        tick,        //（保留不用也没事）
    input  wire        det_main,
    input  wire        det_side,
    input  wire        in_crossing,

    output reg         main_G, main_Y, main_R,
    output reg         side_G, side_Y, side_R,

    output reg  [7:0]  cur_sec,
    output reg  [2:0]  state,

    // ★ 给 VGA 用的"真实数字"：高峰模式的固定绿灯秒数（0~99）
    output wire [7:0]  peak_main_green_s,
    output wire [7:0]  peak_side_green_s,
    
        // 给数码管用的：当前绿灯剩余"虚拟秒"，以及当前绿灯是不是主路
    output reg  [7:0] green_left_s,   // 0~99
    output reg        green_on_main   // 1=主路绿灯，0=辅路绿灯(或无绿灯)

);
    // 模式编码保持和 vga_pic 一致：0=早,1=normal,2=晚
    localparam [1:0] MORNING_PEAK = 2'd0;
    localparam [1:0] NORMAL       = 2'd1;
    localparam [1:0] EVENING_PEAK = 2'd2;

    // 状态机
    localparam S_MG  = 3'd0,
               S_MY  = 3'd1,
               S_AR1 = 3'd2,
               S_SG  = 3'd3,
               S_SY  = 3'd4,
               S_AR2 = 3'd5;

    // ★ 每 1 个"仿真秒"需要的时钟数：真实频率 / 加速倍数
    localparam integer TICKS_PER_SEC = CLK_HZ / SEC_SCALE;
    
    // NORMAL 模式自适应用的计数阈值（仿真秒 → 时钟）
    localparam integer MIN_GREEN_C = MIN_GREEN_S * TICKS_PER_SEC;
    localparam integer MAX_GREEN_C = MAX_GREEN_S * TICKS_PER_SEC;
    localparam integer YELLOW_C    = YELLOW_S    * TICKS_PER_SEC;
    localparam integer ALL_RED_C   = ALL_RED_S   * TICKS_PER_SEC;
    localparam integer GAP_C       = GAP_S       * TICKS_PER_SEC;

    // 高峰模式固定绿灯时间对应的计数阈值（仿真秒 → 时钟）
    localparam integer MORN_MAIN_C = MORN_MAIN_S * TICKS_PER_SEC;
    localparam integer MORN_SIDE_C = MORN_SIDE_S * TICKS_PER_SEC;
    localparam integer EVEN_MAIN_C = EVEN_MAIN_S * TICKS_PER_SEC;
    localparam integer EVEN_SIDE_C = EVEN_SIDE_S * TICKS_PER_SEC;
        // 高峰模式下：主/辅路的"目标绿灯时间"（虚拟秒）
    wire [7:0] main_green_target_s =
        (mode == MORNING_PEAK) ? MORN_MAIN_S[7:0] :
        (mode == EVENING_PEAK) ? EVEN_MAIN_S[7:0] :
                                 8'd0;       // NORMAL 模式没有固定时间，用 0

    wire [7:0] side_green_target_s =
        (mode == MORNING_PEAK) ? MORN_SIDE_S[7:0] :
        (mode == EVENING_PEAK) ? EVEN_SIDE_S[7:0] :
                                 8'd0;

    // 提供给 VGA 的"真实秒数"（截断到 0~255，VGA 里再截到两位数）
    assign peak_main_green_s = (mode == MORNING_PEAK) ? MORN_MAIN_S[7:0] :
                               (mode == EVENING_PEAK) ? EVEN_MAIN_S[7:0] :
                                                        8'd0;

    assign peak_side_green_s = (mode == MORNING_PEAK) ? MORN_SIDE_S[7:0] :
                               (mode == EVENING_PEAK) ? EVEN_SIDE_S[7:0] :
                                                        8'd0;

    reg [31:0] phase_cnt;  // 当前阶段计数
    reg [31:0] gap_cnt;    // 自适应用的 gap 计数
    reg [31:0] sec_div;    // 秒分频计数
    reg [31:0] green_sec_div;   // 绿灯用的 1s 分频
    reg [7:0]  green_elapsed_s; // 当前这段绿灯已经过去的"虚拟秒"

    // 输出灯色组合
    always @* begin
        main_G = 0; main_Y = 0; main_R = 0;
        side_G = 0; side_Y = 0; side_R = 0;
        case(state)
            S_MG:  begin main_G = 1; side_R = 1; end  // 主绿 辅红
            S_MY:  begin main_Y = 1; side_R = 1; end  // 主黄 辅红
            S_AR1: begin main_R = 1; side_R = 1; end  // 全红
            S_SG:  begin side_G = 1; main_R = 1; end  // 主红 辅绿
            S_SY:  begin side_Y = 1; main_R = 1; end  // 主红 辅黄
            S_AR2: begin main_R = 1; side_R = 1; end  // 全红
            default: begin main_R = 1; side_R = 1; end
        endcase
    end
        // 绿灯秒计数：MG / SG 状态下，每 "1 秒" 加一
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            green_sec_div   <= 32'd0;
            green_elapsed_s <= 8'd0;
        end else begin
            // 只有在主绿或辅绿阶段才计数，其它状态清零
            if (state == S_MG || state == S_SG) begin
                // ★ 每经过 1 个"虚拟秒"（现实 1/4 秒），green_elapsed_s+1
                if (green_sec_div >= TICKS_PER_SEC - 1) begin
                    green_sec_div <= 32'd0;
                    if (green_elapsed_s != 8'hFF)
                        green_elapsed_s <= green_elapsed_s + 1'b1;
                end else begin
                    green_sec_div <= green_sec_div + 1'b1;
                end
            end else begin
                green_sec_div   <= 32'd0;
                green_elapsed_s <= 8'd0;
            end
        end
    end
    // 根据当前状态 / 模式，算数码管要显示的绿灯剩余秒数
    always @(*) begin
        green_on_main = 1'b0;
        green_left_s  = 8'd0;

        if (state == S_MG) begin
            // 主路绿灯
            green_on_main = 1'b1;
            if (main_green_target_s > green_elapsed_s)
                green_left_s = main_green_target_s - green_elapsed_s;
            else
                green_left_s = 8'd0;

        end else if (state == S_SG) begin
            // 辅路绿灯
            green_on_main = 1'b0;
            if (side_green_target_s > green_elapsed_s)
                green_left_s = side_green_target_s - green_elapsed_s;
            else
                green_left_s = 8'd0;
        end
        // 其它状态就保持 0
    end

    // 主状态机 + 计数
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_MG;
            phase_cnt <= 32'd0;
            gap_cnt   <= 32'd0;
            sec_div   <= 32'd0;
            cur_sec   <= 8'd0;
        end else begin
            // 相位计数
            phase_cnt <= phase_cnt + 1'b1;

            // gap 计数：只给 NORMAL 模式里的自适应用，其它模式无所谓
            if ( (state == S_SG) ? det_side : det_main )
                gap_cnt <= 32'd0;
            else
                gap_cnt <= gap_cnt + 1'b1;

            // 秒计数 / 行人过街时间（单位：仿真秒）
            if (sec_div >= TICKS_PER_SEC-1) begin       // ★ 用仿真秒的时钟数
                sec_div <= 32'd0;
                if ((state == S_SG) && in_crossing && (cur_sec != 8'hFF))
                    cur_sec <= cur_sec + 1'b1;          // cur_sec 也是"仿真秒"
            end else begin
                if ((state == S_SG) && in_crossing)
                    sec_div <= sec_div + 1'b1;
                else if (state != S_SG)
                    sec_div <= sec_div + 1'b1;
            end

            // 状态机
            case(state)
                // ==================== 主路绿 ====================
                S_MG: begin
                    case (mode)
                        NORMAL: begin
                            // 自适应：至少 MIN_GREEN_S，
                            // 没车则等 GAP_S 再切黄，最长不超过 MAX_GREEN_S
                            if ( (phase_cnt >= MIN_GREEN_C && gap_cnt >= GAP_C) ||
                                 (phase_cnt >= MAX_GREEN_C) ) begin
                                state     <= S_MY;
                                phase_cnt <= 32'd0;
                                gap_cnt   <= 32'd0;
                                sec_div   <= 32'd0;
                            end
                        end

                        MORNING_PEAK: begin
                            // 早高峰：主路绿固定 MORN_MAIN_S 秒
                            if (phase_cnt >= MORN_MAIN_C) begin
                                state     <= S_MY;
                                phase_cnt <= 32'd0;
                                gap_cnt   <= 32'd0;
                                sec_div   <= 32'd0;
                            end
                        end

                        EVENING_PEAK: begin
                            // 晚高峰：主路绿固定 EVEN_MAIN_S 秒
                            if (phase_cnt >= EVEN_MAIN_C) begin
                                state     <= S_MY;
                                phase_cnt <= 32'd0;
                                gap_cnt   <= 32'd0;
                                sec_div   <= 32'd0;
                            end
                        end

                        default: ; // 不会进
                    endcase
                end

                // ==================== 主黄 -> 全红 ====================
                S_MY: begin
                    if (phase_cnt >= YELLOW_C) begin
                        state     <= S_AR1;
                        phase_cnt <= 32'd0;
                        gap_cnt   <= 32'd0;
                        sec_div   <= 32'd0;
                    end
                end

                // ==================== 全红 -> 辅绿 ====================
                S_AR1: begin
                    if (phase_cnt >= ALL_RED_C) begin
                        state     <= S_SG;
                        phase_cnt <= 32'd0;
                        gap_cnt   <= 32'd0;
                        sec_div   <= 32'd0;
                        cur_sec   <= 8'd0;   // 刚放行行人时，walktime 从 0 开始再计
                    end
                end

                S_SG: begin
                    case (mode)
                        NORMAL: begin
                            // 自适应 + FAST_RELEASE
                            if (!det_side && (phase_cnt >= MIN_GREEN_C)) begin
                                if (FAST_RELEASE) begin
                                    state     <= S_MG;   // 直接回主绿
                                    phase_cnt <= 32'd0;
                                    gap_cnt   <= 32'd0;
                                    sec_div   <= 32'd0;
                                end else begin
                                    state     <= S_SY;   // 先黄
                                    phase_cnt <= 32'd0;
                                    gap_cnt   <= 32'd0;
                                    sec_div   <= 32'd0;
                                end
                            end else if ((phase_cnt >= MAX_GREEN_C) && !in_crossing) begin
                                // 最长绿灯也要等行人走完才切
                                state     <= S_SY;
                                phase_cnt <= 32'd0;
                                gap_cnt   <= 32'd0;
                                sec_div   <= 32'd0;
                            end
                        end

                        MORNING_PEAK: begin
                            // ★ 早高峰：至少 MORN_SIDE_S 秒，且行人全部通过(in_crossing==0)才切黄
                            if ((phase_cnt >= MORN_SIDE_C) && !in_crossing) begin
                                state     <= S_SY;
                                phase_cnt <= 32'd0;
                                gap_cnt   <= 32'd0;
                                sec_div   <= 32'd0;
                            end
                        end

                        EVENING_PEAK: begin
                            //  晚高峰：至少 EVEN_SIDE_S 秒，且行人全部通过才切黄
                            if ((phase_cnt >= EVEN_SIDE_C) && !in_crossing) begin
                                state     <= S_SY;
                                phase_cnt <= 32'd0;
                                gap_cnt   <= 32'd0;
                                sec_div   <= 32'd0;
                            end
                        end

                        default: ;
                    endcase
                end

                // ==================== 辅黄 -> 全红 ====================
                S_SY: begin
                    if (phase_cnt >= YELLOW_C) begin
                        state     <= S_AR2;
                        phase_cnt <= 32'd0;
                        gap_cnt   <= 32'd0;
                        sec_div   <= 32'd0;
                    end
                end

                // ==================== 全红 -> 主绿 ====================
                S_AR2: begin
                    if (phase_cnt >= ALL_RED_C) begin
                        state     <= S_MG;
                        phase_cnt <= 32'd0;
                        gap_cnt   <= 32'd0;
                        sec_div   <= 32'd0;
                    end
                end

                default: begin
                    state     <= S_MG;
                    phase_cnt <= 32'd0;
                    gap_cnt   <= 32'd0;
                    sec_div   <= 32'd0;
                end
            endcase
        end
    end

endmodule
