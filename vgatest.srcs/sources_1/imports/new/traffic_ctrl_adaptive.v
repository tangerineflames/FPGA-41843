`timescale 1ns/1ps
module traffic_ctrl_adaptive #(
    parameter integer CLK_HZ      = 25_175_000,
    parameter integer MIN_GREEN_S = 5,
    parameter integer MAX_GREEN_S = 12,
    parameter integer YELLOW_S    = 1,
    parameter integer ALL_RED_S   = 1,
    parameter integer GAP_S       = 1,
    parameter FAST_RELEASE        = 1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  mode,        // ★ 新增：0=早高峰,1=正常,2=晚高峰
    input  wire        tick,        // 你原来的，还可以保留
    input  wire        det_main,
    input  wire        det_side,
    input  wire        in_crossing,

    output reg  main_G, main_Y, main_R,
    output reg  side_G, side_Y, side_R,

    output reg [7:0]   cur_sec,
    output reg [2:0]   state
);

// 状态机
localparam S_MG=3'd0, S_MY=3'd1, S_AR1=3'd2, S_SG=3'd3, S_SY=3'd4, S_AR2=3'd5;

// 根据模式设置绿灯时间
localparam integer MIN_GREEN_C = MIN_GREEN_S * CLK_HZ;
localparam integer MAX_GREEN_C = MAX_GREEN_S * CLK_HZ;
localparam integer YELLOW_C    = YELLOW_S    * CLK_HZ;
localparam integer ALL_RED_C   = ALL_RED_S   * CLK_HZ;
localparam integer GAP_C       = GAP_S       * CLK_HZ;

reg [31:0] phase_cnt;  // 当前阶段计数
reg [31:0] gap_cnt;    // "gap"阶段计数
reg [31:0] sec_div;    // 秒计数

always @* begin
    // 默认状态
    main_G = 0; main_Y = 0; main_R = 0;
    side_G = 0; side_Y = 0; side_R = 0;
    
    case(state)
        S_MG:  begin main_G = 1; side_R = 1; end  // 主车道绿，辅车道红
        S_MY:  begin main_Y = 1; side_R = 1; end  // 主车道黄，辅车道红
        S_AR1: begin main_R = 1; side_R = 1; end  // 主车道红，辅车道红
        S_SG:  begin side_G = 1; main_R = 1; end  // 主车道红，辅车道绿
        S_SY:  begin side_Y = 1; main_R = 1; end  // 主车道红，辅车道黄
        S_AR2: begin main_R = 1; side_R = 1; end  // 主车道红，辅车道红
        default: begin main_R = 1; side_R = 1; end // 默认状态
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= S_MG;
        phase_cnt <= 32'd0;
        gap_cnt <= 32'd0;
        sec_div <= 32'd0;
        cur_sec <= 8'd0;
    end else begin
        phase_cnt <= phase_cnt + 1'b1;

        // 如果是辅车道检测到有车，重置 gap_cnt
        if ( (state == S_SG) ? det_side : det_main )
            gap_cnt <= 32'd0;
        else
            gap_cnt <= gap_cnt + 1'b1;

        // 每秒更新
        if (sec_div >= CLK_HZ-1) begin
            sec_div <= 32'd0;
            if ((state == S_SG) && in_crossing && (cur_sec != 8'hFF))
                cur_sec <= cur_sec + 1'b1;
        end else begin
            if ((state == S_SG) && in_crossing)
                sec_div <= sec_div + 1'b1;
            else if (state != S_SG)
                sec_div <= sec_div + 1'b1;
        end

        // 根据模式不同调整状态机行为
        case(state)
            // 早高峰和晚高峰的固定绿灯时间
            S_MG: begin
                if (mode == 2'b00 || mode == 2'b10) begin // 早高峰 或 晚高峰
                    if (phase_cnt >= MIN_GREEN_C || (phase_cnt >= MAX_GREEN_C)) begin
                        state <= S_MY;
                        phase_cnt <= 32'd0;
                        gap_cnt <= 32'd0;
                        sec_div <= 32'd0;
                    end
                end
                else if (mode == 2'b01) begin // 正常期间
                    if ( (phase_cnt >= MIN_GREEN_C && gap_cnt >= GAP_C) || (phase_cnt >= MAX_GREEN_C) ) begin
                        state <= S_MY;
                        phase_cnt <= 32'd0;
                        gap_cnt <= 32'd0;
                        sec_div <= 32'd0;
                    end
                end
            end

            // 主车道黄灯到全红
            S_MY: begin
                if (phase_cnt >= YELLOW_C) begin
                    state <= S_AR1;
                    phase_cnt <= 32'd0;
                    gap_cnt <= 32'd0;
                    sec_div <= 32'd0;
                end
            end

            // 全红到辅车道绿
            S_AR1: begin
                if (phase_cnt >= ALL_RED_C) begin
                    state <= S_SG;
                    phase_cnt <= 32'd0;
                    gap_cnt <= 32'd0;
                    sec_div <= 32'd0;
                end
            end

            // 辅车道绿到辅车道黄
            S_SG: begin
                if (!det_side && (phase_cnt >= MIN_GREEN_C)) begin
                    if (FAST_RELEASE) begin
                        state <= S_MG;
                        phase_cnt <= 32'd0;
                        gap_cnt <= 32'd0;
                        sec_div <= 32'd0;
                    end else begin
                        state <= S_SY;
                        phase_cnt <= 32'd0;
                        gap_cnt <= 32'd0;
                        sec_div <= 32'd0;
                    end
                end else if (phase_cnt >= MAX_GREEN_C) begin
                    state <= S_SY;
                    phase_cnt <= 32'd0;
                    gap_cnt <= 32'd0;
                    sec_div <= 32'd0;
                end
            end

            // 辅车道黄灯到全红
            S_SY: begin
                if (phase_cnt >= YELLOW_C) begin
                    state <= S_AR2;
                    phase_cnt <= 32'd0;
                    gap_cnt <= 32'd0;
                    sec_div <= 32'd0;
                end
            end

            // 全红到主车道绿
            S_AR2: begin
                if (phase_cnt >= ALL_RED_C) begin
                    state <= S_MG;
                    phase_cnt <= 32'd0;
                    gap_cnt <= 32'd0;
                    sec_div <= 32'd0;
                end
            end
        endcase
    end
end

endmodule
