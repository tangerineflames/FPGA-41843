module Periodchange
(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       key_second_level,   // 建议接 key_filter.key_flag 脉冲
    output reg  [1:0] modenum             // 0:早高峰 1:正常期间 2:晚高峰
);
    // 状态常量：三种时段
    localparam [1:0] MORNING_PEAK = 2'd0;  // 早高峰
    localparam [1:0] NORMAL       = 2'd1;  // 正常期间
    localparam [1:0] EVENING_PEAK = 2'd2;  // 晚高峰

    // 边沿检测（防止持续高电平多次触发）
    reg key_d1; // 寄存器存上一拍电平
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            key_d1 <= 1'b0;
        else
            key_d1 <= key_second_level;
    end
    wire key_rise = key_second_level & ~key_d1;  // 检测上升沿

    // 主状态机：早高峰 -> 正常 -> 晚高峰 -> 再回到早高峰
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            modenum <= NORMAL; // 默认上电：正常期间
        else if (key_rise) begin
            case(modenum)
                MORNING_PEAK: modenum <= NORMAL;
                NORMAL:       modenum <= EVENING_PEAK;
                default:      modenum <= MORNING_PEAK;   
            endcase
        end
    end
endmodule
