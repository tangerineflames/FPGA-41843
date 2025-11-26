module traffic_count_map #(
    // 三种模式下的车辆数配置（可以改）
    parameter integer UP_MORNING_PEAK    = 6,   // 早高峰上行车数量
    parameter integer UP_NORMAL          = 3,   // 正常期间上行车数量
    parameter integer UP_EVENING_PEAK    = 6,   // 晚高峰上行车数量
    parameter integer DN_MORNING_PEAK    = 6,   // 早高峰下行车数量
    parameter integer DN_NORMAL          = 3,   // 正常期间下行车数量
    parameter integer DN_EVENING_PEAK    = 6,   // 晚高峰下行车数量
    parameter integer PEOPLE_MORNING_PEAK = 7,  // 早高峰行人数
    parameter integer PEOPLE_NORMAL       = 2,  // 正常期间行人数（少人）
    parameter integer PEOPLE_EVENING_PEAK = 7,  // 晚高峰行人数
    // ★ 新增：NORMAL 模式"多行人"模式的配置
    parameter integer PEOPLE_NORMAL_ALT   = 5   // 正常期间行人数（多人）
)(
    input  wire [1:0] mode,             // 模式号：0=早高峰 1=正常期间 2=晚高峰

    // ★ 新增：只在 NORMAL 模式下生效的拨码开关选择信号
    // 0 -> 使用 PEOPLE_NORMAL（2 人）
    // 1 -> 使用 PEOPLE_NORMAL_ALT（5 人）
    input  wire       ped_cfg_normal_sel,

    output reg  [7:0] people_count,     // 显示行人数
    output reg  [7:0] car_count_up,     // 上行车数量
    output reg  [7:0] car_count_down    // 下行车数量
);
    localparam [1:0] MORNING_PEAK = 2'd0,
                     NORMAL       = 2'd1,
                     EVENING_PEAK = 2'd2;

    always @(*) begin
        case (mode)
            MORNING_PEAK: begin
                car_count_up   = UP_MORNING_PEAK;
                car_count_down = DN_MORNING_PEAK;
                people_count   = PEOPLE_MORNING_PEAK;   // 早高峰：固定 7 人，不看拨码开关
            end
            NORMAL: begin
                car_count_up   = UP_NORMAL;
                car_count_down = DN_NORMAL;
                // ★ 关键：NORMAL 模式下才看 ped_cfg_normal_sel
                people_count   = ped_cfg_normal_sel ? PEOPLE_NORMAL_ALT
                                                    : PEOPLE_NORMAL;
            end
            EVENING_PEAK: begin
                car_count_up   = UP_EVENING_PEAK;
                car_count_down = DN_EVENING_PEAK;
                people_count   = PEOPLE_EVENING_PEAK;   // 晚高峰：固定 7 人
            end
            default: begin
                car_count_up   = UP_NORMAL;
                car_count_down = DN_NORMAL;
                // default 按 NORMAL 模式处理
                people_count   = ped_cfg_normal_sel ? PEOPLE_NORMAL_ALT
                                                    : PEOPLE_NORMAL;
            end
        endcase
    end
endmodule
