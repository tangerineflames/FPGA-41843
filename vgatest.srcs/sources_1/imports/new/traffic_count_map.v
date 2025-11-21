module traffic_count_map #(
    // 三种模式下的车辆数配置（可以改）
    parameter integer UP_MORNING_PEAK   = 6,   // 早高峰上行车数量
    parameter integer UP_NORMAL          = 3,   // 正常期间上行车数量
    parameter integer UP_EVENING_PEAK    = 6,   // 晚高峰上行车数量
    parameter integer DN_MORNING_PEAK    = 6,   // 早高峰下行车数量
    parameter integer DN_NORMAL          = 3,   // 正常期间下行车数量
    parameter integer DN_EVENING_PEAK    = 6,   // 晚高峰下行车数量
    parameter integer PEOPLE_MORNING_PEAK = 7,   // 早高峰行人数
    parameter integer PEOPLE_NORMAL       = 2,   // 正常期间行人数
    parameter integer PEOPLE_EVENING_PEAK = 7    // 晚高峰行人数
)(
    input  wire [1:0] mode,             // 模式号：0=早高峰 1=正常期间 2=晚高峰
    output reg  [7:0] people_count,     // 显示行人数
    output reg  [7:0] car_count_up,     // 上行车数量
    output reg  [7:0] car_count_down    // 下行车数量
);
    localparam [1:0] MORNING_PEAK = 2'd0, NORMAL = 2'd1, EVENING_PEAK = 2'd2;

    always @(*) begin
        case (mode)
            MORNING_PEAK: begin
                car_count_up   = UP_MORNING_PEAK;
                car_count_down = DN_MORNING_PEAK;
                people_count   = PEOPLE_MORNING_PEAK;
            end
            NORMAL: begin
                car_count_up   = UP_NORMAL;
                car_count_down = DN_NORMAL;
                people_count   = PEOPLE_NORMAL;
            end
            EVENING_PEAK: begin
                car_count_up   = UP_EVENING_PEAK;
                car_count_down = DN_EVENING_PEAK;
                people_count   = PEOPLE_EVENING_PEAK;
            end
            default: begin
                car_count_up   = UP_NORMAL;
                car_count_down = DN_NORMAL;
                people_count   = PEOPLE_NORMAL;
            end
        endcase
    end
endmodule
