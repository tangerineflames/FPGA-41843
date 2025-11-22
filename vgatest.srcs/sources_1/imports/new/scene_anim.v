// scene_anim.v - 车/行人速度改为可注入
`timescale 1ns/1ps
module scene_anim(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tick,          // ~30Hz
    input  wire       main_R,
    input  wire       main_G,
    input  wire       side_G,
    input  wire [3:0] car_speed,     //新增
    input  wire [3:0] ped_speed,     //新增
    input  wire [7:0] people_count,
    input  wire [9:0] ped_phase_step,
    output reg  [9:0] car_x,
    output reg  [9:0] ped_y
);
    localparam [9:0] HMAX=10'd639, VMAX=10'd479;
    localparam [9:0] CAR_X0=10'd10, PED_Y0=10'd20;
    localparam [3:0] CAR_DEF=4'd2, PED_DEF=4'd1;

    wire [3:0] CS = (car_speed==0)? CAR_DEF : car_speed;
    wire [3:0] PS = (ped_speed==0)? PED_DEF : ped_speed;

    localparam [9:0] Y_TOP       = 10'd190;   // 跟 vga_pic、ped_sensor 保持一致
    localparam [9:0] Y_BOTTOM    = 10'd310;   // 同上
    localparam [9:0] PED_WAIT_Y  = Y_TOP - 10'd18; // 等于 ped_sensor 里的 wait_line
    // 也就是 PED_WAIT_Y = 182
    // ★ 队首 y = ped_y + (people_count-1)*ped_phase_step  (环绕到 0~479)
    wire [10:0] ppl_m1      = (people_count == 0) ? 11'd0 : {3'd0, people_count-1};
    wire [10:0] head_offset = ppl_m1 * ped_phase_step;
    wire [10:0] head_sum    = {1'b0, ped_y} + head_offset;
    
    wire [9:0] head_y = (head_sum >= 11'd480) ? head_sum - 11'd480
                                              : head_sum[9:0];

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            car_x <= CAR_X0;
            ped_y <= PED_Y0;
        end else if (tick) begin
        // 车：绿/黄灯才走，红灯保持当前位置
        if (!main_R) begin
            if (car_x >= (HMAX - CS))
                car_x <= car_x + CS - (HMAX + 10'd1);
            else
                car_x <= car_x + CS;
        end else begin
            car_x <= car_x;   // 红灯：停住
        end
        // =================== 行人 ===================
        if (side_G) begin
            // ★ 行人灯绿：整个队伍都往下走（包括正在过马路的）
            if (ped_y >= (VMAX - PS)) ped_y <= 10'd0;
            else                      ped_y <= ped_y + PS;
        end else begin
            // ★ 行人灯红/黄
            if (ped_y > Y_BOTTOM) begin
                // 整队已经完全过完路口，在下面人行道：继续走
                if (ped_y >= (VMAX - PS)) ped_y <= 10'd0;
                else                      ped_y <= ped_y + PS;
            end else begin
                // 还有人没过完路口：按"队首 head_y"的位置来决定是否还能往前挪
                if (head_y < PED_WAIT_Y) begin
                    // 队首还没到等待线，整队继续向等待线靠近
                    ped_y <= ped_y + PS;
                end else begin
                    // 队首已经到达等待线：整队在红灯期间保持不动
                    ped_y <= ped_y;
                end
            end
        end

        end
    end
endmodule
