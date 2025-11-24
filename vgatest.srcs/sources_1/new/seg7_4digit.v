module seg7_4digit (
    input  wire       clk,       // 用 100MHz sys_clk 就行
    input  wire       rst_n,
    input  wire [7:0] value,     // 0~99，当前绿灯剩余"虚拟秒"

    output reg        SEG_CA,
    output reg        SEG_CB,
    output reg        SEG_CC,
    output reg        SEG_CD,
    output reg        SEG_CE,
    output reg        SEG_CF,
    output reg        SEG_CG,
    output reg        SEG_DP,
    output reg        SEG_BIT1,
    output reg        SEG_BIT2,
    output reg        SEG_BIT3,
    output reg        SEG_BIT4
);
    // ===== 把 value 拆成十位 / 个位 =====
    wire [3:0] ones = value % 10;
    wire [3:0] tens = value / 10;

    // 左两位先空白
    wire [3:0] dig0 = tens;      // 最右边
    wire [3:0] dig1 = ones;
    wire [3:0] dig2 = 4'hF;      // F 代表空白
    wire [3:0] dig3 = 4'hF;

    // ===== 扫描节拍 =====
    reg [15:0] scan_cnt;
    reg [1:0]  scan_sel;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_cnt <= 16'd0;
            scan_sel <= 2'd0;
        end else begin
            scan_cnt <= scan_cnt + 16'd1;
            scan_sel <= scan_cnt[15:14];  // 简单取高两位作扫描
        end
    end

    // 当前选择的数字 & 位选
    reg [3:0] cur_digit;
    always @(*) begin
        // 默认全灭
        SEG_BIT1 = 1'b0;
        SEG_BIT2 = 1'b0;
        SEG_BIT3 = 1'b0;
        SEG_BIT4 = 1'b0;
        cur_digit = 4'hF;

        case (scan_sel)
            2'd0: begin SEG_BIT1 = 1'b1; cur_digit = dig0; end // 右 1
            2'd1: begin SEG_BIT2 = 1'b1; cur_digit = dig1; end // 右 2
            2'd2: begin SEG_BIT3 = 1'b1; cur_digit = dig2; end // 左 2
            2'd3: begin SEG_BIT4 = 1'b1; cur_digit = dig3; end // 左 1
        endcase
    end

    // ===== 7 段译码：这里假设"共阴极，段选高电平点亮" =====
    reg [7:0] segs;  // {CA,CB,CC,CD,CE,CF,CG,DP}
    always @(*) begin
        case (cur_digit)
            4'd0: segs = 8'b11111100;
            4'd1: segs = 8'b01100000;
            4'd2: segs = 8'b11011010;
            4'd3: segs = 8'b11110010;
            4'd4: segs = 8'b01100110;
            4'd5: segs = 8'b10110110;
            4'd6: segs = 8'b10111110;
            4'd7: segs = 8'b11100000;
            4'd8: segs = 8'b11111110;
            4'd9: segs = 8'b11110110;
            4'hF: segs = 8'b00000000;      // 空白
            default: segs = 8'b00000000;
        endcase
    end

    always @(*) begin
        SEG_CA = segs[7];
        SEG_CB = segs[6];
        SEG_CC = segs[5];
        SEG_CD = segs[4];
        SEG_CE = segs[3];
        SEG_CF = segs[2];
        SEG_CG = segs[1];
        SEG_DP = segs[0];   // 小数点常灭
    end
endmodule
