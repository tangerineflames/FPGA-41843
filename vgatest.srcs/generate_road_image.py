import numpy as np
from PIL import Image

# 分辨率
H_VALID = 640
V_VALID = 480

# RGB565转RGB888
def rgb565_to_rgb888(color565):
    r5 = (color565 >> 11) & 0x1F
    g6 = (color565 >> 5) & 0x3F
    b5 = color565 & 0x1F
    r8 = (r5 << 3) | (r5 >> 2)
    g8 = (g6 << 2) | (g6 >> 4)
    b8 = (b5 << 3) | (b5 >> 2)
    return (r8, g8, b8)

# 颜色定义（RGB565格式）
BLACK = 0x0000
WHITE = 0xFFFF
YELLOW = 0xFFE0
RED = 0xF800
GREEN = 0x07E0

# 转换为RGB888
COLOR_BLACK = rgb565_to_rgb888(BLACK)
COLOR_WHITE = rgb565_to_rgb888(WHITE)
COLOR_YELLOW = rgb565_to_rgb888(YELLOW)
COLOR_RED = rgb565_to_rgb888(RED)
COLOR_GREEN = rgb565_to_rgb888(GREEN)

# 道路参数（水平方向）
Y_TOP = 190
Y_BOTTOM = 310
SIDE_H = 3
MID_H = 2
Y_MID = (Y_TOP + Y_BOTTOM) >> 1
H_DASH_PERIOD = 64
H_DASH_ON = 24

# 道路参数（垂直方向）
X_LEFT = 300
X_RIGHT = 340
V_SIDE_W = 3
X_MID = (X_LEFT + X_RIGHT) >> 1

# 交通灯参数
LAMP_W = 14
LAMP_H = 14
CAR_LAMP_X0 = (H_VALID >> 1) - (LAMP_W >> 1)
CAR_LAMP_X1 = CAR_LAMP_X0 + LAMP_W
CAR_LAMP_Y0 = Y_MID - (LAMP_H >> 1)
CAR_LAMP_Y1 = CAR_LAMP_Y0 + LAMP_H

PED_LAMP_X0 = X_RIGHT + 6
PED_LAMP_X1 = PED_LAMP_X0 + LAMP_W
PED_LAMP_Y0 = Y_TOP - 20
PED_LAMP_Y1 = PED_LAMP_Y0 + LAMP_H

# 创建图像
img = np.zeros((V_VALID, H_VALID, 3), dtype=np.uint8)

# 填充背景（黑色）
img[:, :] = COLOR_BLACK

# 绘制道路
for y in range(V_VALID):
    for x in range(H_VALID):
        # 判断是否在各个边线上
        top_side = (y >= Y_TOP - SIDE_H) and (y <= Y_TOP + SIDE_H)
        bot_side = (y >= Y_BOTTOM - SIDE_H) and (y <= Y_BOTTOM + SIDE_H)
        left_side = (x >= X_LEFT - V_SIDE_W) and (x <= X_LEFT + V_SIDE_W)
        right_side = (x >= X_RIGHT - V_SIDE_W) and (x <= X_RIGHT + V_SIDE_W)
        h_mid_band = (y >= Y_MID - MID_H) and (y <= Y_MID + MID_H)
        h_dash_on = (x % H_DASH_PERIOD) < H_DASH_ON
        
        # 水平道路边线（白色），但不包括十字路口中心
        if (top_side or bot_side) and not (x >= X_LEFT and x <= X_RIGHT):
            img[y, x] = COLOR_WHITE
        
        # 垂直道路边线（白色），但不包括十字路口中心
        if (left_side or right_side) and not (y >= Y_TOP and y <= Y_BOTTOM):
            img[y, x] = COLOR_WHITE
        
        # 中线黄色虚线（不覆盖边线）
        if h_mid_band and h_dash_on and not (top_side or bot_side):
            img[y, x] = COLOR_YELLOW

# 绘制交通灯（示例：车道灯为红色，人行灯为绿色）
main_light_color = COLOR_RED  # 可以修改为GREEN或YELLOW
ped_light_color = COLOR_GREEN  # 可以修改为RED

# 车道交通灯
img[CAR_LAMP_Y0:CAR_LAMP_Y1, CAR_LAMP_X0:CAR_LAMP_X1] = main_light_color

# 行人交通灯
img[PED_LAMP_Y0:PED_LAMP_Y1, PED_LAMP_X0:PED_LAMP_X1] = ped_light_color

# 保存图像
output_img = Image.fromarray(img)
output_img.save('road_model.png')
print("道路模型已保存为 road_model.png")
print(f"图像尺寸: {H_VALID}x{V_VALID}")
print(f"道路布局:")
print(f"  - 水平道路: Y {Y_TOP} ~ {Y_BOTTOM}")
print(f"  - 垂直道路: X {X_LEFT} ~ {X_RIGHT}")
print(f"  - 车道交通灯位置: ({CAR_LAMP_X0}, {CAR_LAMP_Y0})")
print(f"  - 行人交通灯位置: ({PED_LAMP_X0}, {PED_LAMP_Y0})")

# 可选：显示图像
# output_img.show()


