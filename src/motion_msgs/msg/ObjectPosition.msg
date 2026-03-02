# version: 20250821

float32[] pos       # 表示目标的坐标 0:X; 1:Y; 2:Z

# 0 单轴回零；10 双轴回零(X, Y)
# 1 单轴运动; 2 双轴运动(X, Y)
# 3 振镜运动(Mx, My)
# 4 双轴+振镜 跟随
# 5 双轴+振镜 联动
uint8 mode          # 0:回零; 1:单轴; 2:平面

uint8 axis_num      # 单轴控制轴号，仅供mode 0,1使用

uint8 plane_coord   # 坐标系选择 0 加工幅面 1 振镜 2 加工工件
