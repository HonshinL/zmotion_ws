# 运动类型常量 (对应正运动核心 API)
uint8 TYPE_LINE    = 0   # 直线运动 (使用 MoveSmooth 函数)
uint8 TYPE_ARC     = 1   # 圆弧运动 (使用 MoveCirc 函数)
uint8 TYPE_ELLIPSE = 2   # 椭圆运动 (使用 MEclipse 函数)
uint8 TYPE_SPIRAL  = 3   # 渐开线运动 (使用 MoveSpiral 函数)
uint8 TYPE_SPLINE  = 4   # 样条曲线运动 (使用 MoveSpline 函数)

# 路径段属性
uint8 type                # 运动类型，取值为上述 TYPE_* 常量
geometry_msgs/Point target_pos  # 目标位置坐标
geometry_msgs/Point center_pos  # 中心点坐标：
float64 axis1_radius      # 轴1半径 (仅 TYPE_ELLIPSE 使用)
float64 axis2_radius      # 轴2半径 (仅 TYPE_ELLIPSE 使用)
int8 rot_direction   # 旋转方向：0-逆时针，1-顺时针
uint32 spiral_turns        # 螺旋圈数 (仅 TYPE_SPIRAL 使用)
uint32 segment_id          # 路径段唯一标识符，用于断点续传和错误定位