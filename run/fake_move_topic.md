


          
我需要先检查 `ObjectPosition.msg` 消息的具体结构，然后为您提供发布模拟消息的命令。让我查看消息定义：
        
            
toolName: view_files
            
status: success
          
            
filePath: /home/hongxin/Projects/10_Github/zmotion_ws/src/motion_msgs/msg/ObjectPosition.msg
          
基于 `ObjectPosition.msg` 的消息结构，我来为您提供几种发布模拟消息的命令：

## ObjectPosition 消息结构分析

```yaml
pos: [float32, float32, float32]  # 目标坐标 [X, Y, Z]
mode: uint8                       # 运动模式 (0:回零, 1:单轴, 2:双轴, 3:振镜)
axis_num: uint8                   # 单轴控制轴号
plane_coord: uint8                # 坐标系选择
```

## 发布模拟消息的命令

### 1. **单轴运动测试（模式1）**

```bash
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "
{
  pos: [100.5],
  mode: 1,
  axis_num: 0,
  plane_coord: 0
}" --once
```

### 2. **双轴运动测试（模式2）**

```bash
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "
{
  pos: [150.0, 200.0],
  mode: 2,
  axis_num: 0,
  plane_coord: 0
}" --once
```

### 3. **带Z轴的三维运动测试**

```bash
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "
{
  pos: [100.0, 200.0, 50.0],
  mode: 2,
  axis_num: 0,
  plane_coord: 0
}" --once
```

### 4. **连续发布测试（每秒1次）**

```bash
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "
{
  pos: [100.0, 200.0],
  mode: 2,
  axis_num: 0,
  plane_coord: 0
}" --rate 1
```

### 5. **随机位置测试**

```bash
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "
{
  pos: [$(($RANDOM % 300)), $(($RANDOM % 400)), $(($RANDOM % 100))],
  mode: 2,
  axis_num: 0,
  plane_coord: 0
}" --once
```

### 6. **回零操作测试（模式0）**

```bash
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "
{
  pos: [0.0],
  mode: 0,
  axis_num: 0,
  plane_coord: 0
}" --once
```

### 7. **振镜运动测试（模式3）**

```bash
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "
{
  pos: [10.5, 15.3],
  mode: 3,
  axis_num: 0,
  plane_coord: 1
}" --once
```

## 完整的测试脚本

创建一个测试脚本 `test_object_position.sh`：

```bash
#!/bin/bash

echo "开始测试 ObjectPosition 消息发布..."

# 测试1：单轴运动
echo "测试1: 单轴运动 (X轴移动到100)"
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "
{
  pos: [100.0],
  mode: 1,
  axis_num: 0,
  plane_coord: 0
}" --once
sleep 2

# 测试2：双轴运动  
echo "测试2: 双轴运动 (X=150, Y=200)"
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "
{
  pos: [150.0, 200.0],
  mode: 2,
  axis_num: 0,
  plane_coord: 0
}" --once
sleep 2

# 测试3：三维运动
echo "测试3: 三维运动 (X=100, Y=200, Z=50)"
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "
{
  pos: [100.0, 200.0, 50.0],
  mode: 2,
  axis_num: 0,
  plane_coord: 0
}" --once
sleep 2

# 测试4：回零操作
echo "测试4: 回零操作 (轴0回零)"
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "
{
  pos: [0.0],
  mode: 0,
  axis_num: 0,
  plane_coord: 0
}" --once

echo "测试完成！"
```

运行脚本：
```bash
chmod +x test_object_position.sh
./test_object_position.sh
```

## 监控和验证命令

### 监听消息
```bash
# 监听原始 ObjectPosition 消息
ros2 topic echo /app_pos

# 监听转换后的 Action 请求
ros2 action list
ros2 action info /move_to_position
```

### 查看系统状态
```bash
# 查看节点日志
ros2 topic echo /rosout | grep -E "(object_position|move_to_position)"

# 查看话题信息
ros2 topic info /app_pos
ros2 topic hz /app_pos
```

## 模式说明参考

根据消息注释，各模式的含义：

- **模式0**: 单轴回零（`axis_num` 指定要回零的轴）
- **模式1**: 单轴运动（`axis_num` 指定要运动的轴，`pos[0]` 为目标位置）
- **模式2**: 双轴运动（`pos[0]`=X轴目标，`pos[1]`=Y轴目标）
- **模式3**: 振镜运动（`pos[0]`=Mx目标，`pos[1]`=My目标）
- **模式4**: 双轴+振镜跟随
- **模式5**: 双轴+振镜联动

## 快速测试命令

最简单的测试命令：
```bash
# 快速双轴运动测试
ros2 topic pub /app_pos motion_msgs/msg/ObjectPosition "{pos: [100, 200], mode: 2, axis_num: 0, plane_coord: 0}" --once
```

这些命令可以帮助您测试 `ObjectPosition` 消息的发布和转换功能。选择适合您需求的命令进行测试！
        