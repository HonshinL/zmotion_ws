使用方式
1. 启动系统

bash
# 启动 Action 服务器
ros2 run zmotion_driver zmotion_action_server

# 启动转换节点
ros2 run msgs_bridge object_position_to_action
2. 发布测试消息

bash
# 单轴运动测试
ros2 topic pub /app_pos slms_interface/msg/ObjectPosition "
{
  pos: [100.0],
  mode: 1,
  axis_num: 0,
  plane_coord: 0
}"

# 双轴运动测试
ros2 topic pub /app_pos slms_interface/msg/ObjectPosition "
{
  pos: [100.0, 200.0],
  mode: 2,
  axis_num: 0,
  plane_coord: 0
}"
3. 监控系统状态

bash
# 查看原始消息
ros2 topic echo /app_pos

# 查看 Action 状态
ros2 action list
ros2 action info /move_to_position

# 查看节点日志
ros2 topic echo /rosout
功能特性
1. 完整的消息转换

# 启动 Action 服务器
ros2 run zmotion_driver zmotion_action_server
```

### 测试 Action 功能
```bash
# 查看 Action 列表
ros2 action list

# 查看 Action 信息
ros2 action info /move_to_position

# 发送测试 Action
ros2 action send_goal /move_to_position motion_msgs/action/MoveToPosition "
{
  target_axes: [0, 1],
  target_positions: [100.0, 200.0],
  speed: 50.0,
  acceleration: 100.0,
  deceleration: 100.0
}"
```