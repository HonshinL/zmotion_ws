

```
ros2 topic pub /motion_status motion_msgs/msg/MotionStatus "
{
  joint_state: {
    header: {
      stamp: {sec: $(date +%s), nanosec: 0},
      frame_id: 'base_link'
    },
    name: ['axis_0', 'axis_1', 'axis_2', 'axis_4', 'axis_5'],
    position: [100.5, 200.3, 50.7, 0.0, 0.0],
    velocity: [10.2, 5.5, 0.0, 0.0, 0.0],
    effort: []
  }
}" --rate 1
```

ros2 topic pub /motion_status motion_msgs/msg/MotionStatus "
{
  joint_state: {
    header: {
      stamp: {sec: $(date +%s), nanosec: 0},
      frame_id: 'base_link'
    },
    name: ['axis_0', 'axis_1', 'axis_2'],
    position: [100.5, 200.3, 50.7],
    velocity: [10.2, 5.5, 2.1],
    effort: []
  }
}" --once


ros2 topic pub /motion_status motion_msgs/msg/MotionStatus "
{
  joint_state: {
    header: {
      stamp: {sec: $(date +%s), nanosec: $(date +%N)},
      frame_id: 'zmc_frame'
    },
    name: ['axis_0', 'axis_1', 'axis_2'],
    position: [$(($RANDOM % 200)), $(($RANDOM % 300)), $(($RANDOM % 100))],
    velocity: [$(($RANDOM % 20)), $(($RANDOM % 15)), $(($RANDOM % 10))],
    effort: []
  }
}" --once