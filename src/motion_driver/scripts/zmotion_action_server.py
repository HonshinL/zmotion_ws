#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
import time
import ctypes
import math

# 导入自定义接口
from zmotion_driver.action import MoveToPosition
from zmotion_driver.zmotion.zmcaux import ZAUXDLL

class ZMotionActionServer(Node):
    def __init__(self):
        super().__init__('zmotion_action_server')
        
        # 1. 初始化控制器
        self.zaux = ZAUXDLL()
        # 请根据实际情况修改IP
        if self.zaux.ZAux_OpenEth("192.168.0.11") != 0:
            self.get_logger().error("❌ 无法连接到 ZMotion 控制器!")
        
        self._action_server = ActionServer(
            self,
            MoveToPosition,
            'move_to_position',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info("✅ ZMotion 多轴运动服务已启动")

    def goal_callback(self, goal_request):
        """处理目标接收：检查参数合法性"""
        if len(goal_request.target_axes) != len(goal_request.target_positions):
            self.get_logger().warn("⚠️ 拒绝目标：轴号列表与位置列表长度不一致")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """处理取消请求"""
        self.get_logger().info("🛑 接收到取消请求，正在停止电机...")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """核心执行逻辑"""
        request = goal_handle.request
        feedback_msg = MoveToPosition.Feedback()
        result = MoveToPosition.Result()
        
        # 获取初始位置用于计算进度
        initial_positions = self._get_batch_mpos(request.target_axes)
        
        # 2. 配置并启动多轴运动
        for i, axis in enumerate(request.target_axes):
            self.zaux.ZAux_Direct_SetSpeed(axis, request.speed)
            self.zaux.ZAux_Direct_SetAccel(axis, request.acceleration)
            self.zaux.ZAux_Direct_SetDecel(axis, request.deceleration)
            # 启动绝对运动
            self.zaux.ZAux_Direct_Single_MoveAbs(axis, request.target_positions[i])

        # 3. 监控循环
        self.get_logger().info(f"开始移动轴: {request.target_axes}")
        
        start_time = self.get_clock().now()
        
        while True:
            # A. 检查用户是否取消
            if goal_handle.is_cancel_requested:
                for axis in request.target_axes:
                    self.zaux.ZAux_Direct_Single_Cancel(axis, 2) # 2-减速停止
                goal_handle.canceled()
                result.success = False
                result.message = "运动被用户取消"
                return result

            # B. 读取实时数据
            current_mpos = self._get_batch_mpos(request.target_axes)
            
            # C. 计算进度 (取所有轴进度的平均值或最小值)
            total_progress = 0.0
            for i in range(len(request.target_axes)):
                total_dist = abs(request.target_positions[i] - initial_positions[i])
                if total_dist == 0:
                    prog = 1.0
                else:
                    moved_dist = abs(current_mpos[i] - initial_positions[i])
                    prog = min(moved_dist / total_dist, 1.0)
                total_progress += prog
            
            # D. 发布反馈
            feedback_msg.current_positions = current_mpos
            feedback_msg.progress = total_progress / len(request.target_axes)
            feedback_msg.current_status = "Moving"
            goal_handle.publish_feedback(feedback_msg)

            # E. 检查是否所有轴都已停止 (Idle)
            all_idle = True
            for axis in request.target_axes:
                idle_status = ctypes.c_int()
                self.zaux.ZAux_Direct_GetIfIdle(axis, ctypes.byref(idle_status))
                if idle_status.value == 0: # 0 代表正在运动
                    all_idle = False
                    break
            
            if all_idle:
                break
            
            time.sleep(0.1) # 10Hz 监控频率

        # 4. 完成任务
        goal_handle.succeed()
        result.success = True
        result.end_time = self.get_clock().now().to_msg()
        result.final_positions = self._get_batch_mpos(request.target_axes)
        result.message = "所有轴已成功到达目标位置"
        
        return result

    def _get_batch_mpos(self, axes):
        """辅助函数：批量获取指定轴的位置"""
        # 为了简单，假设控制器最多有16个轴，我们读前16个
        mpos_buff = (ctypes.c_float * 16)()
        self.zaux.ZAux_GetModbusMpos(16, mpos_buff)
        return [mpos_buff[axis] for axis in axes]

def main(args=None):
    rclpy.init(args=args)
    server = ZMotionActionServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.zaux.ZAux_Close()
        rclpy.shutdown()