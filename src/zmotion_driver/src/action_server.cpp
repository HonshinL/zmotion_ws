#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "motion_msgs/action/move_to_position.hpp"
#include "zmotion_driver/zmcaux.h"

class ZMotionActionServer : public rclcpp::Node {
public:
    using MoveToPosition = motion_msgs::action::MoveToPosition;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveToPosition>;

    explicit ZMotionActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("zmotion_action_server", options) {
        
        // 1. 连接控制器
        char ip[] = "192.168.0.11";
        if (ZAux_OpenEth(ip, &handle_) != 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ 无法连接到 ZMotion 控制器 %s", ip);
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ 成功连接到控制器");
        }

        // 2. 创建 Action Server
        this->action_server_ = rclcpp_action::create_server<MoveToPosition>(
            this,
            "move_to_position",
            std::bind(&ZMotionActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ZMotionActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ZMotionActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    ZMC_HANDLE handle_ = nullptr;
    rclcpp_action::Server<MoveToPosition>::SharedPtr action_server_;

    // 接收目标：检查参数
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToPosition::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "收到运动请求，轴数量: %zu", goal->target_axes.size());
        (void)uuid;
        if (goal->target_axes.size() != goal->target_positions.size()) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 处理取消
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "接收到取消请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 接受后开始执行
    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
        // 必须在独立线程中执行，否则会阻塞 ROS 2 的 Executor
        std::thread{std::bind(&ZMotionActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "开始执行运动...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveToPosition::Feedback>();
        auto result = std::make_shared<MoveToPosition::Result>();

        // 记录初始位置计算进度
        std::vector<float> start_positions = get_mpos(goal->target_axes);

        // 1. 设置运动参数并启动
        for (size_t i = 0; i < goal->target_axes.size(); ++i) {
            int axis = goal->target_axes[i];
            ZAux_Direct_SetSpeed(handle_, axis, goal->speed);
            ZAux_Direct_SetAccel(handle_, axis, goal->acceleration);
            ZAux_Direct_SetDecel(handle_, axis, goal->deceleration);
            ZAux_Direct_Single_MoveAbs(handle_, axis, goal->target_positions[i]);
        }

        // 2. 监控循环
        rclcpp::Rate loop_rate(10); // 10Hz
        while (rclcpp::ok()) {
            // 检查是否有取消请求
            if (goal_handle->is_canceling()) {
                for (auto axis : goal->target_axes) ZAux_Direct_Single_Cancel(handle_, axis, 2);
                result->success = false;
                result->message = "运动被中止";
                goal_handle->canceled(result);
                return;
            }

            std::vector<float> current_pos = get_mpos(goal->target_axes);
            
            // 计算进度
            float total_prog = 0.0;
            for (size_t i = 0; i < goal->target_axes.size(); ++i) {
                float dist = std::abs(goal->target_positions[i] - start_positions[i]);
                if (dist < 0.001) total_prog += 1.0;
                else total_prog += std::min(1.0f, std::abs(current_pos[i] - start_positions[i]) / dist);
            }

            feedback->current_positions = current_pos;
            feedback->progress = total_prog / goal->target_axes.size();
            feedback->current_status = "Executing";
            goal_handle->publish_feedback(feedback);

            // 检查是否全部停止
            bool all_idle = true;
            for (auto axis : goal->target_axes) {
                int idle = 0;
                ZAux_Direct_GetIfIdle(handle_, axis, &idle);
                if (idle == 0) { all_idle = false; break; }
            }

            if (all_idle) break;
            loop_rate.sleep();
        }

        // 3. 任务完成
        if (rclcpp::ok()) {
            result->success = true;
            result->final_positions = get_mpos(goal->target_axes);
            result->end_time = this->now();
            result->message = "成功到达目标位置";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "运动完成");
        }
    }

    // 辅助函数：获取位置
    std::vector<float> get_mpos(const std::vector<int32_t>& axes) {
        float buffer[16];
        ZAux_GetModbusMpos(handle_, 16, buffer);
        std::vector<float> res;
        for (auto a : axes) res.push_back(buffer[a]);
        return res;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZMotionActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}