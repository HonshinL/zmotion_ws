#include <iostream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "motion_msgs/action/axis_homing.hpp"

using AxisHoming = motion_msgs::action::AxisHoming;
using GoalHandleHoming = rclcpp_action::ClientGoalHandle<AxisHoming>;

class HomingTestClient : public rclcpp::Node {
public:
    explicit HomingTestClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("homing_test_client", options) {
        
        // 创建Action客户端
        this->action_client_ = rclcpp_action::create_client<AxisHoming>(
            this, "zmc_act/axis_homing");
        
        RCLCPP_INFO(this->get_logger(), "AxisHoming Action客户端已启动");
    }
    
    void send_goal(int axis_id, float velocity_high, float velocity_low, int homing_mode, float timeout = 60.0) {
        // 等待Action服务器
        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action服务器不可用");
            return;
        }
        
        // 创建目标
        auto goal_msg = AxisHoming::Goal();
        goal_msg.axis_id = axis_id;
        goal_msg.velocity_high = velocity_high;
        goal_msg.velocity_low = velocity_low;
        goal_msg.homing_mode = homing_mode;
        goal_msg.timeout = timeout;
        
        RCLCPP_INFO(this->get_logger(), "发送回零请求: 轴=%d, 高速=%.3f, 低速=%.3f, 模式=%d, 超时=%.1f秒", 
                   axis_id, velocity_high, velocity_low, homing_mode, timeout);
        
        // 发送目标
        auto send_goal_options = rclcpp_action::Client<AxisHoming>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&HomingTestClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&HomingTestClient::result_callback, this, std::placeholders::_1);
        
        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
private:
    rclcpp_action::Client<AxisHoming>::SharedPtr action_client_;
    
    void feedback_callback(
        GoalHandleHoming::SharedPtr, 
        const std::shared_ptr<const AxisHoming::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "回零反馈: 状态=%s, 位置=%.3f, 驱动器状态=%d, 已执行时间=%.1f秒", 
                   feedback->current_state.c_str(), feedback->current_pos, feedback->drive_status, feedback->elapsed_time);
    }
    
    void result_callback(const GoalHandleHoming::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "回零成功! 最终位置: %.3f, 错误码: %d", result.result->final_pos, result.result->error_code);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "回零失败: %s, 错误码: %d", result.result->message.c_str(), result.result->error_code);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "回零被取消, 错误码: %d", result.result->error_code);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "回零结果未知");
                break;
        }
        
        // 退出节点
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<HomingTestClient>();
    
    // 测试参数
    int axis_id = 0;
    float velocity_high = 50.0;
    float velocity_low = 10.0;
    int homing_mode = 11; // 回零模式11
    float timeout = 60.0; // 超时时间60秒
    
    // 发送回零请求
    node->send_goal(axis_id, velocity_high, velocity_low, homing_mode, timeout);
    
    // 运行节点
    rclcpp::spin(node);
    
    return 0;
}