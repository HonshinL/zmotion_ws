#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "slms_interface/msg/object_position.hpp"
#include "motion_msgs/action/axes_moving.hpp"
#include "motion_msgs/action/axes_homing.hpp"

class ObjectPositionToActionConverter : public rclcpp::Node {
public:
    using AxesMoving = motion_msgs::action::AxesMoving;
    using GoalHandleMove = rclcpp_action::ClientGoalHandle<AxesMoving>;
    using AxesHoming = motion_msgs::action::AxesHoming;
    using GoalHandleHome = rclcpp_action::ClientGoalHandle<AxesHoming>;

    explicit ObjectPositionToActionConverter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("object_position_to_action_converter", options) {
        
        // 创建 ObjectPosition 消息订阅者
        object_position_sub_ = this->create_subscription<slms_interface::msg::ObjectPosition>(
            "/app_pos", 10,
            std::bind(&ObjectPositionToActionConverter::object_position_callback, this, std::placeholders::_1));
        
        // 创建 AxesMoving Action 客户端
        action_client_ = rclcpp_action::create_client<AxesMoving>(this, "zmc_act/axes_moving");
        
        // 创建 AxesHoming Action 客户端
        homing_client_ = rclcpp_action::create_client<AxesHoming>(this, "zmc_act/axes_homing");
        
        RCLCPP_INFO(this->get_logger(), "ObjectPosition 到 Action 转换节点已启动");
        RCLCPP_INFO(this->get_logger(), "订阅话题: /app_pos");
        RCLCPP_INFO(this->get_logger(), "发布 Action: zmc_act/axes_moving, zmc_act/axes_homing");
    }

private:
    void object_position_callback(const slms_interface::msg::ObjectPosition::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到 ObjectPosition 消息，模式: %d, 轴号: %d, 坐标系: %d", 
                   msg->mode, msg->axis_num, msg->plane_coord);
        
        // 检查消息有效性
        if (msg->pos.empty()) {
            RCLCPP_ERROR(this->get_logger(), "ObjectPosition 消息中位置数据为空");
            return;
        }
        
        // 根据模式处理不同的运动类型
        switch (msg->mode) {
            case 0: // 单轴回零
                RCLCPP_INFO(this->get_logger(), "模式0: 单轴回零，轴号: %d", msg->axis_num);
                send_homing_action(msg->axis_num);
                break;
                
            case 1: // 单轴运动
                if (msg->pos.size() >= 1) {
                    send_single_axis_action(msg->axis_num, msg->pos);
                }
                break;
                
            case 2: // 双轴运动(X, Y)
                if (msg->pos.size() >= 2) {
                    RCLCPP_INFO(this->get_logger(), "模式2: 双轴运动，X: %.3f, Y: %.3f", 
                               msg->pos[0], msg->pos[1]);
                    send_multi_axis_action({0, 1}, {msg->pos[0], msg->pos[1]});
                }
                break;
                
            case 3: // 振镜运动
                RCLCPP_INFO(this->get_logger(), "模式3: 振镜运动");
                // 这里可以实现振镜控制逻辑
                break;
                
            default:
                RCLCPP_WARN(this->get_logger(), "未知的运动模式: %d", msg->mode);
                break;
        }
    }
    
    // 发送单轴运动 Action
    void send_single_axis_action(int32_t axis_num, const std::vector<float>& positions) {
        // 检查轴号是否有效
        int32_t actual_axis = axis_num;
        float target_position = 0.0;
        
        if (axis_num == 0) {
            // 如果轴号是0，取位置数组的第一个值
            if (positions.empty()) {
                RCLCPP_ERROR(this->get_logger(), "位置数组为空");
                return;
            }
            target_position = positions[0];
        } else if (axis_num == 1) {
            // 如果轴号是1，则驱动轴2
            actual_axis = 2;
            RCLCPP_INFO(this->get_logger(), "轴号1被映射到轴2");
            // 位置数组如果只有一个值，则取该值，如果有2个值，则取第二个值
            if (positions.empty()) {
                RCLCPP_ERROR(this->get_logger(), "位置数组为空");
                return;
            } else if (positions.size() >= 2) {
                target_position = positions[1];
            } else {
                target_position = positions[0];
            }
        } else if (axis_num != 2 && axis_num != 4 && axis_num != 5) {
            // 只接受0, 2, 4, 5中的轴号
            RCLCPP_ERROR(this->get_logger(), "无效的轴号: %d，只支持0, 2, 4, 5", axis_num);
            return;
        } else {
            // 其他有效轴号，取位置数组的第一个值
            if (positions.empty()) {
                RCLCPP_ERROR(this->get_logger(), "位置数组为空");
                return;
            }
            target_position = positions[0];
        }
        
        // 记录单轴运动信息
        RCLCPP_INFO(this->get_logger(), "模式1: 单轴运动，轴号: %d, 目标位置: %.3f", 
                    actual_axis, target_position);
        
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action 服务器不可用");
            return;
        }
        
        auto goal_msg = AxesMoving::Goal();
        goal_msg.target_axes = {static_cast<int64_t>(actual_axis)};
        goal_msg.target_positions = {static_cast<double>(target_position)};
        
        auto send_goal_options = rclcpp_action::Client<AxesMoving>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&ObjectPositionToActionConverter::move_goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&ObjectPositionToActionConverter::move_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&ObjectPositionToActionConverter::move_result_callback, this, std::placeholders::_1);
        
        RCLCPP_INFO(this->get_logger(), "发送单轴运动 Action: 轴%d -> %.3f", actual_axis, target_position);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    // 发送多轴运动 Action
    void send_multi_axis_action(const std::vector<int32_t>& axes, const std::vector<double>& positions) {
        // 检查轴号是否有效
        std::vector<int32_t> actual_axes;
        actual_axes.reserve(axes.size());
        
        for (int32_t axis_num : axes) {
            if (axis_num == 1) {
                // 如果轴号是1，则驱动轴2
                actual_axes.push_back(2);
                RCLCPP_INFO(this->get_logger(), "轴号1被映射到轴2");
            } else if (axis_num != 0 && axis_num != 2 && axis_num != 4 && axis_num != 5) {
                // 只接受0, 2, 4, 5中的轴号
                RCLCPP_ERROR(this->get_logger(), "无效的轴号: %d，只支持0, 2, 4, 5", axis_num);
                return;
            } else {
                actual_axes.push_back(axis_num);
            }
        }
        
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action 服务器不可用");
            return;
        }
        
        auto goal_msg = AxesMoving::Goal();
        // 将std::vector<int>转换为std::vector<int64_t>
        std::vector<int64_t> target_axes_long;
        target_axes_long.reserve(actual_axes.size());
        for (int axis : actual_axes) {
            target_axes_long.push_back(static_cast<int64_t>(axis));
        }
        goal_msg.target_axes = target_axes_long;
        // 将std::vector<float>转换为std::vector<double>
        std::vector<double> positions_double;
        positions_double.reserve(positions.size());
        for (float pos : positions) {
            positions_double.push_back(static_cast<double>(pos));
        }
        goal_msg.target_positions = positions_double;
        // 速度、加速度和减速度从参数服务器读取，不在action消息中传递
        
        auto send_goal_options = rclcpp_action::Client<AxesMoving>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&ObjectPositionToActionConverter::move_goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&ObjectPositionToActionConverter::move_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&ObjectPositionToActionConverter::move_result_callback, this, std::placeholders::_1);
        
        // 构建轴号和位置的字符串
        std::string axes_str = "[";
        std::string positions_str = "[";
        for (size_t i = 0; i < actual_axes.size(); ++i) {
            axes_str += std::to_string(actual_axes[i]);
            positions_str += std::to_string(positions[i]);
            if (i < actual_axes.size() - 1) {
                axes_str += ", ";
                positions_str += ", ";
            }
        }
        axes_str += "]";
        positions_str += "]";
        
        RCLCPP_INFO(this->get_logger(), "发送多轴运动 Action: 轴%s -> %s", axes_str.c_str(), positions_str.c_str());
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    // 发送单轴回零 Action
    void send_homing_action(int32_t axis_num) {
        if (!homing_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Homing Action 服务器不可用");
            return;
        }
        
        auto goal_msg = AxesHoming::Goal();
        goal_msg.homing_axes = {static_cast<int64_t>(axis_num)};
        // 其他参数从参数服务器读取，不在action消息中传递
        
        auto send_goal_options = rclcpp_action::Client<AxesHoming>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&ObjectPositionToActionConverter::homing_goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&ObjectPositionToActionConverter::homing_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&ObjectPositionToActionConverter::homing_result_callback, this, std::placeholders::_1);
        
        RCLCPP_INFO(this->get_logger(), "发送单轴回零 Action: 轴%d", axis_num);
        homing_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    // 运动 Action 回调函数
    void move_goal_response_callback(const GoalHandleMove::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "运动目标被拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "运动目标被接受");
        }
    }
    
    void move_feedback_callback(
        GoalHandleMove::SharedPtr,
        const std::shared_ptr<const AxesMoving::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "运动进度: %.1f%%", feedback->progress * 100);
    }
    
    void move_result_callback(const GoalHandleMove::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "运动完成，结果: %s", result.result->success ? "成功" : "失败");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "运动被中止");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "运动被取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知结果");
                break;
        }
    }
    
    // Homing Action 回调函数
    void homing_goal_response_callback(const GoalHandleHome::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "回零目标被拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "回零目标被接受");
        }
    }
    
    void homing_feedback_callback(
        GoalHandleHome::SharedPtr,
        const std::shared_ptr<const AxesHoming::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "回零进度: %s, 耗时: %.1f 秒", 
                   feedback->current_phase.c_str(), feedback->elapsed_time);
    }
    
    void homing_result_callback(const GoalHandleHome::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "回零完成，结果: %s", result.result->success ? "成功" : "失败");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "回零被中止");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "回零被取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知结果");
                break;
        }
    }
    
    rclcpp::Subscription<slms_interface::msg::ObjectPosition>::SharedPtr object_position_sub_;
    rclcpp_action::Client<AxesMoving>::SharedPtr action_client_;
    rclcpp_action::Client<AxesHoming>::SharedPtr homing_client_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectPositionToActionConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}