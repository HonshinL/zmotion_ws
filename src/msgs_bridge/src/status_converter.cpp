#include "rclcpp/rclcpp.hpp"
#include "motion_msgs/msg/motion_status.hpp"
#include "motion_msgs/msg/axis_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class StatusConverter : public rclcpp::Node {
public:
    StatusConverter() : Node("status_converter_node") {
        // 创建发布者，发布到 axis_status 话题
        axis_status_pub_ = this->create_publisher<motion_msgs::msg::AxisStatus>("axis_status", 10);

        // 创建订阅者，订阅 motion_status 话题
        motion_status_sub_ = this->create_subscription<motion_msgs::msg::MotionStatus>(
            "motion_status", 10,
            std::bind(&StatusConverter::motion_status_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "状态转换节点已启动: motion_status -> axis_status");
    }

private:
    void motion_status_callback(const motion_msgs::msg::MotionStatus::SharedPtr msg) {
        // 创建 AxisStatus 消息
        auto axis_status_msg = motion_msgs::msg::AxisStatus();
        
        // 转换逻辑：从 MotionStatus 的 JointState 提取数据到 AxisStatus
        const auto& joint_state = msg->joint_state;
        
        // 提取轴状态和位置信息
        if (!joint_state.name.empty() && !joint_state.position.empty()) {
            // 假设每个轴的状态为0（正常状态），位置从joint_state.position获取
            for (size_t i = 0; i < joint_state.name.size(); ++i) {
                // 添加轴状态（默认正常状态）
                axis_status_msg.status.push_back(0);
                
                // 添加轴位置
                axis_status_msg.pos.push_back(static_cast<float>(joint_state.position[i]));
            }
            
            // 计算合成速度（所有轴速度的绝对值之和）
            float total_speed = 0.0f;
            if (!joint_state.velocity.empty()) {
                for (size_t i = 0; i < joint_state.velocity.size(); ++i) {
                    total_speed += std::abs(static_cast<float>(joint_state.velocity[i]));
                }
            }
            axis_status_msg.speed = total_speed;
            
            RCLCPP_DEBUG(this->get_logger(), "转换成功: %zu 个轴, 合成速度: %.3f", 
                        axis_status_msg.status.size(), axis_status_msg.speed);
        } else {
            RCLCPP_WARN(this->get_logger(), "MotionStatus消息中缺少轴数据");
        }

        // 发布转换后的 AxisStatus 消息
        axis_status_pub_->publish(axis_status_msg);
    }

    rclcpp::Publisher<motion_msgs::msg::AxisStatus>::SharedPtr axis_status_pub_;
    rclcpp::Subscription<motion_msgs::msg::MotionStatus>::SharedPtr motion_status_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusConverter>());
    rclcpp::shutdown();
    return 0;
}