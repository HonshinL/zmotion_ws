#include "rclcpp/rclcpp.hpp"
#include "zmotion_driver/zmc_controller.h"
#include <iostream>
#include <string>

class TestMoveNode : public rclcpp::Node {
public:
    TestMoveNode() : Node("test_move_node") {
        // 初始化ZMC控制器
        controller_ = std::make_shared<ZmcController>();
        
        // 连接到控制器
        std::string ip = this->declare_parameter("controller_ip", "192.168.0.11");
        if (controller_->connect(ip)) {
            RCLCPP_INFO(this->get_logger(), "成功连接到ZMC控制器: %s", ip.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "连接ZMC控制器失败: %s", ip.c_str());
            rclcpp::shutdown();
        }
    }
    
    ~TestMoveNode() {
        controller_->disconnect();
    }
    
    void testMove(const std::string& move_type) {
        RCLCPP_INFO(this->get_logger(), "测试运动类型: %s", move_type.c_str());
        
        if (move_type == "line") {
            testLineMove();
        } else if (move_type == "arc") {
            testArcMove();
        } else if (move_type == "path") {
            testPathMove();
        } else {
            RCLCPP_ERROR(this->get_logger(), "未知的运动类型: %s", move_type.c_str());
            RCLCPP_INFO(this->get_logger(), "支持的运动类型: line, arc, path");
        }
    }
    
private:
    void testLineMove() {
        RCLCPP_INFO(this->get_logger(), "测试直线运动");
        
        // 测试单轴运动
        std::vector<int64_t> axes = {0};
        std::vector<double> positions = {100.0};
        
        if (controller_->moveAxes(axes, positions)) {
            RCLCPP_INFO(this->get_logger(), "单轴直线运动测试成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "单轴直线运动测试失败");
        }
        
        // 测试多轴运动
        axes = {0, 2};
        positions = {50.0, 50.0};
        
        if (controller_->moveAxes(axes, positions)) {
            RCLCPP_INFO(this->get_logger(), "多轴直线运动测试成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "多轴直线运动测试失败");
        }
    }
    
    void testArcMove() {
        RCLCPP_INFO(this->get_logger(), "测试圆弧运动");
        
        // 创建action客户端
        auto action_client = rclcpp_action::create_client<motion_msgs::action::MovePath>(
            this, "move_path");
        
        // 等待action服务器可用
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action服务器不可用");
            return;
        }
        
        // 创建圆弧运动的路径段
        auto goal_msg = motion_msgs::action::MovePath::Goal();
        
        // 创建圆弧段
        motion_msgs::msg::PathSegment arc_segment;
        arc_segment.type = 1; // TYPE_ARC
        arc_segment.target_pos.x = 100.0;
        arc_segment.target_pos.y = 0.0;
        arc_segment.center_pos.x = 50.0;
        arc_segment.center_pos.y = 50.0;
        
        goal_msg.segments.push_back(arc_segment);
        goal_msg.global_speed = 50.0;
        goal_msg.laser_power = 0;
        goal_msg.start_segment_id = 0;
        goal_msg.corner_mode = 0;
        
        // 发送路径运动请求
        RCLCPP_INFO(this->get_logger(), "发送圆弧运动请求");
        auto send_goal_options = rclcpp_action::Client<motion_msgs::action::MovePath>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            [this](const rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "圆弧运动请求被拒绝");
                } else {
                    RCLCPP_INFO(this->get_logger(), "圆弧运动请求被接受");
                }
            };
        
        send_goal_options.feedback_callback = 
            [this](rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::SharedPtr, 
                   const std::shared_ptr<const motion_msgs::action::MovePath::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "圆弧运动进度: %.2f%%, 当前段: %d", 
                           feedback->progress * 100, feedback->current_segment_id);
            };
        
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "圆弧运动执行成功，完成段数: %d", result.result->last_completed_id);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "圆弧运动执行失败: %s", result.result->error_msg.c_str());
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(), "圆弧运动被取消");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "圆弧运动执行结果未知");
                        break;
                }
            };
        
        action_client->async_send_goal(goal_msg, send_goal_options);
        
        // 等待运动完成
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    
    void testPathMove() {
        RCLCPP_INFO(this->get_logger(), "测试路径运动");
        
        // 创建action客户端
        auto action_client = rclcpp_action::create_client<motion_msgs::action::MovePath>(
            this, "move_path");
        
        // 等待action服务器可用
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action服务器不可用");
            return;
        }
        
        // 创建路径运动的goal
        auto goal_msg = motion_msgs::action::MovePath::Goal();
        
        // 创建直线段
        motion_msgs::msg::PathSegment line_segment;
        line_segment.type = 0; // TYPE_LINE
        line_segment.target_pos.x = 50.0;
        line_segment.target_pos.y = 50.0;
        
        goal_msg.segments.push_back(line_segment);
        
        // 创建圆弧段
        motion_msgs::msg::PathSegment arc_segment;
        arc_segment.type = 1; // TYPE_ARC
        arc_segment.target_pos.x = 100.0;
        arc_segment.target_pos.y = 0.0;
        arc_segment.center_pos.x = 50.0;
        arc_segment.center_pos.y = 50.0;
        
        goal_msg.segments.push_back(arc_segment);
        
        goal_msg.global_speed = 50.0;
        goal_msg.laser_power = 0;
        goal_msg.start_segment_id = 0;
        goal_msg.corner_mode = 0;
        
        // 发送路径运动请求
        RCLCPP_INFO(this->get_logger(), "发送路径运动请求");
        auto send_goal_options = rclcpp_action::Client<motion_msgs::action::MovePath>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            [this](const rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "路径运动请求被拒绝");
                } else {
                    RCLCPP_INFO(this->get_logger(), "路径运动请求被接受");
                }
            };
        
        send_goal_options.feedback_callback = 
            [this](rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::SharedPtr, 
                   const std::shared_ptr<const motion_msgs::action::MovePath::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "路径运动进度: %.2f%%, 当前段: %d", 
                           feedback->progress * 100, feedback->current_segment_id);
            };
        
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "路径运动执行成功，完成段数: %d", result.result->last_completed_id);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "路径运动执行失败: %s", result.result->error_msg.c_str());
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(), "路径运动被取消");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "路径运动执行结果未知");
                        break;
                }
            };
        
        action_client->async_send_goal(goal_msg, send_goal_options);
        
        // 等待运动完成
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
    
    std::shared_ptr<ZmcController> controller_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 解析命令行参数
    std::string move_type = "line"; // 默认测试直线运动
    if (argc > 1) {
        move_type = argv[1];
    }
    
    auto node = std::make_shared<TestMoveNode>();
    node->testMove(move_type);
    
    // 运行节点，处理回调
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}