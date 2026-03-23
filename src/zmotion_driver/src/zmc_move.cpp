#include "rclcpp/rclcpp.hpp"
#include "zmotion_driver/zmc_controller.h"
#include <iostream>
#include <string>

class TestMoveNode : public rclcpp::Node {
public:
    TestMoveNode() : Node("test_node") {
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
        // 退出时将轴移动到0,0位置
        moveToHomePosition();
        
        // 断开连接
        controller_->disconnect();
    }
    
    // 运动类型枚举
    enum class MoveType {
        LINE,
        ARC,
        PATH,
        UNKNOWN
    };

    // 将字符串转换为运动类型枚举
    MoveType stringToMoveType(const std::string& move_type) {
        if (move_type == "line") return MoveType::LINE;
        if (move_type == "arc") return MoveType::ARC;
        if (move_type == "path") return MoveType::PATH;
        return MoveType::UNKNOWN;
    }

    void testMove(const std::string& move_type) {
        RCLCPP_INFO(this->get_logger(), "测试运动类型: %s", move_type.c_str());
        
        MoveType type = stringToMoveType(move_type);
        
        switch (type) {
            case MoveType::LINE:
                testLineMove();
                break;
            case MoveType::ARC:
                testArcMove();
                break;
            case MoveType::PATH:
                testPathMove();
                break;
            case MoveType::UNKNOWN:
            default:
                RCLCPP_ERROR(this->get_logger(), "未知的运动类型: %s", move_type.c_str());
                RCLCPP_INFO(this->get_logger(), "支持的运动类型: line, arc, path");
                break;
        }
    }
    
private:
    void testLineMove() {
        RCLCPP_INFO(this->get_logger(), "测试直线运动");
        
        // 创建action客户端
        auto action_client = rclcpp_action::create_client<motion_msgs::action::MovePath>(
            this, "zmc/move_path");
        
        // 等待action服务器可用
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action服务器不可用");
            return;
        }
        
        // 创建直线运动的goal
        auto goal_msg = motion_msgs::action::MovePath::Goal();
        
        // 创建直线段
        motion_msgs::msg::PathSegment line_segment;
        line_segment.type = 0; // TYPE_LINE
        line_segment.target_pos.x = 50.0;
        line_segment.target_pos.y = 50.0;
        
        goal_msg.segments.push_back(line_segment);
        goal_msg.global_speed = 50.0;
        goal_msg.laser_power = 0;
        goal_msg.start_segment_id = 0;
        goal_msg.corner_mode = 0;
        
        // 发送路径运动请求
        RCLCPP_INFO(this->get_logger(), "发送直线运动请求");
        auto send_goal_options = rclcpp_action::Client<motion_msgs::action::MovePath>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            [this](const rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "直线运动请求被拒绝");
                } else {
                    RCLCPP_INFO(this->get_logger(), "直线运动请求被接受");
                }
            };
        
        send_goal_options.feedback_callback = 
            [this](rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::SharedPtr, 
                   const std::shared_ptr<const motion_msgs::action::MovePath::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "直线运动进度: %.2f%%, 当前段: %d", 
                           feedback->progress * 100, feedback->current_segment_id);
            };
        
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "直线运动执行成功，完成段数: %d", result.result->last_completed_id);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "直线运动执行失败: %s", result.result->error_msg.c_str());
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(), "直线运动被取消");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "直线运动执行结果未知");
                        break;
                }
            };
        
        action_client->async_send_goal(goal_msg, send_goal_options);
        
        // 等待运动完成
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    
    void testArcMove() {
        RCLCPP_INFO(this->get_logger(), "测试圆弧运动");
        
        // 创建action客户端
        auto action_client = rclcpp_action::create_client<motion_msgs::action::MovePath>(
            this, "zmc/move_path");
        
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
            this, "zmc/move_path");
        
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
    
    // 将轴移动到0,0位置
    void moveToHomePosition() {
        RCLCPP_INFO(this->get_logger(), "退出前将轴移动到0,0位置");
        
        // 创建action客户端
        auto action_client = rclcpp_action::create_client<motion_msgs::action::MovePath>(
            this, "zmc/move_path");
        
        // 等待action服务器可用
        if (action_client->wait_for_action_server(std::chrono::seconds(5))) {
            // 创建直线运动的goal
            auto goal_msg = motion_msgs::action::MovePath::Goal();
            
            // 创建直线段，目标位置为0,0
            motion_msgs::msg::PathSegment line_segment;
            line_segment.type = 0; // TYPE_LINE
            line_segment.target_pos.x = 0.0;
            line_segment.target_pos.y = 0.0;
            
            goal_msg.segments.push_back(line_segment);
            goal_msg.global_speed = 50.0;
            goal_msg.laser_power = 0;
            goal_msg.start_segment_id = 0;
            goal_msg.corner_mode = 0;
            
            // 发送路径运动请求
            RCLCPP_INFO(this->get_logger(), "发送移动到0,0位置的请求");
            auto send_goal_options = rclcpp_action::Client<motion_msgs::action::MovePath>::SendGoalOptions();
            
            // 用于同步等待的标志
            std::atomic<bool> goal_completed(false);
            
            send_goal_options.result_callback = 
                [this, &goal_completed](const rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::WrappedResult & result) {
                    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                        RCLCPP_INFO(this->get_logger(), "成功移动到0,0位置");
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "移动到0,0位置失败");
                    }
                    goal_completed.store(true);
                };
            
            action_client->async_send_goal(goal_msg, send_goal_options);
            
            // 等待运动完成，最多等待10秒
            int wait_count = 0;
            while (!goal_completed.load() && wait_count < 100) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                wait_count++;
            }
            
            if (!goal_completed.load()) {
                RCLCPP_WARN(this->get_logger(), "移动到0,0位置超时");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Action服务器不可用，无法移动轴到0,0位置");
        }
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