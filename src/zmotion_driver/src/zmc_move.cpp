#include "rclcpp/rclcpp.hpp"
#include "zmotion_driver/zmc_controller.h"
#include <iostream>
#include <string>

class TestMoveNode : public rclcpp::Node {
public:
    TestMoveNode() : Node("test_node"), motion_completed_(false) {
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
        // 析构函数保持简单，不执行阻塞操作
        // 清理操作在主函数中显式处理
    }
    
    void cleanup() {
        // 退出时将轴移动到0,0位置
        moveToHomePosition();
        
        // 断开连接
        controller_->disconnect();
    }
    
    // 显式断开控制器连接
    void disconnectController() {
        controller_->disconnect();
    }
    
    // 运动类型枚举
    enum class MoveType {
        LINE,
        ARC,
        PATH,
        CIRC,
        UNKNOWN
    };

    // 将字符串转换为运动类型枚举
    MoveType stringToMoveType(const std::string& move_type) {
        if (move_type == "line") return MoveType::LINE;
        if (move_type == "arc") return MoveType::ARC;
        if (move_type == "path") return MoveType::PATH;
        if (move_type == "circ") return MoveType::CIRC;
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
            case MoveType::CIRC:
                testCircMove();
                break;
            case MoveType::UNKNOWN:
            default:
                RCLCPP_ERROR(this->get_logger(), "未知的运动类型: %s", move_type.c_str());
                RCLCPP_INFO(this->get_logger(), "支持的运动类型: line, arc, path, circ");
                break;
        }
        
        // 标记运动完成
        motion_completed_ = true;
    }
    
    bool isMotionCompleted() const {
        return motion_completed_;
    }
    
private:
    void testLineMove() {
        RCLCPP_INFO(this->get_logger(), "测试直线运动");
        
        // 创建action客户端
        auto action_client = rclcpp_action::create_client<motion_msgs::action::MovePath>(
            this, "zmc_act/move_path");
        
        // 等待action服务器可用
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action服务器不可用，请先运行: ros2 run zmotion_driver zmotion_node");
            RCLCPP_INFO(this->get_logger(), "检查Action服务器状态: ros2 action list");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Action服务器连接成功");
        
        // 创建路径运动的goal
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
        
        RCLCPP_INFO(this->get_logger(), "Goal参数: 直线段到(%.1f, %.1f), 速度%.1f", 
                   line_segment.target_pos.x, line_segment.target_pos.y, goal_msg.global_speed);
        
        // 发送路径运动请求
        RCLCPP_INFO(this->get_logger(), "发送路径运动请求");
        
        auto send_goal_options = rclcpp_action::Client<motion_msgs::action::MovePath>::SendGoalOptions();
        
        // 设置反馈回调
        send_goal_options.feedback_callback = 
            [this](rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::SharedPtr goal_handle,
                   const std::shared_ptr<const motion_msgs::action::MovePath::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "路径运动反馈: 当前段ID=%u, 进度=%f", 
                           feedback->current_segment_id, feedback->progress);
            };
        
        // 设置结果回调
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "路径运动完成");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "路径运动被中止");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "路径运动被取消");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "路径运动未知结果");
                        break;
                }
                
                // 标记运动完成
                motion_completed_ = true;
            };
        
        // 发送goal
        RCLCPP_INFO(this->get_logger(), "正在发送goal...");
        auto future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
        
        // 等待goal被接受
        RCLCPP_INFO(this->get_logger(), "等待goal被接受...");
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_goal_handle) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "发送goal失败，可能Action服务器没有响应");
            return;
        }
        
        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "goal被拒绝，可能参数不完整或Action服务器验证失败");
            RCLCPP_INFO(this->get_logger(), "可以尝试手动测试: ros2 action send_goal /zmc_act/move_path motion_msgs/action/MovePath \"{segments: [{type: 0, target_pos: {x: 50.0, y: 50.0}}], global_speed: 50.0, laser_power: 0, start_segment_id: 0, corner_mode: 0}\" --feedback");
            RCLCPP_INFO(this->get_logger(), "检查Action服务器日志以了解拒绝原因");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "goal被接受，等待结果...");
        
        // 等待结果
        auto future_result = action_client->async_get_result(goal_handle);
        auto result_status = rclcpp::spin_until_future_complete(this->shared_from_this(), future_result);
        
        if (result_status == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future_result.get();
            RCLCPP_INFO(this->get_logger(), "路径运动执行完成");
        } else {
            RCLCPP_ERROR(this->get_logger(), "等待结果超时");
        }
        
        // 标记运动完成
        motion_completed_ = true;        
    }
    
    void testArcMove() {
        RCLCPP_INFO(this->get_logger(), "测试圆弧运动");
        
        // 创建action客户端
        auto action_client = rclcpp_action::create_client<motion_msgs::action::MovePath>(
            this, "zmc_act/move_path");
        
        // 等待action服务器可用
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action服务器不可用，请先运行: ros2 run zmotion_driver zmotion_node");
            RCLCPP_INFO(this->get_logger(), "检查Action服务器状态: ros2 action list");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Action服务器连接成功");
        
        // 创建路径运动的goal
        auto goal_msg = motion_msgs::action::MovePath::Goal();
        
        // 创建圆弧段
        motion_msgs::msg::PathSegment arc_segment;
        arc_segment.type = 1; // TYPE_ARC
        arc_segment.center_pos.x = 50.0;
        arc_segment.center_pos.y = 0.0;
        arc_segment.target_pos.x = 100.0;
        arc_segment.target_pos.y = 0.0;
        
        goal_msg.segments.push_back(arc_segment);
        goal_msg.global_speed = 50.0;
        goal_msg.laser_power = 0;
        goal_msg.start_segment_id = 0;
        goal_msg.corner_mode = 0;
        
        RCLCPP_INFO(this->get_logger(), "Goal参数: 圆弧段，圆心(%.1f, %.1f)，目标点(%.1f, %.1f)，速度%.1f", 
                   arc_segment.center_pos.x, arc_segment.center_pos.y, 
                   arc_segment.target_pos.x, arc_segment.target_pos.y, 
                   goal_msg.global_speed);
        
        // 发送路径运动请求
        RCLCPP_INFO(this->get_logger(), "发送圆弧运动请求");
        
        auto send_goal_options = rclcpp_action::Client<motion_msgs::action::MovePath>::SendGoalOptions();
        
        // 设置反馈回调
        send_goal_options.feedback_callback = 
            [this](rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::SharedPtr goal_handle,
                   const std::shared_ptr<const motion_msgs::action::MovePath::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "圆弧运动反馈: 当前段ID=%u, 进度=%.2f%%, 当前位置(%.2f, %.2f)", 
                           feedback->current_segment_id, feedback->progress * 100,
                           feedback->current_pos.x, feedback->current_pos.y);
            };
        
        // 设置结果回调
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        if (result.result) {
                            RCLCPP_INFO(this->get_logger(), "圆弧运动执行成功，完成段数: %d", result.result->last_completed_id);
                        } else {
                            RCLCPP_INFO(this->get_logger(), "圆弧运动执行成功");
                        }
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        if (result.result) {
                            RCLCPP_ERROR(this->get_logger(), "圆弧运动执行失败: %s", result.result->error_msg.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "圆弧运动执行失败");
                        }
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "圆弧运动被取消");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "圆弧运动未知结果");
                        break;
                }
                
                // 标记运动完成
                motion_completed_ = true;
            };
        
        // 发送goal
        RCLCPP_INFO(this->get_logger(), "正在发送goal...");
        auto future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
        
        // 等待goal被接受
        RCLCPP_INFO(this->get_logger(), "等待goal被接受...");
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_goal_handle) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "发送goal失败，可能Action服务器没有响应");
            return;
        }
        
        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "goal被拒绝，可能参数不完整或Action服务器验证失败");
            RCLCPP_INFO(this->get_logger(), "可以尝试手动测试: ros2 action send_goal /zmc_act/move_path motion_msgs/action/MovePath \"{segments: [{type: 1, center_pos: {x: 50.0, y: 0.0}, target_pos: {x: 100.0, y: 0.0}}], global_speed: 50.0, laser_power: 0, start_segment_id: 0, corner_mode: 0}\" --feedback");
            RCLCPP_INFO(this->get_logger(), "检查Action服务器日志以了解拒绝原因");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "goal被接受，等待结果...");
        
        // 等待结果
        auto future_result = action_client->async_get_result(goal_handle);
        auto result_status = rclcpp::spin_until_future_complete(this->shared_from_this(), future_result);
        
        if (result_status == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future_result.get();
            RCLCPP_INFO(this->get_logger(), "圆弧运动执行完成");
        } else {
            RCLCPP_ERROR(this->get_logger(), "等待结果超时");
        }
        
        // 标记运动完成
        motion_completed_ = true;
    }
    
    void testCircMove() {
        RCLCPP_INFO(this->get_logger(), "测试画圆运动");
        
        // 创建action客户端
        auto action_client = rclcpp_action::create_client<motion_msgs::action::MovePath>(
            this, "zmc_act/move_path");
        
        // 等待action服务器可用
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action服务器不可用，请先运行: ros2 run zmotion_driver zmotion_node");
            RCLCPP_INFO(this->get_logger(), "检查Action服务器状态: ros2 action list");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Action服务器连接成功");
        
        // 创建路径运动的goal
        auto goal_msg = motion_msgs::action::MovePath::Goal();
        
        // 创建圆弧段 - 画圆（起点和终点相同）
        motion_msgs::msg::PathSegment circle_segment;
        circle_segment.type = 1; // TYPE_ARC
        circle_segment.center_pos.x = 50.0;  // 圆心X坐标
        circle_segment.center_pos.y = 0.0;  // 圆心Y坐标
        circle_segment.target_pos.x = 0.1;  // 终点X坐标（与起点相同）
        circle_segment.target_pos.y = 0.0;  // 终点Y坐标（与起点相同）
        
        goal_msg.segments.push_back(circle_segment);
        goal_msg.global_speed = 30.0;  // 降低速度以便观察
        goal_msg.laser_power = 0;
        goal_msg.start_segment_id = 0;
        goal_msg.corner_mode = 0;
        
        RCLCPP_INFO(this->get_logger(), "Goal参数: 画圆运动，圆心(%.1f, %.1f)，半径%.1f，速度%.1f", 
                   circle_segment.center_pos.x, circle_segment.center_pos.y,
                   50.0,  // 半径（圆心到起点/终点的距离）
                   goal_msg.global_speed);
        
        // 发送路径运动请求
        RCLCPP_INFO(this->get_logger(), "发送画圆运动请求");
        
        auto send_goal_options = rclcpp_action::Client<motion_msgs::action::MovePath>::SendGoalOptions();
        
        // 设置反馈回调
        send_goal_options.feedback_callback = 
            [this](rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::SharedPtr goal_handle,
                   const std::shared_ptr<const motion_msgs::action::MovePath::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "画圆运动反馈: 当前段ID=%u, 进度=%.2f%%, 当前位置(%.2f, %.2f)", 
                           feedback->current_segment_id, feedback->progress * 100,
                           feedback->current_pos.x, feedback->current_pos.y);
            };
        
        // 设置结果回调
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        if (result.result) {
                            RCLCPP_INFO(this->get_logger(), "画圆运动执行成功，完成段数: %d", result.result->last_completed_id);
                        } else {
                            RCLCPP_INFO(this->get_logger(), "画圆运动执行成功");
                        }
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        if (result.result) {
                            RCLCPP_ERROR(this->get_logger(), "画圆运动执行失败: %s", result.result->error_msg.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "画圆运动执行失败");
                        }
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "画圆运动被取消");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "画圆运动未知结果");
                        break;
                }
                
                // 标记运动完成
                motion_completed_ = true;
            };
        
        // 发送goal
        RCLCPP_INFO(this->get_logger(), "正在发送goal...");
        auto future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
        
        // 等待goal被接受
        RCLCPP_INFO(this->get_logger(), "等待goal被接受...");
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_goal_handle) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "发送goal失败，可能Action服务器没有响应");
            return;
        }
        
        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "goal被拒绝，可能参数不完整或Action服务器验证失败");
            RCLCPP_INFO(this->get_logger(), "可以尝试手动测试: ros2 action send_goal /zmc_act/move_path motion_msgs/action/MovePath \"{segments: [{type: 1, center_pos: {x: 50.0, y: 50.0}, target_pos: {x: 50.0, y: 50.0}}], global_speed: 30.0, laser_power: 0, start_segment_id: 0, corner_mode: 0}\" --feedback");
            RCLCPP_INFO(this->get_logger(), "检查Action服务器日志以了解拒绝原因");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "goal被接受，等待结果...");
        
        // 等待结果
        auto future_result = action_client->async_get_result(goal_handle);
        auto result_status = rclcpp::spin_until_future_complete(this->shared_from_this(), future_result);
        
        if (result_status == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future_result.get();
            RCLCPP_INFO(this->get_logger(), "画圆运动执行完成");
        } else {
            RCLCPP_ERROR(this->get_logger(), "等待结果超时");
        }
        
        // 标记运动完成
        motion_completed_ = true;
    }

    void testPathMove() {
        RCLCPP_INFO(this->get_logger(), "测试路径运动（三个直线段）");
        
        // 创建action客户端
        auto action_client = rclcpp_action::create_client<motion_msgs::action::MovePath>(
            this, "zmc_act/move_path");
        
        // 等待action服务器可用
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action服务器不可用，请先运行: ros2 run zmotion_driver zmotion_node");
            RCLCPP_INFO(this->get_logger(), "检查Action服务器状态: ros2 action list");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Action服务器连接成功");
        
        // 创建路径运动的goal
        auto goal_msg = motion_msgs::action::MovePath::Goal();
        
        // 创建第一个直线段：到 (150.0, 0.0)
        motion_msgs::msg::PathSegment line_segment1;
        line_segment1.type = 0; // TYPE_LINE
        line_segment1.target_pos.x = 150.0;
        line_segment1.target_pos.y = 0.0;
        goal_msg.segments.push_back(line_segment1);
        
        // 创建第二个直线段：到 (100.0, 50.0)
        motion_msgs::msg::PathSegment line_segment2;
        line_segment2.type = 0; // TYPE_LINE
        line_segment2.target_pos.x = 100.0;
        line_segment2.target_pos.y = 50.0;
        goal_msg.segments.push_back(line_segment2);
        
        // 创建第三个直线段：到 (0.0, 50.0)
        motion_msgs::msg::PathSegment line_segment3;
        line_segment3.type = 0; // TYPE_LINE
        line_segment3.target_pos.x = 0.0;
        line_segment3.target_pos.y = 50.0;
        goal_msg.segments.push_back(line_segment3);
        
        goal_msg.global_speed = 50.0;
        goal_msg.laser_power = 0;
        goal_msg.start_segment_id = 0;
        goal_msg.corner_mode = 0;
        
        RCLCPP_INFO(this->get_logger(), "Goal参数: 三个直线段路径");
        RCLCPP_INFO(this->get_logger(), "  段1: 到(%.1f, %.1f)", line_segment1.target_pos.x, line_segment1.target_pos.y);
        RCLCPP_INFO(this->get_logger(), "  段2: 到(%.1f, %.1f)", line_segment2.target_pos.x, line_segment2.target_pos.y);
        RCLCPP_INFO(this->get_logger(), "  段3: 到(%.1f, %.1f)", line_segment3.target_pos.x, line_segment3.target_pos.y);
        RCLCPP_INFO(this->get_logger(), "  速度: %.1f", goal_msg.global_speed);
        
        // 发送路径运动请求
        RCLCPP_INFO(this->get_logger(), "发送路径运动请求");
        
        auto send_goal_options = rclcpp_action::Client<motion_msgs::action::MovePath>::SendGoalOptions();
        
        // 设置反馈回调
        send_goal_options.feedback_callback = 
            [this](rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::SharedPtr goal_handle,
                   const std::shared_ptr<const motion_msgs::action::MovePath::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "路径运动反馈: 当前段ID=%u, 进度=%.2f%%, 当前位置(%.2f, %.2f)", 
                           feedback->current_segment_id, feedback->progress * 100,
                           feedback->current_pos.x, feedback->current_pos.y);
            };
        
        // 设置结果回调
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<motion_msgs::action::MovePath>::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        if (result.result) {
                            RCLCPP_INFO(this->get_logger(), "路径运动执行成功，完成段数: %d", result.result->last_completed_id);
                        } else {
                            RCLCPP_INFO(this->get_logger(), "路径运动执行成功");
                        }
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        if (result.result) {
                            RCLCPP_ERROR(this->get_logger(), "路径运动执行失败: %s", result.result->error_msg.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "路径运动执行失败");
                        }
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "路径运动被取消");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "路径运动未知结果");
                        break;
                }
                
                // 标记运动完成
                motion_completed_ = true;
            };
        
        // 发送goal
        RCLCPP_INFO(this->get_logger(), "正在发送goal...");
        auto future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
        
        // 等待goal被接受
        RCLCPP_INFO(this->get_logger(), "等待goal被接受...");
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_goal_handle) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "发送goal失败，可能Action服务器没有响应");
            return;
        }
        
        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "goal被拒绝，可能参数不完整或Action服务器验证失败");
            RCLCPP_INFO(this->get_logger(), "可以尝试手动测试: ros2 action send_goal /zmc_act/move_path motion_msgs/action/MovePath \"{segments: [{type: 0, target_pos: {x: 150.0, y: 0.0}}, {type: 0, target_pos: {x: 100.0, y: 50.0}}, {type: 0, target_pos: {x: 0.0, y: 50.0}}], global_speed: 50.0, laser_power: 0, start_segment_id: 0, corner_mode: 0}\" --feedback");
            RCLCPP_INFO(this->get_logger(), "检查Action服务器日志以了解拒绝原因");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "goal被接受，等待结果...");
        
        // 等待结果
        auto future_result = action_client->async_get_result(goal_handle);
        auto result_status = rclcpp::spin_until_future_complete(this->shared_from_this(), future_result);
        
        if (result_status == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future_result.get();
            RCLCPP_INFO(this->get_logger(), "路径运动执行完成");
        } else {
            RCLCPP_ERROR(this->get_logger(), "等待结果超时");
        }
        
        // 标记运动完成
        motion_completed_ = true;
    }
    
    void moveToHomePosition() {
        RCLCPP_INFO(this->get_logger(), "退出前将轴移动到0,0位置");
        
        // 直接使用控制器的moveAxes方法，避免在节点上下文失效时创建Action客户端
        std::vector<int64_t> target_axes = {5, 4}; // 假设X轴是轴0，Y轴是轴2
        std::vector<double> target_positions = {0.0, 0.0};
        
        if (controller_->moveAxes(target_axes, target_positions)) {
            RCLCPP_INFO(this->get_logger(), "成功启动移动到0,0位置的命令");
            
            // 等待运动完成，最多等待10秒
            int wait_count = 0;
            bool all_reached = false;
            
            while (!all_reached && wait_count < 100) {
                all_reached = true;
                for (size_t i = 0; i < target_axes.size(); ++i) {
                    if (!controller_->isAxisAtPosition(target_axes[i], target_positions[i])) {
                        all_reached = false;
                        break;
                    }
                }
                
                if (!all_reached) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    wait_count++;
                }
            }
            
            if (all_reached) {
                RCLCPP_INFO(this->get_logger(), "成功移动到0,0位置");
            } else {
                RCLCPP_WARN(this->get_logger(), "移动到0,0位置超时");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "无法启动移动到0,0位置的命令");
        }
    }
    
    std::shared_ptr<ZmcController> controller_;
    std::atomic<bool> motion_completed_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 解析命令行参数
    std::string move_type = "line"; // 默认测试直线运动
    if (argc > 1) {
        move_type = argv[1];
    }
    
    {
        auto node = std::make_shared<TestMoveNode>();
        node->testMove(move_type);
        
        // 运行节点，处理回调，但检查运动是否完成
        rclcpp::Rate rate(10); // 10Hz
        while (rclcpp::ok() && !node->isMotionCompleted()) {
            rclcpp::spin_some(node);
            rate.sleep();
        }
        
        RCLCPP_INFO(node->get_logger(), "运动完成，执行清理操作");
        
        // 显式执行清理操作
        node->cleanup();
        
        RCLCPP_INFO(node->get_logger(), "清理完成，程序退出");
    }
    
    rclcpp::shutdown();
    return 0;
}