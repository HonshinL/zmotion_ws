void ZmcController::executeAxesMoving(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::AxesMoving>> goal_handle) {
    
    // 设置执行状态
    action_running_ = true;
    current_axes_moving_goal_handle_ = goal_handle;
    
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<motion_msgs::action::AxesMoving::Result>();
    auto feedback = std::make_shared<motion_msgs::action::AxesMoving::Feedback>();
    
    RCLCPP_INFO(this->get_logger(), "开始异步执行轴移动Action");
    
    try {
        // 检查控制器连接状态
        if (!is_connected_) {
            throw std::runtime_error("控制器未连接");
        }
        
        // 设置运动参数（使用Action目标中的参数）
        for (size_t i = 0; i < goal->target_axes.size(); ++i) {
            int64_t axis = goal->target_axes[i];
            
            // 设置轴速度
            if (!checkError(ZAux_Direct_SetSpeed(handle_, static_cast<int>(axis), goal->speed[i]))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 速度失败");
            }
            
            // 设置加速度
            if (!checkError(ZAux_Direct_SetAccel(handle_, static_cast<int>(axis), goal->acceleration[i]))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 加速度失败");
            }
            
            // 设置减速度
            if (!checkError(ZAux_Direct_SetDecel(handle_, static_cast<int>(axis), goal->deceleration[i]))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 减速度失败");
            }
        }
        
        // 启动运动
        for (size_t i = 0; i < goal->target_axes.size(); ++i) {
            int64_t axis = goal->target_axes[i];
            double target_position = goal->target_positions[i];
            
            if (!checkError(ZAux_Direct_Single_MoveAbs(handle_, static_cast<int>(axis), target_position))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 目标位置失败");
            }
            
            RCLCPP_INFO(this->get_logger(), "轴 %ld 开始移动到位置 %.3f，速度: %.3f，加速度: %.3f", 
                        axis, target_position, goal->speed[i], goal->acceleration[i]);
        }
        
        // 监控运动过程
        bool all_axes_completed = false;
        auto start_time = std::chrono::steady_clock::now();
        
        while (action_running_ && !all_axes_completed) {
            // 检查Action是否被取消
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Action被用户取消";
                result->end_time = this->now();
                goal_handle->canceled(result);
                action_running_ = false;
                RCLCPP_INFO(this->get_logger(), "Action执行被取消");
                return;
            }
            
            // 更新反馈信息
            feedback->current_positions.clear();
            feedback->current_velocities.clear();
            
            int completed_axes = 0;
            all_axes_completed = true;
            
            for (size_t i = 0; i < goal->target_axes.size(); ++i) {
                int64_t axis = goal->target_axes[i];
                double target_position = goal->target_positions[i];
                
                double current_position = 0.0;
                double current_velocity = 0.0;
                
                // 读取当前位置和速度
                if (getMpos(static_cast<int>(axis), current_position) && getCurSpeed(static_cast<int>(axis), current_velocity)) {
                    feedback->current_positions.push_back(current_position);
                    feedback->current_velocities.push_back(current_velocity);
                    
                    // 检查是否到达目标位置
                    if (isAxisAtPosition(axis, target_position)) {
                        completed_axes++;
                    } else {
                        all_axes_completed = false;
                    }
                } else {
                    all_axes_completed = false;
                    RCLCPP_WARN(this->get_logger(), "无法读取轴 %ld 的当前位置和速度", axis);
                }
            }
            
            // 计算进度
            feedback->progress = static_cast<float>(completed_axes) / goal->target_axes.size();
            feedback->current_status = "移动中，已完成 " + std::to_string(completed_axes) + "/" + 
                                      std::to_string(goal->target_axes.size()) + " 个轴";
            
            // 发布反馈
            goal_handle->publish_feedback(feedback);
            
            // 计算耗时
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            
            RCLCPP_DEBUG(this->get_logger(), "运动进度: %.1f%%, 耗时: %ld秒", 
                        feedback->progress * 100, elapsed.count());
            
            // 短暂休眠，避免过度占用CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        if (action_running_) {
            // 所有轴都到达目标位置
            result->end_time = this->now();
            result->success = true;
            result->message = "所有轴成功到达目标位置";
            result->final_positions = feedback->current_positions;
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "轴移动Action执行成功完成");
        }
        
    } catch (const std::exception& e) {
        result->end_time = this->now();
        result->success = false;
        result->message = "轴移动Action执行失败: " + std::string(e.what());
        
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "轴移动Action执行失败: %s", e.what());
    }
    
    // 清理执行状态
    action_running_ = false;
    current_axes_moving_goal_handle_.reset();
}
