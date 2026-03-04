void ZmcController::executeAxesHoming(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::AxesHoming>> goal_handle) {
    
    // 设置执行状态
    action_running_ = true;
    current_axes_homing_goal_handle_ = goal_handle;
    
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<motion_msgs::action::AxesHoming::Result>();
    auto feedback = std::make_shared<motion_msgs::action::AxesHoming::Feedback>();
    
    RCLCPP_INFO(this->get_logger(), "开始异步执行多轴回零Action");
    
    try {
        // 检查控制器连接状态
        if (!is_connected_) {
            throw std::runtime_error("控制器未连接");
        }
        
        // 执行多轴回零
        if (!homeAxes(goal->axes)) {
            throw std::runtime_error("启动多轴回零失败");
        }
        
        // 等待所有轴回零完成
        auto start_time = std::chrono::steady_clock::now();
        bool all_homed = false;
        
        while (action_running_ && !all_homed) {
            // 检查Action是否被取消
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Action被用户取消";
                result->error_codes = std::vector<int64_t>(goal->axes.size(), -1);
                goal_handle->canceled(result);
                action_running_ = false;
                RCLCPP_INFO(this->get_logger(), "Action执行被取消");
                return;
            }
            
            // 检查所有轴的回零状态
            all_homed = true;
            feedback->current_positions.clear();
            feedback->drive_statuses.clear();
            
            for (int64_t axis : goal->axes) {
                int status = 0;
                uint32 home_status = 0;
                if (!checkError(ZAux_BusCmd_GetHomeStatus(handle_, static_cast<int>(axis), &home_status))) {
                    all_homed = false;
                }
                if (home_status != 1) { // 1表示回零完成
                    all_homed = false;
                }
                
                // 获取当前位置
                float current_position = 0.0;
                if (!getMpos(static_cast<int>(axis), current_position)) {
                    RCLCPP_WARN(this->get_logger(), "无法获取轴 %ld 的当前位置", axis);
                }
                
                feedback->current_positions.push_back(current_position);
                feedback->drive_statuses.push_back(static_cast<int32_t>(home_status));
            }
            
            // 更新反馈信息
            feedback->current_state = all_homed ? "所有轴回零完成" : "回零中";
            
            // 计算已执行时间
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            feedback->elapsed_time = static_cast<double>(elapsed.count());
            
            // 发布反馈
            goal_handle->publish_feedback(feedback);
            
            // 检查超时（使用第一个轴的超时时间作为整个回零过程的超时时间）
            if (!goal->homing_timeout.empty() && feedback->elapsed_time > goal->homing_timeout[0]) {
                throw std::runtime_error("回零超时");
            }
            
            // 短暂休眠，避免过度占用CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (action_running_) {
            // 所有轴回零完成，获取最终位置
            result->final_positions.clear();
            result->error_codes.clear();
            
            for (int64_t axis : goal->axes) {
                float position = 0.0;
                if (getMpos(static_cast<int>(axis), position)) {
                    result->final_positions.push_back(position);
                } else {
                    result->final_positions.push_back(0.0);
                }
                result->error_codes.push_back(0); // 0表示成功
            }
            
            result->success = true;
            result->message = "所有轴回零成功";
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "多轴回零Action执行成功完成");
        }
        
    } catch (const std::exception& e) {
        result->success = false;
        result->message = "多轴回零Action执行失败: " + std::string(e.what());
        result->error_codes = std::vector<int64_t>(goal->axes.size(), -1);
        
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "多轴回零Action执行失败: %s", e.what());
    }
    
    // 清理执行状态
    action_running_ = false;
    current_axes_homing_goal_handle_.reset();
}
