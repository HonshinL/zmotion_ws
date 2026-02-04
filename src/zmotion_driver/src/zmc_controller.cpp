#include "zmotion_driver/zmc_controller.h"
#include <cstring>
#include <iostream>

ZmcController::ZmcController() : Node("zmc_controller"), handle_(nullptr), is_connected_(false) {
    initROS();
}

ZmcController::ZmcController(const std::string& node_name) : Node(node_name), handle_(nullptr), is_connected_(false) {
    initROS();
}

ZmcController::~ZmcController() {
    disconnect();
}

bool ZmcController::connect(const std::string& ip) {
    // 确保之前的连接已关闭
    if (is_connected_) {
        disconnect();
    }

    controller_ip_ = ip;
    
    // 准备IP地址缓冲区
    char ip_buffer[16]; // IPv4地址最多需要15个字符
    std::strncpy(ip_buffer, ip.c_str(), sizeof(ip_buffer) - 1);
    ip_buffer[sizeof(ip_buffer) - 1] = '\0'; // 确保字符串以null结尾

    // 连接控制器
    int32 result = ZAux_OpenEth(ip_buffer, &handle_);
    if (checkError(result)) {
        is_connected_ = true;
        RCLCPP_INFO(this->get_logger(), "成功连接到ZMC控制器: %s", ip.c_str());
        return true;
    } else {
        handle_ = nullptr;
        is_connected_ = false;
        RCLCPP_ERROR(this->get_logger(), "无法连接到ZMC控制器: %s", ip.c_str());
        return false;
    }
}

void ZmcController::disconnect() {
    if (is_connected_ && handle_) {
        ZAux_Close(handle_);
        handle_ = nullptr;
        is_connected_ = false;
        RCLCPP_INFO(this->get_logger(), "已断开与ZMC控制器的连接: %s", controller_ip_.c_str());
    }
}

bool ZmcController::isConnected() const {
    return is_connected_;
}

ZMC_HANDLE ZmcController::getHandle() const {
    return handle_;
}

// 位置相关方法
bool ZmcController::getDpos(int axis, float& position) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetDpos(handle_, axis, &position);
    return checkError(result);
}

bool ZmcController::setDpos(int axis, float position) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_SetDpos(handle_, axis, position);
    return checkError(result);
}

bool ZmcController::getMpos(int axis, float& position) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetMpos(handle_, axis, &position);
    return checkError(result);
}

// 速度相关方法
bool ZmcController::getCurSpeed(int axis, float& speed) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetVpSpeed(handle_, axis, &speed);
    return checkError(result);
}

bool ZmcController::getAccel(int axis, float& accel) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetAccel(handle_, axis, &accel);
    return checkError(result);
}

bool ZmcController::setAccel(int axis, float accel) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_SetAccel(handle_, axis, accel);
    return checkError(result);
}

bool ZmcController::getDecel(int axis, float& decel) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetDecel(handle_, axis, &decel);
    return checkError(result);
}

bool ZmcController::setDecel(int axis, float decel) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_SetDecel(handle_, axis, decel);
    return checkError(result);
}

// IO相关方法
bool ZmcController::getInput(int ionum, uint32& value) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetIn(handle_, ionum, &value);
    return checkError(result);
}

bool ZmcController::setOutput(int ionum, uint32 value) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_SetOp(handle_, ionum, value);
    return checkError(result);
}

bool ZmcController::getOutput(int ionum, uint32& value) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetOp(handle_, ionum, &value);
    return checkError(result);
}

// 轴状态相关方法
bool ZmcController::getAxisStatus(int axis, int& status) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetAxisStatus(handle_, axis, &status);
    return checkError(result);
}

bool ZmcController::getAxisEnable(int axis, int& enabled) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetAxisEnable(handle_, axis, &enabled);
    return checkError(result);
}

bool ZmcController::setAxisEnable(int axis, int enabled) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_SetAxisEnable(handle_, axis, enabled);
    return checkError(result);
}

// 新方法实现
void ZmcController::initROS() {
    // 声明并获取参数
    std::string ip = this->declare_parameter<std::string>("controller_ip", "192.168.0.11");
    axis_ = this->declare_parameter<int>("monitoring_axis", 0);

    // 创建发布者 (Publisher)
    // 发布运动状态
    motion_status_pub_ = this->create_publisher<motion_msgs::msg::MotionStatus>("zmc/motion_status", 10);

    // 连接控制器
    if (connect(ip)) {
        RCLCPP_INFO(this->get_logger(), "已连接控制器: %s, 正在监控轴 %d", ip.c_str(), axis_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "无法连接控制器!");
    }
}

void ZmcController::startPublishing() {
    // 创建定时器 (WallTimer)
    // 每 20 毫秒执行一次 timer_callback (50Hz)
    if (!timer_) {
        timer_ = this->create_wall_timer(20ms, std::bind(&ZmcController::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "开始发布控制器数据");
    }
}

void ZmcController::stopPublishing() {
    if (timer_) {
        timer_.reset();
        RCLCPP_INFO(this->get_logger(), "停止发布控制器数据");
    }
}

void ZmcController::timer_callback() {
    if (!is_connected_) return;

    float dpos_val = 0.0;
    float mpos_val = 0.0;
    float speed_val = 0.0;

    // 从硬件读取数据
    if (getDpos(axis_, dpos_val) && getMpos(axis_, mpos_val) && getCurSpeed(axis_, speed_val)) {
        // 创建MotionStatus消息
        auto motion_status_msg = motion_msgs::msg::MotionStatus();
        
        // 填充Header
        motion_status_msg.header.stamp = this->now();
        motion_status_msg.header.frame_id = "zmc_controller";
        
        // 填充JointState
        auto& joint_state = motion_status_msg.joint_state;
        joint_state.header = motion_status_msg.header;
        
        // 设置关节名称 (假设有一个关节，名称为"axis_X"，其中X是轴号)
        joint_state.name.push_back("axis_" + std::to_string(axis_));
        
        // 设置关节位置 (使用MPOS作为实际位置)
        joint_state.position.push_back(mpos_val);
        
        // 设置关节速度
        joint_state.velocity.push_back(speed_val);
        
        // 设置关节加速度 (暂不支持，留空)
        // joint_state.effort.push_back(0.0); // 不支持力/力矩
        
        // 发布MotionStatus消息
        motion_status_pub_->publish(motion_status_msg);

        // (可选) 调试打印，实际运行时建议注释掉以节省 CPU
        // RCLCPP_DEBUG(this->get_logger(), "Published MPOS: %.3f, Speed: %.3f", mpos_val, speed_val);
    } else {
        RCLCPP_WARN(this->get_logger(), "无法读取轴 %d 的数据", axis_);
    }
}

// 私有方法
bool ZmcController::checkError(int32 error_code) const {
    if (error_code == ERR_OK) {
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "ZMC控制器错误: 错误码 = %d", error_code);
        return false;
    }
}