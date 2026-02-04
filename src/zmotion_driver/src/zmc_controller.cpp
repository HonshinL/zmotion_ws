#include "zmotion_driver/zmc_controller.h"
#include <cstring>
#include <iostream>

ZmcController::ZmcController() : handle_(nullptr), is_connected_(false) {
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
        std::cout << "成功连接到ZMC控制器: " << ip << std::endl;
        return true;
    } else {
        handle_ = nullptr;
        is_connected_ = false;
        std::cerr << "无法连接到ZMC控制器: " << ip << std::endl;
        return false;
    }
}

void ZmcController::disconnect() {
    if (is_connected_ && handle_) {
        ZAux_Close(handle_);
        handle_ = nullptr;
        is_connected_ = false;
        std::cout << "已断开与ZMC控制器的连接: " << controller_ip_ << std::endl;
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

// 私有方法
bool ZmcController::checkError(int32 error_code) const {
    if (error_code == ERR_OK) {
        return true;
    } else {
        std::cerr << "ZMC控制器错误: 错误码 = " << error_code << std::endl;
        return false;
    }
}