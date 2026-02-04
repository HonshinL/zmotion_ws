#include "zmotion_driver/zmc_controller.h"
#include <cstring>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <thread>
#include <atomic>

ZmcController::ZmcController(const std::string& node_name) : Node(node_name), handle_(nullptr), is_connected_(false), connecting_(false) {
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
    connect_search_timeout_ms_ = this->declare_parameter<int>("controller_connect_search_timeout_ms", 1000);
    
    // 初始化轴列表（假设支持4个轴）
    axes_ = {0, 1, 2, 3};

    // 创建发布者 (Publisher)
    // 发布运动状态
    motion_status_pub_ = this->create_publisher<motion_msgs::msg::MotionStatus>("zmc_pub/motion_status", 10);

    // 创建DXF到XML转换服务
    convert_dxf_to_xml_service_ = this->create_service<motion_msgs::srv::ConvertDxfToXml>(
        "zmc_srv/convert_dxf_to_xml",
        std::bind(&ZmcController::handleConvertDxfToXml, this, std::placeholders::_1, std::placeholders::_2));

    // 创建DXF转换状态发布者（异步任务状态通知）
    convert_status_pub_ = this->create_publisher<std_msgs::msg::String>("zmc_pub/convert_dxf_to_xml/status", 10);

    // 不在构造/初始化阶段进行阻塞性连接，使用显式的 start() 方法进行连接和启动发布
}

void ZmcController::start() {
    if (connecting_.load()) {
        RCLCPP_WARN(this->get_logger(), "连接已在进行中");
        return;
    }

    connecting_.store(true);
    int timeout_ms = connect_search_timeout_ms_;

    std::thread([this, timeout_ms]() {
        std::string ip = this->get_parameter("controller_ip").as_string();
        RCLCPP_INFO(this->get_logger(), "搜索控制器 %s (超时 %d ms)", ip.c_str(), timeout_ms);

        int32 search_res = ZAux_SearchEth(ip.c_str(), static_cast<uint32>(timeout_ms));
        if (search_res != ERR_OK) {
            RCLCPP_ERROR(this->get_logger(), "控制器 %s 无响应（%d ms 超时），不尝试连接", ip.c_str(), timeout_ms);
            connecting_.store(false);
            return;
        }

        // 搜索到设备，尝试建立连接（此调用依然是阻塞性的库调用）
        if (connect(ip)) {
            RCLCPP_INFO(this->get_logger(), "已连接控制器: %s, 正在监控轴 %d", ip.c_str(), axis_);
            startPublishing();
        } else {
            RCLCPP_ERROR(this->get_logger(), "无法连接控制器!");
        }

        connecting_.store(false);
    }).detach();
}

void ZmcController::stop() {
    stopPublishing();
    disconnect();
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

    // 创建MotionStatus消息
    auto motion_status_msg = motion_msgs::msg::MotionStatus();
    
    // 填充Header
    motion_status_msg.header.stamp = this->now();
    motion_status_msg.header.frame_id = "zmc_status";
    
    // 填充JointState
    auto& joint_state = motion_status_msg.joint_state;
    joint_state.header = motion_status_msg.header;
    
    bool all_axes_success = true;
    
    // 读取所有轴的数据
    for (int axis : axes_) {
        float dpos_val = 0.0;
        float mpos_val = 0.0;
        float speed_val = 0.0;
        
        // 从硬件读取数据
        bool axis_success = getDpos(axis, dpos_val) && getMpos(axis, mpos_val) && getCurSpeed(axis, speed_val);
        
        if (axis_success) {
            // 设置关节名称
            joint_state.name.push_back("axis_" + std::to_string(axis));
            
            // 设置关节位置 (使用MPOS作为实际位置)
            joint_state.position.push_back(mpos_val);
            
            // 设置关节速度
            joint_state.velocity.push_back(speed_val);
            
            // 设置关节加速度 (暂不支持，留空)
            // joint_state.effort.push_back(0.0); // 不支持力/力矩
            
            // 调试信息
            RCLCPP_DEBUG(this->get_logger(), "轴 %d: MPOS=%.3f, Speed=%.3f", axis, mpos_val, speed_val);
        } else {
            RCLCPP_WARN(this->get_logger(), "无法读取轴 %d 的数据", axis);
            all_axes_success = false;
        }
    }
    
    // 如果至少有一个轴读取成功，则发布消息
    if (!joint_state.name.empty()) {
        motion_status_pub_->publish(motion_status_msg);
        
        if (all_axes_success) {
            RCLCPP_DEBUG(this->get_logger(), "成功发布 %zu 个轴的状态数据", joint_state.name.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "部分轴数据读取失败，成功发布 %zu 个轴的状态数据", joint_state.name.size());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "所有轴数据读取失败，无法发布状态消息");
    }
}

// DXF到XML转换服务实现
void ZmcController::handleConvertDxfToXml(const std::shared_ptr<motion_msgs::srv::ConvertDxfToXml::Request> request,
                                         std::shared_ptr<motion_msgs::srv::ConvertDxfToXml::Response> response) {
    
    std::string dxf_file_path = request->dxf_file_path;
    
    RCLCPP_INFO(this->get_logger(), "收到DXF到XML转换请求，文件路径: %s", dxf_file_path.c_str());
    
    // 检查输入文件是否存在
    if (!std::filesystem::exists(dxf_file_path)) {
        response->success = false;
        response->message = "输入DXF文件不存在: " + dxf_file_path;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    // 检查文件扩展名是否为.dxf
    std::filesystem::path input_path(dxf_file_path);
    if (input_path.extension() != ".dxf" && input_path.extension() != ".DXF") {
        response->success = false;
        response->message = "输入文件不是DXF格式，扩展名应为.dxf: " + dxf_file_path;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    // 生成输出XML文件路径（与输入文件相同目录，文件名相同但扩展名为.xml）
    std::filesystem::path output_path = input_path.parent_path() / input_path.stem().concat(".xml");
    std::string xml_file_path = output_path.string();
    
    // 发布状态：启动
    std_msgs::msg::String start_msg;
    start_msg.data = "started: " + dxf_file_path + " -> " + xml_file_path;
    convert_status_pub_->publish(start_msg);

    // 启动异步转换任务（立即返回响应，实际转换在后台执行）
    std::string input = dxf_file_path;
    std::string output = xml_file_path;
    std::thread([this, input, output]() {
        try {
            auto start_time = std::chrono::high_resolution_clock::now();
            std::ifstream dxf_file(input, std::ios::binary);
            if (!dxf_file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "无法打开DXF文件: %s", input.c_str());
                std_msgs::msg::String fail_msg;
                fail_msg.data = "failed: cannot open input: " + input;
                convert_status_pub_->publish(fail_msg);
                return;
            }
            std::ofstream xml_file(output);
            if (!xml_file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "无法创建XML文件: %s", output.c_str());
                dxf_file.close();
                std_msgs::msg::String fail_msg;
                fail_msg.data = "failed: cannot create output: " + output;
                convert_status_pub_->publish(fail_msg);
                return;
            }
            xml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
            xml_file << "<!-- Converted from DXF file: " << std::filesystem::path(input).filename().string() << " -->\n";
            xml_file << "<MotionConfig>\n";
            std::string line;
            int line_count = 0;
            while (std::getline(dxf_file, line)) {
                line_count++;
                if (line.empty()) continue;
                if (line[0] == '#') {
                    xml_file << "<!-- " << line.substr(1) << " -->\n";
                    continue;
                }
                size_t equals_pos = line.find('=');
                if (equals_pos != std::string::npos) {
                    std::string key = line.substr(0, equals_pos);
                    std::string value = line.substr(equals_pos + 1);
                    key.erase(0, key.find_first_not_of(" \t\n\r\f\v"));
                    key.erase(key.find_last_not_of(" \t\n\r\f\v") + 1);
                    value.erase(0, value.find_first_not_of(" \t\n\r\f\v"));
                    value.erase(value.find_last_not_of(" \t\n\r\f\v") + 1);
                    xml_file << "  <" << key << ">" << value << "</" << key << ">\n";
                } else {
                    xml_file << "  <!-- DXF Line " << line_count << ": " << line << " -->\n";
                }
            }
            xml_file << "</MotionConfig>\n";
            dxf_file.close();
            xml_file.close();
            auto end_time = std::chrono::high_resolution_clock::now();
            double duration_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time - start_time).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "格式转换耗时: " << std::fixed << std::setprecision(3) << duration_ms << " ms");
            RCLCPP_INFO(this->get_logger(), "异步转换完成，输出文件: %s", output.c_str());

            std_msgs::msg::String done_msg;
            done_msg.data = "completed: " + output + " lines=" + std::to_string(line_count);
            convert_status_pub_->publish(done_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "异步转换过程中发生错误: %s", e.what());
            std_msgs::msg::String err_msg;
            err_msg.data = std::string("failed: exception: ") + e.what();
            convert_status_pub_->publish(err_msg);
        }
    }).detach();

    response->success = true;
    response->message = "已启动异步转换，输出文件: " + xml_file_path;
    response->xml_file_path = xml_file_path;

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    RCLCPP_INFO(this->get_logger(), "输出XML文件路径: %s", xml_file_path.c_str());
    return;
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