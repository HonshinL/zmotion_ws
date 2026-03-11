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
        // RCLCPP_INFO(this->get_logger(), "成功连接到ZMC控制器: %s", ip.c_str());
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

// 重载版本：支持double类型
bool ZmcController::getMpos(int axis, double& position) {
    if (!is_connected_) return false;
    
    float pos = 0.0;
    int32 result = ZAux_Direct_GetMpos(handle_, axis, &pos);
    if (checkError(result)) {
        position = static_cast<double>(pos);
        return true;
    }
    return false;
}

// 速度相关方法
bool ZmcController::getCurSpeed(int axis, float& speed) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetVpSpeed(handle_, axis, &speed);
    return checkError(result);
}

// 重载版本：支持double类型
bool ZmcController::getCurSpeed(int axis, double& speed) {
    if (!is_connected_) return false;
    
    float spd = 0.0;
    int32 result = ZAux_Direct_GetVpSpeed(handle_, axis, &spd);
    if (checkError(result)) {
        speed = static_cast<double>(spd);
        return true;
    }
    return false;
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
    // 声明参数，默认值将被配置文件覆盖
    this->declare_parameter<std::string>("controller_ip", "192.168.0.11");
    this->declare_parameter<std::vector<int64_t>>("running_axes", {0});
    this->declare_parameter<int64_t>("search_timeout_ms", 10000);
    
    // 控制器连接搜索超时时间(毫秒)，用于在网络断开后重新搜索连接
    this->declare_parameter<int64_t>("controller_connect_search_timeout_ms", 10000);
    
    // 轴参数
    this->declare_parameter<std::vector<int64_t>>("holding_axes", {0, 1, 2, 4, 5});
    this->declare_parameter<std::vector<double>>("axis_moving_pulse_equivalent", {13107.2, 13107.2, 13107.2, 1000.0, 1000.0});
    this->declare_parameter<std::vector<double>>("axis_moving_max_speed", {50.0, 50.0, 50.0, 50.0, 50.0});
    this->declare_parameter<std::vector<double>>("axis_moving_acceleration", {150.0, 150.0, 150.0, 150.0, 150.0});
    this->declare_parameter<std::vector<double>>("axis_moving_deceleration", {150.0, 150.0, 150.0, 150.0, 150.0});
    
    // 回零参数
    this->declare_parameter<bool>("auto_homing_on_start", true);
    this->declare_parameter<std::vector<int64_t>>("auto_homing_axes", {0});
    this->declare_parameter<std::vector<double>>("auto_homing_timeout", {60.0, 60.0, 60.0, 60.0, 60.0});
    this->declare_parameter<std::vector<int64_t>>("axis_homing_mode", {11, 11, 11, 11, 11});
    this->declare_parameter<std::vector<double>>("axis_homing_velocity_high", {50.0, 50.0, 50.0, 50.0, 50.0});
    this->declare_parameter<std::vector<double>>("axis_homing_velocity_low", {10.0, 10.0, 10.0, 10.0, 10.0});
    this->declare_parameter<std::vector<double>>("axis_homing_velocity_creep", {5.0, 5.0, 5.0, 5.0, 5.0});
    this->declare_parameter<std::vector<double>>("axis_homing_timeout", {60.0, 60.0, 60.0, 60.0, 60.0});
    
    // 回零后移动参数
    this->declare_parameter<bool>("move_after_homing", true);
    this->declare_parameter<std::vector<double>>("move_after_homing_positions", {240.0, 0.0, 0.0, 0.0, 0.0});

    // 创建发布者 (Publisher)
    // 发布运动状态
    axes_state_pub_ = this->create_publisher<motion_msgs::msg::AxesState>("zmc_pub/axes_state", 10);

    // 创建DXF到XML转换服务
    convert_dxf_to_xml_service_ = this->create_service<motion_msgs::srv::ConvertDxfToXml>(
        "zmc_srv/convert_dxf_to_xml",
        std::bind(&ZmcController::handleConvertDxfToXml, this, std::placeholders::_1, std::placeholders::_2));

    // 创建DXF转换状态发布者（异步任务状态通知）
    convert_status_pub_ = this->create_publisher<std_msgs::msg::String>("zmc_pub/convert_dxf_to_xml/status", 10);

    // 创建轴移动Action服务器
    axes_moving_action_server_ = rclcpp_action::create_server<motion_msgs::action::AxesMoving>(
        this,
        "zmc_act/axes_moving",
        std::bind(&ZmcController::handleAxesMovingGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ZmcController::handleAxesMovingCancel, this, std::placeholders::_1),
        std::bind(&ZmcController::handleAxesMovingAccepted, this, std::placeholders::_1));
    
    // 创建轴回零Action服务器
    axes_homing_action_server_ = rclcpp_action::create_server<motion_msgs::action::AxesHoming>(
        this,
        "zmc_act/axes_homing",
        std::bind(&ZmcController::handleAxesHomingGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ZmcController::handleAxesHomingCancel, this, std::placeholders::_1),
        std::bind(&ZmcController::handleAxesHomingAccepted, this, std::placeholders::_1));
    
    // 初始化Action状态
    action_running_ = false;

    // 不在构造/初始化阶段进行阻塞性连接，使用显式的 start() 方法进行连接和启动发布
}

void ZmcController::start() {
    
    if (connecting_.load()) {
        RCLCPP_WARN(this->get_logger(), "连接已在进行中，跳过本次连接尝试");
        return;
    }

    if (is_connected_) {
        RCLCPP_INFO(this->get_logger(), "控制器已连接，无需重新连接");
        return;
    }

    connecting_.store(true);
    
    // 从参数服务器获取IP地址
    std::string ip = this->get_parameter("controller_ip").as_string();
    
    RCLCPP_INFO(this->get_logger(), "开始尝试连接控制器: %s", ip.c_str());

    std::thread([this, ip]() {
        // 直接尝试连接，不进行搜索
        bool connect_success = connect(ip);
        
        if (connect_success) {
            RCLCPP_INFO(this->get_logger(), "✅ 成功连接到控制器: %s", ip.c_str());
            RCLCPP_INFO(this->get_logger(), "📊 开始监控 %ld 个轴: [%ld, %ld, %ld, %ld, %ld]", 
                       NUM_AXES, AXES[0], AXES[1], AXES[2], AXES[3], AXES[4]);
            
            // 初始化轴参数
            initializeAxisParameters();
            
            // 启动时自动回零
            bool auto_homing = this->get_parameter("auto_homing_on_start").as_bool();
            if (auto_homing) {
                auto homing_axes = this->get_parameter("auto_homing_axes").as_integer_array();
                auto homing_timeout_array = this->get_parameter("auto_homing_timeout").as_double_array();
                double homing_timeout = 60.0; // 默认60秒
                if (!homing_timeout_array.empty()) {
                    homing_timeout = homing_timeout_array[0]; // 使用第一个值
                }
                
                RCLCPP_INFO(this->get_logger(), "开始启动时自动回零");
                if (homeAxes(homing_axes)) {
                    RCLCPP_INFO(this->get_logger(), "所有轴回零成功");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "部分轴回零失败");
                }
            }
            
            // 回零后移动已在homeAxes函数中处理，此处不再重复执行
            // 如需修改回零后移动逻辑，请修改homeAxes函数中的相关代码
            RCLCPP_DEBUG(this->get_logger(), "回零后移动逻辑已集成到homeAxes函数中");
            
            // 启动数据发布
            startPublishing();
            
            RCLCPP_INFO(this->get_logger(), "🚀 控制器已启动并开始发布数据");
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 连接控制器失败: %s", ip.c_str());
            RCLCPP_ERROR(this->get_logger(), "💡 请检查以下事项:");
            RCLCPP_ERROR(this->get_logger(), "  1. 控制器电源是否打开");
            RCLCPP_ERROR(this->get_logger(), "  2. 网络连接是否正常");
            RCLCPP_ERROR(this->get_logger(), "  3. IP地址 %s 是否正确", ip.c_str());
            RCLCPP_ERROR(this->get_logger(), "  4. 防火墙设置是否允许连接");
        }

        connecting_.store(false);
    }).detach();
}

void ZmcController::stop() {

    moveAxes({0, 2}, {240.0, 0.0});
    stopPublishing();
    disconnect();
}

void ZmcController::startPublishing() {
    // 创建定时器 (WallTimer)
    if (!timer_) {
        timer_ = this->create_wall_timer(500ms, std::bind(&ZmcController::timer_callback, this));
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

    // 创建AxesState消息
    auto axes_state_msg = motion_msgs::msg::AxesState();
    
    // 设置JointState的Header
    auto& joint_state = axes_state_msg.joint_state;
    joint_state.header.stamp = this->now();
    joint_state.header.frame_id = "zmc_status";
    
    bool all_axes_success = true;
    float total_speed = 0.0f;
    int valid_axes_count = 0;
    
    // 读取所有轴的数据
    for (int64_t axis : running_axes_) {
        float dpos_val = 0.0;
        float mpos_val = 0.0;
        float speed_val = 0.0;
        
        // 使用ZMotion SDK函数获取轴数据
        bool axis_success = true;
        
        // 获取命令位置 (DPOS)
        if (ZAux_Direct_GetDpos(handle_, axis, &dpos_val) != ERR_OK) {
            axis_success = false;
            RCLCPP_WARN(this->get_logger(), "无法读取轴 %ld 的命令位置", axis);
        }
        
        // 获取实际位置 (MPOS)
        if (axis_success && ZAux_Direct_GetMpos(handle_, axis, &mpos_val) != ERR_OK) {
            axis_success = false;
            RCLCPP_WARN(this->get_logger(), "无法读取轴 %ld 的实际位置", axis);
        }
        
        // 获取速度
        if (axis_success && ZAux_Direct_GetVpSpeed(handle_, axis, &speed_val) != ERR_OK) {
            axis_success = false;
            RCLCPP_WARN(this->get_logger(), "无法读取轴 %ld 的速度", axis);
        }
        
        if (axis_success) {
            // 添加轴数据到JointState
            joint_state.name.push_back("axis_" + std::to_string(axis));
            joint_state.position.push_back(mpos_val);  // 使用实际位置作为关节位置
            joint_state.velocity.push_back(speed_val);
            
            // 累加速度用于计算合成速度
            total_speed += std::abs(speed_val);
            valid_axes_count++;
            
            // 调试信息
            RCLCPP_DEBUG(this->get_logger(), "轴 %ld: MPOS=%.3f, DPOS=%.3f, 速度=%.3f", 
                        axis, mpos_val, dpos_val, speed_val);
        } else {
            // 如果读取失败，添加默认值
            joint_state.name.push_back("axis_" + std::to_string(axis));
            joint_state.position.push_back(0.0);
            joint_state.velocity.push_back(0.0);
            all_axes_success = false;
            RCLCPP_WARN(this->get_logger(), "轴 %ld 数据读取失败，使用默认值", axis);
        }
    }
    
    // 计算合成速度（轴0和轴1的合成速度，假设垂直）
    if (valid_axes_count >= 2) {
        // 获取轴0和轴1的速度
        float speed_axis0 = 0.0f;
        float speed_axis1 = 0.0f;
        
        // 重新读取轴0和轴1的速度（确保数据一致性）
        bool speed_read_success = true;
        if (ZAux_Direct_GetVpSpeed(handle_, 0, &speed_axis0) != ERR_OK) {
            speed_read_success = false;
            RCLCPP_WARN(this->get_logger(), "无法读取轴0的速度用于合成速度计算");
        }
        if (ZAux_Direct_GetVpSpeed(handle_, 1, &speed_axis1) != ERR_OK) {
            speed_read_success = false;
            RCLCPP_WARN(this->get_logger(), "无法读取轴1的速度用于合成速度计算");
        }
        
        if (speed_read_success) {
            // 合成速度（假设轴0和轴1垂直）
            // 注意：AxesState消息没有speed字段，这个计算可以用于内部使用或日志
            float composite_speed = std::sqrt(speed_axis0 * speed_axis0 + speed_axis1 * speed_axis1);
            RCLCPP_DEBUG(this->get_logger(), "合成速度计算: 轴0=%.3f, 轴1=%.3f, 合成=%.3f", 
                        speed_axis0, speed_axis1, composite_speed);
        } else {
            RCLCPP_WARN(this->get_logger(), "速度读取失败，无法计算合成速度");
        }
    }
    //  else {
    //     RCLCPP_WARN(this->get_logger(), "有效轴数量不足，无法计算合成速度");
    // }
    
    // 发布AxesState消息
    if (!joint_state.name.empty()) {
        axes_state_pub_->publish(axes_state_msg);
        
        if (all_axes_success) {
            RCLCPP_DEBUG(this->get_logger(), "成功发布 %zu 个轴的状态数据到AxesState话题", 
                        joint_state.name.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "部分轴数据读取失败，成功发布 %zu 个轴的状态数据", 
                       joint_state.name.size());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "所有轴数据读取失败，无法发布AxesState消息");
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

// Action相关方法实现

// 处理轴移动Action目标请求
rclcpp_action::GoalResponse ZmcController::handleAxesMovingGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const motion_msgs::action::AxesMoving::Goal> goal) {
        
    RCLCPP_INFO(this->get_logger(), "收到轴移动Action请求");
    
    // 检查控制器是否连接
    if (!is_connected_) {
        RCLCPP_ERROR(this->get_logger(), "控制器未连接，拒绝Action请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // 检查是否有正在执行的Action
    if (action_running_) {
        RCLCPP_WARN(this->get_logger(), "已有Action正在执行，拒绝新请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // 检查目标参数有效性
        if (goal->target_axes.empty() || goal->target_positions.empty()) {
            RCLCPP_ERROR(this->get_logger(), "目标轴号或位置列表为空，拒绝Action请求");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        if (goal->target_axes.size() != goal->target_positions.size()) {
            RCLCPP_ERROR(this->get_logger(), "目标轴号数量(%zu)与位置数量(%zu)不匹配", 
                         goal->target_axes.size(), goal->target_positions.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
    
    RCLCPP_INFO(this->get_logger(), "接受轴移动Action请求");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 处理轴移动Action取消请求
rclcpp_action::CancelResponse ZmcController::handleAxesMovingCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::AxesMoving>> goal_handle) {
    
    RCLCPP_INFO(this->get_logger(), "收到轴移动Action取消请求");
    
    if (action_running_ && current_axes_moving_goal_handle_ == goal_handle) {
        // 停止所有轴的运动
        auto goal = goal_handle->get_goal();
        for (int64_t axis : goal->target_axes) {
            ZAux_Direct_Single_Cancel(handle_, static_cast<int>(axis), 0);
        }
        
        action_running_ = false;
        RCLCPP_INFO(this->get_logger(), "轴移动Action已取消");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    RCLCPP_WARN(this->get_logger(), "没有正在执行的轴移动Action可以取消");
    return rclcpp_action::CancelResponse::REJECT;
}

// 接受并执行轴移动Action
void ZmcController::handleAxesMovingAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::AxesMoving>> goal_handle) {
    
    RCLCPP_INFO(this->get_logger(), "轴移动Action目标被接受，启动异步执行");
    
    // 启动独立线程执行，避免阻塞ROS2主线程
    std::thread{std::bind(&ZmcController::executeAxesMoving, this, goal_handle)}.detach();
}

void ZmcController::executeAxesMoving(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::AxesMoving>> goal_handle) {
    
    // 设置执行状态
    action_running_ = true;
    current_axes_moving_goal_handle_ = goal_handle;
    
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<motion_msgs::action::AxesMoving::Result>();
    auto feedback = std::make_shared<motion_msgs::action::AxesMoving::Feedback>();
    
    // 记录开始时间
    auto start_time = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(this->get_logger(), "开始异步执行轴移动Action");
    
    try {
        
        // 检查控制器连接状态
        if (!is_connected_) {
            throw std::runtime_error("控制器未连接");
        }
        
        // 从参数服务器读取运动参数并设置
        setAxisMoveParameters();
        
        // 启动运动
        if (!moveAxes(goal->target_axes, goal->target_positions)) {
            throw std::runtime_error("执行运动失败");
        }
        
        RCLCPP_INFO(this->get_logger(), "运动已启动");
        
        // 记录起始位置
        std::vector<double> start_positions;
        for (int64_t axis : goal->target_axes) {
            double start_pos = 0.0;
            getMpos(static_cast<int>(axis), start_pos);
            start_positions.push_back(start_pos);
        }
        
        // 监控运动过程
        bool all_axes_completed = false;
        
        while (action_running_ && !all_axes_completed) {
            // 检查Action是否被取消
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Action被用户取消";
                auto end_time = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
                result->duration = duration;
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
            double total_progress = 0.0;
            
            for (size_t i = 0; i < goal->target_axes.size(); ++i) {
                int64_t axis = goal->target_axes[i];
                double target_position = goal->target_positions[i];
                double start_position = start_positions[i];
                
                double current_position = 0.0;
                double current_velocity = 0.0;
                
                // 读取当前位置和速度
                if (getMpos(static_cast<int>(axis), current_position) && getCurSpeed(static_cast<int>(axis), current_velocity)) {
                    feedback->current_positions.push_back(current_position);
                    feedback->current_velocities.push_back(current_velocity);
                    
                    // 检查是否到达目标位置
                    if (isAxisAtPosition(axis, target_position)) {
                        completed_axes++;
                        total_progress += 1.0; // 已完成的轴贡献100%进度
                    } else {
                        all_axes_completed = false;
                        // 计算当前轴的进度（基于位置）
                        double total_distance = fabs(target_position - start_position);
                        if (total_distance > 0.001) { // 避免除以零
                            double completed_distance = fabs(current_position - start_position);
                            double axis_progress = completed_distance / total_distance;
                            total_progress += axis_progress;
                        } else {
                            total_progress += 1.0; // 如果距离很小，视为已完成
                        }
                    }
                } else {
                    all_axes_completed = false;
                    RCLCPP_WARN(this->get_logger(), "无法读取轴 %ld 的当前位置和速度", axis);
                }
            }
            
            // 计算总进度
            feedback->progress = static_cast<float>(total_progress) / goal->target_axes.size();
            if (feedback->progress > 1.0) feedback->progress = 1.0; // 确保进度不超过100%
            
            // 计算已执行时间
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count();
            feedback->elapsed_time = elapsed;
            
            // 发布反馈
            goal_handle->publish_feedback(feedback);
            
            RCLCPP_DEBUG(this->get_logger(), "运动进度: %.1f%%, 耗时: %.2f秒", 
                        feedback->progress * 100, elapsed);
            
            // 短暂休眠，避免过度占用CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        if (action_running_) {
            // 所有轴都到达目标位置
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
            result->duration = duration;
            result->success = true;
            result->message = "所有轴成功到达目标位置";
            result->final_positions = feedback->current_positions;
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "轴移动Action执行成功完成，耗时: %.2f秒", duration);
        }
        
    } catch (const std::exception& e) {
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
        result->duration = duration;
        result->success = false;
        result->message = "轴移动Action执行失败: " + std::string(e.what());
        
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "轴移动Action执行失败: %s, 耗时: %.2f秒", e.what(), duration);
    }
    
    // 清理执行状态
    action_running_ = false;
    current_axes_moving_goal_handle_.reset();
}

// 执行轴运动（支持单轴或多轴）
bool ZmcController::moveAxes(const std::vector<int64_t>& axes, const std::vector<double>& positions) {
    if (!is_connected_) {
        RCLCPP_ERROR(this->get_logger(), "控制器未连接");
        return false;
    }
    
    // 检查参数有效性
    if (axes.empty() || positions.empty()) {
        RCLCPP_ERROR(this->get_logger(), "目标轴或位置列表为空");
        return false;
    }
    
    if (axes.size() != positions.size()) {
        RCLCPP_ERROR(this->get_logger(), "目标轴数量与位置数量不匹配");
        return false;
    }
    
    // 为所有运行轴设置默认运动参数
    setAxisMoveParameters();
    
    size_t axis_count = axes.size();
    
    if (axis_count == 1) {
        // 单轴运动
        int64_t axis = axes[0];
        double target_position = positions[0];
        
        RCLCPP_INFO(this->get_logger(), "执行单轴运动: 轴 %ld 移动到 %.3f", axis, target_position);
        
        if (!checkError(ZAux_Direct_Single_MoveAbs(handle_, static_cast<int>(axis), target_position))) {
            RCLCPP_ERROR(this->get_logger(), "设置轴 %ld 目标位置失败", axis);
            return false;
        }
    } else if (axis_count == 2 || axis_count == 3) {
        // 合成运动
        RCLCPP_INFO(this->get_logger(), "执行合成运动: 轴列表 %s", vectorToString(axes).c_str());
        
        // 创建转换后的数组
        std::vector<int> axis_vec(axis_count);
        std::vector<float> position_vec(axis_count);
        
        // 转换数据类型
        for (size_t i = 0; i < axis_count; ++i) {
            axis_vec[i] = static_cast<int>(axes[i]);
            position_vec[i] = static_cast<float>(positions[i]);
        }
        
        // 设置基轴列表
        if (!checkError(ZAux_Direct_Base(handle_, static_cast<int>(axis_count), axis_vec.data()))) {
            RCLCPP_ERROR(this->get_logger(), "设置基轴列表失败");
            return false;
        }
        
        // 执行合成运动
        if (!checkError(ZAux_Direct_MoveAbs(handle_, static_cast<int>(axis_count), axis_vec.data(), position_vec.data()))) {
            RCLCPP_ERROR(this->get_logger(), "执行合成运动失败");
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "不支持的轴数量: %zu，只支持1-3个轴", axis_count);
        return false;
    }
    
    // 启动监控线程
    std::thread{std::bind(&ZmcController::monitorAxesMotion, this, axes, positions)}.detach();
    
    return true;
}

// 检查轴是否到达目标位置
bool ZmcController::isAxisAtPosition(int axis, double target_position, double tolerance) {
    if (!is_connected_) return false;
    
    double current_position = 0.0;
    if (!getMpos(axis, current_position)) {
        return false;
    }
    
    return std::abs(current_position - target_position) <= tolerance;
}

// 通用轴监控函数（内部方法）
template <typename CheckFunc>
bool ZmcController::monitorAxes(const std::vector<int64_t>& axes, double timeout, CheckFunc check_func, const std::string& operation_name) {
    auto start_time = std::chrono::steady_clock::now();
    bool all_axes_completed = false;
    
    while (!all_axes_completed) {
        all_axes_completed = true;
        int completed_axes = 0;
        
        for (int axis : axes) {
            bool completed = check_func(axis);
            if (!completed) {
                all_axes_completed = false;
            } else {
                completed_axes++;
            }
        }
        
        // 检查超时
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        if (elapsed.count() > timeout) {
            RCLCPP_ERROR(this->get_logger(), "%s 超时", operation_name.c_str());
            return false;
        }
        
        float progress = static_cast<float>(completed_axes) / axes.size() * 100.0f;
        RCLCPP_DEBUG(this->get_logger(), "%s 进度: %.1f%%, 耗时: %lds", operation_name.c_str(), progress, elapsed.count());
        
        if (!all_axes_completed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "%s 完成", operation_name.c_str());
    return true;
}

// 监控轴运动进度（内部方法）
void ZmcController::monitorAxesMotion(const std::vector<int64_t>& target_axes, const std::vector<double>& target_positions) {
    auto start_time = std::chrono::steady_clock::now();
    bool all_axes_completed = false;
    
    while (!all_axes_completed) {
        all_axes_completed = true;
        int completed_axes = 0;
        
        for (size_t i = 0; i < target_axes.size(); ++i) {
            int64_t axis = target_axes[i];
            double target_position = target_positions[i];
            
            if (!isAxisAtPosition(static_cast<int>(axis), target_position)) {
                all_axes_completed = false;
            } else {
                completed_axes++;
            }
        }
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        
        float progress = static_cast<float>(completed_axes) / target_axes.size() * 100.0f;
        
        RCLCPP_DEBUG(this->get_logger(), "运动进度: %.1f%%, 耗时: %lds", progress, elapsed.count());
        
        if (!all_axes_completed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "轴运动完成");
}

// 处理多轴回零Action目标请求
rclcpp_action::GoalResponse ZmcController::handleAxesHomingGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const motion_msgs::action::AxesHoming::Goal> goal) {
    
    RCLCPP_INFO(this->get_logger(), "收到多轴回零Action请求");
    
    // 检查控制器是否连接
    if (!is_connected_) {
        RCLCPP_ERROR(this->get_logger(), "控制器未连接，拒绝Action请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // 检查是否有正在执行的Action
    if (action_running_) {
        RCLCPP_WARN(this->get_logger(), "已有Action正在执行，拒绝新请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // 检查目标参数有效性
    if (goal->homing_axes.empty()) {
        RCLCPP_ERROR(this->get_logger(), "轴列表为空，拒绝Action请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    for (int64_t axis : goal->homing_axes) {
        if (axis < 0 || axis > 5) {
            RCLCPP_ERROR(this->get_logger(), "轴号 %ld 无效，拒绝Action请求", axis);
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    
    // 其他参数从参数服务器读取，不在action消息中传递
    
    RCLCPP_INFO(this->get_logger(), "接受多轴回零Action请求");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 处理多轴回零Action取消请求
rclcpp_action::CancelResponse ZmcController::handleAxesHomingCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::AxesHoming>> goal_handle) {
    
    RCLCPP_INFO(this->get_logger(), "收到多轴回零Action取消请求");
    
    if (action_running_ && current_axes_homing_goal_handle_ == goal_handle) {
            // 停止所有轴的运动
        auto goal = goal_handle->get_goal();
        for (int64_t axis : goal->homing_axes) {
            ZAux_Direct_Single_Cancel(handle_, static_cast<int>(axis), 0);
        }
            
            action_running_ = false;
            RCLCPP_INFO(this->get_logger(), "多轴回零Action已取消");
            return rclcpp_action::CancelResponse::ACCEPT;
        }
    
    RCLCPP_WARN(this->get_logger(), "没有正在执行的多轴回零Action可以取消");
    return rclcpp_action::CancelResponse::REJECT;
}

// 接受并执行多轴回零Action
void ZmcController::handleAxesHomingAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::AxesHoming>> goal_handle) {
    
    RCLCPP_INFO(this->get_logger(), "多轴回零Action目标被接受，启动异步执行");
    
    // 启动独立线程执行，避免阻塞ROS2主线程
    std::thread{std::bind(&ZmcController::executeAxesHoming, this, goal_handle)}.detach();
}

// 执行多轴回零的Action（异步执行）
void ZmcController::executeAxesHoming(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::AxesHoming>> goal_handle) {
    
    // 设置执行状态
    action_running_ = true;
    current_axes_homing_goal_handle_ = goal_handle;
    
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<motion_msgs::action::AxesHoming::Result>();
    auto feedback = std::make_shared<motion_msgs::action::AxesHoming::Feedback>();
    
    RCLCPP_INFO(this->get_logger(), "开始异步执行多轴回零Action");
    
    // 记录开始时间
    auto start_time = std::chrono::steady_clock::now();
    
    try {
        // 检查控制器连接状态
        if (!is_connected_) {
            throw std::runtime_error("控制器未连接");
        }
        
        // 验证轴参数有效性
        if (goal->homing_axes.empty()) {
            throw std::runtime_error("轴列表为空");
        }
        
        for (int64_t axis : goal->homing_axes) {
            if (axis < 0 || axis > 5) {
                throw std::runtime_error("轴号 " + std::to_string(axis) + " 无效");
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "开始执行多轴回零，轴列表: %s", vectorToString(goal->homing_axes).c_str());
        
        // 存储每个轴的回零状态
        struct AxisHomingStatus {
            bool completed = false;
            bool success = false;
        };
        std::map<int64_t, AxisHomingStatus> axis_status;
        
        // 设置所有轴的回零参数
        setAxisHomeParameters();
        
        // 读取回零模式参数
        auto homing_modes = this->get_parameter("axis_homing_mode").as_integer_array();
        
        // 从参数服务器读取回零超时参数，设置默认值
        auto timeout_array = this->get_parameter("axis_homing_timeout").as_double_array();
        
        // 串行执行回零操作：一个轴完成后再启动下一个轴
        for (int64_t axis : goal->homing_axes) {
            // 检查Action是否被取消
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Action被用户取消";
                auto end_time = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
                result->duration = duration;
                goal_handle->canceled(result);
                action_running_ = false;
                RCLCPP_INFO(this->get_logger(), "Action执行被取消，耗时: %.2f秒", duration);
                return;
            }
            
            // 再次检查控制器连接状态
            if (!is_connected_) {
                throw std::runtime_error("控制器连接断开");
            }
            
            // 获取轴在holding_axes中的索引
            int64_t holding_index = getAxisHoldingIndex(axis);
            
            // 获取回零模式
            int64_t homing_mode = 11;
            if (holding_index >= 0 && static_cast<size_t>(holding_index) < homing_modes.size()) {
                homing_mode = static_cast<int>(homing_modes[holding_index]);
            }
            
            RCLCPP_INFO(this->get_logger(), "开始轴 %ld 回零，模式: %ld", axis, homing_mode);
            
            // 执行回零操作（使用总线命令）
            if (!checkError(ZAux_BusCmd_Datum(handle_, static_cast<int>(axis), homing_mode))) {
                throw std::runtime_error("轴 " + std::to_string(axis) + " 启动回零失败");
            }
            
            // 初始化轴状态
            axis_status[axis] = AxisHomingStatus();
            
            // 等待当前轴回零完成
            bool axis_completed = false;
            auto axis_start_time = std::chrono::steady_clock::now();
            
            while (!axis_completed && action_running_) {
                // 检查Action是否被取消
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->message = "Action被用户取消";
                    goal_handle->canceled(result);
                    action_running_ = false;
                    RCLCPP_INFO(this->get_logger(), "Action执行被取消");
                    return;
                }
                
                // 再次检查控制器连接状态
                if (!is_connected_) {
                    throw std::runtime_error("控制器连接断开");
                }
                
                // 读取回零状态
                uint32 homing_status = 0;
                bool home_status_ok = checkError(ZAux_BusCmd_GetHomeStatus(handle_, static_cast<int>(axis), &homing_status));
                
                // 读取轴状态（物理停止状态）
                int32 idle_status = 0;
                bool idle_status_ok = checkError(ZAux_Direct_GetIfIdle(handle_, static_cast<int>(axis), &idle_status));
                
                // 读取运动类型（逻辑任务状态）
                int32 mtype = 0;
                bool mtype_ok = checkError(ZAux_Direct_GetMtype(handle_, static_cast<int>(axis), &mtype));
                
                // 判断回零完成条件
                if (home_status_ok && idle_status_ok && mtype_ok) {
                    if (homing_status == 1 && idle_status == -1 && mtype == 0) {
                        // 回零成功完成：回零状态正常 + 物理停止 + 逻辑任务结束
                        axis_status[axis].completed = true;
                        axis_status[axis].success = true;
                        RCLCPP_INFO(this->get_logger(), "轴 %ld 回零成功", axis);
                        
                        // 检查是否需要在回零后移动
                        bool move_after_homing = this->get_parameter("move_after_homing").as_bool();
                        if (move_after_homing) {
                            // 获取回零后移动位置
                            float position = getAxisHomingMovePosition(axis);
                            
                            // 执行移动操作
                            RCLCPP_INFO(this->get_logger(), "轴 %ld 开始回零后移动到位置 %.3f", axis, position);
                            if (!moveAxes({axis}, {position})) {
                                RCLCPP_ERROR(this->get_logger(), "轴 %ld 回零后移动失败", axis);
                                // 回零后移动失败不影响回零成功状态
                            } else {
                                RCLCPP_INFO(this->get_logger(), "轴 %ld 回零后移动成功", axis);
                            }
                        }
                        
                        axis_completed = true;
                    } else if (homing_status == 0) {
                        // 回零还在进行中，继续等待
                        RCLCPP_DEBUG(this->get_logger(), "轴 %ld 回零状态: 进行中 (home_status=%d, idle=%d, mtype=%d)", 
                                    axis, homing_status, idle_status, mtype);
                    } else {
                        // 其他状态值，视为异常
                        axis_status[axis].completed = true;
                        axis_status[axis].success = false;
                        RCLCPP_WARN(this->get_logger(), "轴 %ld 获取状态失败: home=%d, idle=%d, mtype=%d", 
                                axis, homing_status, idle_status, mtype);
                        axis_completed = true;
                    }
                } else {
                    // 状态获取失败，继续尝试
                    RCLCPP_WARN(this->get_logger(), "轴 %ld 获取状态失败: home=%d, idle=%d, mtype=%d", 
                                axis, home_status_ok ? 1 : 0, idle_status_ok ? 1 : 0, mtype_ok ? 1 : 0);
                }
                
                // 更新反馈信息
                feedback->current_positions.clear();
                
                // 获取当前轴的位置
                float current_position = 0.0;
                if (!getMpos(static_cast<int>(axis), current_position)) {
                    RCLCPP_WARN(this->get_logger(), "无法获取轴 %ld 的当前位置", axis);
                }
                
                feedback->current_positions.push_back(current_position);
                
                // 更新反馈状态
                feedback->current_phase = axis_completed ? "轴回零完成" : "回零中";
                
                // 计算已执行时间
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - axis_start_time);
                feedback->elapsed_time = static_cast<double>(elapsed.count());
                
                // 发布反馈
                goal_handle->publish_feedback(feedback);
                
                // 检查超时
                double axis_timeout = 30.0; // 默认30秒
                if (!timeout_array.empty() && holding_index >= 0 && static_cast<size_t>(holding_index) < timeout_array.size()) {
                    axis_timeout = timeout_array[holding_index];
                }
                if (feedback->elapsed_time > axis_timeout) {
                    throw std::runtime_error("轴 " + std::to_string(axis) + " 回零超时");
                }
                
                // 短暂休眠，避免过度占用CPU
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            RCLCPP_INFO(this->get_logger(), "轴 %ld 回零完成，继续下一个轴", axis);
        }
        
        if (action_running_) {
            // 所有轴回零完成，获取最终位置
            result->final_positions.clear();
            
            for (int64_t axis : goal->homing_axes) {
                float position = 0.0;
                if (getMpos(static_cast<int>(axis), position)) {
                    result->final_positions.push_back(position);
                } else {
                    result->final_positions.push_back(0.0);
                }
            }
            
            result->success = true;
            result->message = "所有轴回零成功";
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
            result->duration = duration;
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "多轴回零Action执行成功完成，耗时: %.2f秒", duration);
        }
        
    } catch (const std::exception& e) {
        result->success = false;
        result->message = "多轴回零Action执行失败: " + std::string(e.what());
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
        result->duration = duration;
        
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "多轴回零Action执行失败: %s, 耗时: %.2f秒", e.what(), duration);
    }
    
    // 清理执行状态
    action_running_ = false;
    current_axes_homing_goal_handle_.reset();
}

// 执行轴回零操作
bool ZmcController::homeAxes(const std::vector<int64_t>& axes) {
    if (!is_connected_) {
        RCLCPP_ERROR(this->get_logger(), "控制器未连接");
        return false;
    }
    
    // 验证轴参数有效性
    if (axes.empty()) {
        RCLCPP_ERROR(this->get_logger(), "轴列表为空");
        return false;
    }
    
    for (int64_t axis : axes) {
        if (axis < 0 || axis > 5) {
            RCLCPP_ERROR(this->get_logger(), "轴号 %ld 无效", axis);
            return false;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "开始执行多轴回零，轴列表: %s", vectorToString(axes).c_str());
    
    // 存储每个轴的回零状态
    struct AxisHomingStatus {
        bool completed = false;
        bool success = false;
    };
    std::map<int64_t, AxisHomingStatus> axis_status;
    
    // 设置所有轴的回零参数
    setAxisHomeParameters();
    
    // 读取回零模式参数
    auto homing_modes = this->get_parameter("axis_homing_mode").as_integer_array();
    
    // 从参数服务器读取回零超时参数，设置默认值
    double timeout = 30.0; // 默认30秒
    
    // 读取超时参数数组
    try {
        auto timeout_array = this->get_parameter("axis_homing_timeout").as_double_array();
        if (!timeout_array.empty()) {
            timeout = timeout_array[0]; // 使用第一个值作为默认超时
        }
    } catch (const rclcpp::exceptions::InvalidParameterTypeException& e) {
        RCLCPP_WARN(this->get_logger(), "无法读取回零超时参数，使用默认值 %.1f 秒", timeout);
    }
    
    // 验证超时参数有效性
    if (timeout <= 0) {
        timeout = 30.0; // 默认30秒
        RCLCPP_WARN(this->get_logger(), "回零超时参数无效，使用默认值 %.1f 秒", timeout);
    }
    
    RCLCPP_INFO(this->get_logger(), "回零超时设置为 %.1f 秒", timeout);
    
    // 串行执行回零操作：一个轴完成后再启动下一个轴
    for (int64_t axis : axes) {
        // 再次检查控制器连接状态
        if (!is_connected_) {
            RCLCPP_ERROR(this->get_logger(), "控制器连接断开");
            return false;
        }
        
        // 获取轴在holding_axes中的索引
        int64_t holding_index = getAxisHoldingIndex(axis);
        
        // 获取回零模式
        int64_t homing_mode = 11;
        if (holding_index >= 0 && static_cast<size_t>(holding_index) < homing_modes.size()) {
            homing_mode = static_cast<int>(homing_modes[holding_index]);
        }
        
        RCLCPP_INFO(this->get_logger(), "开始轴 %ld 回零，模式: %ld", axis, homing_mode);
        
        // 获取回零开始前的位置
        float start_position = 0.0;
        getMpos(static_cast<int>(axis), start_position);
        float target_position = 0.0; // 回零的目标位置通常是0
        float total_distance = fabs(start_position - target_position);
        
        RCLCPP_INFO(this->get_logger(), "轴 %ld 回零开始，起始位置: %.3f, 目标位置: %.3f, 总距离: %.3f", 
                   axis, start_position, target_position, total_distance);
        
        // 执行回零操作（使用总线命令）
        if (!checkError(ZAux_BusCmd_Datum(handle_, axis, homing_mode))) {
            RCLCPP_ERROR(this->get_logger(), "轴 %ld 启动回零失败", axis);
            return false;
        }
        
        // 初始化轴状态
        axis_status[axis] = AxisHomingStatus();
        
        // 等待当前轴回零完成
        bool axis_completed = false;
        auto start_time = std::chrono::steady_clock::now();
        int last_progress = -1; // 用于跟踪上次输出的进度，避免重复输出
        
        while (!axis_completed) {
            // 再次检查控制器连接状态
            if (!is_connected_) {
                RCLCPP_ERROR(this->get_logger(), "控制器连接断开");
                axis_status[axis].completed = true;
                axis_status[axis].success = false;
                return false;
            }
            
            // 读取回零状态
            uint32 homing_status = 0;
            bool home_status_ok = checkError(ZAux_BusCmd_GetHomeStatus(handle_, axis, &homing_status));
            
            // 读取轴状态（物理停止状态）
            int32 idle_status = 0;
            bool idle_status_ok = checkError(ZAux_Direct_GetIfIdle(handle_, axis, &idle_status));
            
            // 读取运动类型（逻辑任务状态）
            int32 mtype = 0;
            bool mtype_ok = checkError(ZAux_Direct_GetMtype(handle_, axis, &mtype));
            
            // 计算已执行时间
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            double elapsed_seconds = static_cast<double>(elapsed.count());
            
            // 读取当前位置
            float current_position = 0.0;
            getMpos(static_cast<int>(axis), current_position);
            
            // 计算基于位置的进度
            int progress = 0;
            if (total_distance > 0.001) { // 避免除以零
                float completed_distance = fabs(start_position - current_position);
                progress = static_cast<int>((completed_distance / total_distance) * 100.0);
                if (progress > 100) progress = 100;
            } else {
                progress = 100; // 如果总距离很小，直接认为已完成
            }
            
            // 判断回零完成条件
            if (home_status_ok && idle_status_ok && mtype_ok) {
                if (homing_status == 1 && idle_status == -1 && mtype == 0) {
                    // 回零成功完成：回零状态正常 + 物理停止 + 逻辑任务结束
                    axis_status[axis].completed = true;
                    axis_status[axis].success = true;
                    RCLCPP_INFO(this->get_logger(), "轴 %ld 回零成功，耗时: %.1f 秒，最终位置: %.3f", axis, elapsed_seconds, current_position);
                    
                    // 检查是否需要在回零后移动
                    bool move_after_homing = this->get_parameter("move_after_homing").as_bool();
                    if (move_after_homing) {
                        // 获取回零后移动位置
                        float move_position = getAxisHomingMovePosition(axis);
                        
                        // 计算移动距离
                        float move_start_position = 0.0; // 回零后的位置
                        getMpos(static_cast<int>(axis), move_start_position);
                        float move_total_distance = fabs(move_start_position - move_position);
                        
                        RCLCPP_INFO(this->get_logger(), "轴 %ld 开始回零后移动到位置 %.3f，起始位置: %.3f, 总距离: %.3f", 
                                   axis, move_position, move_start_position, move_total_distance);
                        
                        // 执行移动操作
                        if (!moveAxes({axis}, {move_position})) {
                            RCLCPP_ERROR(this->get_logger(), "轴 %ld 回零后移动失败", axis);
                            // 回零后移动失败不影响回零成功状态
                        } else {
                            RCLCPP_INFO(this->get_logger(), "轴 %ld 回零后移动成功", axis);
                        }
                    }
                    
                    axis_completed = true;
                } else if (homing_status == 0) {
                    // 回零还在进行中，继续等待
                    // 每10%进度或状态变化时输出一次进度信息
                    if (progress % 10 == 0 && progress != last_progress) {
                        RCLCPP_INFO(this->get_logger(), "轴 %ld 回零进度: %d%%, 耗时: %.1f 秒, 已移动: %.3f", 
                                   axis, progress, elapsed_seconds, 
                                   fabs(start_position - current_position));
                        last_progress = progress;
                    }
                    RCLCPP_DEBUG(this->get_logger(), "轴 %ld 回零状态: 进行中 (home_status=%d, idle=%d, mtype=%d)", 
                                axis, homing_status, idle_status, mtype);
                    // 短暂休眠，避免过度占用CPU
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                } else {
                    // 其他状态值，视为异常
                    axis_status[axis].completed = true;
                    axis_status[axis].success = false;
                    RCLCPP_WARN(this->get_logger(), "轴 %ld 获取状态失败: home=%d, idle=%d, mtype=%d", 
                            axis, homing_status, idle_status, mtype);
                    axis_completed = true;
                }
            } else {
                // 状态获取失败，继续尝试
                RCLCPP_WARN(this->get_logger(), "轴 %ld 获取状态失败: home=%d, idle=%d, mtype=%d", 
                            axis, home_status_ok ? 1 : 0, idle_status_ok ? 1 : 0, mtype_ok ? 1 : 0);
                // 短暂休眠，避免过度占用CPU
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // 检查超时
            double axis_timeout = timeout; // 使用默认超时
            
            // 尝试从超时参数数组中获取当前轴的超时值
            try {
                auto timeout_array = this->get_parameter("axis_homing_timeout").as_double_array();
                if (!timeout_array.empty() && holding_index >= 0 && static_cast<size_t>(holding_index) < timeout_array.size()) {
                    axis_timeout = timeout_array[holding_index];
                }
            } catch (const rclcpp::exceptions::InvalidParameterTypeException& e) {
                // 使用默认超时
            }
            
            if (elapsed_seconds > axis_timeout) {
                RCLCPP_ERROR(this->get_logger(), "轴 %ld 回零超时，耗时: %.1f 秒，当前位置: %.3f", axis, elapsed_seconds, current_position);
                axis_status[axis].completed = true;
                axis_status[axis].success = false;
                return false;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "轴 %ld 回零完成，继续下一个轴", axis);
    }
    
    // 检查是否所有轴都回零成功
    bool all_success = true;
    for (const auto& [axis, status] : axis_status) {
        if (!status.success) {
            all_success = false;
            RCLCPP_ERROR(this->get_logger(), "轴 %ld 回零失败", axis);
        }
    }
    
    if (all_success) {
        RCLCPP_INFO(this->get_logger(), "所有轴回零成功");
    } else {
        RCLCPP_ERROR(this->get_logger(), "部分轴回零失败");
    }
    
    return all_success;
}

// 初始化轴参数
void ZmcController::initializeAxisParameters() {

    // 读取running_axes参数并赋值
    running_axes_ = this->get_parameter("running_axes").as_integer_array();

    std::string axes_str = "实际可控轴: [" + vectorToString(running_axes_) + "]";
    RCLCPP_INFO(this->get_logger(), "%s", axes_str.c_str());

    // 读取轴参数
    auto pulse_equivalent = this->get_parameter("axis_moving_pulse_equivalent").as_double_array();
    
    RCLCPP_INFO(this->get_logger(), "开始初始化轴参数");
    
    // 设置轴参数（脉冲当量）
    for (size_t i = 0; i < running_axes_.size(); ++i) {
        int64_t axis = running_axes_[i];
        
        // 获取轴在holding_axes中的索引
        int64_t holding_index = getAxisHoldingIndex(axis);
        
        // 检查轴是否在holding_axes中
        if (holding_index < 0) {
            RCLCPP_WARN(this->get_logger(), "轴 %ld 不在holding_axes中，跳过参数设置", axis);
            continue;
        }
        
        // 检查索引是否在参数数组范围内
        size_t hold_idx = static_cast<size_t>(holding_index);
        
        // 设置脉冲当量（参数）
        if (hold_idx < pulse_equivalent.size()) {
            if (checkError(ZAux_Direct_SetUnits(handle_, axis, pulse_equivalent[hold_idx]))) {
                // RCLCPP_INFO(this->get_logger(), "轴 %d: 脉冲当量设置为 %.3f", axis, pulse_equivalent[hold_idx]);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "轴 %ld 的holding_axes索引 %ld 超出脉冲当量参数数组范围", axis, holding_index);
        }
    }
    
    // 叠加轴2到轴1
    RCLCPP_INFO(this->get_logger(), "叠加轴2到轴1");
    if (checkError(ZAux_Direct_Single_Addax(handle_, 1, 2))) {
        RCLCPP_INFO(this->get_logger(), "轴2叠加到轴1设置成功");
    } else {
        RCLCPP_ERROR(this->get_logger(), "轴2叠加到轴1设置失败");
    }
    
    RCLCPP_INFO(this->get_logger(), "轴参数初始化完成");
}

// 辅助函数：将向量转换为字符串
std::string ZmcController::vectorToString(const std::vector<int>& vec) const {
    std::stringstream ss;
    for (size_t i = 0; i < vec.size(); ++i) {
        ss << vec[i];
        if (i < vec.size() - 1) ss << ", ";
    }
    return ss.str();
}

// 重载版本：支持int64_t类型
std::string ZmcController::vectorToString(const std::vector<int64_t>& vec) const {
    std::stringstream ss;
    for (size_t i = 0; i < vec.size(); ++i) {
        ss << vec[i];
        if (i < vec.size() - 1) ss << ", ";
    }
    return ss.str();
}

// 辅助函数：获取轴在holding_axes中的索引
int64_t ZmcController::getAxisHoldingIndex(int64_t axis) const {
    // 读取holding_axes参数
    std::vector<int64_t> holding_axes = this->get_parameter("holding_axes").as_integer_array();
    
    // 查找轴在holding_axes中的索引
    for (size_t i = 0; i < holding_axes.size(); ++i) {
        if (holding_axes[i] == axis) {
            return static_cast<int64_t>(i);
        }
    }
    
    // 轴不在holding_axes中
    RCLCPP_WARN(this->get_logger(), "轴 %ld 不在holding_axes中", axis);
    return static_cast<int64_t>(-1);
}

// 辅助函数：为所有运行轴设置默认运动参数
void ZmcController::setAxisMoveParameters() {
    // 从参数服务器获取参数
    auto max_speed = this->get_parameter("axis_moving_max_speed").as_double_array();
    auto acceleration = this->get_parameter("axis_moving_acceleration").as_double_array();
    auto deceleration = this->get_parameter("axis_moving_deceleration").as_double_array();
    
    // 遍历所有运行轴
    for (int64_t axis : running_axes_) {
        // 获取轴在holding_axes中的索引
        int64_t holding_index = getAxisHoldingIndex(axis);
        
        // 获取速度参数
        double speed = max_speed.empty() ? 50.0 : (holding_index >= 0 && static_cast<size_t>(holding_index) < max_speed.size() ? max_speed[holding_index] : max_speed[0]);
        
        // 获取加速度参数
        double accel = acceleration.empty() ? 150.0 : (holding_index >= 0 && static_cast<size_t>(holding_index) < acceleration.size() ? acceleration[holding_index] : acceleration[0]);
        
        // 获取减速度参数
        double decel = deceleration.empty() ? 150.0 : (holding_index >= 0 && static_cast<size_t>(holding_index) < deceleration.size() ? deceleration[holding_index] : deceleration[0]);
        
        // 设置轴速度
        if (checkError(ZAux_Direct_SetSpeed(handle_, axis, static_cast<float>(speed)))) {
            // RCLCPP_INFO(this->get_logger(), "轴 %d: 最大速度设置为 %.3f", axis, speed);
        }
        
        // 设置轴加速度
        if (checkError(ZAux_Direct_SetAccel(handle_, axis, static_cast<float>(accel)))) {
            // RCLCPP_INFO(this->get_logger(), "轴 %d: 加速度设置为 %.3f", axis, accel);
        }
        
        // 设置轴减速度
        if (checkError(ZAux_Direct_SetDecel(handle_, axis, static_cast<float>(decel)))) {
            // RCLCPP_INFO(this->get_logger(), "轴 %d: 减速度设置为 %.3f", axis, decel);
        }
    }
}

// 辅助函数：获取轴的回零后移动位置
float ZmcController::getAxisHomingMovePosition(int64_t axis) {
    // 从参数服务器获取参数
    auto move_positions = this->get_parameter("move_after_homing_positions").as_double_array();
    
    // 获取轴在holding_axes中的索引
    int64_t holding_index = getAxisHoldingIndex(axis);
    
    // 获取位置参数
    if (holding_index >= 0 && static_cast<size_t>(holding_index) < move_positions.size()) {
        return static_cast<float>(move_positions[holding_index]);
    }
    
    return 0.0f;
}

// 辅助函数：设置所有运行轴的回零参数（不包括回零模式）
void ZmcController::setAxisHomeParameters() {
    // 读取回零参数
    auto homing_velocities_high = this->get_parameter("axis_homing_velocity_high").as_double_array();
    auto homing_velocities_low = this->get_parameter("axis_homing_velocity_low").as_double_array();
    auto homing_velocities_creep = this->get_parameter("axis_homing_velocity_creep").as_double_array();
    
    // 遍历所有运行轴
    for (size_t i = 0; i < running_axes_.size(); ++i) {
        int64_t axis = running_axes_[i];
        
        // 设置默认值
        float velocity_high = 50.0;
        float velocity_low = 10.0;
        float velocity_creep = 5.0;
        
        // 获取轴在holding_axes中的索引
        int64_t holding_index = getAxisHoldingIndex(axis);
        
        // 使用轴在holding_axes中的索引获取参数，否则使用默认值
        if (holding_index >= 0) {
            if (static_cast<size_t>(holding_index) < homing_velocities_high.size()) {
                velocity_high = homing_velocities_high[holding_index];
            }
            if (static_cast<size_t>(holding_index) < homing_velocities_low.size()) {
                velocity_low = homing_velocities_low[holding_index];
            }
            if (static_cast<size_t>(holding_index) < homing_velocities_creep.size()) {
                velocity_creep = homing_velocities_creep[holding_index];
            }
        }
        
        // 设置回零低速
        if (velocity_low > 0) {
            if (!checkError(ZAux_Direct_SetSpeed(handle_, axis, velocity_low))) {
                RCLCPP_WARN(this->get_logger(), "设置轴 %ld 回零低速失败", axis);
                // 继续执行，不因为低速设置失败而终止回零
            } else {
                RCLCPP_INFO(this->get_logger(), "轴 %ld 回零低速设置为 %.3f", axis, velocity_low);
            }
        }
        
        // 设置回零蠕动速度
        if (velocity_creep > 0) {
            if (!checkError(ZAux_Direct_SetCreep(handle_, axis, velocity_creep))) {
                RCLCPP_WARN(this->get_logger(), "设置轴 %ld 回零蠕动速度失败", axis);
                // 继续执行，不因为蠕动速度设置失败而终止回零
            } else {
                RCLCPP_INFO(this->get_logger(), "轴 %ld 回零蠕动速度设置为 %.3f", axis, velocity_creep);
            }
        }
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