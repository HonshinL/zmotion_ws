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
    connect_search_timeout_ms_ = this->declare_parameter<int>("controller_connect_search_timeout_ms", 10000);
    
    // 轴参数
    this->declare_parameter<std::vector<double>>("axis_pulse_equivalent", {13107.2, 13107.2, 13107.2, 1000.0, 1000.0});
    this->declare_parameter<std::vector<double>>("axis_max_speed", {50.0, 50.0, 50.0, 50.0, 50.0});
    this->declare_parameter<std::vector<double>>("axis_acceleration", {150.0, 150.0, 150.0, 150.0, 150.0});
    this->declare_parameter<std::vector<double>>("axis_deceleration", {150.0, 150.0, 150.0, 150.0, 150.0});
    
    // 回零参数
    this->declare_parameter<std::vector<int>>("axis_homing_mode", {11, 11, 11, 11, 11});
    this->declare_parameter<std::vector<double>>("axis_homing_velocity_high", {50.0, 50.0, 50.0, 50.0, 50.0});
    this->declare_parameter<std::vector<double>>("axis_homing_velocity_low", {10.0, 10.0, 10.0, 10.0, 10.0});
    this->declare_parameter<std::vector<double>>("axis_homing_velocity_creep", {5.0, 5.0, 5.0, 5.0, 5.0});
    this->declare_parameter<std::vector<double>>("axis_homing_timeout", {60.0, 60.0, 60.0, 60.0, 60.0});
    
    // 启动回零配置
    this->declare_parameter<bool>("auto_homing_on_start", true);
    this->declare_parameter<std::vector<int>>("auto_homing_axes", {0, 1, 2, 4, 5});
    this->declare_parameter<double>("auto_homing_timeout", 60.0);
    
    // 初始化轴列表（假设支持4个轴）
    axes_ = {0, 1, 2, 4, 5}; // 示例轴号，根据实际情况调整

    // 创建发布者 (Publisher)
    // 发布运动状态
    motion_status_pub_ = this->create_publisher<motion_msgs::msg::MotionStatus>("zmc_pub/motion_status", 10);

    // 创建ObjectPosition消息订阅者
    object_position_sub_ = this->create_subscription<motion_msgs::msg::ObjectPosition>(
        "/app_pos", 10,
        std::bind(&ZmcController::handleObjectPosition, this, std::placeholders::_1));

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
    axis_homing_action_server_ = rclcpp_action::create_server<motion_msgs::action::AxisHoming>(
        this,
        "zmc_act/axis_homing",
        std::bind(&ZmcController::handleAxisHomingGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ZmcController::handleAxisHomingCancel, this, std::placeholders::_1),
        std::bind(&ZmcController::handleAxisHomingAccepted, this, std::placeholders::_1));
    
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
    
    // 直接使用固定IP地址 192.168.0.11
    std::string ip = "192.168.0.11";
    
    RCLCPP_INFO(this->get_logger(), "开始尝试连接控制器: %s", ip.c_str());

    std::thread([this, ip]() {
        // 直接尝试连接，不进行搜索
        bool connect_success = connect(ip);
        
        if (connect_success) {
            RCLCPP_INFO(this->get_logger(), "✅ 成功连接到控制器: %s", ip.c_str());
            RCLCPP_INFO(this->get_logger(), "📊 开始监控 %d 个轴: [%d, %d, %d, %d, %d]", 
                       NUM_AXES, AXES[0], AXES[1], AXES[2], AXES[3], AXES[4]);
            
            // 初始化轴参数
            initializeAxisParameters();
            
            // 启动时自动回零
            bool auto_homing = this->get_parameter("auto_homing_on_start").as_bool();
            if (auto_homing) {
                auto homing_axes = this->get_parameter("auto_homing_axes").as_integer_array();
                double homing_timeout = this->get_parameter("auto_homing_timeout").as_double();
                
                RCLCPP_INFO(this->get_logger(), "开始启动时自动回零");
                if (homeMultipleAxes(homing_axes, homing_timeout)) {
                    RCLCPP_INFO(this->get_logger(), "所有轴回零成功");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "部分轴回零失败");
                }
            }
            
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
    stopPublishing();
    disconnect();
}

void ZmcController::startPublishing() {
    // 创建定时器 (WallTimer)
    // 每 20 毫秒执行一次 timer_callback (50Hz)
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

    // 创建MotionStatus消息
    auto motion_status_msg = motion_msgs::msg::MotionStatus();
    
    // 设置JointState的Header
    auto& joint_state = motion_status_msg.joint_state;
    joint_state.header.stamp = this->now();
    joint_state.header.frame_id = "zmc_status";
    
    bool all_axes_success = true;
    float total_speed = 0.0f;
    int valid_axes_count = 0;
    
    // 读取所有轴的数据
    for (int axis : axes_) {
        float dpos_val = 0.0;
        float mpos_val = 0.0;
        float speed_val = 0.0;
        
        // 使用ZMotion SDK函数获取轴数据
        bool axis_success = true;
        
        // 获取命令位置 (DPOS)
        if (ZAux_Direct_GetDpos(handle_, axis, &dpos_val) != ERR_OK) {
            axis_success = false;
            RCLCPP_WARN(this->get_logger(), "无法读取轴 %d 的命令位置", axis);
        }
        
        // 获取实际位置 (MPOS)
        if (axis_success && ZAux_Direct_GetMpos(handle_, axis, &mpos_val) != ERR_OK) {
            axis_success = false;
            RCLCPP_WARN(this->get_logger(), "无法读取轴 %d 的实际位置", axis);
        }
        
        // 获取速度
        if (axis_success && ZAux_Direct_GetVpSpeed(handle_, axis, &speed_val) != ERR_OK) {
            axis_success = false;
            RCLCPP_WARN(this->get_logger(), "无法读取轴 %d 的速度", axis);
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
            RCLCPP_DEBUG(this->get_logger(), "轴 %d: MPOS=%.3f, DPOS=%.3f, 速度=%.3f", 
                        axis, mpos_val, dpos_val, speed_val);
        } else {
            // 如果读取失败，添加默认值
            joint_state.name.push_back("axis_" + std::to_string(axis));
            joint_state.position.push_back(0.0);
            joint_state.velocity.push_back(0.0);
            all_axes_success = false;
            RCLCPP_WARN(this->get_logger(), "轴 %d 数据读取失败，使用默认值", axis);
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
            // 注意：MotionStatus消息没有speed字段，这个计算可以用于内部使用或日志
            float composite_speed = std::sqrt(speed_axis0 * speed_axis0 + speed_axis1 * speed_axis1);
            RCLCPP_DEBUG(this->get_logger(), "合成速度计算: 轴0=%.3f, 轴1=%.3f, 合成=%.3f", 
                        speed_axis0, speed_axis1, composite_speed);
        } else {
            RCLCPP_WARN(this->get_logger(), "速度读取失败，无法计算合成速度");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "有效轴数量不足，无法计算合成速度");
    }
    
    // 发布MotionStatus消息
    if (!joint_state.name.empty()) {
        motion_status_pub_->publish(motion_status_msg);
        
        if (all_axes_success) {
            RCLCPP_DEBUG(this->get_logger(), "成功发布 %zu 个轴的状态数据到MotionStatus话题", 
                        joint_state.name.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "部分轴数据读取失败，成功发布 %zu 个轴的状态数据", 
                       joint_state.name.size());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "所有轴数据读取失败，无法发布MotionStatus消息");
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
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const motion_msgs::action::AxesMoving::Goal> goal) {
    
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
    
    if (goal->speed <= 0 || goal->acceleration <= 0 || goal->deceleration <= 0) {
        RCLCPP_ERROR(this->get_logger(), "速度或加速度参数无效，拒绝Action请求");
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
        for (int axis : axes_) {
            ZAux_Direct_Single_Cancel(handle_, axis, 0);
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
    
    RCLCPP_INFO(this->get_logger(), "开始异步执行轴移动Action");
    
    try {
        // 检查控制器连接状态
        if (!is_connected_) {
            throw std::runtime_error("控制器未连接");
        }
        
        // 设置运动参数
        for (size_t i = 0; i < goal->target_axes.size(); ++i) {
            int axis = goal->target_axes[i];
            
            // 设置轴速度
            if (!checkError(ZAux_Direct_SetSpeed(handle_, axis, goal->speed))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 速度失败");
            }
            
            // 设置加速度
            if (!checkError(ZAux_Direct_SetAccel(handle_, axis, goal->acceleration))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 加速度失败");
            }
            
            // 设置减速度
            if (!checkError(ZAux_Direct_SetDecel(handle_, axis, goal->deceleration))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 减速度失败");
            }
        }
        
        // 启动运动
        for (size_t i = 0; i < goal->target_axes.size(); ++i) {
            int axis = goal->target_axes[i];
            float target_position = goal->target_positions[i];
            
            if (!checkError(ZAux_Direct_Single_Move(handle_, axis, target_position))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 目标位置失败");
            }
            
            RCLCPP_INFO(this->get_logger(), "轴 %d 开始移动到位置 %.3f", axis, target_position);
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
                int axis = goal->target_axes[i];
                float target_position = goal->target_positions[i];
                
                float current_position = 0.0;
                float current_velocity = 0.0;
                
                // 读取当前位置和速度
                if (getMpos(axis, current_position) && getCurSpeed(axis, current_velocity)) {
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
                    RCLCPP_WARN(this->get_logger(), "无法读取轴 %d 的当前位置和速度", axis);
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

// 执行单轴移动
bool ZmcController::moveSingleAxis(int axis, float target_position, float speed, float acceleration, float deceleration) {
    if (!is_connected_) return false;
    
    // 设置运动参数
    if (!checkError(ZAux_Direct_SetSpeed(handle_, axis, speed)) ||
        !checkError(ZAux_Direct_SetAccel(handle_, axis, acceleration)) ||
        !checkError(ZAux_Direct_SetDecel(handle_, axis, deceleration)) ||
        !checkError(ZAux_Direct_Single_Move(handle_, axis, target_position))) {
        return false;
    }
    
    return true;
}

// 检查轴是否到达目标位置
bool ZmcController::isAxisAtPosition(int axis, float target_position, float tolerance) {
    if (!is_connected_) return false;
    
    float current_position = 0.0;
    if (!getMpos(axis, current_position)) {
        return false;
    }
    
    return std::abs(current_position - target_position) <= tolerance;
}

// ObjectPosition消息处理
void ZmcController::handleObjectPosition(const motion_msgs::msg::ObjectPosition::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "收到ObjectPosition消息，模式: %d, 轴数: %d, 坐标系: %d", 
               msg->mode, msg->axis_num, msg->plane_coord);
    
    // 检查消息有效性
    if (msg->pos.empty()) {
        RCLCPP_ERROR(this->get_logger(), "ObjectPosition消息中位置数据为空");
        return;
    }
    
    // 根据模式处理不同的运动类型
    switch (msg->mode) {
        case 0: // 单轴回零
            RCLCPP_INFO(this->get_logger(), "执行单轴回零，轴号: %d", msg->axis_num);
            // 这里可以实现回零逻辑
            break;
            
        case 1: // 单轴运动
            if (msg->pos.size() >= 1) {
                RCLCPP_INFO(this->get_logger(), "执行单轴运动，轴号: %d, 目标位置: %.3f", 
                           msg->axis_num, msg->pos[0]);
                // 单轴运动
                std::vector<float> target_positions(NUM_AXES, 0.0f);
                if (msg->axis_num < NUM_AXES) {
                    target_positions[msg->axis_num] = msg->pos[0];
                    moveToPositions(target_positions);
                }
            }
            break;
            
        case 2: // 双轴运动(X, Y)
            if (msg->pos.size() >= 2) {
                RCLCPP_INFO(this->get_logger(), "执行双轴运动，X: %.3f, Y: %.3f", 
                           msg->pos[0], msg->pos[1]);
                // 双轴运动 - 假设轴0为X轴，轴1为Y轴
                std::vector<float> target_positions(NUM_AXES, 0.0f);
                target_positions[0] = msg->pos[0]; // X轴
                target_positions[1] = msg->pos[1]; // Y轴
                moveToPositions(target_positions);
            }
            break;
            
        case 3: // 振镜运动
            RCLCPP_INFO(this->get_logger(), "执行振镜运动");
            // 这里可以实现振镜控制逻辑
            break;
            
        default:
            RCLCPP_WARN(this->get_logger(), "未知的运动模式: %d", msg->mode);
            break;
    }
}

// 执行多轴运动到目标位置
bool ZmcController::moveToPositions(const std::vector<float>& target_positions, float speed, 
                                   float acceleration, float deceleration) {
    if (!is_connected_) {
        RCLCPP_ERROR(this->get_logger(), "控制器未连接，无法执行运动");
        return false;
    }
    
    if (target_positions.size() != NUM_AXES) {
        RCLCPP_ERROR(this->get_logger(), "目标位置数量(%zu)与轴数量(%d)不匹配", 
                    target_positions.size(), NUM_AXES);
        return false;
    }
    
    // 检查是否有正在执行的Action
    if (action_running_) {
        RCLCPP_WARN(this->get_logger(), "有Action正在执行，等待完成后执行新运动");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "开始执行多轴运动");
    
    // 设置运动参数
    for (int i = 0; i < NUM_AXES; ++i) {
        int axis = AXES[i];
        
        // 设置运动参数
        if (!checkError(ZAux_Direct_SetSpeed(handle_, axis, speed)) ||
            !checkError(ZAux_Direct_SetAccel(handle_, axis, acceleration)) ||
            !checkError(ZAux_Direct_SetDecel(handle_, axis, deceleration))) {
            RCLCPP_ERROR(this->get_logger(), "设置轴 %d 运动参数失败", axis);
            return false;
        }
        
        // 执行绝对运动
        if (!checkError(ZAux_Direct_Single_MoveAbs(handle_, axis, target_positions[i]))) {
            RCLCPP_ERROR(this->get_logger(), "设置轴 %d 目标位置失败", axis);
            return false;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "轴 %d 开始移动到位置 %.3f", axis, target_positions[i]);
    }
    
    RCLCPP_INFO(this->get_logger(), "多轴运动已启动，正在监控执行进度...");
    
    // 启动监控线程（可选）
    std::thread([this, target_positions]() {
        this->monitorMultiAxisMotion(target_positions);
    }).detach();
    
    return true;
}

// 监控多轴运动进度（内部方法）
void ZmcController::monitorMultiAxisMotion(const std::vector<float>& target_positions) {
    auto start_time = std::chrono::steady_clock::now();
    bool all_axes_completed = false;
    
    while (!all_axes_completed) {
        all_axes_completed = true;
        int completed_axes = 0;
        
        for (int i = 0; i < NUM_AXES; ++i) {
            int axis = AXES[i];
            
            if (!isAxisAtPosition(axis, target_positions[i])) {
                all_axes_completed = false;
            } else {
                completed_axes++;
            }
        }
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        
        float progress = static_cast<float>(completed_axes) / NUM_AXES * 100.0f;
        
        RCLCPP_DEBUG(this->get_logger(), "多轴运动进度: %.1f%%, 耗时: %lds", progress, elapsed.count());
        
        if (!all_axes_completed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "多轴运动完成");
}

// 轴回零Action相关方法实现

// 处理轴回零Action目标请求
rclcpp_action::GoalResponse ZmcController::handleAxisHomingGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const motion_msgs::action::AxisHoming::Goal> goal) {
    
    RCLCPP_INFO(this->get_logger(), "收到轴回零Action请求");
    
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
    if (goal->axis_id < 0 || goal->axis_id > 5) {
        RCLCPP_ERROR(this->get_logger(), "轴号无效，拒绝Action请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (goal->velocity_high <= 0 || goal->velocity_low <= 0) {
        RCLCPP_ERROR(this->get_logger(), "速度参数无效，拒绝Action请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (goal->timeout <= 0) {
        RCLCPP_ERROR(this->get_logger(), "超时时间无效，拒绝Action请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    RCLCPP_INFO(this->get_logger(), "接受轴回零Action请求");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 处理轴回零Action取消请求
rclcpp_action::CancelResponse ZmcController::handleAxisHomingCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::AxisHoming>> goal_handle) {
    
    RCLCPP_INFO(this->get_logger(), "收到轴回零Action取消请求");
    
    if (action_running_ && current_homing_goal_handle_ == goal_handle) {
        // 停止轴的运动
        auto goal = goal_handle->get_goal();
        ZAux_Direct_Single_Cancel(handle_, goal->axis_id, 0);
        
        action_running_ = false;
        RCLCPP_INFO(this->get_logger(), "轴回零Action已取消");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    RCLCPP_WARN(this->get_logger(), "没有正在执行的轴回零Action可以取消");
    return rclcpp_action::CancelResponse::REJECT;
}

// 接受并执行轴回零Action
void ZmcController::handleAxisHomingAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::AxisHoming>> goal_handle) {
    
    RCLCPP_INFO(this->get_logger(), "轴回零Action目标被接受，启动异步执行");
    
    // 启动独立线程执行，避免阻塞ROS2主线程
    std::thread{std::bind(&ZmcController::executeAxisHoming, this, goal_handle)}.detach();
}

void ZmcController::executeAxisHoming(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::AxisHoming>> goal_handle) {
    
    // 设置执行状态
    action_running_ = true;
    current_homing_goal_handle_ = goal_handle;
    
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<motion_msgs::action::AxisHoming::Result>();
    auto feedback = std::make_shared<motion_msgs::action::AxisHoming::Feedback>();
    
    RCLCPP_INFO(this->get_logger(), "开始异步执行轴回零Action");
    
    try {
        // 检查控制器连接状态
        if (!is_connected_) {
            throw std::runtime_error("控制器未连接");
        }
        
        // 确定回零参数
        int axis = goal->axis_id;
        float velocity_high = goal->velocity_high;
        float velocity_low = goal->velocity_low;
        int homing_mode = goal->homing_mode;
        float timeout = goal->timeout;
        
        // 如果用户未指定回零模式，使用参数文件中的默认值
        if (homing_mode == 0 && axis < homing_modes_.size()) {
            homing_mode = static_cast<int>(homing_modes_[axis]);
            RCLCPP_INFO(this->get_logger(), "使用默认回零模式: %d", homing_mode);
        }
        
        // 如果用户未指定速度，使用参数文件中的默认值
        if (velocity_high <= 0 && axis < homing_velocities_high_.size()) {
            velocity_high = homing_velocities_high_[axis];
            RCLCPP_INFO(this->get_logger(), "使用默认回零高速: %.3f", velocity_high);
        }
        if (velocity_low <= 0 && axis < homing_velocities_low_.size()) {
            velocity_low = homing_velocities_low_[axis];
            RCLCPP_INFO(this->get_logger(), "使用默认回零低速: %.3f", velocity_low);
        }
        
        // 如果用户未指定超时时间，使用参数文件中的默认值
        if (timeout <= 0 && axis < homing_timeouts_.size()) {
            timeout = homing_timeouts_[axis];
            RCLCPP_INFO(this->get_logger(), "使用默认回零超时: %.1f秒", timeout);
        }
        
        // 确定回零蠕动速度
        float velocity_creep = 0.0;
        if (axis < homing_velocities_creep_.size()) {
            velocity_creep = homing_velocities_creep_[axis];
        }
        
        // 执行回零操作
        if (!homeSingleAxis(axis, velocity_high, velocity_low, velocity_creep, homing_mode)) {
            throw std::runtime_error("启动回零操作失败");
        }
        
        // RCLCPP_INFO(this->get_logger(), "轴 %d 开始回零，模式: %d, 高速: %.3f, 低速: %.3f, 超时: %.1f秒", 
                //    axis, homing_mode, velocity_high, velocity_low, timeout);
        
        // 监控回零过程
        bool homing_completed = false;
        bool homing_success = false;
        auto start_time = std::chrono::steady_clock::now();
        
        while (action_running_ && !homing_completed) {
            // 检查Action是否被取消
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Action被用户取消";
                result->final_pos = 0.0;
                result->error_code = 1;
                goal_handle->canceled(result);
                action_running_ = false;
                RCLCPP_INFO(this->get_logger(), "轴回零Action执行被取消");
                return;
            }
            
            // 检查是否超时
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
            auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            if (elapsed_sec.count() > timeout) {
                result->success = false;
                result->message = "回零操作超时";
                result->final_pos = 0.0;
                result->error_code = 2;
                goal_handle->abort(result);
                action_running_ = false;
                RCLCPP_ERROR(this->get_logger(), "轴 %d 回零操作超时", goal->axis_id);
                return;
            }
            
            // 读取回零状态（使用总线命令）
            uint32 homing_status = 0;
            bool home_status_ok = checkError(ZAux_BusCmd_GetHomeStatus(handle_, goal->axis_id, &homing_status));
            
            // 读取轴状态（物理停止状态）
            int32 idle_status = 0;
            bool idle_status_ok = checkError(ZAux_Direct_GetIfIdle(handle_, goal->axis_id, &idle_status));
            
            // 读取运动类型（逻辑任务状态）
            int32 mtype = -1;
            bool mtype_ok = checkError(ZAux_Direct_GetMtype(handle_, goal->axis_id, &mtype));
            
            // 判断回零完成条件
            if (home_status_ok && idle_status_ok && mtype_ok) {
                if (homing_status == 1 && idle_status == -1 && mtype == 0) {
                    // 回零成功完成：回零状态正常 + 物理停止 + 逻辑任务结束
                    homing_completed = true;
                    homing_success = true;
                    RCLCPP_DEBUG(this->get_logger(), "轴 %d 回零状态: 成功完成", goal->axis_id);
                } else if (homing_status == 0) {
                    // 回零还在进行中，继续等待
                    RCLCPP_DEBUG(this->get_logger(), "轴 %d 回零状态: 进行中 (home_status=%d, idle=%d, mtype=%d)", 
                                goal->axis_id, homing_status, idle_status, mtype);
                } else {
                    // 其他状态值，视为异常
                    homing_completed = true;
                    homing_success = false;
                    RCLCPP_WARN(this->get_logger(), "轴 %d 回零状态异常: home_status=%d, idle=%d, mtype=%d", 
                                goal->axis_id, homing_status, idle_status, mtype);
                }
            } else {
                // 状态获取失败，继续尝试
                RCLCPP_WARN(this->get_logger(), "轴 %d 获取状态失败: home=%d, idle=%d, mtype=%d", 
                            goal->axis_id, home_status_ok ? 1 : 0, idle_status_ok ? 1 : 0, mtype_ok ? 1 : 0);
            }
            
            // 读取当前位置和驱动器状态
            float current_pos = 0.0;
            int drive_status = 0;
            
            if (getMpos(goal->axis_id, current_pos)) {
                feedback->current_pos = current_pos;
            }
            
            if (getAxisStatus(goal->axis_id, drive_status)) {
                feedback->drive_status = drive_status;
            }
            
            // 计算已执行时间（带小数）
            feedback->elapsed_time = static_cast<float>(elapsed_ms.count()) / 1000.0f;
            
            // 更新反馈信息
            if (!homing_completed) {
                feedback->current_state = "SEARCHING";
            } else if (homing_success) {
                feedback->current_state = "COMPLETED";
            } else {
                feedback->current_state = "FAILED";
            }
            
            // 发布反馈
            goal_handle->publish_feedback(feedback);
            
            RCLCPP_DEBUG(this->get_logger(), "回零中... 轴 %d, 位置: %.3f, 状态: %s, 驱动器状态: %d, 耗时: %.2f秒", 
                        goal->axis_id, feedback->current_pos, feedback->current_state.c_str(), 
                        feedback->drive_status, feedback->elapsed_time);
            
            // 短暂休眠，避免过度占用CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (action_running_) {
            // 读取最终位置
            float final_pos = 0.0;
            getMpos(goal->axis_id, final_pos);
            
            if (homing_success) {
                result->success = true;
                result->message = "回零成功";
                result->final_pos = final_pos;
                result->error_code = 0;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "轴 %d 回零成功，最终位置: %.3f", goal->axis_id, final_pos);
            } else {
                result->success = false;
                result->message = "回零失败";
                result->final_pos = final_pos;
                result->error_code = 3;
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "轴 %d 回零失败", goal->axis_id);
            }
        }
        
    } catch (const std::exception& e) {
        result->success = false;
        result->message = "Action执行失败: " + std::string(e.what());
        result->final_pos = 0.0;
        result->error_code = 999;
        
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "轴回零Action执行失败: %s", e.what());
    }
    
    // 清理执行状态
    action_running_ = false;
    current_homing_goal_handle_.reset();
}

// 执行单轴回零操作
bool ZmcController::homeSingleAxis(int axis, float velocity_high, float velocity_low, float velocity_creep, int homing_mode) {
    if (!is_connected_) return false;
    
    // 设置回零蠕动速度
    if (velocity_creep > 0) {
        if (!checkError(ZAux_Direct_SetCreep(handle_, axis, velocity_creep))) {
            RCLCPP_WARN(this->get_logger(), "设置轴 %d 回零蠕动速度失败", axis);
            // 继续执行，不因为蠕动速度设置失败而终止回零
        } else {
            RCLCPP_INFO(this->get_logger(), "轴 %d 回零蠕动速度设置为 %.3f", axis, velocity_creep);
        }
    }
    
    // 执行回零操作（使用总线命令）
    if (!checkError(ZAux_BusCmd_Datum(handle_, axis, homing_mode))) {
        return false;
    }
    
    return true;
}

// 执行多轴回零操作
bool ZmcController::homeMultipleAxes(const std::vector<long int>& axes, double timeout) {
    if (!is_connected_) return false;
    
    RCLCPP_INFO(this->get_logger(), "开始执行多轴回零");
    
    // 存储每个轴的回零状态
    struct AxisHomingStatus {
        bool completed = false;
        bool success = false;
    };
    std::map<int, AxisHomingStatus> axis_status;
    
    // 启动所有轴的回零操作
    for (int axis : axes) {
        // 获取默认回零参数
        int homing_mode = 11;
        float velocity_high = 50.0;
        float velocity_low = 10.0;
        float velocity_creep = 5.0;
        
        if (axis < homing_modes_.size()) {
            homing_mode = static_cast<int>(homing_modes_[axis]);
        }
        if (axis < homing_velocities_high_.size()) {
            velocity_high = homing_velocities_high_[axis];
        }
        if (axis < homing_velocities_low_.size()) {
            velocity_low = homing_velocities_low_[axis];
        }
        if (axis < homing_velocities_creep_.size()) {
            velocity_creep = homing_velocities_creep_[axis];
        }
        
        // 启动回零
        if (!homeSingleAxis(axis, velocity_high, velocity_low, velocity_creep, homing_mode)) {
            RCLCPP_ERROR(this->get_logger(), "轴 %d 启动回零失败", axis);
            return false;
        }
        
        axis_status[axis] = AxisHomingStatus();
    }
    
    // 等待所有轴回零完成
    auto start_time = std::chrono::steady_clock::now();
    bool all_completed = false;
    
    while (!all_completed) {
        all_completed = true;
        
        for (auto& [axis, status] : axis_status) {
            if (!status.completed) {
                // 读取回零状态
                uint32 homing_status = 0;
                if (checkError(ZAux_BusCmd_GetHomeStatus(handle_, axis, &homing_status))) {
                    if (homing_status == 1) {
                        status.completed = true;
                        status.success = true;
                        RCLCPP_INFO(this->get_logger(), "轴 %d 回零成功", axis);
                    } else if (homing_status == 0) {
                        status.completed = true;
                        status.success = false;
                        RCLCPP_ERROR(this->get_logger(), "轴 %d 回零失败", axis);
                    }
                }
                
                if (!status.completed) {
                    all_completed = false;
                }
            }
        }
        
        // 检查超时
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        if (elapsed.count() > timeout) {
            RCLCPP_ERROR(this->get_logger(), "多轴回零超时");
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 检查是否所有轴都回零成功
    bool all_success = true;
    for (const auto& [axis, status] : axis_status) {
        if (!status.success) {
            all_success = false;
        }
    }
    
    return all_success;
}

// 初始化轴参数
void ZmcController::initializeAxisParameters() {
    // 读取轴参数
    auto pulse_equivalent = this->get_parameter("axis_pulse_equivalent").as_double_array();
    auto max_speed = this->get_parameter("axis_max_speed").as_double_array();
    auto acceleration = this->get_parameter("axis_acceleration").as_double_array();
    auto deceleration = this->get_parameter("axis_deceleration").as_double_array();
    
    // 读取回零参数并存储为成员变量
    homing_modes_ = this->get_parameter("axis_homing_mode").as_integer_array();
    homing_velocities_high_ = this->get_parameter("axis_homing_velocity_high").as_double_array();
    homing_velocities_low_ = this->get_parameter("axis_homing_velocity_low").as_double_array();
    homing_velocities_creep_ = this->get_parameter("axis_homing_velocity_creep").as_double_array();
    homing_timeouts_ = this->get_parameter("axis_homing_timeout").as_double_array();
    
    RCLCPP_INFO(this->get_logger(), "回零参数初始化完成");
    for (size_t i = 0; i < homing_modes_.size(); ++i) {
        // RCLCPP_INFO(this->get_logger(), "轴 %d: 回零模式=%ld, 高速=%.3f, 低速=%.3f, 蠕动速度=%.3f, 超时=%.1f秒", 
        //            i, homing_modes_[i], 
        //            i < homing_velocities_high_.size() ? homing_velocities_high_[i] : 0.0, 
        //            i < homing_velocities_low_.size() ? homing_velocities_low_[i] : 0.0, 
        //            i < homing_velocities_creep_.size() ? homing_velocities_creep_[i] : 0.0, 
        //            i < homing_timeouts_.size() ? homing_timeouts_[i] : 0.0);
    }
    
    RCLCPP_INFO(this->get_logger(), "开始初始化轴参数");
    
    // 设置轴参数
        for (size_t i = 0; i < axes_.size(); ++i) {
            int axis = axes_[i];
            
            // 设置脉冲当量
            if (i < pulse_equivalent.size()) {
                if (checkError(ZAux_Direct_SetUnits(handle_, axis, pulse_equivalent[i]))) {
                    // RCLCPP_INFO(this->get_logger(), "轴 %d: 脉冲当量设置为 %.3f", axis, pulse_equivalent[i]);
                }
            }
            
            // 设置速度
            if (i < max_speed.size()) {
                if (checkError(ZAux_Direct_SetSpeed(handle_, axis, max_speed[i]))) {
                    // RCLCPP_INFO(this->get_logger(), "轴 %d: 最大速度设置为 %.3f", axis, max_speed[i]);
                }
            }
            
            // 设置加速度
            if (i < acceleration.size()) {
                if (checkError(ZAux_Direct_SetAccel(handle_, axis, acceleration[i]))) {
                    // RCLCPP_INFO(this->get_logger(), "轴 %d: 加速度设置为 %.3f", axis, acceleration[i]);
                }
            }
            
            // 设置减速度
            if (i < deceleration.size()) {
                if (checkError(ZAux_Direct_SetDecel(handle_, axis, deceleration[i]))) {
                    // RCLCPP_INFO(this->get_logger(), "轴 %d: 减速度设置为 %.3f", axis, deceleration[i]);
                }
            }
        }
    
    RCLCPP_INFO(this->get_logger(), "轴参数初始化完成");
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