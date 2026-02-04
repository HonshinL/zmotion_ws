#ifndef ZMC_CONTROLLER_H
#define ZMC_CONTROLLER_H

#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "zmotion_driver/zmcaux.h"

using namespace std::chrono_literals;

class ZmcController : public rclcpp::Node {
public:
    ZmcController();
    ZmcController(const std::string& node_name);
    ~ZmcController();

    /**
     * @brief 通过以太网连接到ZMC控制器
     * @param ip 控制器IP地址
     * @return true 连接成功，false 连接失败
     */
    bool connect(const std::string& ip);

    /**
     * @brief 断开与控制器的连接
     */
    void disconnect();

    /**
     * @brief 检查控制器是否已连接
     * @return true 已连接，false 未连接
     */
    bool isConnected() const;

    /**
     * @brief 获取控制器句柄
     * @return 控制器句柄
     */
    ZMC_HANDLE getHandle() const;

    /**
     * @brief 初始化ROS2相关功能（参数、发布者、定时器）
     */
    void initROS();

    /**
     * @brief 启动数据发布循环
     */
    void startPublishing();

    /**
     * @brief 停止数据发布循环
     */
    void stopPublishing();

    // 位置相关方法
    /**
     * @brief 获取指定轴的命令位置
     * @param axis 轴号
     * @param position 返回的命令位置
     * @return true 成功，false 失败
     */
    bool getDpos(int axis, float& position);

    /**
     * @brief 设置指定轴的命令位置
     * @param axis 轴号
     * @param position 命令位置
     * @return true 成功，false 失败
     */
    bool setDpos(int axis, float position);

    /**
     * @brief 获取指定轴的反馈位置
     * @param axis 轴号
     * @param position 返回的反馈位置
     * @return true 成功，false 失败
     */
    bool getMpos(int axis, float& position);

    // 速度相关方法
    /**
     * @brief 获取指定轴的当前速度
     * @param axis 轴号
     * @param speed 返回的当前速度
     * @return true 成功，false 失败
     */
    bool getCurSpeed(int axis, float& speed);

    /**
     * @brief 获取指定轴的加速度
     * @param axis 轴号
     * @param accel 返回的加速度
     * @return true 成功，false 失败
     */
    bool getAccel(int axis, float& accel);

    /**
     * @brief 设置指定轴的加速度
     * @param axis 轴号
     * @param accel 加速度
     * @return true 成功，false 失败
     */
    bool setAccel(int axis, float accel);

    /**
     * @brief 获取指定轴的减速度
     * @param axis 轴号
     * @param decel 返回的减速度
     * @return true 成功，false 失败
     */
    bool getDecel(int axis, float& decel);

    /**
     * @brief 设置指定轴的减速度
     * @param axis 轴号
     * @param decel 减速度
     * @return true 成功，false 失败
     */
    bool setDecel(int axis, float decel);

    // IO相关方法
    /**
     * @brief 读取输入信号
     * @param ionum 输入口编号
     * @param value 返回的输入口状态
     * @return true 成功，false 失败
     */
    bool getInput(int ionum, uint32& value);

    /**
     * @brief 设置输出信号
     * @param ionum 输出口编号
     * @param value 输出口状态
     * @return true 成功，false 失败
     */
    bool setOutput(int ionum, uint32 value);

    /**
     * @brief 读取输出信号
     * @param ionum 输出口编号
     * @param value 返回的输出口状态
     * @return true 成功，false 失败
     */
    bool getOutput(int ionum, uint32& value);

    // 轴状态相关方法
    /**
     * @brief 获取指定轴的状态
     * @param axis 轴号
     * @param status 返回的轴状态
     * @return true 成功，false 失败
     */
    bool getAxisStatus(int axis, int& status);

    /**
     * @brief 获取指定轴的使能状态
     * @param axis 轴号
     * @param enabled 返回的使能状态
     * @return true 成功，false 失败
     */
    bool getAxisEnable(int axis, int& enabled);

    /**
     * @brief 设置指定轴的使能状态
     * @param axis 轴号
     * @param enabled 使能状态
     * @return true 成功，false 失败
     */
    bool setAxisEnable(int axis, int enabled);

private:
    /**
     * @brief 定时器回调函数，定期读取并发布数据
     */
    void timer_callback();

    /**
     * @brief 检查错误码并返回结果
     * @param error_code ZMC API返回的错误码
     * @return true 成功，false 失败
     */
    bool checkError(int32 error_code) const;

    // ZMC控制器相关成员
    ZMC_HANDLE handle_;      ///< 控制器句柄
    bool is_connected_;      ///< 连接状态
    std::string controller_ip_;  ///< 控制器IP地址

    // ROS2相关成员
    int axis_;  ///< 监控的轴号
    rclcpp::TimerBase::SharedPtr timer_;  ///< 定时器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dpos_pub_;  ///< 命令位置发布者
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mpos_pub_;  ///< 反馈位置发布者
};

#endif // ZMC_CONTROLLER_H