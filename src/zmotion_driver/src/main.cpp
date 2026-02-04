#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp" // 简单起见使用 Float64，实际项目常用 JointState
#include "zmotion_driver/zmc_controller.h"

using namespace std::chrono_literals;

class ZmcStatusPublisher : public rclcpp::Node {
public:
    ZmcStatusPublisher() : Node("zmc_status_publisher") {
        // 1. 声明并获取参数
        std::string ip = this->declare_parameter<std::string>("controller_ip", "192.168.0.11");
        axis_ = this->declare_parameter<int>("monitoring_axis", 0);

        // 2. 连接控制器
        if (controller_.connect(ip)) {
            RCLCPP_INFO(this->get_logger(), "已连接控制器: %s, 正在监控轴 %d", ip.c_str(), axis_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "无法连接控制器!");
        }

        // 3. 创建发布者 (Publisher)
        // 发布指令位置 (Command Position)
        dpos_pub_ = this->create_publisher<std_msgs::msg::Float64>("zmc/dpos", 10);
        // 发布反馈位置 (Actual Position)
        mpos_pub_ = this->create_publisher<std_msgs::msg::Float64>("zmc/mpos", 10);

        // 4. 创建定时器 (WallTimer)
        // 每 20 毫秒执行一次 timer_callback (50Hz)
        timer_ = this->create_wall_timer(20ms, std::bind(&ZmcStatusPublisher::timer_callback, this));
    }

    ~ZmcStatusPublisher() {
        // ZmcController 的析构函数会自动断开连接
    }

private:
    void timer_callback() {
        if (!controller_.isConnected()) return;

        float dpos_val = 0.0;
        float mpos_val = 0.0;

        // 从硬件读取数据
        if (controller_.getDpos(axis_, dpos_val) && controller_.getMpos(axis_, mpos_val)) {
            // 封装并发布 DPOS
            auto dpos_msg = std_msgs::msg::Float64();
            dpos_msg.data = dpos_val;
            dpos_pub_->publish(dpos_msg);

            // 封装并发布 MPOS
            auto mpos_msg = std_msgs::msg::Float64();
            mpos_msg.data = mpos_val;
            mpos_pub_->publish(mpos_msg);

            // (可选) 调试打印，实际运行时建议注释掉以节省 CPU
            // RCLCPP_DEBUG(this->get_logger(), "Published DPOS: %.3f, MPOS: %.3f", dpos_val, mpos_val);
        } else {
            RCLCPP_WARN(this->get_logger(), "无法读取轴 %d 的位置数据", axis_);
        }
    }

    // 成员变量
    ZmcController controller_;
    int axis_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dpos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mpos_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZmcStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}