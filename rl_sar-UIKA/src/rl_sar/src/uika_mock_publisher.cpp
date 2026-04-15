/*
 * ============================================================
 * UIKA 模拟数据发布器 (Mock Publisher)
 * ============================================================
 *
 * 【用途】
 *   用于测试RL节点，在没有真实硬件时模拟传感器数据
 *
 * 【发布话题】
 *
 *   /uika/joint_state (std_msgs/Float64MultiArray)
 *     - 24个float: [q0,q1,...,q11, dq0,dq1,...,dq11]
 *     - 前12个: 关节位置 (rad)
 *     - 后12个: 关节速度 (rad/s)
 *
 *   /uika/imu (std_msgs/Float64MultiArray)
 *     - 7个float: [qw, qx, qy, qz, gx, gy, gz]
 *     - 前4个: 四元数姿态 (w,x,y,z)
 *     - 后3个: 陀螺仪角速度 (rad/s)
 *
 *   /uika/commands (std_msgs/Float64MultiArray)
 *     - 3个float: [vx, vy, wz]
 *     - vx: 前进/后退速度 (m/s)
 *     - vy: 左/右横移速度 (m/s)
 *     - wz: 偏航角速度 (rad/s)
 *
 * 【数据内容】
 *
 *   关节位置:
 *     - 基准位置: 默认站立姿态 [-0.70, 0.20, 0.70, ...]
 *     - 叠加小幅正弦波动: 0.05 * sin(t * (i+1))
 *     - 模拟机器人轻微晃动
 *
 *   关节速度:
 *     - 小幅余弦波动: 0.1 * cos(t * (i+1))
 *     - 速度值较小，模拟静止/微动状态
 *
 *   IMU姿态:
 *     - 四元数: [0, 0, 0, 1] 表示无旋转
 *     - 陀螺仪: [0, 0, 0] 表示无角速度
 *
 *   遥控命令:
 *     - 前进速度: 0.5 m/s
 *     - 无侧向移动
 *     - 无旋转
 *
 * 【发布频率】
 *   200Hz (5ms间隔)
 *
 * 【使用场景】
 *   - 开发测试
 *   - 无硬件时验证ROS2接口
 *   - 验证ONNX模型推理流程
 *
 * ============================================================
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <cmath>
#include <memory>

/**
 * @brief 模拟UIKA传感器数据发布器
 *
 * 定时以200Hz频率发布模拟的关节状态和IMU数据
 */
class MockPublisher : public rclcpp::Node
{
public:
    MockPublisher()
        : Node("uika_mock_publisher"),
          count_(0),
          t_(0.0)
    {
        // ============================================================
        // 1. 创建发布器
        // ============================================================

        // 关节状态发布器: /uika/joint_state
        // 数据: [q(12), dq(12)] = 24个float
        joint_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/uika/joint_state", 10);

        // IMU发布器: /uika/imu
        // 数据: [quat(4), gyro(3)] = 7个float
        imu_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/uika/imu", 10);

        // 遥控命令发布器: /uika/commands
        // 数据: [vx, vy, wz] = 3个float
        commands_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/uika/commands", 10);

        // ============================================================
        // 2. 初始化默认关节位置（站立姿态）
        // ============================================================
        // 对应12个关节:
        //   FR: hip, thigh, calf (右前)
        //   FL: hip, thigh, calf (左前)
        //   RR: hip, thigh, calf (右后)
        //   RL: hip, thigh, calf (左后)
        default_pos_ = {
            -0.70,  0.20,  0.70,   // FR: hip, thigh, calf
             0.70, -0.20, -0.70,   // FL: hip, thigh, calf (镜像)
             0.70,  0.20,  0.70,   // RR: hip, thigh, calf
            -0.70, -0.20, -0.70    // RL: hip, thigh, calf (镜像)
        };

        // ============================================================
        // 3. 创建定时器 (200Hz = 5ms)
        // ============================================================
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),  // 5ms间隔 = 200Hz
            std::bind(&MockPublisher::TimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Mock Publisher started");
        RCLCPP_INFO(this->get_logger(), "  Publishing /uika/joint_state at 200Hz");
        RCLCPP_INFO(this->get_logger(), "  Publishing /uika/imu at 200Hz");
        RCLCPP_INFO(this->get_logger(), "  Publishing /uika/commands at 200Hz");
        RCLCPP_INFO(this->get_logger(), "  Commands: vx=%.2f, vy=%.2f, wz=%.2f",
                    vx_, vy_, wz_);
        RCLCPP_INFO(this->get_logger(), "  Default joint positions:");
        RCLCPP_INFO(this->get_logger(), "    FR: hip=%.2f, thigh=%.2f, calf=%.2f",
                    default_pos_[0], default_pos_[1], default_pos_[2]);
        RCLCPP_INFO(this->get_logger(), "    FL: hip=%.2f, thigh=%.2f, calf=%.2f",
                    default_pos_[3], default_pos_[4], default_pos_[5]);
    }

private:
    /**
     * @brief 定时器回调 - 发布模拟数据
     *
     * 每5ms调用一次，发布当前时刻的关节状态和IMU数据
     */
    void TimerCallback()
    {
        count_++;
        t_ = count_ * 0.005;  // 时间 (s)

        // ========================================================
        // 发布关节状态 /uika/joint_state
        // ========================================================
        std_msgs::msg::Float64MultiArray joint_msg;
        joint_msg.data.resize(24);  // 12位置 + 12速度

        for (int i = 0; i < 12; ++i)
        {
            // 位置: 默认位置 + 小幅正弦波动
            // 波动幅度 0.05rad ≈ 3度
            // 每个关节的波动频率不同 (i+1)
            joint_msg.data[i] = default_pos_[i] + 0.05 * std::sin(t_ * (i + 1));

            // 速度: 小幅余弦波动
            // 波动幅度 0.1 rad/s
            joint_msg.data[i + 12] = 0.1 * std::cos(t_ * (i + 1));
        }
        joint_state_pub_->publish(joint_msg);

        // ========================================================
        // 发布IMU数据 /uika/imu
        // ========================================================
        std_msgs::msg::Float64MultiArray imu_msg;
        imu_msg.data.resize(7);  // 4四元数 + 3陀螺仪

        // 四元数: [0, 0, 0, 1] 表示无旋转（水平姿态）
        // 这是单位四元数，表示从自身坐标系到世界坐标系无旋转
        imu_msg.data[0] = 0.0f;   // qx
        imu_msg.data[1] = 0.0f;   // qy
        imu_msg.data[2] = 0.0f;   // qz
        imu_msg.data[3] = 1.0f;   // qw

        // 陀螺仪: 模拟z轴轻微旋转
        // 模拟机器人有轻微的yaw晃动
        imu_msg.data[4] = 0.0f;                      // gx: 无roll
        imu_msg.data[5] = 0.0f;                      // gy: 无pitch
        imu_msg.data[6] = 0.05 * std::sin(t_);       // gz: 轻微yaw晃动

        imu_pub_->publish(imu_msg);

        // ========================================================
        // 发布遥控命令 /uika/commands
        // ========================================================
        std_msgs::msg::Float64MultiArray commands_msg;
        commands_msg.data.resize(3);  // vx, vy, wz
        commands_msg.data[0] = vx_;   // 前进速度
        commands_msg.data[1] = vy_;   // 侧向速度
        commands_msg.data[2] = wz_;   // 偏航角速度

        commands_pub_->publish(commands_msg);

        // 每秒打印一次状态
        if (count_ % 200 == 0)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Mock data: q[0]=%.3f, dq[0]=%.3f, gyro_z=%.3f",
                joint_msg.data[0],      // FR hip位置
                joint_msg.data[12],     // FR hip速度
                imu_msg.data[6]);       // z轴角速度
        }
    }

    // 发布器
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commands_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 默认站立姿态
    std::vector<double> default_pos_;

    // 计数器和时间
    int count_;
    double t_;

    // 遥控命令参数
    double vx_ = 0.5;   // 前进速度 (m/s)
    double vy_ = 0.0;   // 侧向速度 (m/s)
    double wz_ = 0.0;   // 偏航角速度 (rad/s)
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MockPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
