#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

class PIDControllerNode : public rclcpp::Node
{
public:
    PIDControllerNode();

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void compute_and_publish_command();
    void declare_and_get_params();

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // PID parameters
    double kp_, ki_, kd_;
    std::vector<double> target_positions_;
    std::vector<double> prev_errors_;
    std::vector<double> integrals_;

    // Latest joint states
    std::vector<double> current_positions_;
    bool state_received_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  // 기타 멤버 변수...
};

#endif
