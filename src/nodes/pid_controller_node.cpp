#include "mujoco_ros2/pid_controller.hpp"

PIDControllerNode::PIDControllerNode()
: Node("pid_controller_node"),
  state_received_(false)
{
    declare_and_get_params();

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_state", 10,
        std::bind(&PIDControllerNode::joint_state_callback, this, std::placeholders::_1)
    );

    joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/joint_commands", 10
    );

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PIDControllerNode::compute_and_publish_command, this)
    );
    /**
     파라미터 변경 감지 콜백 (수정됨)
    this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            for (const auto &p : params) {
                if (p.get_name() == "kp") kp_ = p.as_double();
                else if (p.get_name() == "ki") ki_ = p.as_double();
                else if (p.get_name() == "kd") kd_ = p.as_double();
                else if (p.get_name() == "target_positions") target_positions_ = p.as_double_array();
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "";
            return result;
        }
    );
  */
      this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            for (const auto &p : params) {
                if (p.get_name() == "kp") kp_ = p.as_double();
                else if (p.get_name() == "ki") ki_ = p.as_double();
                else if (p.get_name() == "kd") kd_ = p.as_double();
                else if (p.get_name() == "target_positions") target_positions_ = p.as_double_array();
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "";
            return result;
        }
    );
}
}

void PIDControllerNode::declare_and_get_params()
{
    this->declare_parameter<double>("kp", 50.0);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 2.0);
    this->declare_parameter<std::vector<double>>("target_positions", {1.0});

    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    target_positions_ = this->get_parameter("target_positions").as_double_array();

    prev_errors_.resize(target_positions_.size(), 0.0);
    integrals_.resize(target_positions_.size(), 0.0);
}

void PIDControllerNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    current_positions_ = msg->position;
    if (!state_received_) {
        prev_errors_.assign(msg->position.size(), 0.0);
        integrals_.assign(msg->position.size(), 0.0);
        state_received_ = true;
    }
}

void PIDControllerNode::compute_and_publish_command()
{
    if (!state_received_) return;

    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data.resize(target_positions_.size());

    double dt = 0.01; // 10ms

    for (size_t i = 0; i < target_positions_.size(); ++i) {
        double error = target_positions_[i] - current_positions_[i];
        integrals_[i] += error * dt;
        double derivative = (error - prev_errors_[i]) / dt;

        cmd_msg.data[i] = kp_ * error + ki_ * integrals_[i] + kd_ * derivative;
        prev_errors_[i] = error;
    }

    joint_command_pub_->publish(cmd_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
