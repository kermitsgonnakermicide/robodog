#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include "leg_walker/ik_solver.hpp"

class WalkNode : public rclcpp::Node {
public:
  WalkNode() : Node("walk_node") {
    declare_parameter("L1", 0.1);
    declare_parameter("L2", 0.1);
    get_parameter("L1", L1_);
    get_parameter("L2", L2_);

    solver_ = std::make_shared<IKSolver>(L1_, L2_);
    pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    sub_ = create_subscription<geometry_msgs::msg::Point>(
      "target_point", 10,
      std::bind(&WalkNode::targetCallback, this, std::placeholders::_1)
    );
  }

private:
  void targetCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = {
      "leg1_rot","leg1_up","leg1_low",
      "leg2_rot","leg2_up","leg2_low",
      "leg3_rot","leg3_up","leg3_low",
      "leg4_rot","leg4_up","leg4_low"
    };
    js.position.resize(12);

    for (int i = 0; i < 4; ++i) {
      double x = msg->x;
      double y = msg->y + ((i < 2) ? 0.1 : -0.1);  // front/back offset
      double z = -0.2;

      Eigen::Vector3d foot_pos(x, y, z);
      Eigen::Vector3d angles;

      if (solver_->solve(foot_pos, angles)) {
        for (int j = 0; j < 3; ++j)
          js.position[i * 3 + j] = angles[j];
      } else {
        RCLCPP_WARN(this->get_logger(), "IK failed for leg %d", i+1);
      }
    }

    pub_->publish(js);
  }

  double L1_, L2_;
  std::shared_ptr<IKSolver> solver_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkNode>());
  rclcpp::shutdown();
  return 0;
}
