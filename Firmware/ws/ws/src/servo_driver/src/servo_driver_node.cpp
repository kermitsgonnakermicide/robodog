#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "servo_driver/pwm_controller.hpp"

PCA9685 driver0(0x40);
PCA9685 driver1(0x41);

std::map<std::string, std::pair<int, int>> joint_map = {
  {"leg1_rot", {0x40, 0}}, {"leg1_up", {0x40, 1}}, {"leg1_low", {0x40, 2}},
  {"leg2_rot", {0x40, 3}}, {"leg2_up", {0x40, 4}}, {"leg2_low", {0x40, 5}},
  {"leg3_rot", {0x41, 0}}, {"leg3_up", {0x41, 1}}, {"leg3_low", {0x41, 2}},
  {"leg4_rot", {0x41, 3}}, {"leg4_up", {0x41, 4}}, {"leg4_low", {0x41, 5}}
};

class ServoDriver : public rclcpp::Node {
public:
  ServoDriver() : Node("servo_driver") {
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&ServoDriver::callback, this, std::placeholders::_1)
    );
    driver0.setPWMFreq(50);
    driver1.setPWMFreq(50);
  }

  void callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); i++) {
      auto name = msg->name[i];
      float angle = msg->position[i];
      if (!joint_map.count(name)) continue;

      auto [addr, ch] = joint_map[name];
      if (addr == 0x40)
        driver0.setAngle(ch, angle);
      else
        driver1.setAngle(ch, angle);
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoDriver>());
  rclcpp::shutdown();
  return 0;
}
