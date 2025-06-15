#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <string>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node {
public:
  MinimalParam() : Node("minimap_param_node") {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is mine!";
    this->declare_parameter("my_parameter", "world", param_desc);
    timer_ = this->create_wall_timer(1s, std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback() {
    std::string my_param = this->get_parameter("my_parameter").as_string();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    // std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameter(rclcpp::Parameter("my_parameter", "world"));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}
