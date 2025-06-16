#include <chrono>
#include <functional>
#include <memory>
#include <math.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class DynamicFrameBroadcaster : public rclcpp::Node {
public:
  DynamicFrameBroadcaster() : Node("dynamic_frame_tf2_broadcaster") {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(100ms, std::bind(&DynamicFrameBroadcaster::broadcaster_timer_callback, this));
  }

private:
  void broadcaster_timer_callback() {
    geometry_msgs::msg::TransformStamped t;

    auto now = this->get_clock()->now();
    t.header.stamp = now;
    t.header.frame_id = "turtle1";
    t.child_frame_id = "carrot1";

    double x = now.seconds() / 10 * M_PI;

    t.transform.translation.x = 5 * sin(x);
    t.transform.translation.y = 5 * cos(x);
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
  rclcpp::shutdown();
  
  return 0;
}
