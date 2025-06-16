#include <memory>
#include <functional>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

class FrameListener : public rclcpp::Node {
public:
  FrameListener(char* transformation[]) : Node("turtle_tf2_broadcaster") {
    // Declare and acquire `turtlename` parameter
    turtlename_ = this->declare_parameter("turtlename", "trutle");

    // Init the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    std::ostringstream stream;
    stream << "/" << turtlename_.c_str() << "/pose";
    std::string topic_name = stream.str();

    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      topic_name, 10,
      std::bind(&FrameListener::handle_turtle_pose, this, std::placeholders::_1)
    );
  }

private:
  void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg) {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtlename_.c_str();

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  std::string turtlename_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>(argv));
  rclcpp::shutdown();

  return 0;
}
