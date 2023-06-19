#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>
#include "tf2_ros/static_transform_broadcaster.h"

#include <memory>
#include <string>

using std::placeholders::_1;

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("pub_tf")
  {
    // Initialize the transform broadcaster
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    pub_tf();
    timer_ = this-> create_wall_timer(std::chrono::milliseconds(10),std::bind(&FramePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    pub_tf();
  }

  void pub_tf()
  {
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = now;
    t.header.frame_id = "base_link";
    t.child_frame_id = "laser_link";

    t.transform.translation.x = 0.1;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.05;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    // Send the transformation
    tf_static_broadcaster_->sendTransform(t);
  }
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //rclcpp::Rate loop_rate(10);
  while (rclcpp::ok())
  {
    rclcpp::spin(std::make_shared<FramePublisher>());
    //loop_rate.sleep();

  }
  
  rclcpp::shutdown();
  return 0;
}