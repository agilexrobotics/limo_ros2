#include "limo_base/limo_messenger.h"
#include "limo_base/limo_params.h"

#include <ugv_sdk/limo_base.h>


#include <rclcpp/rclcpp.hpp>

using namespace agx;
using namespace westonrobot;

std::shared_ptr<LimoBase> robot;

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("limo_base");

  RCLCPP_INFO(node->get_logger(), "limo_node start...");

  robot = std::make_shared<LimoBase>();
  LimoROSMessenger messenger(robot.get(), node);

  std::string port_name;
  rclcpp::Parameter parameter;
  node->get_parameter("port_name", parameter);
  port_name = parameter.as_string();
  node->get_parameter("odom_frame", parameter);
  messenger.odom_frame_ = parameter.as_string();
  node->get_parameter("base_frame", parameter);
  messenger.base_frame_=parameter.as_string();
  node->get_parameter("odom_topic_name", parameter);
  messenger.odom_topic_name_=parameter.as_string();

  messenger.SetupSubscription();

  // connect to the serial port
  if(port_name.find("tty") != port_name.npos){
    port_name= "/dev/" + port_name;
    robot->Connect(port_name,460800);
    robot->SetBaudRate(460800);
    robot->EnableCommandedMode();
    RCLCPP_INFO(node->get_logger(), "open the serial port: %s", port_name.c_str());
  }

  rclcpp::Rate rate(50);
  while (rclcpp::ok()) {
    rclcpp::spin(node);
    messenger.PublishStateToROS();
    // robot->SetMotionCommand(0.1,0.38);
    rate.sleep();
  }

  return 0;
}
