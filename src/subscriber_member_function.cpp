// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <zed_interfaces/msg/objects_stamped.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
      "/zed2/zed_node/obj_det/objects", 1000, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(zed_interfaces::msg::ObjectsStamped::ConstSharedPtr msg) 
  {
    for (auto obj : msg->objects){
      float x = obj.position[0];
      float y = obj.position[1];
      float z = obj.position[2];
      RCLCPP_INFO(this->get_logger(), "detect position: '%f' '%f' '%f' " + obj.label, x, y, z);
    }
  }
  rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
