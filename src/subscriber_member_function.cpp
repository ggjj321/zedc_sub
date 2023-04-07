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
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("transfrom_zed2_3dod_to_2d", 1000);
    
    subscription_ = this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
      "/zed2/zed_node/obj_det/objects", 1000, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(zed_interfaces::msg::ObjectsStamped::ConstSharedPtr msg) 
  {
    for (auto obj : msg->objects){
      float x = obj.position[0];
      float y = obj.position[1];

      int coordinateX = int(250 + y * 100);
      int coordinateY = int(50 + x * 150);

      if (coordinateX < 0){
        coordinateX = 0;
      }
      else if (coordinateX > 500){
        coordinateX = 500;
      }

      if (coordinateY < 0){
        coordinateY = 0;
      }
      else if (coordinateY > 500){
        coordinateY = 500;
      }

      auto message = std_msgs::msg::String();
      message.data = "2d: " + std::to_string(coordinateX) + " " + std::to_string(coordinateY) + " ";
      message.data += "label: " + obj.label + " ";
      message.data += "label_id: " + std::to_string(obj.label_id) + " ";
      RCLCPP_INFO(this->get_logger(), message.data);
      publisher_->publish(message);
    }
  }
  rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
