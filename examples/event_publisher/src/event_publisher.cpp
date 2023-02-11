// Copyright 2023 Graph Power Solutions LLC.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

const std::string TOPIC_NAME = "EVENTS_TOPIC";
const int MAX_OBJECT_ID = 25;
const int MAX_OBJECT_COUNT = 4;
const std::chrono::microseconds TIMER = 300ms;

class EventPublisher : public rclcpp::Node
{
public:
  EventPublisher()
      : Node("event_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(TOPIC_NAME, 10);

    std::random_device rd;
    std::mt19937 gen(rd());
    rd_ = gen;

    std::uniform_int_distribution<> id_distr(1, MAX_OBJECT_ID);
    id_distr_ = id_distr;

    std::uniform_int_distribution<> num_distr(1, MAX_OBJECT_COUNT);
    num_distr_ = num_distr;

    timer_ = this->create_wall_timer(
        TIMER, std::bind(&EventPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std_msgs::msg::Int32MultiArray message;
    message.data.clear();

    for (int i = 0; i < num_distr_(rd_); i++)
    {
      message.data.push_back(id_distr_(rd_));
    }

    // RCLCPP_INFO(this->get_logger(), "Publishing: %s", std::to_string(message.data[0]).c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", std::to_string(message.data.size()).c_str());

    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 rd_;
  std::uniform_int_distribution<> id_distr_;
  std::uniform_int_distribution<> num_distr_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventPublisher>());
  rclcpp::shutdown();
  return 0;
}
