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

#include <cstdio>
#include <functional>
#include <memory>

#include "mgclient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using std::placeholders::_1;

const std::string TOPIC_NAME = "EVENTS_TOPIC";

class EventGraphWriter : public rclcpp::Node
{
public:
  EventGraphWriter(std::string host, int port)
      : Node("event_graph_writer")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        TOPIC_NAME, 10, std::bind(&EventGraphWriter::topic_callback, this, _1));

    mg::Client::Init();
    mg::Client::Params params;
    params.host = host;
    params.port = port;

    db_client_ = mg::Client::Connect(params);
    if (!db_client_)
    {
      throw rclcpp::exceptions::InvalidParametersException("Failed to connect.");
    }
    RCLCPP_INFO(this->get_logger(), "Connected to Memgraph.");
  }

  ~EventGraphWriter()
  {
    db_client_.reset(nullptr);
    mg::Client::Finalize();

    RCLCPP_INFO(this->get_logger(), "Connection terminated.");
  }

private:
  void topic_callback(const std_msgs::msg::Int32MultiArray &message) const
  {
    RCLCPP_INFO(this->get_logger(), "Got: '%s'", std::to_string(message.data.size()).c_str());
    rclcpp::Time t = this->now();

    std::string query = "CREATE (e:Event {timestamp: " + std::to_string(t.seconds()) + " }) \n";
    for (int i = 0; i < message.data.size(); i++)
    {
      query += "MERGE (p" + std::to_string(i) + ":Object {id:" + std::to_string(message.data[i]) + " })  \n";
      query += "CREATE (e) -[:I_SEE]-> (p" + std::to_string(i) + ")  \n";
    }
    RCLCPP_INFO(this->get_logger(), query.c_str());
    if (!db_client_->Execute(query))
    {
      std::cerr << "Failed to execute query.";
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Executed.");
    }
    db_client_->FetchOne();
  }

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
  std::unique_ptr<mg::Client> db_client_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventGraphWriter>("localhost", 7687));
  rclcpp::shutdown();
  return 0;
}