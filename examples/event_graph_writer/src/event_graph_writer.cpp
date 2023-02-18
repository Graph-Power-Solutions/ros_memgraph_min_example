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

  void drop_db()
  {
    if (!db_client_->Execute("MATCH (n) DETACH DELETE n"))
    {
      std::cerr << "Failed to drop db.";
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Drop db done.");
    }
    db_client_->FetchOne();
  }

private:
  void topic_callback(const std_msgs::msg::Int32MultiArray &message) const
  {
    RCLCPP_INFO(this->get_logger(), "Got: '%s'", std::to_string(message.data.size()).c_str());
    rclcpp::Time t = this->now();

    mg::Map query_params(2);

    query_params.Insert("timestamp", mg::Value(std::to_string(t.nanoseconds())));

    std::vector<mg::Value> vec_values(message.data.begin(), message.data.end());
    // std::vector<std::string> vec_str = {"a", "w"};
    // std::vector<mg::Value> vec_str_values(vec_str.begin(), vec_str.end());
    mg::List object_ids(
        vec_values);
    query_params.Insert("object_ids", mg::Value(std::move(object_ids)));

    std::string query = "\nCREATE (e:Event) \n"
                        "SET e.timestamp = toInteger($timestamp) \n"
                        "WITH e \n"
                        "UNWIND $object_ids AS object_id \n"
                        "MERGE (p:Object {id: object_id}) \n"
                        "CREATE (e) -[:I_SEE]-> (p)";

    RCLCPP_INFO(this->get_logger(), query.c_str());
    if (!db_client_->Execute(query, query_params.AsConstMap()))
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
  auto graph_writer = std::make_shared<EventGraphWriter>("localhost", 7687);
  graph_writer->drop_db();
  rclcpp::spin(graph_writer);
  rclcpp::shutdown();
  return 0;
}