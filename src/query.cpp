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
#include <rclcpp/rclcpp.hpp>
#include <mgclient.hpp>

// See https://github.com/memgraph/mgclient/blob/master/examples/advanced.cpp

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  mg::Client::Init();

  mg::Client::Params params;
  params.host = "localhost";
  params.port = 7687;
  auto client = mg::Client::Connect(params);
  if (!client)
  {
    std::cerr << "Failed to connect." << std::endl;
    return 1;
  }

  if (!client->Execute("MATCH (n) RETURN n LIMIT 10"))
  {
    std::cerr << "Failed to execute query!";
    return 1;
  }

  while (const auto maybe_result = client->FetchOne())
  {
    const auto result = *maybe_result;
    if (result.size() < 1)
    {
      continue;
    }
    const auto value = result[0];
    if (value.type() == mg::Value::Type::Node)
    {
      const auto node = value.ValueNode();
      auto labels = node.labels();
      std::string labels_str = std::accumulate(
          labels.begin(), labels.end(), std::string(""),
          [](const std::string &acc, const std::string_view value)
          {
            return acc + ":" + std::string(value);
          });
      const auto props = node.properties();
      std::string props_str =
          std::accumulate(
              props.begin(), props.end(), std::string("{"),
              [](const std::string &acc, const auto &key_value)
              {
                const auto &[key, value] = key_value;
                std::string value_str;
                if (value.type() == mg::Value::Type::Int)
                {
                  value_str = std::to_string(value.ValueInt());
                }
                else if (value.type() == mg::Value::Type::String)
                {
                  value_str = value.ValueString();
                }
                else if (value.type() == mg::Value::Type::Bool)
                {
                  value_str = std::to_string(value.ValueBool());
                }
                else if (value.type() == mg::Value::Type::Double)
                {
                  value_str = std::to_string(value.ValueDouble());
                }
                else
                {
                  std::cerr
                      << "Uncovered converstion from data type to a string"
                      << std::endl;
                  std::exit(1);
                }
                return acc + " " + std::string(key) + ": " + value_str;
              }) +
          " }";
      std::cout << labels_str << " " << props_str << std::endl;
    }
  }

  // Deallocate the client because mg_finalize has to be called globally.
  client.reset(nullptr);

  mg::Client::Finalize();

  printf("hello graph\n");
  return 0;
}
