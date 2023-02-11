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
#include <mgclient.hpp>

#include "rclcpp/rclcpp.hpp"

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
  std::cout << "Connected." << std::endl;

  return 0;
}
