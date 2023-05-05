// Copyright (c) 2021 Xiaomi Corporation
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
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cyberdog_aft/protocol/aft_client.hpp"

namespace cyberdog
{
void Run()
{
  grpc::string ip = std::string("localhost:50051");
  auto channel = grpc::CreateChannel(ip, grpc::InsecureChannelCredentials());

  auto client = std::make_shared<protocol::AFTClient>(channel);
  ::grpcapi::AFTRequest request;

  std::string msg = "{\"name\":\"菜鸟教程\"}";

  request.set_name_code(1001);
  request.set_params(msg);
  bool send_ok = client->SendRequest(request);
  if (send_ok) {
    std::cout << "Send message ok." << std::endl;
  }
}  // namespace
}  // namespace cyberdog

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  cyberdog::Run();
  rclcpp::shutdown();
  return 0;
}
