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

#include "cyberdog_aft/factory_tool.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_debug/backtrace.hpp"

namespace cyberdog
{
void Run()
{
  auto auto_factory_tool = std::make_shared<FactoryTool>();
  rclcpp::spin(auto_factory_tool);
}
}

int main(int argc, char * argv[])
{
  cyberdog::debug::register_signal();
  LOGGER_MAIN_INSTANCE("CyberdogAFT");
  rclcpp::init(argc, argv);
  cyberdog::Run();
  rclcpp::shutdown();
  return 0;
}
