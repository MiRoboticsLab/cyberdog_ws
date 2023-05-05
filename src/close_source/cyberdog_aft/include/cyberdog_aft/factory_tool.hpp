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

#ifndef CYBERDOG_AFT__FACTORY_TOOL_HPP_
#define CYBERDOG_AFT__FACTORY_TOOL_HPP_

#include <functional>
#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include <map>
#include <deque>
#include <string>

#include "cyberdog_aft/motion/motion.hpp"
#include "cyberdog_aft/navigation/navigation.hpp"
#include "cyberdog_aft/protocol/protocol.hpp"
#include "cyberdog_aft/slam/slam.hpp"
#include "cyberdog_aft/system/system.hpp"
#include "cyberdog_aft/visualization/visualization.hpp"
// #include "cyberdog_aft/config_manager.hpp"

#include "rclcpp/rclcpp.hpp"

namespace cyberdog
{
class FactoryTool : public rclcpp::Node
{
public:
  FactoryTool();
  ~FactoryTool();

  FactoryTool(const FactoryTool &) = delete;
  FactoryTool & operator=(const FactoryTool &) = delete;

  bool Start(std::string & module_name);
  bool Start();
  bool Pause();
  bool Finished();

  // ParameterManager * parameter_manager();

private:
  void RunTask();

  bool CreateModules();

  // UI config
  void CheckUIStart();
  bool StartUI();

  // std::shared_ptr<ParameterManager> parameters_ptr_ {nullptr};
  std::shared_ptr<motion::Motion> motion_ptr_ {nullptr};
  std::shared_ptr<navigation::Navigation> navigation_ptr_ {nullptr};
  std::shared_ptr<protocol::Protocol> protocol_ptr_ {nullptr};
  std::shared_ptr<slam::SLAM> slam_ptr_ {nullptr};

  std::shared_ptr<visualization::Visualization> visualization_ptr_ {nullptr};

  std::mutex ui_mutex_;
  rclcpp::TimerBase::SharedPtr ui_timer_;
  bool initialized_ {false};
  bool start_ui_ {false};
};

}  // namespace cyberdog

#endif  // CYBERDOG_AFT__FACTORY_TOOL_HPP_
