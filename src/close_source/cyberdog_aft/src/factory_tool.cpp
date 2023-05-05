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

#include <string>
#include <memory>

#include "cyberdog_aft/factory_tool.hpp"
#include "cyberdog_aft/utils/utils.hpp"

using namespace std::chrono_literals;

namespace cyberdog
{

// std::string kAFTDefaultConfigFilename = "aft_default.toml";

FactoryTool::FactoryTool()
: Node("aft")
{
  initialized_ = CreateModules();
  if (!initialized_) {
    ERROR("FactoryTool failed.");
  }

  // 1 Load config
  // std::string filename = utils::GetDefaultPath() + "/config" + kAFTDefaultConfigFilename;
  // bool paraser_ok = parameters_ptr_->LoadUnchangeableDefaultConfig(filename);
  // if (!paraser_ok) {
  //     ERROR("Load default config file error");
  // }

  // UI
  // this->declare_parameter<bool>("start_ui", false);
  // ui_timer_ = this->create_wall_timer(1000ms, std::bind(&FactoryTool::CheckUIStart, this));
}

FactoryTool::~FactoryTool()
{
}

void FactoryTool::RunTask()
{
  while (true) {
    if (start_ui_) {
      StartUI();
    }

    // // 2 Test motion
    // bool motion_ok = motion_ptr_->RunDefaultTestCommmandActions();
    // if (motion_ok) {
    //     INFO("TEST Motion success.");
    // } else {
    //     ERROR("TEST Motion failed.");
    // }

    // // 3 Test Nav
    // bool nav_ok = navigation_ptr_->RunDefaultTestCommmandActions();
    // if (nav_ok) {
    //     INFO("TEST Navigation success.");
    // } else {
    //     ERROR("TEST Navigation failed.");
    // }

    // // 4 Test SLAM
    // bool slam_ok = slam_ptr_->RunDefaultTestCommmandActions();
    // if (slam_ok) {
    //     INFO("TEST SLAM success.");
    // } else {
    //     ERROR("TEST SLAM failed.");
    // }

    // // 5 Test system
    // bool system_ok = system_ptr_->RunDefaultTestCommmandActions();;
    // if (system_ok) {
    //     INFO("TEST System[Audio & Image Transmission] success.");
    // } else {
    //     ERROR("TEST System[Audio & Image Transmission] failed.");
    // }
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
}

bool FactoryTool::CreateModules()
{
  protocol_ptr_ = std::make_shared<protocol::Protocol>();
  if (protocol_ptr_ == nullptr) {
    ERROR("Create Protocol GRPC error.");
    return false;
  }

  // motion_ptr_ = std::make_shared<motion::Motion>();
  // if (motion_ptr_ == nullptr) {
  //   return false;
  // }

  // navigation_ptr_ = std::make_shared<navigation::Navigation>();
  // if (navigation_ptr_ == nullptr) {
  //   return false;
  // }

  // slam_ptr_ = std::make_shared<slam::SLAM>();
  // if (slam_ptr_ == nullptr) {
  //   return false;
  // }

  // visualization_ptr_ = std::make_shared<visualization::Visualization>();
  // if (visualization_ptr_ == nullptr) {
  //   return false;
  // }

  return true;
}

// ParameterManager * FactoryTool::parameter_manager()
// {
//   return parameters_ptr_.get();
// }


void FactoryTool::CheckUIStart()
{
  this->get_parameter("start_ui", start_ui_);
}

bool FactoryTool::StartUI()
{
  return true;
}

}  // namespace cyberdog
