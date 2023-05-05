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

#ifndef CYBERDOG_AFT__NAVIGATION__NAVIGATION_HPP_
#define CYBERDOG_AFT__NAVIGATION__NAVIGATION_HPP_


#include <chrono>
#include <memory>
#include <deque>
#include <map>

#include "protocol/action/navigation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace cyberdog
{
namespace navigation
{

enum class CommmandAction
{
  kNavTypeAB,       // AB点导航
  kNavTypeFollow       // 跟随模式
};

class Navigation
{
public:
  Navigation();
  ~Navigation();

  Navigation(const Navigation &) = delete;
  Navigation & operator=(const Navigation &) = delete;

  bool LoadDefaultConfig();
  bool RunDefaultTestCommmandActions();

  bool RunCommandAction(const CommmandAction & command);
  bool AddCommandAction(const CommmandAction & command);
  bool RemoveCommandAction(const CommmandAction & command);
  bool HasContain(const CommmandAction & command);

  bool Reset();
  bool CheckAllFinished();
  bool RemoveAllCommmandAction();

private:
  std::shared_ptr<rclcpp::Node> node_ptr_ {nullptr};
  rclcpp_action::Client<::protocol::action::Navigation>::SharedPtr client_ptr_ {nullptr};
  std::deque<CommmandAction> queue_;
  std::map<CommmandAction, bool> results_;
};


}  // namespace navigation
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__NAVIGATION__NAVIGATION_HPP_
