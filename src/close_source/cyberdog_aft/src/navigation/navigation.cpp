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

#include "cyberdog_aft/navigation/navigation.hpp"

namespace cyberdog
{
namespace navigation
{

Navigation::Navigation()
{
  node_ptr_ = rclcpp::Node::make_shared("cyberdog_aft_navigation");
}

Navigation::~Navigation()
{
}

bool Navigation::LoadDefaultConfig()
{
  return true;
}

bool Navigation::RunDefaultTestCommmandActions()
{
  return true;
}

bool Navigation::RunCommandAction(const CommmandAction & command)
{
  switch (command) {
    case CommmandAction::kNavTypeAB:
      break;

    case CommmandAction::kNavTypeFollow:
      break;

    default:
      break;
  }
  return true;
}

bool Navigation::AddCommandAction(const CommmandAction & command)
{
  return true;
}

bool Navigation::RemoveCommandAction(const CommmandAction & command)
{
  return true;
}

bool Navigation::HasContain(const CommmandAction & command)
{
  return true;
}

bool Navigation::Reset()
{
  return true;
}

bool Navigation::CheckAllFinished()
{
  return true;
}

bool Navigation::RemoveAllCommmandAction()
{
  return true;
}

}  // namespace navigation
}  // namespace cyberdog
