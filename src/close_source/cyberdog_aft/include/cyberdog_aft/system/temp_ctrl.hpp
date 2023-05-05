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

#ifndef CYBERDOG_AFT__SYSTEM__TEMP_CTRL_HPP_
#define CYBERDOG_AFT__SYSTEM__TEMP_CTRL_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <unordered_map>
#include <map>

#include "cyberdog_aft/system/hardware.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_aft/system/report_result.hpp"

namespace cyberdog
{
namespace system
{

class TempCtrl
{
public:
  TempCtrl();
  ~TempCtrl();

  TempCtrl(const TempCtrl &) = delete;
  TempCtrl & operator=(const TempCtrl &) = delete;

  enum class Type : u_int32_t
  {
    kCPU,
    kGPU,
    kFan,
    kIMU
  };

  using Callback = std::function<bool ()>;
  bool RunTest(const Type & type);

private:
  bool TestCPU();
  bool TestGPU();
  bool TestFan();
  bool TestIMU();

  std::map<Type, Callback> functions_;

  std::shared_ptr<rclcpp::Node> node_ptr_ {nullptr};
};


}  // namespace system
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__SYSTEM__TEMP_CTRL_HPP_
