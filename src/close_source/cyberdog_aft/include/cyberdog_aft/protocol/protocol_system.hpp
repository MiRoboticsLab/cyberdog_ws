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

#ifndef CYBERDOG_AFT__PROTOCOL__PROTOCOL_SYSTEM_HPP_
#define CYBERDOG_AFT__PROTOCOL__PROTOCOL_SYSTEM_HPP_

#include <cstdint>
#include <chrono>
#include <memory>
#include <deque>
#include <thread>
#include <vector>
#include <functional>
#include <unordered_map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cyberdog_aft/system/system.hpp"
#include "cyberdog_aft/system/report_result.hpp"

namespace cyberdog
{
namespace protocol
{

class ProtocolSystem
{
public:
  using ReportResult = system::ReportResult;

  ProtocolSystem();
  ~ProtocolSystem();

  ProtocolSystem(const ProtocolSystem &) = delete;
  ProtocolSystem & operator=(const ProtocolSystem &) = delete;

  using SystemType = system::System::Type;
  bool RunCommand(const SystemType & type);

  /**
   * @brief Default config test
   *
   * @param type
   * @return true
   * @return false
   */
  bool RunDefaultCommand(const SystemType & type);

  /**
   * @brief Initialize config default test
   *
   * @param type
   */
  void InitializeDefaultTestModules(const SystemType & type);
  void SenorStatusIniti();

  /**
   * @brief Get the Test Report Result object
   *
   * @param type
   * @param report
   * @return true
   * @return false
   */
  bool GetTestReportResult(const SystemType & type, ReportResult & report);
  bool ClearStaticStageOneReport(const SystemType & type, ReportResult & report);
  bool ClearStaticStageTwoReport(const SystemType & type, ReportResult & report);
  struct State
  {
    SystemType current_type;
    std::deque<SystemType> remain_motions;
    std::vector<SystemType> total_motions;
    std::unordered_map<SystemType, bool> results;

    State()
    : current_type{SystemType::kUnknown}
    {
    }
  };

  // Get system version
  std::string GetSystemVersion();

  // Get Sn info
  std::string GetSnInfo();

  // Get Bms info
  std::string GetBmsVolt();
  std::string GetBmsSoc();
  std::string GetBmsStatus();

  // Get CPU temp
  int GetCpuTemp();

  // turn off/on the fan and moniter cpu tempture
  bool TurnOffFanMoniterTemp();
  bool TurnOnFanMoniterTemp();
  bool CpuTest();
  bool FanTestSuccess();

  // play white noise
  bool PlayWhiteNoise(const int & volume);
  bool PlayWhiteNoiseOnce(const int & volume);

  // lamp
  void LampChange();
  void LameStatusChange();

  // stop white noise
  void StopPlayWhiteNoise();
  void StartPlayWhiteNoise();
  void EmplaceSuccessBatteryTest();
  void EmplaceFailedBatteryTest();

  // callback stop white noise
  void StopWhiteNoiseCb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

private:
  std::shared_ptr<rclcpp::Node> stop_noise_node_{nullptr};

  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_noise_service_{nullptr};

  std::shared_ptr<system::System> system_ptr_{nullptr};
  State state_;
};

}   //  namespace protocol
}  //  namespace cyberdog

#endif  // CYBERDOG_AFT__PROTOCOL__PROTOCOL_SYSTEM_HPP_
