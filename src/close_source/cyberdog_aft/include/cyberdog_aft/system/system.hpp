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

#ifndef CYBERDOG_AFT__SYSTEM__SYSTEM_HPP_
#define CYBERDOG_AFT__SYSTEM__SYSTEM_HPP_

#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <unordered_map>

#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_aft/system/hardware.hpp"
#include "cyberdog_aft/system/storage.hpp"
#include "cyberdog_aft/system/temp_ctrl.hpp"
#include "protocol/msg/bms_status.hpp"
#include "protocol/srv/device_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_aft/system/report_result.hpp"

namespace cyberdog
{
namespace system
{
class System
{
public:
  System();
  ~System();

  System(const System &) = delete;
  System & operator=(const System &) = delete;

  enum class Type : u_int32_t
  {
    kBattery,
    kStorage,
    kHardwareDevice,
    kUnknown
  };

  using SystemType = System::Type;
  using SubsystemHardwareType = Hardware::Type;
  using SubsystemStorageType = Storage::Type;
  using SubsystemTempCtrlType = TempCtrl::Type;

  using Callback = std::function<bool ()>;
  bool RunTest(const Type & type);

  bool RunSubsystemTest(const SubsystemHardwareType & hardware_type);
  bool RunSubsystemTest(const SubsystemStorageType & storage_type);

  /**
   * @brief Run default test
   *
   * @param what Which system type
   * @return true
   * @return false
   */
  bool RunDefaultTest(const SystemType & what);

  using BmsStatus = ::protocol::msg::BmsStatus;
  using DeviceInfo = ::protocol::srv::DeviceInfo::Response;

  bool GetBmsStatusInfo(BmsStatus & bms_info);

  // Initialize which module test items
  bool InitializeModuleTestItems(const Type & what, const std::vector<std::string> & items);

  // Default test modules
  void InitializeDefaultTestModules(const Type & what_type);
  void SenorStatusIniti();

  // Hardware device report
  bool GetHardwareReport(ReportResult & report);
  bool ClearHardwareReport(ReportResult & report);

  // Storage report
  bool GetStorageReport(ReportResult & report);
  bool ClearStorageReport(ReportResult & report);

  std::string & GetSnInfo();

  // System version
  std::string GetSystemVersion();

  // battery
  uint16_t GetBmsVolt();
  uint8_t GetBmsSoc();
  uint8_t GetBmsStatus();
  int16_t GetBmsTemp();

  // cpu temperature
  int GetCpuTemp();

  bool TestTurnOffFan();
  bool TestTurnOnFan();
  bool FanTestSuccess();
  bool PlayWhiteNoise(const int & volume);
  bool PlayWhiteNoiseOnce(const int & volume);
  void LedsTestStatusChange();
  void LedsChangeTest();
  void StopPlayWhiteNoise();
  void StartPlayWhiteNoise();
  void EmplaceSuccessBatteryTest();
  void EmplaceFailedBatteryTest();
  bool CpuTest();

private:
  void Spin();
  void HandleBmsStatusMessage(const ::protocol::msg::BmsStatus::SharedPtr msg);

  bool TestStorage();
  bool TestHardwareDevice();

  std::shared_ptr<Hardware> hardware_ptr_ {nullptr};
  std::shared_ptr<Storage> storage_ptr_ {nullptr};
  std::shared_ptr<TempCtrl> temp_ctrl_ptr_ {nullptr};

  std::shared_ptr<rclcpp::Node> node_ptr_ {nullptr};
  rclcpp::Subscription<::protocol::msg::BmsStatus>::SharedPtr bms_sub_ {nullptr};
  rclcpp::Client<::protocol::srv::DeviceInfo>::SharedPtr sn_cli_ {nullptr};

  std::shared_ptr<std::thread> spin_thread_ptr_ {nullptr};
  std::mutex mutex_;
  BmsStatus bms_status_;
  std::string sn_info_;

  int cpu_temp_;
};

}  // namespace system
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__SYSTEM__SYSTEM_HPP_
