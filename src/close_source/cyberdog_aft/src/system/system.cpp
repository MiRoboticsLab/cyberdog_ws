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

#include <memory>
#include <vector>
#include <string>
#include <utility>

#include "cyberdog_aft/system/system.hpp"
#include "cyberdog_aft/utils/shell_command.hpp"
#include "cyberdog_aft/utils/string_util.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

using namespace std::chrono_literals;

namespace cyberdog
{
namespace system
{

System::System()
{
  node_ptr_ = rclcpp::Node::make_shared("cyberdog_aft_system");

  hardware_ptr_ = std::make_shared<Hardware>(node_ptr_);
  storage_ptr_ = std::make_shared<Storage>();
  temp_ctrl_ptr_ = std::make_shared<TempCtrl>();

  bms_sub_ = node_ptr_->create_subscription<::protocol::msg::BmsStatus>(
    "bms_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&System::HandleBmsStatusMessage, this, std::placeholders::_1));

  sn_cli_ = node_ptr_->create_client<::protocol::srv::DeviceInfo>("query_divice_info");

  spin_thread_ptr_ = std::make_shared<std::thread>(&System::Spin, this);
}

System::~System()
{
}

bool System::RunTest(const Type & type)
{
  if (type == Type::kStorage) {
    return TestStorage();
  } else if (type == Type::kHardwareDevice) {
    return TestHardwareDevice();
  }
  return false;
}

bool System::RunSubsystemTest(const SubsystemHardwareType & hardware_type)
{
  bool ok = hardware_ptr_->RunTest(hardware_type);

  if (!ok) {
    return false;
  }
  return true;
}

bool System::RunSubsystemTest(const SubsystemStorageType & storage_type)
{
  bool ok = storage_ptr_->RunTest(storage_type);
  if (!ok) {
    return false;
  }
  return true;
}

bool System::RunDefaultTest(const SystemType & what)
{
  if (what == Type::kStorage) {
    return storage_ptr_->RunAllTest();
  } else if (what == Type::kHardwareDevice) {
    return hardware_ptr_->RunAllTest();
  } 
  return false;
}

bool System::GetBmsStatusInfo(BmsStatus & bms_info)
{
  std::lock_guard<std::mutex> lock(mutex_);
  bms_info = bms_status_;
  return true;
}

std::string & System::GetSnInfo()
{
  const std::string sn_info_commmand =
    "ros2 service call /`ros2 node list | grep 'mi_' | head -n 1 | cut -f 2 -d '/'`/query_divice_info protocol/srv/DeviceInfo '{enables: [true]}'";
  bool success = false;

  sn_info_ = RunShellCommand(
    sn_info_commmand.c_str(), success);

  if (success == false) {
    sn_info_ = "sn_version fails";
  }
  INFO("sn:%s", sn_info_.c_str());
  return sn_info_;
}

bool System::InitializeModuleTestItems(const Type & what, const std::vector<std::string> & items)
{
  switch (what) {
    case Type::kStorage:
      return storage_ptr_->InitializeTotalModuleTests(items);
      break;

    case Type::kHardwareDevice:
      return hardware_ptr_->InitializeTotalModuleTests(items);
      break;

    default:
      break;
  }

  return true;
}

void System::InitializeDefaultTestModules(const Type & what_type)
{
  switch (what_type) {
    case Type::kStorage:
      storage_ptr_->InitializeTestModules();
      break;

    case Type::kHardwareDevice:
      hardware_ptr_->InitializeTestModules();
      break;

    default:
      break;
  }
}

void System::SenorStatusIniti()
{
  hardware_ptr_->SenorStatusIniti();
}

bool System::GetHardwareReport(ReportResult & report)
{
  bool flag = hardware_ptr_->GetReport(report);
       hardware_ptr_->ReportToString(report);
  return flag;
}

bool System::ClearHardwareReport(ReportResult & report)
{
  bool flag = hardware_ptr_->ClearReport(report);
  return flag;
}

bool System::GetStorageReport(ReportResult & report)
{
  bool flag = storage_ptr_->GetReport(report);
       storage_ptr_->ReportToString(report);
  return flag;
}

bool System::ClearStorageReport(ReportResult & report)
{
  bool flag = storage_ptr_->ClearReport(report);
  return flag;
}

std::string System::GetSystemVersion()
{
  std::string version = "unkown";
  bool success = false;
  std::string cmd =
    "factory-tool -f /usr/share/factory_cmd_config/version.xml -i 'Pure Version NX Board'";
  INFO("run cmd:%s", cmd.c_str());
  version = RunShellCommand(cmd.c_str(), success);
  if (success) {
    version = StringUtil::strip(version);
  } else {
    std::ifstream fin("/etc/sw-version");
    while (getline(fin, version)) {
      bool find = false;
      std::vector<std::string> result = StringUtil::split_filter_empty(version, ':');
      for (decltype(result.size()) i = 0; i < result.size(); i++) {
        if (result[i] == "Release") {
          version = StringUtil::strip(result[i + 1]);
          find = true;
          break;
        }
      }
      if (find) {
        break;
      }
    }
  }
  INFO("SystemVersion:%s", version.c_str());
  return version;
}

uint16_t System::GetBmsVolt()
{
  INFO("Battery_volt:%d", bms_status_.batt_volt);
  return bms_status_.batt_volt;
}

uint8_t System::GetBmsSoc()
{
  // INFO("Battery_soc:%d", bms_status_.batt_soc);
  return bms_status_.batt_soc;
}

uint8_t System::GetBmsStatus()
{
  INFO("Battery_status:%d", bms_status_.bms_state_one);
  return bms_status_.bms_state_one;
}

int16_t System::GetBmsTemp()
{
  INFO("CPU_temperature:%d", bms_status_.batt_temp);
  return bms_status_.batt_temp;
}

int System::GetCpuTemp()
{
  std::string cmd = "cat /sys/class/thermal/thermal_zone0/temp";
  bool success = false;
  std::string cpu_temp = RunShellCommand(cmd.c_str(), success);
  cpu_temp_ = std::stoi(cpu_temp);
  return cpu_temp_;
}

bool System::TestTurnOffFan()
{
  return hardware_ptr_->TestTurnOffFan();
}

bool System::TestTurnOnFan()
{
  return hardware_ptr_->TestTurnOnFan();
}

bool System::FanTestSuccess()
{
  return storage_ptr_->TestFanCase();
}

bool System::PlayWhiteNoise(const int& volume)
{
  return hardware_ptr_->PlayWhiteNoise(volume);
}

void System::LedsChangeTest()
{
  return hardware_ptr_->LedsChangeTest();
}

void System::LedsTestStatusChange()
{
  return hardware_ptr_->LedsTestStatusChange();
}

bool System::PlayWhiteNoiseOnce(const int& volume)
{
  return hardware_ptr_->PlayWhiteNoiseOnce(volume);
}

void System::StopPlayWhiteNoise()
{
   hardware_ptr_->StopPlayWhiteNoise();
}

void System::StartPlayWhiteNoise()
{
  hardware_ptr_->StartPlayWhiteNoise();
}

void System::EmplaceSuccessBatteryTest()
{
   hardware_ptr_->EmplaceSuccessBatteryTest();
}

void System::EmplaceFailedBatteryTest()
{
  hardware_ptr_->EmplaceFailedBatteryTest();
}

bool System::CpuTest()
{
  return storage_ptr_->TestCPUCase();
}

void System::Spin()
{ 
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_ptr_);
  executor.spin();
  // rclcpp::spin(node_ptr_);
}

void System::HandleBmsStatusMessage(const ::protocol::msg::BmsStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  bms_status_ = *msg;
}

bool System::TestStorage()
{
  // memory
  bool flag = RunSubsystemTest(SubsystemStorageType::kMemory);
  if (!flag) {
    INFO("Storage kMemory error");
  }

  // emmc
  flag = RunSubsystemTest(SubsystemStorageType::kEmmc);
  if (!flag) {
    INFO("Storage kEmmc error");
  }

  // sdcard
  flag = RunSubsystemTest(SubsystemStorageType::kSDCard);
  if (!flag) {
    INFO("Storage kSDCard error");
  }
  return true;
}

bool System::TestHardwareDevice()
{
  // touch
  bool flag = RunSubsystemTest(SubsystemHardwareType::kTouch);
  if (!flag) {
    INFO("Hardware touch error");
  }

  // lidar
  flag = RunSubsystemTest(SubsystemHardwareType::kLidar);
  if (!flag) {
    INFO("Hardware lidar error");
  }

  // head left tof
  flag = RunSubsystemTest(SubsystemHardwareType::kHeadLeftTOF);
  if (!flag) {
    INFO("Hardware head left tof error");
  }
  
  // head right tof
  flag = RunSubsystemTest(SubsystemHardwareType::kHeadRightTOF);
  if (!flag) {
    INFO("Hardware head left tof error");
  }

  // gps
  flag = RunSubsystemTest(SubsystemHardwareType::kGPS);
  if (!flag) {
    INFO("Hardware gps error");
  }

  // ultrasonic
  flag = RunSubsystemTest(SubsystemHardwareType::kUltrasound);
  if (!flag) {
    INFO("Hardware ultrasonic error");
  }

  // wifi
  flag = RunSubsystemTest(SubsystemHardwareType::kWIFI);
  if (!flag) {
    INFO("Hardware wifi error");
  }

  return flag;
}

}   // namespace system
} // namespace cyberdog
