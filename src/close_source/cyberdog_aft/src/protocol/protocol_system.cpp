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

#include "cyberdog_aft/protocol/protocol_system.hpp"
namespace cyberdog
{
namespace protocol
{

ProtocolSystem::ProtocolSystem()
{
  system_ptr_ = std::make_shared<system::System>();
  stop_noise_node_ = rclcpp::Node::make_shared("stop_noise_node");
  callback_group_ = stop_noise_node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  INFO("Create stop noise service......");
  stop_noise_service_ =
    stop_noise_node_->create_service<std_srvs::srv::SetBool>(
    "stop_noise_service",
    std::bind(
      &cyberdog::protocol::ProtocolSystem::StopWhiteNoiseCb, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default,
    callback_group_);
  std::thread{[this] {rclcpp::spin(stop_noise_node_);}}.detach();
}

ProtocolSystem::~ProtocolSystem()
{
}

bool ProtocolSystem::RunCommand(const SystemType & type)
{
  bool ok = system_ptr_->RunTest(type);

  if (!ok) {
    return false;
  }
  return true;
}

bool ProtocolSystem::RunDefaultCommand(const SystemType & type)
{
  if (type == SystemType::kStorage) {
    return system_ptr_->RunDefaultTest(SystemType::kStorage);
  } else if (type == SystemType::kHardwareDevice) {
    return system_ptr_->RunDefaultTest(SystemType::kHardwareDevice);
  }
  return true;
}

void ProtocolSystem::InitializeDefaultTestModules(const SystemType & type)
{
  system_ptr_->InitializeDefaultTestModules(type);
}

void ProtocolSystem::SenorStatusIniti()
{
  system_ptr_->SenorStatusIniti();
}

bool ProtocolSystem::GetTestReportResult(const SystemType & type, ReportResult & report)
{
  if (type == SystemType::kStorage) {
    return system_ptr_->GetStorageReport(report);
  } else if (type == SystemType::kHardwareDevice) {
    return system_ptr_->GetHardwareReport(report);
  }
  return true;
}

bool ProtocolSystem::ClearStaticStageOneReport(const SystemType & type, ReportResult & report)
{
  if (type == SystemType::kStorage) {
    return system_ptr_->ClearStorageReport(report);
  } else if (type == SystemType::kHardwareDevice) {
    return system_ptr_->ClearHardwareReport(report);
  }
  return true;
}

bool ProtocolSystem::ClearStaticStageTwoReport(const SystemType & type, ReportResult & report)
{
  if (type == SystemType::kStorage) {
    return system_ptr_->ClearStorageReport(report);
  } else if (type == SystemType::kHardwareDevice) {
    return system_ptr_->ClearHardwareReport(report);
  }
  return true;
}

std::string ProtocolSystem::GetSystemVersion()
{
  return system_ptr_->GetSystemVersion();
}

std::string ProtocolSystem::GetSnInfo()
{
  return system_ptr_->GetSnInfo();
}

std::string ProtocolSystem::GetBmsVolt()
{
  return std::to_string(system_ptr_->GetBmsVolt());
}

std::string ProtocolSystem::GetBmsSoc()
{
  return std::to_string(system_ptr_->GetBmsSoc());
}

std::string ProtocolSystem::GetBmsStatus()
{
  // auto bms_status = system_ptr_->GetBmsStatus();
  // switch (bms_status)
  // {
  // case 0:// 正常模式
  //   return "normal";
  //   break;

  // case 1:// 有线正在充电
  //   return "wire charging";
  //   break;

  // case 2:// 充电完成
  //   return "charge completely";
  //   break;

  // case 3:// 电机掉电
  //   break;

  // case 4:// 软关机
  //   break;

  // case 5:// 无线充在位
  //   break;

  // case 6:// 无线充电中
  //   break;

  // case 7:// 外部供电
  //   break;

  // default:
  //   break;
  // }
  return std::to_string(system_ptr_->GetBmsStatus());
}

int ProtocolSystem::GetCpuTemp()
{
  return system_ptr_->GetCpuTemp();
}

bool ProtocolSystem::TurnOffFanMoniterTemp()
{
  return system_ptr_->TestTurnOffFan();
}

bool ProtocolSystem::TurnOnFanMoniterTemp()
{
  return system_ptr_->TestTurnOnFan();
}

bool ProtocolSystem::CpuTest()
{
  return system_ptr_->CpuTest();
}

bool ProtocolSystem::FanTestSuccess()
{
  return system_ptr_->FanTestSuccess();
}

void ProtocolSystem::LampChange()
{
  return system_ptr_->LedsChangeTest();
}

void ProtocolSystem::LameStatusChange()
{
  return system_ptr_->LedsTestStatusChange();
}

bool ProtocolSystem::PlayWhiteNoise(const int & volume)
{
  return system_ptr_->PlayWhiteNoise(volume);
}

bool ProtocolSystem::PlayWhiteNoiseOnce(const int & volume)
{
  return system_ptr_->PlayWhiteNoiseOnce(volume);
}

void ProtocolSystem::StopPlayWhiteNoise()
{
  system_ptr_->StopPlayWhiteNoise();
}

void ProtocolSystem::StartPlayWhiteNoise()
{
  system_ptr_->StartPlayWhiteNoise();
}

void ProtocolSystem::EmplaceSuccessBatteryTest()
{
  system_ptr_->EmplaceSuccessBatteryTest();
}

void ProtocolSystem::EmplaceFailedBatteryTest()
{
  system_ptr_->EmplaceFailedBatteryTest();
}

void ProtocolSystem::StopWhiteNoiseCb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    StopPlayWhiteNoise();
  }
  response->success = true;
  response->message = "White noise will stop within 1min";
  INFO("%s", response->message.c_str());
}

}  // namespace protocol
}  // namespace cyberdog
