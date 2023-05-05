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

#ifndef CYBERDOG_AFT__PROTOCOL__PROTOCOL_GRPC_HPP_
#define CYBERDOG_AFT__PROTOCOL__PROTOCOL_GRPC_HPP_
#define UNUSED(x) (void)(x)

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <iostream>
#include <random>
#include <string>
#include <deque>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <memory>
#include <thread>

#include "std_msgs/msg/bool.hpp"
#include "cyberdog_aft/protocol/protocol_interface.hpp"
#include "cyberdog_aft/protocol/aft_client.hpp"
#include "cyberdog_aft/protocol/aft_server.hpp"
#include "cyberdog_aft/protocol/protocol_motion.hpp"
#include "protocol/srv/audio_volume_set.hpp"
#include "protocol/srv/connect_ip.hpp"
#include "protocol/srv/connect_pc.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "protocol/msg/audio_play.hpp"
#include "protocol/msg/connector_status.hpp"
#include "protocol/msg/self_check_status.hpp"
#include "cyberdog_aft/protocol/protocol_system.hpp"
#include "cyberdog_aft/system/report_result.hpp"
#include "./cyberdog_aft.grpc.pb.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

#define RegisterConfig  "./aft_register_uri.toml"

namespace cyberdog
{
namespace protocol
{

struct ProtocolStaticState
{
  system::ReportResult storage_results;
  system::ReportResult battery_results;
  system::ReportResult hardware_device_results;
  std::deque<std::string> command_items;
};

struct ProtocolDynamicState
{
  std::unordered_map<std::string, std::string> success_results;
  std::unordered_map<std::string, std::string> failure_results;
  std::deque<std::string> command_items;
  std::vector<std::string> total_modules;
};

class ProtocolGrpc
{
public:
  using ReportResult = system::ReportResult;
  using StaticStageOneReportResult = system::StaticStageOneReportResult;
  using StaticStageTwoReportResult = system::StaticStageTwoReportResult;
  using DynamicReportResult = system::DynamicReportResult;

  ProtocolGrpc();
  ~ProtocolGrpc();

  enum class TestStatus : int16_t
  {
    Busy = 0,
    Error = 1,
    Idle = 2
  };

  ProtocolGrpc(const ProtocolGrpc &) = delete;
  ProtocolGrpc & operator=(const ProtocolGrpc &) = delete;

  void HandleMessages(
    const ::grpcapi::AFTRequest * request,
    ::grpc::ServerWriter<::grpcapi::AFTResponse> * writer);

  using ProtocolSystemState = ProtocolSystem::State;
  using ProtocolMotionState = ProtocolMotion::State;
  using ProtocolSystemType = ProtocolSystem::SystemType;
  using ProtocolMotionType = ProtocolMotion::MotionType;

  void Spin();

  /**
   * @brief Convert ReportResult type to json format
   *
   * @param report
   * @param json_response
   */
  void ReportResult2Rapidjson(
    const ReportResult & report,
    rapidjson::Document & json_response);

  /**
   * @brief Convert StaticStageOneReportResult type to json format
   *
   * @param report
   * @param json_response
   */
  void ReportResult2Rapidjson(
    const StaticStageOneReportResult & report,
    rapidjson::Document & json_response);

  /**
   * @brief Convert StaticStageTwoReportResult type to json format
   *
   * @param report
   * @param json_response
   */
  void ReportResult2Rapidjson(
    const StaticStageTwoReportResult & report,
    rapidjson::Document & json_response);

  /**
   * @brief Convert DynamicReportResult type to json format
   *
   * @param report
   * @param json_response
   */
  void ReportResult2Rapidjson(
    const DynamicReportResult & report,
    rapidjson::Document & json_response);

private:
  // template<typename Ty>
  // void RunServer(Ty response);
  void RunIpServer(::protocol::srv::ConnectIp::Response::SharedPtr response);
  void RunPcServer(::protocol::srv::ConnectPc::Response::SharedPtr response);

  // If GRPC run error, handle it
  void RetrunErrorGrpc(::grpc::ServerWriter<::grpcapi::AFTResponse> * writer);

  // Static & Dynamic test
  bool RunStaticTestStageOne();
  bool RunStaticTestStageTwo();
  bool RunDynamicTest();

  // Get report
  bool GetStaticStageOneReport(StaticStageOneReportResult & report);
  bool GetStaticStageTwoReport(StaticStageTwoReportResult & report);
  bool GetDynamicReport(DynamicReportResult & report);

  // Clear report
  bool ClearStaticStageOneReport(StaticStageOneReportResult & report);
  bool ClearStaticStageTwoReport(StaticStageTwoReportResult & report);

  // Check running state
  bool GetStaticTestsStageOneRunningState();
  bool GetStaticTestsStageTwoRunningState();
  bool GetDynamicTestsRunningState();

  // Initial
  bool InitializeStaticTestsStageOne();
  bool InitializeStaticTestsStageTwo();
  bool InitializeDynamicTests();

  void EmplaceSuccessBatteryTest();
  void EmplaceFailedBatteryTest();

  /**
   * @brief Debug ReportResult to string
   *
   * @param report
   */
  void DebugToString(const ReportResult & report);

  /**
   * @brief Debug ReportResult to string
   *
   * @param report
   */
  void DebugToString(const StaticStageOneReportResult & report);

  /**
   * @brief Debug ReportResult to string
   *
   * @param report
   */
  void DebugToString(const StaticStageTwoReportResult & report);

  /**
   * @brief Debug ReportResult to string
   *
   * @param report
   */
  void DebugToString(const DynamicReportResult & report);

  /**
   * @brief For debug and test
   *
   * @param msg
   */
  void HandleStageTest(const std_msgs::msg::Int32::SharedPtr msg);
  // void HandleConnectorStatus(const ::protocol::msg::ConnectorStatus::SharedPtr msg);
  void HandleConnectPc(
    ::protocol::srv::ConnectPc::Request::SharedPtr request,
    ::protocol::srv::ConnectPc::Response::SharedPtr response);
  void HandleConnectIp(
    ::protocol::srv::ConnectIp::Request::SharedPtr request,
    ::protocol::srv::ConnectIp::Response::SharedPtr response);
  void HandleSelfCheckStatus(const ::protocol::msg::SelfCheckStatus::SharedPtr msg);
  bool StaticTestParaInitFromToml(const std::string & config_file);
  bool RegisterParaInitFromToml(const std::string & config_file);

  // Specker broadcast to mention worker
  bool SpeckerBroadcast(int ID);

  std::shared_ptr<grpc::Server> server_{nullptr};
  std::shared_ptr<AFTClient> register_stub_{nullptr};
  std::shared_ptr<std::thread> server_thread_ptr_{nullptr};

  //
  bool construct_tests_ptr_{false};
  bool insuffcient_power_broadcast_{false};

  // module
  std::shared_ptr<ProtocolMotion> motion_ptr_{nullptr};
  std::shared_ptr<ProtocolSystem> system_ptr_{nullptr};
  StaticStageOneReportResult grpc_report_one_;
  StaticStageTwoReportResult grpc_report_two_;

  ProtocolDynamicState dynamic_states_;
  ProtocolStaticState static_states_stage_one_;
  ProtocolStaticState static_states_stage_two_;

  std::string wifi_name_;
  std::string register_server_uri_, sn_;

  bool is_delete_;
  std::string document_path_;

  int charge_duration_;
  int CPU_temperature_threhold_;
  int mic_volume_;

  std::shared_ptr<rclcpp::Node> test_node_ptr_{nullptr};
  std::shared_ptr<rclcpp::Node> node_for_audio_{nullptr};

  // 这3个 订阅者的回调函数 没有同时调用的需求
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stage_test_sub_{nullptr};
  rclcpp::Subscription<::protocol::msg::SelfCheckStatus>::SharedPtr selfCheck_status_sub_{nullptr};

  rclcpp::Subscription<::protocol::msg::ConnectorStatus>::SharedPtr connector_status_sub_{nullptr};
  rclcpp::Service<::protocol::srv::ConnectPc>::SharedPtr connect_pc_{nullptr};
  rclcpp::Service<::protocol::srv::ConnectIp>::SharedPtr connect_ip_{nullptr};
  rclcpp::CallbackGroup::SharedPtr connect_callback_group_{nullptr};

  // For mention stage1 and stage2 is finished
  rclcpp::Client<::protocol::srv::AudioVolumeSet>::SharedPtr speech_volume_set_client_{nullptr};
  rclcpp::Publisher<::protocol::msg::AudioPlay>::SharedPtr stage_test_finished_{nullptr};

  // For reset audio after finishing
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_audio_{nullptr};

  // Cpu temp
  int cpu_temp_{};

  // Static test time
  int static_test_time_{0};

  // Check dog's test status
  TestStatus test_status_{TestStatus::Idle};

  // Self check status
  bool self_check_pass_ {false};

  // Connector status
  bool connector_status_{false};
  int connector_count_{0};
};
}   // namespace protocol
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__PROTOCOL__PROTOCOL_GRPC_HPP_
