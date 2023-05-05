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
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_aft/protocol/protocol_grpc.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"


namespace cyberdog
{
namespace protocol
{

const int kStaticTestStageOne = 1;
const int kStaticTestStageTwo = 2;

ProtocolGrpc::ProtocolGrpc()
{
  test_node_ptr_ = rclcpp::Node::make_shared("cyberdog_aft_stage_test");

  const std::string toml_file = ament_index_cpp::get_package_share_directory(
    "cyberdog_aft") + "/config/static_para_setting.toml";
  StaticTestParaInitFromToml(toml_file);

  stage_test_sub_ = test_node_ptr_->create_subscription<std_msgs::msg::Int32>(
    "stage_test", rclcpp::SystemDefaultsQoS(),
    std::bind(&ProtocolGrpc::HandleStageTest, this, std::placeholders::_1));

  selfCheck_status_sub_ =
    test_node_ptr_->create_subscription<::protocol::msg::SelfCheckStatus>(
    "self_check_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&ProtocolGrpc::HandleSelfCheckStatus, this, std::placeholders::_1));

  // connector_status_sub_ = test_node_ptr_->create_subscription<::protocol::msg::ConnectorStatus>(
  //   "connector_state", rclcpp::SystemDefaultsQoS(),
  //   std::bind(&ProtocolGrpc::HandleConnectorStatus, this, std::placeholders::_1));

  connect_callback_group_ = test_node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  connect_pc_ = test_node_ptr_->create_service<::protocol::srv::ConnectPc>(
    "connect_pc", std::bind(
      &ProtocolGrpc::HandleConnectPc, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default,
    connect_callback_group_);

  connect_ip_ = test_node_ptr_->create_service<::protocol::srv::ConnectIp>(
    "connect_ip", std::bind(
      &ProtocolGrpc::HandleConnectIp, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default,
    connect_callback_group_);
}

ProtocolGrpc::~ProtocolGrpc()
{
}

void ProtocolGrpc::HandleMessages(
  const ::grpcapi::AFTRequest * request,
  ::grpc::ServerWriter<::grpcapi::AFTResponse> * writer)
{
  if (motion_ptr_ == nullptr) {
    node_for_audio_ = rclcpp::Node::make_shared("cyberdog_aft_stage_test1");
    speech_volume_set_client_ =
      node_for_audio_->create_client<::protocol::srv::AudioVolumeSet>("audio_volume_set");
    stage_test_finished_ = node_for_audio_->create_publisher<::protocol::msg::AudioPlay>(
      "speech_play", 10);  // speech_play_extend
    reset_audio_ = node_for_audio_->create_publisher<std_msgs::msg::Bool>(
      "audio_restore_settings", 10);
    motion_ptr_ = std::make_shared<ProtocolMotion>();
  }

  if (system_ptr_ == nullptr) {
    system_ptr_ = std::make_shared<ProtocolSystem>();
  }

  ::grpcapi::AFTResponse respond;
  rapidjson::Document json_resquest(rapidjson::kObjectType);
  rapidjson::Document json_response(rapidjson::kObjectType);
  std::string rsp_string;
  json_resquest.Parse<0>(request->params().c_str());

  if (json_resquest.HasParseError()) {
    ERROR("Parse Error");
    RetrunErrorGrpc(writer);
    return;
  }

  INFO("name_code: %d, %s", request->name_code(), request->params().c_str());
  switch (request->name_code()) {
    case ::grpcapi::AFTRequest::AFT_VERSION_REQUEST:
      respond.set_name_code(::grpcapi::AFTRequest::AFT_VERSION_RESPONSE);
      common::CyberdogJson::Add(json_response, "aft_version", system_ptr_->GetSystemVersion());
      common::CyberdogJson::Add(json_response, "serial_number", system_ptr_->GetSnInfo());

      INFO("request aft version");
      // json to string and response to 上位机 by grpc
      if (!common::CyberdogJson::Document2String(json_response, rsp_string)) {
        ERROR("error while encoding to json");
        RetrunErrorGrpc(writer);
        return;
      }

      respond.set_data(rsp_string);
      writer->Write(respond);
      break;

    case ::grpcapi::AFTRequest::AFT_BMS_REQUEST:

      respond.set_name_code(::grpcapi::AFTRequest::AFT_BMS_RESPONSE);
      // 电池内部电压
      common::CyberdogJson::Add(json_response, "batt_volt", system_ptr_->GetBmsVolt());     // NOLINT

      // 剩余电量
      common::CyberdogJson::Add(json_response, "batt_soc", system_ptr_->GetBmsSoc());     // NOLINT

      // bit 0  正常模式
      // bit 1  有线充电
      // bit 2  充电完成
      // bit 3  电机掉电
      common::CyberdogJson::Add(json_response, "batt_st", system_ptr_->GetBmsStatus());     // NOLINT

      INFO("request bms status");
      if (!common::CyberdogJson::Document2String(json_response, rsp_string)) {
        ERROR("error while encoding to json");
        RetrunErrorGrpc(writer);
        return;
      }

      respond.set_data(rsp_string);
      writer->Write(respond);
      break;

    case ::grpcapi::AFTRequest::AFT_MODULES_TEST_REQUEST:
      std::string module_name;
      std::string command_name;
      std::string command_time;
      // 从上位机下发的request（json格式）中 读取 “module” 并赋值给 std::string module_name
      common::CyberdogJson::Get(json_resquest, "module", module_name);
      common::CyberdogJson::Get(json_resquest, "command", command_name);
      respond.set_name_code(::grpcapi::AFTRequest::AFT_MODULES_TEST_RESPONSE);

      std::stringstream ss;
      using std::chrono::system_clock;
      std::time_t tt = system_clock::to_time_t(system_clock::now());
      struct std::tm * ptm = std::localtime(&tt);
      ss << std::put_time(ptm, "%c");
      std::string str_time = ss.str();
      common::CyberdogJson::Add(json_response, "timestamp", str_time);

      INFO("module : %s", module_name.c_str());
      INFO("command_name : %s", command_name.c_str());
      bool cmd_ok = false;

      if (command_name == "kCommandStart") {
        if (module_name == "kMotion") {
          common::CyberdogJson::Add(json_response, "module", "kMotion");
          if (!motion_ptr_->RunDefaultActions()) {
            INFO("Execute default motion action error.");
          }
        } else if (module_name == "kSystem") {
          common::CyberdogJson::Add(json_response, "module", "kSystem");
        } else if (module_name == "kStaticStageOne") {
          if (test_status_ == TestStatus::Idle) {
            test_status_ = TestStatus::Busy;
            INFO("kStaticStageOne +========================+++++++++");
            while (1) {
              // 有线充电后才开始静态测试
              if (std::stoi(system_ptr_->GetBmsStatus()) == 3 ||
                std::stoi(system_ptr_->GetBmsStatus()) == 7 ||
                std::stoi(system_ptr_->GetBmsStatus()) == 10 ||
                std::stoi(system_ptr_->GetBmsStatus()) == 2)
              {
                system_ptr_->StartPlayWhiteNoise();
                std::shared_ptr<std::thread> play_noise_thread = std::make_shared<std::thread>(
                  [this]()
                  {
                    system_ptr_->PlayWhiteNoise(mic_volume_);
                  });
                play_noise_thread->detach();

                std::shared_ptr<std::thread> lamp_change_thread = std::make_shared<std::thread>(
                  [this]()
                  {
                    system_ptr_->LampChange();
                  });
                lamp_change_thread->detach();

                // stage one
                InitializeStaticTestsStageOne();
                cmd_ok = RunStaticTestStageOne();
                system_ptr_->StopPlayWhiteNoise();
                system_ptr_->LameStatusChange();
                std::this_thread::sleep_for(std::chrono::seconds(3));
                bool report_ok = GetStaticStageOneReport(grpc_report_one_);
                INFO("report_ok is..........:%d", report_ok);
                ReportResult2Rapidjson(grpc_report_one_, json_response);
                ClearStaticStageOneReport(grpc_report_one_);
                if (report_ok) {
                  common::CyberdogJson::Add(json_response, "module", "kStaticStageOne");
                } else {
                  cmd_ok = false;
                }
                test_status_ = TestStatus::Idle;
                INFO("Static test stage one finished.");
                break;
              } else {
                SpeckerBroadcast(31049);
              }
            }
          }
        } else if (module_name == "kStaticStageTwo") {
          if (test_status_ == TestStatus::Idle) {
            test_status_ = TestStatus::Busy;
            INFO("kStaticStageTwo +========================+++++++++");
            while (1) {
              // 有线充电后才开始静态测试
              if (std::stoi(system_ptr_->GetBmsStatus()) == 3 ||
                std::stoi(system_ptr_->GetBmsStatus()) == 7 ||
                std::stoi(system_ptr_->GetBmsStatus()) == 10 ||
                std::stoi(system_ptr_->GetBmsStatus()) == 2)
              {
                system_ptr_->StartPlayWhiteNoise();
                std::shared_ptr<std::thread> play_noise_thread = std::make_shared<std::thread>(
                  [this]()
                  {
                    system_ptr_->PlayWhiteNoise(mic_volume_);
                  });
                play_noise_thread->detach();

                system_ptr_->LameStatusChange();
                std::this_thread::sleep_for(std::chrono::seconds(3));
                std::shared_ptr<std::thread> lamp_change_thread = std::make_shared<std::thread>(
                  [this]()
                  {
                    system_ptr_->LampChange();
                  });
                lamp_change_thread->detach();

                // stage two
                InitializeStaticTestsStageTwo();
                cmd_ok = RunStaticTestStageTwo();
                system_ptr_->StopPlayWhiteNoise();
                system_ptr_->LameStatusChange();
                std::this_thread::sleep_for(std::chrono::seconds(3));
                GetStaticStageTwoReport(grpc_report_two_);
                ReportResult2Rapidjson(grpc_report_two_, json_response);
                ClearStaticStageTwoReport(grpc_report_two_);
                common::CyberdogJson::Add(json_response, "module", "kStaticStageTwo");
                // all tests finish, the status of the dog switch to "Idle"
                test_status_ = TestStatus::Idle;
                INFO("Static test stage two finished.");
                break;
              } else {
                // std::string wire_charge_not_satify = "连接电源线后自动开始静态测试";
                // SpeckerOnlineBroadcast(wire_charge_not_satify);
                SpeckerBroadcast(31049);
              }
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
            auto reset_audio_message = std_msgs::msg::Bool();
            reset_audio_message.data = true;
            reset_audio_->publish(reset_audio_message);
          }
        } else if (module_name == "kDynamic") {
          if (test_status_ == TestStatus::Idle) {
            test_status_ = TestStatus::Busy;
            INFO("module_name ====: %s", module_name.c_str());
            while (1) {
              // for debug: 5; for dev: 90
              if (std::stoi(system_ptr_->GetBmsSoc()) > 90) {
                // 播报语音 “电量充足，开始动态测试，请拔电源线”
                SpeckerBroadcast(31047);

                // std::string power_sufficient = "电量充足请拔电源线后开始动态测试";
                // SpeckerOnlineBroadcast(power_sufficient);
                // 7 represents wire charge and full charge
                while (std::stoi(system_ptr_->GetBmsStatus()) == 3 ||
                  std::stoi(system_ptr_->GetBmsStatus()) == 7 ||
                  std::stoi(system_ptr_->GetBmsStatus()) == 10 ||
                  std::stoi(system_ptr_->GetBmsStatus()) == 2)
                {
                  // std::string battery_not_satify = "当前为有线充电不满足动态测试请拔电源线";
                  // SpeckerOnlineBroadcast(battery_not_satify);
                  // "当前为有线充电不满足动态测试请拔电源线";
                  SpeckerBroadcast(31048);
                }
                // check the battery status again
                if (std::stoi(system_ptr_->GetBmsStatus()) != 3 &&
                  std::stoi(system_ptr_->GetBmsStatus()) != 7 &&
                  std::stoi(system_ptr_->GetBmsStatus()) != 10 &&
                  std::stoi(system_ptr_->GetBmsStatus()) != 2)
                {
                  system_ptr_->StartPlayWhiteNoise();
                  std::shared_ptr<std::thread> play_noise_thread = std::make_shared<std::thread>(
                    [this]()
                    {
                      system_ptr_->PlayWhiteNoise(mic_volume_);
                    });
                  play_noise_thread->detach();

                  system_ptr_->LameStatusChange();
                  std::this_thread::sleep_for(std::chrono::seconds(3));
                  std::shared_ptr<std::thread> lamp_change_thread = std::make_shared<std::thread>(
                    [this]()
                    {
                      system_ptr_->LampChange();
                    });
                  lamp_change_thread->detach();

                  cmd_ok = RunDynamicTest();
                  INFO("Dynamic cmd_ok is:%d", cmd_ok);
                  system_ptr_->StopPlayWhiteNoise();
                  system_ptr_->LameStatusChange();
                  std::this_thread::sleep_for(std::chrono::seconds(3));
                  break;
                } else {
                  ERROR("Battery fail, status error");
                }
              } else {
                // 播报语音 “电量不足”（只播放一次）
                if (!insuffcient_power_broadcast_) {
                  // std::string keep_charge = "电量不足请继续充电";
                  // SpeckerOnlineBroadcast(keep_charge);
                  SpeckerBroadcast(31040);
                  insuffcient_power_broadcast_ = true;
                }
              }
              std::this_thread::sleep_for(std::chrono::seconds(9));
            }
            test_status_ = TestStatus::Idle;
          }
          common::CyberdogJson::Add(json_response, "module", "kDynamic");
          std::this_thread::sleep_for(std::chrono::seconds(3));
          INFO("Dynamic test stage finished.");
          // cmd_ok = true;
        }

        if (!cmd_ok) {
          INFO("Run test error.");
          common::CyberdogJson::Add(json_response, "status", "kError");
          common::CyberdogJson::Add(json_response, "result", false);
        } else {
          INFO("Run test success.");
          common::CyberdogJson::Add(json_response, "status", "kSuccess");
          common::CyberdogJson::Add(json_response, "result", true);
        }
      } else if (command_name == "kCommandQuery") {
        common::CyberdogJson::Add(json_response, "command", "kCommandQuery");

        if (module_name == "kStaticStageOne") {
          common::CyberdogJson::Add(json_response, "module", "kStaticStageOne");

          ProtocolStaticState running_state;
          bool state_ok = GetStaticTestsStageOneRunningState();
          if (state_ok) {
            common::CyberdogJson::Add(json_response, "status", "kSuccess");
            common::CyberdogJson::Add(json_response, "result", true);
          } else {
            common::CyberdogJson::Add(json_response, "status", "kSuccess");
            common::CyberdogJson::Add(json_response, "result", true);
          }
        } else if (module_name == "kStaticStageTwo") {
          common::CyberdogJson::Add(json_response, "module", "kStaticStageTwo");

          ProtocolStaticState running_state;
          bool state_ok = GetStaticTestsStageTwoRunningState();
          if (state_ok) {
            common::CyberdogJson::Add(json_response, "status", "kSuccess");
            common::CyberdogJson::Add(json_response, "result", true);
          }
        }
      } else if (command_name == "kCommandReport") {
        bool report_ok = false;
        if (module_name == "kStaticStageOne") {
          StaticStageOneReportResult report;
          INFO("Get static stage tests report.");
          // report_ok = GetStaticStageOneReport(grpc_report_one_);
          report_ok = GetStaticStageOneReport(report);
          common::CyberdogJson::Add(json_response, "module", "kStaticStageOne");
          // ReportResult2Rapidjson(grpc_report_one_, json_response);
          ReportResult2Rapidjson(report, json_response);
          // print message
          // DebugToString(grpc_report_one_);
          DebugToString(report);
          // bool report_clear = ClearStaticStageOneReport(grpc_report_one_);
        }
        if (module_name == "kStaticStageTwo") {
          report_ok = false;
          if (module_name == "kStaticStageTwo") {
            INFO("Get static stage two report.");
            common::CyberdogJson::Add(json_response, "module", "kStaticStageTwo");
            StaticStageTwoReportResult report;
            // report_ok = GetStaticStageTwoReport(grpc_report_two_);
            report_ok = GetStaticStageTwoReport(report);
            // ReportResult2Rapidjson(grpc_report_two_, json_response);
            ReportResult2Rapidjson(report, json_response);
            // print message
            // DebugToString(grpc_report_two_);
            DebugToString(report);
          }
          // bool report_clear = ClearStaticStageTwoReport(grpc_report_two_);
        } else if (module_name == "kDynamic") {
          INFO("Get Dynamic Report.");
          DynamicReportResult report;
          report_ok = GetDynamicReport(report);
          ReportResult2Rapidjson(report, json_response);
          // print message
          DebugToString(report);
        }

        if (!report_ok) {
          common::CyberdogJson::Add(json_response, "status", "kFail");
          common::CyberdogJson::Add(json_response, "result", false);
        } else {
          common::CyberdogJson::Add(json_response, "status", "kSuccess");
          common::CyberdogJson::Add(json_response, "result", true);
        }
      }

      if (!common::CyberdogJson::Document2String(json_response, rsp_string)) {
        ERROR("error while encoding to json");
        RetrunErrorGrpc(writer);
        return;
      }

      INFO("..................................");
      INFO("rsp_string: %s", rsp_string.c_str());

      std::this_thread::sleep_for(std::chrono::seconds(3));

      respond.set_data(rsp_string);
      writer->Write(respond);
      INFO("grpc send complete");

      break;
  }
}

void ProtocolGrpc::Spin()
{
  rclcpp::spin(test_node_ptr_);
}

void ProtocolGrpc::RunPcServer(::protocol::srv::ConnectPc::Response::SharedPtr response)
{
  auto channel = grpc::CreateChannel(register_server_uri_, grpc::InsecureChannelCredentials());
  register_stub_ = std::make_shared<AFTClient>(channel);
  grpcapi::AFTRequest request;
  request.set_name_code(10001);
  request.set_params(sn_);
  bool connect = register_stub_->SendRequest(request);

  int connect_count = 0;
  while (!connect && rclcpp::ok()) {
    connect = register_stub_->SendRequest(request);
    connect_count++;
    if (connect) {
      response->success = true;
      INFO("Connect PC successfully");
      break;
    }

    INFO("Connect PC count: %d", connect_count);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  const std::string server_address = "0.0.0.0:60060";
  AFTServer service(server_address);
  service.SetRequesProcess(this);

  ::grpc::ServerBuilder builder;

  builder.AddListeningPort(server_address, ::grpc::InsecureServerCredentials());
  builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS, 1000);
  builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 1000);
  builder.AddChannelArgument(GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS, 500);
  builder.AddChannelArgument(GRPC_ARG_HTTP2_MIN_SENT_PING_INTERVAL_WITHOUT_DATA_MS, 1000);
  builder.RegisterService(&service);
  server_ = std::move(builder.BuildAndStart());
  INFO("Server listening on %s", server_address.c_str());
  server_->Wait();
  INFO("after wait");
}

void ProtocolGrpc::RunIpServer(::protocol::srv::ConnectIp::Response::SharedPtr response)
{
  auto channel = grpc::CreateChannel(register_server_uri_, grpc::InsecureChannelCredentials());
  register_stub_ = std::make_shared<AFTClient>(channel);
  grpcapi::AFTRequest request;
  request.set_name_code(10001);
  request.set_params(sn_);
  bool connect = register_stub_->SendRequest(request);

  int connect_count = 0;
  while (!connect && rclcpp::ok()) {
    connect = register_stub_->SendRequest(request);
    connect_count++;
    if (connect) {
      response->success = true;
      INFO("Connect PC successfully");
      break;
    }

    INFO("Connect PC count: %d", connect_count);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  const std::string server_address = "0.0.0.0:60060";
  AFTServer service(server_address);
  service.SetRequesProcess(this);

  ::grpc::ServerBuilder builder;

  builder.AddListeningPort(server_address, ::grpc::InsecureServerCredentials());
  builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS, 1000);
  builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 1000);
  builder.AddChannelArgument(GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS, 500);
  builder.AddChannelArgument(GRPC_ARG_HTTP2_MIN_SENT_PING_INTERVAL_WITHOUT_DATA_MS, 1000);
  builder.RegisterService(&service);
  server_ = std::move(builder.BuildAndStart());
  INFO("Server listening on %s", server_address.c_str());
  server_->Wait();
  INFO("after wait");
}

void ProtocolGrpc::RetrunErrorGrpc(::grpc::ServerWriter<::grpcapi::AFTResponse> * writer)
{
  ::grpcapi::AFTResponse grpc_respond;
  grpc_respond.set_data("ERROR");
  writer->Write(grpc_respond);
}

bool ProtocolGrpc::RunStaticTestStageOne()
{
  static_test_time_++;
  while (!static_states_stage_one_.command_items.empty()) {
    auto test_item = static_states_stage_one_.command_items.front();
    static_states_stage_one_.command_items.pop_front();

    if (test_item == "kBattery") {
      system_ptr_->RunCommand(ProtocolSystem::SystemType::kBattery);
    } else if (test_item == "kHardwareDevice") {
      INFO("Static test stage one start:");
      INFO("Hardware test start:");
      // initial all hardware tests and activate realsense and stereo_camera
      system_ptr_->SenorStatusIniti();
      system_ptr_->InitializeDefaultTestModules(ProtocolSystem::SystemType::kHardwareDevice);
      std::this_thread::sleep_for(std::chrono::seconds(120));
      INFO("query result...................");
      system_ptr_->RunDefaultCommand(ProtocolSystem::SystemType::kHardwareDevice);
      INFO("Battery test start:");
      int battery_soc = std::stoi(system_ptr_->GetBmsSoc());
      if (battery_soc != 100) {
        // debug:30 dev:300
        std::this_thread::sleep_for(std::chrono::seconds(charge_duration_));
        int charged_battery_soc = std::stoi(system_ptr_->GetBmsSoc());
        if (charged_battery_soc >= battery_soc) {
          INFO("   kBattery: Success");
          EmplaceSuccessBatteryTest();
        } else {
          INFO("   kBattery: Failed");
          EmplaceFailedBatteryTest();
        }
      } else {
        // debug:30 dev:300
        std::this_thread::sleep_for(std::chrono::seconds(charge_duration_));
        int charged_battery_soc = std::stoi(system_ptr_->GetBmsSoc());
        if (charged_battery_soc == battery_soc) {
          INFO("   kBattery: Success");
          EmplaceSuccessBatteryTest();
        } else {
          INFO("   kBattery: Failed");
          EmplaceFailedBatteryTest();
        }
      }
    } else if (test_item == "kStorage") {
      INFO("CPU test start, stop the fan");
      system_ptr_->InitializeDefaultTestModules(ProtocolSystem::SystemType::kStorage);
      cpu_temp_ = system_ptr_->GetCpuTemp();
      INFO("The temp of the CPU before testing:%d", (cpu_temp_ / 1000));
      system_ptr_->TurnOffFanMoniterTemp();
      // cpu test(5min) start
      system_ptr_->CpuTest();
      cpu_temp_ = system_ptr_->GetCpuTemp();
      INFO("The temp of the CPU after testing:%d", (cpu_temp_ / 1000));
      // reactivate the fan and record the temp of cpu
      INFO("The fan is activated and monitor the temp of CPU");
      system_ptr_->TurnOnFanMoniterTemp();
      // dev:69000 debug:52000
      while (cpu_temp_ >= CPU_temperature_threhold_) {
        cpu_temp_ = system_ptr_->GetCpuTemp();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        INFO("Current temperature of cpu is:%d", (cpu_temp_ / 1000));
      }
      // fan test success
      system_ptr_->FanTestSuccess();
      INFO("Storage test start:");
      system_ptr_->RunDefaultCommand(ProtocolSystem::SystemType::kStorage);
    }
  }

  INFO("Send speech info to mention stage one is finished.");
  // send speech info to mention stage one is finished
  auto set_volume_request = std::make_shared<::protocol::srv::AudioVolumeSet::Request>();
  set_volume_request->volume = mic_volume_;
  while (!speech_volume_set_client_->wait_for_service(std::chrono::seconds(5))) {
    ERROR("Specker Error, static test stage one get 'set specker volume' service' fail");
  }
  auto result = speech_volume_set_client_->async_send_request(set_volume_request);

  if (rclcpp::spin_until_future_complete(
      node_for_audio_,
      result) == rclcpp::FutureReturnCode::SUCCESS && static_test_time_ == 1)
  {
    SpeckerBroadcast(3001);
    INFO("Static stage finished.");
  } else {
    SpeckerBroadcast(3003);
    INFO("Static stages finished.");
    std::this_thread::sleep_for(std::chrono::seconds(3));
    auto reset_audio_message = std_msgs::msg::Bool();
    reset_audio_message.data = true;
    reset_audio_->publish(reset_audio_message);
  }
  return true;
}

bool ProtocolGrpc::RunStaticTestStageTwo()
{
  while (!static_states_stage_two_.command_items.empty()) {
    auto test_item = static_states_stage_two_.command_items.front();
    static_states_stage_two_.command_items.pop_front();

    if (test_item == "kBattery") {
      system_ptr_->RunCommand(ProtocolSystem::SystemType::kBattery);
    } else if (test_item == "kHardwareDevice") {
      INFO("Static test stage two start:");
      INFO("Hardware test start:");
      // initial all hardware tests and activate realsense and stereo_camera
      system_ptr_->InitializeDefaultTestModules(ProtocolSystem::SystemType::kHardwareDevice);
      std::this_thread::sleep_for(std::chrono::seconds(55));
      INFO("query result...................");
      system_ptr_->RunDefaultCommand(ProtocolSystem::SystemType::kHardwareDevice);
      INFO("Battery test start:");
      int battery_soc = std::stoi(system_ptr_->GetBmsSoc());
      if (battery_soc != 100) {
        // debug:30 dev:300
        std::this_thread::sleep_for(std::chrono::seconds(charge_duration_));
        int charged_battery_soc = std::stoi(system_ptr_->GetBmsSoc());
        if (charged_battery_soc > battery_soc) {
          INFO("   kBattery: Success");
          EmplaceSuccessBatteryTest();
        } else {
          INFO("   kBattery: Failed");
          EmplaceFailedBatteryTest();
        }
      } else {
        // debug:30 dev:300
        std::this_thread::sleep_for(std::chrono::seconds(charge_duration_));
        int charged_battery_soc = std::stoi(system_ptr_->GetBmsSoc());
        if (charged_battery_soc == battery_soc) {
          INFO("   kBattery: Success");
          EmplaceSuccessBatteryTest();
        } else {
          INFO("   kBattery: Failed");
          EmplaceFailedBatteryTest();
        }
      }
    } else if (test_item == "kStorage") {
      INFO("CPU test start, stop the fan");
      system_ptr_->InitializeDefaultTestModules(ProtocolSystem::SystemType::kStorage);
      cpu_temp_ = system_ptr_->GetCpuTemp();
      INFO("The temp of the CPU before testing:%d", (cpu_temp_ / 1000));
      system_ptr_->TurnOffFanMoniterTemp();
      // cpu test(5min) start
      system_ptr_->CpuTest();
      cpu_temp_ = system_ptr_->GetCpuTemp();
      INFO("The temp of the CPU after testing:%d", (cpu_temp_ / 1000));
      // reactivate the fan and record the temp of cpu
      INFO("The fan is activated and monitor the temp of CPU");
      system_ptr_->TurnOnFanMoniterTemp();
      // dev:69000 debug:52000
      while (cpu_temp_ >= CPU_temperature_threhold_) {
        cpu_temp_ = system_ptr_->GetCpuTemp();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        INFO("Current temp of cpu is:%d", (cpu_temp_ / 1000));
      }
      // fan test success
      system_ptr_->FanTestSuccess();
      INFO("Storage test start:");
      system_ptr_->RunDefaultCommand(ProtocolSystem::SystemType::kStorage);
    }
  }
  // send speech info to mention stage two is finished
  auto set_volume_request = std::make_shared<::protocol::srv::AudioVolumeSet::Request>();
  set_volume_request->volume = mic_volume_;
  while (!speech_volume_set_client_->wait_for_service(std::chrono::seconds(5))) {
    ERROR("Specker Error, static test stage two get 'set specker volume' service' fail");
  }
  auto result = speech_volume_set_client_->async_send_request(set_volume_request);

  if (rclcpp::spin_until_future_complete(
      node_for_audio_,
      result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    SpeckerBroadcast(31039);
    INFO("Static stage two is finished.");
  } else {
    ERROR("Specker Error, static test stage one 'set specker volume' fail");
  }
  return true;
}

bool ProtocolGrpc::RunDynamicTest()
{
  // check battery soc and motors's temp during entire dynamic test
  return motion_ptr_->RunDefaultActions();
}

bool ProtocolGrpc::GetStaticStageOneReport(StaticStageOneReportResult & report)
{
  return system_ptr_->GetTestReportResult(
    ProtocolSystemType::kBattery,
    report.battery) && system_ptr_->GetTestReportResult(
    ProtocolSystemType::kHardwareDevice,
    report.hardware) &&
         system_ptr_->GetTestReportResult(ProtocolSystemType::kStorage, report.storage);
}

bool ProtocolGrpc::ClearStaticStageOneReport(StaticStageOneReportResult & report)
{
  return system_ptr_->ClearStaticStageOneReport(
    ProtocolSystemType::kHardwareDevice,
    report.hardware) &&
         system_ptr_->ClearStaticStageOneReport(ProtocolSystemType::kStorage, report.storage);
}

bool ProtocolGrpc::GetStaticStageTwoReport(StaticStageTwoReportResult & report)
{
  return system_ptr_->GetTestReportResult(ProtocolSystemType::kHardwareDevice, report.hardware) &&
         system_ptr_->GetTestReportResult(ProtocolSystemType::kStorage, report.storage);
}

bool ProtocolGrpc::ClearStaticStageTwoReport(StaticStageTwoReportResult & report)
{
  return system_ptr_->ClearStaticStageTwoReport(
    ProtocolSystemType::kHardwareDevice,
    report.hardware) &&
         system_ptr_->ClearStaticStageTwoReport(ProtocolSystemType::kStorage, report.storage);
}

bool ProtocolGrpc::GetDynamicReport(DynamicReportResult & report)
{
  return motion_ptr_->GetTestReportResult(report);
}

bool ProtocolGrpc::GetStaticTestsStageOneRunningState()
{
  return static_states_stage_one_.command_items.empty() ? true : false;
}

bool ProtocolGrpc::GetStaticTestsStageTwoRunningState()
{
  return static_states_stage_two_.command_items.empty() ? true : false;
}

bool ProtocolGrpc::GetDynamicTestsRunningState()
{
  return dynamic_states_.command_items.empty() ? true : false;
}

bool ProtocolGrpc::InitializeStaticTestsStageOne()
{
  // const std::string kTestModuleBattery = "kBattery";
  const std::string kTestModuleHardware = "kHardwareDevice";
  const std::string kTestModuleStorage = "kStorage";

  // static_states_stage_one_.command_items.push_back(kTestModuleBattery);
  static_states_stage_one_.command_items.push_back(kTestModuleHardware);
  static_states_stage_one_.command_items.push_back(kTestModuleStorage);

  return true;
}

bool ProtocolGrpc::InitializeStaticTestsStageTwo()
{
  const std::string kTestModuleHardware = "kHardwareDevice";
  const std::string kTestModuleStorage = "kStorage";

  static_states_stage_two_.command_items.push_back(kTestModuleHardware);
  static_states_stage_two_.command_items.push_back(kTestModuleStorage);
  return true;
}

bool ProtocolGrpc::InitializeDynamicTests()
{
  dynamic_states_.total_modules =
  {
    "kReturnStandUp",
    "kSelfFrequencyWalking",
    "kGoFast",
    "kQuadrupedJump",
    "kJumpBackAndForth",
    "kHighDampingDown"};

  for (const auto & items : dynamic_states_.total_modules) {
    dynamic_states_.command_items.push_back(items);
  }
  return true;
}

void ProtocolGrpc::EmplaceSuccessBatteryTest()
{
  system_ptr_->EmplaceSuccessBatteryTest();
}

void ProtocolGrpc::EmplaceFailedBatteryTest()
{
  system_ptr_->EmplaceFailedBatteryTest();
}

void ProtocolGrpc::DebugToString(const ReportResult & report)
{
  INFO("[total_modules]: module number is %ld", report.total_modules.size());
  for (auto m : report.total_modules) {
    INFO("  %s", m.c_str());
  }

  INFO("[success_results]: module number is %ld", report.success_results.size());
  for (auto & module : report.success_results) {
    INFO("  %s: %s", module.first.c_str(), module.second.c_str());
  }

  INFO("[failure_results]: module number is %ld", report.failure_results.size());
  for (auto & module : report.failure_results) {
    INFO("  %s: %s", module.first.c_str(), module.second.c_str());
  }
}

void ProtocolGrpc::DebugToString(const StaticStageOneReportResult & report)
{
  INFO("[hardware]");
  DebugToString(report.hardware);

  INFO("[storage]");
  DebugToString(report.storage);
}

void ProtocolGrpc::DebugToString(const StaticStageTwoReportResult & report)
{
  INFO("[hardware]");
  DebugToString(report.hardware);

  INFO("[storage]");
  DebugToString(report.storage);
}

void ProtocolGrpc::DebugToString(const DynamicReportResult & report)
{
  INFO("[motion response error code]");
  for (auto response_code_pair : report.motion.motion_response) {
    INFO("  %s: %s", response_code_pair.first.c_str(), response_code_pair.second.c_str());
  }

  INFO("[motion response overheating temperature]");
  for (auto response_temperature_pair : report.motion.motion_response) {
    INFO(
      "  %s: %s", response_temperature_pair.first.c_str(),
      response_temperature_pair.second.c_str());
  }
}

// void ProtocolGrpc::HandleConnectorStatus(const ::protocol::msg::ConnectorStatus::SharedPtr msg)
// {
//   const std::string toml_file = ament_index_cpp::get_package_share_directory(
//     "cyberdog_aft") + "/config/aft_register_uri.toml";

//   if (RegisterParaInitFromToml(toml_file)) {
//     if (msg->is_connected && msg->is_internet && msg->ssid == wifi_name_ && self_check_pass_)
//     {
//       INFO("Start factory auto test for robot connect PC Server.");
//       if (server_thread_ptr_ == nullptr) {
//         INFO("Create RunServer() function handle connect PC.");
//         server_thread_ptr_ = std::make_shared<std::thread>(
//           &ProtocolGrpc::RunServer, this, response);
//       }
//     }
//   }
// }

// 先执行联网操作，等待自检完成，执行py脚本 客户端发送 request
void ProtocolGrpc::HandleConnectPc(
  ::protocol::srv::ConnectPc::Request::SharedPtr request,
  ::protocol::srv::ConnectPc::Response::SharedPtr response)
{
  switch (request->connect_which) {
    case 1:
      register_server_uri_ = "192.168.31.218:50053";
      if (server_thread_ptr_ == nullptr) {
        INFO("Connect pc by pc id, ip address: %s", register_server_uri_.c_str());
        INFO("Create RunServer() function handle connect PC.");
        server_thread_ptr_ = std::make_shared<std::thread>(
          &ProtocolGrpc::RunPcServer, this,
          response);
        response->success = true;
      }
      break;

    case 2:
      register_server_uri_ = "192.168.31.254:50053";
      if (server_thread_ptr_ == nullptr) {
        INFO("Connect pc by pc id, ip address: %s", register_server_uri_.c_str());
        INFO("Create RunServer() function handle connect PC.");
        server_thread_ptr_ = std::make_shared<std::thread>(
          &ProtocolGrpc::RunPcServer, this,
          response);
        response->success = true;
      }
      break;

    case 3:
      register_server_uri_ = "192.168.31.68:50053";
      if (server_thread_ptr_ == nullptr) {
        INFO("Connect pc by pc id, ip address: %s", register_server_uri_.c_str());
        INFO("Create RunServer() function handle connect PC.");
        server_thread_ptr_ = std::make_shared<std::thread>(
          &ProtocolGrpc::RunPcServer, this,
          response);
        response->success = true;
      }
      break;

    default:
      break;
  }
}

void ProtocolGrpc::HandleConnectIp(
  ::protocol::srv::ConnectIp::Request::SharedPtr request,
  ::protocol::srv::ConnectIp::Response::SharedPtr response)
{
  register_server_uri_ = request->ip_address;
  if (server_thread_ptr_ == nullptr) {
    INFO("Connect pc by pc ip, ip address: %s", register_server_uri_.c_str());
    INFO("Create RunServer() function handle connect PC.");
    server_thread_ptr_ = std::make_shared<std::thread>(&ProtocolGrpc::RunIpServer, this, response);
    response->success = true;
  }
}

void ProtocolGrpc::HandleSelfCheckStatus(const ::protocol::msg::SelfCheckStatus::SharedPtr msg)
{
  if (msg->code == 0) {
    self_check_pass_ = true;
  }
}

bool ProtocolGrpc::RegisterParaInitFromToml(const std::string & config_file)
{
  toml::value config;
  if (!common::CyberdogToml::ParseFile(config_file, config)) {
    ERROR("Toml error, Cannot parse %s", config_file.c_str());
    return false;
  }
  toml::value value;
  bool get_value = common::CyberdogToml::Get(config, "address", value);
  bool get_address = common::CyberdogToml::Get(value, "register_address", register_server_uri_);
  bool get_wifi_name = common::CyberdogToml::Get(value, "wifi_name", wifi_name_);
  return get_value && get_address && get_wifi_name;
}

bool ProtocolGrpc::StaticTestParaInitFromToml(const std::string & config_file)
{
  toml::value config;
  if (!common::CyberdogToml::ParseFile(config_file, config)) {
    ERROR("Toml error, Cannot parse %s", config_file.c_str());
    return false;
  }

  toml::value others_value;
  bool get_others_value = common::CyberdogToml::Get(config, "others", others_value);
  bool get_charge_duration = common::CyberdogToml::Get(
    others_value, "charge_duration",
    charge_duration_);
  bool get_CPU_temperature_threhold = common::CyberdogToml::Get(
    others_value,
    "CPU_temperature_threhold",
    CPU_temperature_threhold_);
  bool get_mic_volume = common::CyberdogToml::Get(others_value, "mic_volume", mic_volume_);

  return get_others_value && get_charge_duration && get_CPU_temperature_threhold && get_mic_volume;
}

void ProtocolGrpc::HandleStageTest(const std_msgs::msg::Int32::SharedPtr msg)
{
  INFO("Handle receive command: %d", msg->data);
  constexpr int kStaticStage_1 = 1;
  constexpr int kStaticStage_2 = 2;
  constexpr int kDynamicStage = 3;
  constexpr int kAllTest = 5;
  // constexpr int kStaticStage_1_Report = 5;
  if (!construct_tests_ptr_) {
    node_for_audio_ = rclcpp::Node::make_shared("cyberdog_aft_stage_test1");
    speech_volume_set_client_ =
      node_for_audio_->create_client<::protocol::srv::AudioVolumeSet>("audio_volume_set");
    stage_test_finished_ = node_for_audio_->create_publisher<::protocol::msg::AudioPlay>(
      "speech_play", 10);
    reset_audio_ = node_for_audio_->create_publisher<std_msgs::msg::Bool>(
      "audio_restore_settings", 10);
    motion_ptr_ = std::make_shared<ProtocolMotion>();
    system_ptr_ = std::make_shared<ProtocolSystem>();
    construct_tests_ptr_ = true;
  }

  if (msg->data == kStaticStage_1) {
    test_status_ = TestStatus::Busy;
    // 电池内部电压
    INFO("BmsVolt is %s", system_ptr_->GetBmsVolt().c_str());      // NOLINT

    // 剩余电量
    INFO("BmsSoc is %s", system_ptr_->GetBmsSoc().c_str());      // NOLINT

    // bit 0  正常模式
    // bit 1  有线充电
    // bit 2  充电完成
    // bit 3  电机掉电
    INFO("BmsStatus is %s", system_ptr_->GetBmsStatus().c_str());      // NOLINT
    system_ptr_->StartPlayWhiteNoise();
    std::shared_ptr<std::thread> play_noise_thread = std::make_shared<std::thread>(
      [this]()
      {
        system_ptr_->PlayWhiteNoise(mic_volume_);
      });
    play_noise_thread->detach();

    std::shared_ptr<std::thread> lamp_change_thread = std::make_shared<std::thread>(
      [this]()
      {
        system_ptr_->LampChange();
      });
    lamp_change_thread->detach();

    InitializeStaticTestsStageOne();
    RunStaticTestStageOne();
    system_ptr_->StopPlayWhiteNoise();
    system_ptr_->LameStatusChange();
    StaticStageOneReportResult report;
    GetStaticStageOneReport(report);
    ClearStaticStageOneReport(report);
    std::this_thread::sleep_for(std::chrono::seconds(3));
  } else if (msg->data == kStaticStage_2) {
    while (1) {
      // 有线充电后才开始第三阶段测试
      if (std::stoi(system_ptr_->GetBmsStatus()) == 3 ||
        std::stoi(system_ptr_->GetBmsStatus()) == 7 ||
        std::stoi(system_ptr_->GetBmsStatus()) == 10 ||
        std::stoi(system_ptr_->GetBmsStatus()) == 2)
      {
        system_ptr_->StartPlayWhiteNoise();
        std::shared_ptr<std::thread> play_noise_thread = std::make_shared<std::thread>(
          [this]()
          {
            system_ptr_->PlayWhiteNoise(mic_volume_);
          });
        play_noise_thread->detach();

        system_ptr_->LameStatusChange();
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::shared_ptr<std::thread> lamp_change_thread = std::make_shared<std::thread>(
          [this]()
          {
            system_ptr_->LampChange();
          });
        lamp_change_thread->detach();

        InitializeStaticTestsStageTwo();
        RunStaticTestStageTwo();
        system_ptr_->StopPlayWhiteNoise();
        system_ptr_->LameStatusChange();
        StaticStageTwoReportResult report;
        GetStaticStageTwoReport(report);
        ClearStaticStageTwoReport(report);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        // all tests finish, the status of the dog switch to "Idle"
        test_status_ = TestStatus::Idle;
        break;
      } else {
        // std::string wire_charge_not_satify = "连接电源线后自动开始静态测试";
        // SpeckerOnlineBroadcast(wire_charge_not_satify);
        SpeckerBroadcast(31049);
      }
    }
    // all tests finish, the status of the dog switch to "Idle"
    test_status_ = TestStatus::Idle;
  } else if (msg->data == kDynamicStage) {
    // check battery status before dynamic test
    while (1) {
      // dev: 90   debug: 40
      if (std::stoi(system_ptr_->GetBmsSoc()) > 90) {
        // 播报语音 “电量充足，开始动态测试，请拔电源线”
        // std::string power_sufficient = "电量充足请拔电源线后开始动态测试";
        // SpeckerOnlineBroadcast(power_sufficient);
        SpeckerBroadcast(31047);

        // std::stoi(system_ptr_->GetBmsStatus());
        // keep check bms status util bms status is not 3 or 7(wire charge)
        // 电源不处于 有线充电模式(不等于3、7) 才开始动态测试
        while (std::stoi(system_ptr_->GetBmsStatus()) == 3 ||
          std::stoi(system_ptr_->GetBmsStatus()) == 7 ||
          std::stoi(system_ptr_->GetBmsStatus()) == 10 ||
          std::stoi(system_ptr_->GetBmsStatus()) == 2)
        {
          // std::string battery_not_satify = "当前为有线充电不满足动态测试请拔电源线";
          // SpeckerOnlineBroadcast(battery_not_satify);
          SpeckerBroadcast(31048);
        }
        // check the battery status again
        if (std::stoi(system_ptr_->GetBmsStatus()) != 3 &&
          std::stoi(system_ptr_->GetBmsStatus()) != 7 &&
          std::stoi(system_ptr_->GetBmsStatus()) != 10 &&
          std::stoi(system_ptr_->GetBmsStatus()) != 2)
        {
          system_ptr_->StartPlayWhiteNoise();
          std::shared_ptr<std::thread> play_noise_thread = std::make_shared<std::thread>(
            [this]()
            {
              system_ptr_->PlayWhiteNoise(mic_volume_);
            });
          play_noise_thread->detach();

          system_ptr_->LameStatusChange();
          std::this_thread::sleep_for(std::chrono::seconds(3));
          std::shared_ptr<std::thread> lamp_change_thread = std::make_shared<std::thread>(
            [this]()
            {
              system_ptr_->LampChange();
            });
          lamp_change_thread->detach();

          RunDynamicTest();
          system_ptr_->StopPlayWhiteNoise();
          system_ptr_->LameStatusChange();
          std::this_thread::sleep_for(std::chrono::seconds(3));
          break;
        } else {
          ERROR("Battery fail, status error");
        }
      } else {
        // 播报语音 “电量不足”（只播放一次）
        if (!insuffcient_power_broadcast_) {
          // std::string keep_charge = "电量不足请继续充电";
          // SpeckerOnlineBroadcast(keep_charge);
          SpeckerBroadcast(31040);
          insuffcient_power_broadcast_ = true;
        }
      }
      std::this_thread::sleep_for(std::chrono::seconds(9));
    }
    // change the status of "keep broadcast" to stop broadcast white noise
    // system_ptr_->StopPlayWhiteNoise();
  } else if (msg->data == kAllTest) {
    {
      test_status_ = TestStatus::Busy;
      std::shared_ptr<std::thread> play_noise_thread = std::make_shared<std::thread>(
        [this]()
        {
          system_ptr_->PlayWhiteNoise(mic_volume_);
        });
      play_noise_thread->detach();

      std::shared_ptr<std::thread> lamp_change_thread = std::make_shared<std::thread>(
        [this]()
        {
          system_ptr_->LampChange();
        });
      lamp_change_thread->detach();

      InitializeStaticTestsStageOne();
      RunStaticTestStageOne();
      system_ptr_->StopPlayWhiteNoise();
      system_ptr_->LameStatusChange();
      std::this_thread::sleep_for(std::chrono::seconds(3));
      StaticStageOneReportResult report;
      GetStaticStageOneReport(report);
      ClearStaticStageOneReport(report);
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    {
      while (1) {
        if (std::stoi(system_ptr_->GetBmsSoc()) > 90) {
          // std::string power_sufficient = "电量充足请拔电源线后开始动态测试";
          // SpeckerOnlineBroadcast(power_sufficient);
          SpeckerBroadcast(31047);
          while (std::stoi(system_ptr_->GetBmsStatus()) == 3 ||
            std::stoi(system_ptr_->GetBmsStatus()) == 7 ||
            std::stoi(system_ptr_->GetBmsStatus()) == 10 ||
            std::stoi(system_ptr_->GetBmsStatus()) == 2)
          {
            // std::string battery_not_satify = "当前为有线充电不满足动态测试请拔电源线";
            // SpeckerOnlineBroadcast(battery_not_satify);
            SpeckerBroadcast(31048);
          }
          if (std::stoi(system_ptr_->GetBmsStatus()) != 3 &&
            std::stoi(system_ptr_->GetBmsStatus()) != 7 &&
            std::stoi(system_ptr_->GetBmsStatus()) != 10 &&
            std::stoi(system_ptr_->GetBmsStatus()) != 2)
          {
            system_ptr_->StartPlayWhiteNoise();
            std::shared_ptr<std::thread> play_noise_thread = std::make_shared<std::thread>(
              [this]()
              {
                system_ptr_->PlayWhiteNoise(mic_volume_);
              });
            play_noise_thread->detach();

            system_ptr_->LameStatusChange();
            std::shared_ptr<std::thread> lamp_change_thread = std::make_shared<std::thread>(
              [this]()
              {
                system_ptr_->LampChange();
              });
            lamp_change_thread->detach();

            RunDynamicTest();
            INFO("Dynamic test finish");
            system_ptr_->StopPlayWhiteNoise();
            system_ptr_->LameStatusChange();
            std::this_thread::sleep_for(std::chrono::seconds(3));
            break;
          } else {
            ERROR("Battery fail, status error");
          }
        } else {
          if (!insuffcient_power_broadcast_) {
            // std::string keep_charge = "电量不足请继续充电";
            // SpeckerOnlineBroadcast(keep_charge);
            SpeckerBroadcast(31040);
            insuffcient_power_broadcast_ = true;
          }
        }
        std::this_thread::sleep_for(std::chrono::seconds(9));
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    {
      while (1) {
        // 有线充电后才开始第三阶段测试
        if (std::stoi(system_ptr_->GetBmsStatus()) == 3 ||
          std::stoi(system_ptr_->GetBmsStatus()) == 7 ||
          std::stoi(system_ptr_->GetBmsStatus()) == 10 ||
          std::stoi(system_ptr_->GetBmsStatus()) == 2)
        {
          system_ptr_->StartPlayWhiteNoise();
          std::shared_ptr<std::thread> play_noise_thread = std::make_shared<std::thread>(
            [this]()
            {
              system_ptr_->PlayWhiteNoise(mic_volume_);
            });
          play_noise_thread->detach();

          system_ptr_->LameStatusChange();
          std::shared_ptr<std::thread> lamp_change_thread = std::make_shared<std::thread>(
            [this]()
            {
              system_ptr_->LampChange();
            });
          lamp_change_thread->detach();

          InitializeStaticTestsStageTwo();
          RunStaticTestStageTwo();
          system_ptr_->StopPlayWhiteNoise();
          system_ptr_->LameStatusChange();
          std::this_thread::sleep_for(std::chrono::seconds(3));
          StaticStageTwoReportResult report;
          GetStaticStageTwoReport(report);
          ClearStaticStageTwoReport(report);
          // all tests finish, the status of the dog switch to "Idle"
          test_status_ = TestStatus::Idle;
          break;
        } else {
          // std::string wire_charge_not_satify = "连接电源线后将自动开始静态测试";
          // SpeckerOnlineBroadcast(wire_charge_not_satify);
          SpeckerBroadcast(31049);
        }
      }
    }
  }
  // std::this_thread::sleep_for(std::chrono::seconds(3));
  // auto reset_audio_message = std_msgs::msg::Bool();
  // reset_audio_message.data = true;
  // reset_audio_->publish(reset_audio_message);
}

bool ProtocolGrpc::SpeckerBroadcast(int ID)
{
  auto message = ::protocol::msg::AudioPlay();
  message.module_name = "test";
  message.play_id = ID;
  stage_test_finished_->publish(message);
  std::this_thread::sleep_for(std::chrono::seconds(5));
  return true;
}

void ProtocolGrpc::ReportResult2Rapidjson(
  const ReportResult & report,
  rapidjson::Document & json_response)
{
  INFO("Function ReportResult2Rapidjson().");
  // success_results
  rapidjson::Document::AllocatorType & allocator = json_response.GetAllocator();

  rapidjson::Document json_success_result(rapidjson::kObjectType);
  for (const auto &[key, value] : report.success_results) {
    common::CyberdogJson::Add(json_success_result, key, value);
  }
  common::CyberdogJson::Add(json_response, "success_result", json_success_result);

  // failure_results
  rapidjson::Document json_failure_result(rapidjson::kObjectType);
  for (const auto &[key, value] : report.failure_results) {
    common::CyberdogJson::Add(json_failure_result, key, value);
  }
  common::CyberdogJson::Add(json_response, "failure_result", json_failure_result);

  // total_modules
  rapidjson::Value json_total_modules(rapidjson::kArrayType);
  if (report.total_modules.size() >= 2) {
    for (int i = 0; i < static_cast<int>(report.total_modules.size() - 2); ++i) {
      json_total_modules.PushBack(rapidjson::StringRef(report.total_modules[i].c_str()), allocator);
    }
  } else {
    for (auto & total_module : report.total_modules) {
      json_total_modules.PushBack(rapidjson::StringRef(total_module.c_str()), allocator);
    }
  }

  common::CyberdogJson::Add(json_response, "total_modules", json_total_modules);
}

void ProtocolGrpc::ReportResult2Rapidjson(
  const StaticStageOneReportResult & report,
  rapidjson::Document & json_response)
{
  INFO("Convert static stage tests report result to json");

  rapidjson::Document battery_document(rapidjson::kObjectType);
  rapidjson::Document hardware_document(rapidjson::kObjectType);
  rapidjson::Document storage_document(rapidjson::kObjectType);

  // every module to json format
  ReportResult2Rapidjson(report.battery, battery_document);
  ReportResult2Rapidjson(report.hardware, hardware_document);
  ReportResult2Rapidjson(report.storage, storage_document);

  // all to json format
  common::CyberdogJson::Add(json_response, "battery", battery_document);
  common::CyberdogJson::Add(json_response, "hardware", hardware_document);
  common::CyberdogJson::Add(json_response, "storage", storage_document);
}

void ProtocolGrpc::ReportResult2Rapidjson(
  const StaticStageTwoReportResult & report,
  rapidjson::Document & json_response)
{
  INFO("Convert static stage test report result to json");
  rapidjson::Document hardware_document(rapidjson::kObjectType);
  rapidjson::Document storage_document(rapidjson::kObjectType);

  // every module to json format
  ReportResult2Rapidjson(report.hardware, hardware_document);
  ReportResult2Rapidjson(report.storage, storage_document);

  // all to json format

  common::CyberdogJson::Add(json_response, "hardware", hardware_document);
  common::CyberdogJson::Add(json_response, "storage", storage_document);
}

void ProtocolGrpc::ReportResult2Rapidjson(
  const DynamicReportResult & report,
  rapidjson::Document & json_response)
{
  INFO("Convert dynamic's report result to json");

  // motion module to json format()
  INFO("Dynamic ReportResult2Rapidjson().");

  rapidjson::Document json_motion_response(rapidjson::kObjectType);
  for (const auto &[key, value] : report.motion.motion_response) {
    common::CyberdogJson::Add(json_motion_response, key, value);
  }
  common::CyberdogJson::Add(json_response, "motion_code", json_motion_response);

  // motion temperature response
  rapidjson::Document json_temperature_response(rapidjson::kObjectType);
  for (const auto &[key, value] : report.motion.temperature_response) {
    common::CyberdogJson::Add(
      json_temperature_response, key, value);
  }
  common::CyberdogJson::Add(json_response, "motor_temperature", json_temperature_response);

  // all to json format
  common::CyberdogJson::Add(json_response, "code", json_motion_response);
  common::CyberdogJson::Add(json_response, "temperature", json_temperature_response);
}

}   // namespace protocol
}  // namespace cyberdog
