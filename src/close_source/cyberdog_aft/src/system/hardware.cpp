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
#include <vector>
#include <memory>
#include <utility>

#include "cyberdog_aft/system/hardware.hpp"
#include "cyberdog_aft/utils/shell_command.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace cyberdog
{
namespace system
{
namespace
{
template<typename MessageType>
::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
  void (Hardware::* handler)(typename MessageType::SharedPtr),
  const std::string & topic,
  ::rclcpp::Node::SharedPtr node_handle, Hardware * const hardware, rclcpp::SubscriptionOptions options)
{
  // 之前为RELIABLE rclcpp::SensorDataQoS()
  return node_handle->create_subscription<MessageType>(
    topic, rclcpp::QoS(15),
    [hardware, handler, topic](typename MessageType::SharedPtr msg)
    {
      (hardware->*handler)(msg);
    }, options);
}
}  // namespace

int Hardware::lidar_frame_count = 0;
int Hardware::lidar_dissatisfied_count = 0;

int Hardware::head_left_tof_frame_count = 0;
int Hardware::head_left_tof_dissatisfied_count = 0;

int Hardware::head_right_tof_frame_count = 0;
int Hardware::head_right_tof_dissatisfied_count = 0;

int Hardware::rear_left_tof_frame_count = 0;
int Hardware::rear_left_tof_dissatisfied_count = 0;

int Hardware::rear_right_tof_frame_count = 0;
int Hardware::rear_right_tof_dissatisfied_count = 0;

int Hardware::gps_frame_count = 0;
int Hardware::gps_dissatisfied_count = 0;

int Hardware::ultrasound_frame_count = 0;
int Hardware::ultrasound_dissatisfied_count = 0;

int Hardware::wifi_frame_count = 0;
int Hardware::wifi_dissatisfied_count = 0;

int Hardware::realsense_imu_frame_count = 0;
int Hardware::realsense_imu_dissatisfied_count = 0;

int Hardware::realsense_depth_frame_count = 0;
int Hardware::realsense_depth_dissatisfied_count = 0;

int Hardware::realsense_raw1_frame_count = 0;
int Hardware::realsense_raw1_dissatisfied_count = 0;

int Hardware::realsense_raw2_frame_count = 0;
int Hardware::realsense_raw2_dissatisfied_count = 0;

int Hardware::rgb_frame_count = 0;
int Hardware::rgb_dissatisfied_count = 0;

int Hardware::left_fish_frame_count = 0;
int Hardware::left_fish_dissatisfied_count = 0;

int Hardware::right_fish_frame_count = 0;
int Hardware::right_fish_dissatisfied_count = 0;

Hardware::Hardware()
{
  functions_.emplace(std::make_pair("kUnknow", []() {return true;}));
  functions_.emplace(std::make_pair("kTouch", std::bind(&Hardware::TestTouch, this)));
  functions_.emplace(std::make_pair("kLidar", std::bind(&Hardware::TestLidar, this)));
  functions_.emplace(std::make_pair("kHeadLeftTOF", std::bind(&Hardware::TestHeadLeftTOF, this)));
  functions_.emplace(std::make_pair("kHeadRightTOF", std::bind(&Hardware::TestHeadRightTOF, this)));
  functions_.emplace(std::make_pair("kRearLeftTOF", std::bind(&Hardware::TestRearLeftTOF, this)));
  functions_.emplace(std::make_pair("kRearRightTOF", std::bind(&Hardware::TestRearRightTOF, this)));
  functions_.emplace(std::make_pair("kUltrasound", std::bind(&Hardware::TestUltrasound, this)));
  functions_.emplace(std::make_pair("kGPS", std::bind(&Hardware::TestGPS, this)));
  functions_.emplace(std::make_pair("kWIFI", std::bind(&Hardware::TestWIFI, this)));
  functions_.emplace(std::make_pair("kFan", std::bind(&Hardware::TestFan, this)));
  functions_.emplace(std::make_pair("kAICamera", std::bind(&Hardware::TestAICamera, this)));
  functions_.emplace(std::make_pair("kRealSenseImu", std::bind(&Hardware::TestRealSense, this)));
  functions_.emplace(std::make_pair("kRealSenseDepth", std::bind(&Hardware::TestRealSense, this)));
  functions_.emplace(std::make_pair("kRealSenseRaw1", std::bind(&Hardware::TestRealSense, this)));
  functions_.emplace(std::make_pair("kRealSenseRaw2", std::bind(&Hardware::TestRealSense, this)));
  functions_.emplace(std::make_pair("kRGBCamera", std::bind(&Hardware::TestRGBCamera, this)));
  functions_.emplace(
    std::make_pair(
      "kLeftFisheyeCamera",
      std::bind(&Hardware::TestLeftFisheyeCamera, this)));
  functions_.emplace(
    std::make_pair(
      "kRightFisheyeCamera",
      std::bind(&Hardware::TestRightFisheyeCamera, this)));
}

Hardware::Hardware(std::shared_ptr<rclcpp::Node> node)
: node_ptr_(node)
{
  // Create Led client
  led_switch_client_ =
    node_ptr_->create_client<::protocol::srv::LedExecute>("led_execute");
  // subscribe all sensor topics and run these callback
  InitializeSensorSubscriptionTopics();
  
  // 从toml中读取可配置文件
  std::string toml_file = ament_index_cpp::get_package_share_directory(
    "cyberdog_aft") + "/config/static_para_setting.toml";
  InitializeTestParas(toml_file);

  functions_.clear();
  functions_.emplace(std::make_pair("kUnknow", []() {return true;}));
  functions_.emplace(std::make_pair("kTouch", std::bind(&Hardware::TestTouch, this)));
  functions_.emplace(std::make_pair("kLidar", std::bind(&Hardware::TestLidar, this)));
  functions_.emplace(std::make_pair("kHeadLeftTOF", std::bind(&Hardware::TestHeadLeftTOF, this)));
  functions_.emplace(std::make_pair("kHeadRightTOF", std::bind(&Hardware::TestHeadRightTOF, this)));
  functions_.emplace(std::make_pair("kRearLeftTOF", std::bind(&Hardware::TestRearLeftTOF, this)));
  functions_.emplace(std::make_pair("kRearRightTOF", std::bind(&Hardware::TestRearRightTOF, this)));
  functions_.emplace(std::make_pair("kUltrasound", std::bind(&Hardware::TestUltrasound, this)));
  functions_.emplace(std::make_pair("kGPS", std::bind(&Hardware::TestGPS, this)));
  functions_.emplace(std::make_pair("kWIFI", std::bind(&Hardware::TestWIFI, this)));
  functions_.emplace(std::make_pair("kFan", std::bind(&Hardware::TestFan, this)));
  functions_.emplace(std::make_pair("kAICamera", std::bind(&Hardware::TestAICamera, this)));
  
  functions_.emplace(std::make_pair("kRealSenseImu", std::bind(&Hardware::TestRealSense, this)));
  functions_.emplace(std::make_pair("kRealSenseDepth", std::bind(&Hardware::TestRealSense, this)));
  functions_.emplace(std::make_pair("kRealSenseRaw1", std::bind(&Hardware::TestRealSense, this)));
  functions_.emplace(std::make_pair("kRealSenseRaw2", std::bind(&Hardware::TestRealSense, this)));
  
  functions_.emplace(std::make_pair("kRGBCamera", std::bind(&Hardware::TestRGBCamera, this)));
  functions_.emplace(
    std::make_pair(
      "kLeftFisheyeCamera",
      std::bind(&Hardware::TestLeftFisheyeCamera, this)));
  functions_.emplace(
    std::make_pair(
      "kRightFisheyeCamera",
      std::bind(&Hardware::TestRightFisheyeCamera, this)));

  // Create ai camera sensor turn on and turn off client
  ai_camera_switch_client_ =
    node_ptr_->create_client<::protocol::srv::CameraService>("camera_service");
  
  // Create speech volume set client
  speech_volume_set_client_ =
    node_ptr_->create_client<::protocol::srv::AudioVolumeSet>("audio_volume_set");

  // Create speech play pub
  speech_play_publisher_ =
    node_ptr_->create_publisher<::protocol::msg::AudioPlayExtend>("speech_play_extend", 10);
  // set initialize test modules
  // InitializeTestModules();
}

Hardware::~Hardware()
{
}

bool Hardware::RunTest(const Type & hardware_type)
{
  return true;
}

bool Hardware::RunTest(const std::string& hardware_type)
{
  auto it = functions_.find(hardware_type);
  if (it == functions_.end()) {
    return false;
  }
  
  // get result
  bool cmd_ok = functions_[hardware_type]();
  running_state_.total_modules.emplace_back(hardware_type);
  // INFO("Run hardware module : %s", hardware_type.c_str());

  if (!cmd_ok) {
    running_state_.failure_results.emplace(hardware_type, "Failed");
  } else {
    running_state_.success_results.emplace(hardware_type, "Success");
  }

  return cmd_ok;
}

bool Hardware::InitializeTotalModuleTests(const std::vector<std::string> & modules)
{
  for (const auto & item : modules) {
    commands_queue_.push_back(item);
  }
  return true;
}

bool Hardware::GetReport(ReportResult & report)
{
  report = running_state_;
  if(running_state_.failure_results.size()!= 0){
    return false;
  } else{
  return true;
  }
}

bool Hardware::CheckAllFinished()
{
  return commands_queue_.empty();
}

bool Hardware::RunAllTest()
{
  while (!commands_queue_.empty()) {
    if (stoped_.load()) {
      break;
    }
    const std::string cmd = commands_queue_.front();
    commands_queue_.pop_front();
    INFO("Current test :%s", cmd.c_str());
    
    if(cmd == "kAICamera"){
    std::thread kAICamera_test(&Hardware::TestAICamera, this); 
    kAICamera_test.detach();
    // 阻塞180s(timeout)，若180s内没有收到 条件变量的 notify，std::cv_status返回timeout，当前测试项失败。
    std::unique_lock<std::mutex> lock_wait_AICamera_test(mutex_wait_);
    std::cv_status cvsts_AICamera_test = cond_wait_.wait_for(lock_wait_AICamera_test, std::chrono::seconds(timeout_));
    if (cvsts_AICamera_test == std::cv_status::timeout){
    //AI相机测试项直接返回失败
      kAICamera_valid_ = false;
    }
    lock_wait_AICamera_test.unlock();
   } else{
    RunTest(cmd);
   }
  }

  if (active_stereocamera_success_)
  ShutdownStereoCamera();
  INFO("Stereocamera shutdown success");

  std::this_thread::sleep_for(std::chrono::seconds(3));
  if (active_realsense_success_)
  ShutdownRealSense();
  INFO("Realsense shutdown success");
  return true;
}

std::string Hardware::ToString(const Type & type)
{
  std::string type_str = "";

  switch (type) {
    case Type::kTouch:
      type_str = "kTouch";
      break;

    case Type::kLidar:
      type_str = "kLidar";
      break;

    case Type::kHeadLeftTOF:
      type_str = "kHeadLeftTOF";
      break;
    
    case Type::kHeadRightTOF:
      type_str = "kHeadRightTOF";
      break;
    
    case Type::kRearLeftTOF:
      type_str = "kRearLeftTOF";
      break;
    
    case Type::kRearRightTOF:
      type_str = "kRearRightTOF";
      break;

    case Type::kUltrasound:
      type_str = "kUltrasound";
      break;

    case Type::kGPS:
      type_str = "kGPS";
      break;

    case Type::kWIFI:
      type_str = "kWIFI";
      break;

    case Type::kFan:
      type_str = "kFan";
      break;

    case Type::kAICamera:
      type_str = "kAICamera";
      break;

    case Type::kRGBCamera:
      type_str = "kRGBCamera";
      break;

    case Type::kRealSenseImu:
      type_str = "kRealSenseImu";
      break;
    
    case Type::kRealSenseDepth:
      type_str = "kRealSenseDepth";
      break;

    case Type::kRealSenseRaw1:
      type_str = "kRealSenseRaw1";
      break;
    
    case Type::kRealSenseRaw2:
      type_str = "kRealSenseRaw2";
      break;

    case Type::kLeftFisheyeCamera:
      type_str = "kLeftFisheyeCamera";
      break;

    case Type::kRightFisheyeCamera:
      type_str = "kRightFisheyeCamera";
      break;

    case Type::kMIC:
      type_str = "kMIC";
      break;

    case Type::kBattery:
      type_str = "kBattery";
      break;

    default:
      break;
  }
  return type_str;
}

Hardware::Type Hardware::FromString(const std::string & type)
{
  if (type == "kTouch") {
    INFO("kTouch");
    return Hardware::Type::kTouch;
  } else if (type == "kLidar") {
    INFO("kLidar");
    return Hardware::Type::kLidar;
  } else if (type == "kHeadLeftTOF") {
    INFO("kHeadLeftTOF");
    return Hardware::Type::kHeadLeftTOF;
  } else if (type == "kHeadRightTOF") {
    INFO("kHeadRightTOF");
    return Hardware::Type::kHeadRightTOF;
  } else if (type == "kRearLeftTOF") {
    INFO("kRearLeftTOF");
    return Hardware::Type::kRearLeftTOF;
  } else if (type == "kRearRightTOF") {
    INFO("kRearRightTOF");
    return Hardware::Type::kRearRightTOF;
  } else if (type == "kUltrasound") {
    INFO("kUltrasound");
    return Hardware::Type::kUltrasound;
  } else if (type == "kGPS") {
    return Hardware::Type::kGPS;
  } else if (type == "kWIFI") {
    return Hardware::Type::kWIFI;
  } else if (type == "kFan") {
    return Hardware::Type::kFan;
  } else if (type == "kAICamera") {
    return Hardware::Type::kAICamera;
  } else if (type == "kRGBCamera") {
    return Hardware::Type::kRGBCamera;
  } else if (type == "kRealSenseImu") {
    return Hardware::Type::kRealSenseImu;
  } else if (type == "kRealSenseDepth") {
    return Hardware::Type::kRealSenseDepth;
  } else if (type == "kRealSenseRaw1") {
    return Hardware::Type::kRealSenseRaw1;
  } else if (type == "kRealSenseRaw2") {
    return Hardware::Type::kRealSenseRaw2;
  } else if (type == "kLeftFisheyeCamera") {
    return Hardware::Type::kLeftFisheyeCamera;
  } else if (type == "kRightFisheyeCamera") {
    return Hardware::Type::kRightFisheyeCamera;
  } else if (type == "kMIC") {
    return Hardware::Type::kMIC;
  } 
  return Hardware::Type::kUnknow;
}

void Hardware::ReportToString(ReportResult & report)
{
  INFO("[total_modules]: module num = %ld", report.total_modules.size());
  for (auto & module : report.total_modules) {
    INFO("    %s", module.c_str());
  }

  INFO("[success_modules]: module num = %ld", report.success_results.size());
  for (auto & module : report.success_results) {
    INFO("    %s: %s", module.first.c_str(), module.second.c_str());
  }

  INFO("[failure_modules]: module num = %ld", report.failure_results.size());
  for (auto & module : report.failure_results) {
    INFO("    %s: %s", module.first.c_str(), module.second.c_str());
  }
}

bool Hardware::ClearReport(ReportResult & report)
{
  report.total_modules.clear();
  report.success_results.clear();
  report.failure_results.clear();
  return true;
}

bool Hardware::TestTouch()
{
  std::string test_touch =
    "factory-tool -f /usr/share/factory_cmd_config/system.xml -i 'Touch Checktest'|grep Failed";  // NOLINT
  bool test_touch_success;
  std::string return_turnOff_result = RunShellCommand(test_touch.c_str(), test_touch_success);

  if (return_turnOff_result.size() != 0) {
    kTouch_valid_ = false;
    return false;
  } else {
    kTouch_valid_ = true;
    return true;
  }
}

bool Hardware::TestLidar()
{
  return kLidar_valid_;
}

bool Hardware::TestHeadLeftTOF()
{
  return kHeadLeftTOF_valid_;
}

bool Hardware::TestHeadRightTOF()
{
  return kHeadRightTOF_valid_;
}

bool Hardware::TestRearLeftTOF()
{
  return kRearLeftTOF_valid_;
}

bool Hardware::TestRearRightTOF()
{
  return kRearRightTOF_valid_;
}

bool Hardware::TestGPS()
{
  return kGPS_valid_;
}

bool Hardware::TestUltrasound()
{
  return kUltrasound_valid_;
}

bool Hardware::TestWIFI()
{
  return kWIFI_valid_;
}

bool Hardware::TestFan()
{
  return true;
}

bool Hardware::TestBatterySuccess()
{
  return true;
}

bool Hardware::TestBatteryFailed()
{
  return false;
}

bool Hardware::TestTurnOffFan()
{
  std::string turn_off_fan =
    "factory-tool -f /usr/share/factory_cmd_config/system.xml -i  'Turn Off Fan'| grep successfully";  // NOLINT
  bool fan_turnOff_success = false;

  std::string return_turnOff_result = RunShellCommand(turn_off_fan.c_str(), fan_turnOff_success);

  if (return_turnOff_result.size() != 0) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    return true;
  }
  return false;
}

bool Hardware::TestTurnOnFan()
{
  std::string turn_on_fan =
    "factory-tool -f /usr/share/factory_cmd_config/system.xml -i  'Turn On Fan'| grep successfully";  // NOLINT
  bool fan_turnOn_success = false;

  std::string return_turnOff_result = RunShellCommand(turn_on_fan.c_str(), fan_turnOn_success);

  if (return_turnOff_result.size() != 0) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    return true;
  }
  return false;
}

bool Hardware::FanTestSuccess()
{
  functions_.emplace(std::make_pair("kFan", std::bind(&Hardware::TestFan, this)));
  kFan_valid_ = true;
  return true;
}

bool Hardware::PlayWhiteNoise(const int& volume)
{
  while (!speech_volume_set_client_->wait_for_service(3s)) {
      ERROR("Specker Error, cannot get specker service");
      kMIC_valid_ = false;
      return false;
  }

 int current_volume = volume;
 auto set_volume_request = std::make_shared<::protocol::srv::AudioVolumeSet::Request>();
 set_volume_request->volume = current_volume;
 auto result = speech_volume_set_client_->async_send_request(set_volume_request);
 std::this_thread::sleep_for(std::chrono::seconds(1));

 auto message = ::protocol::msg::AudioPlayExtend();
 message.module_name = "speech_test";
 message.is_online = false;
 message.speech.module_name = "test";
 message.speech.play_id = 3000;
 message.text = "no";

 auto response = result.get()->success;
 if (!response) {
    ERROR("Specker Error, volume set fail");
    kMIC_valid_ = false;
    return false;
 }

  while(keep_broadcast_){
    speech_play_publisher_->publish(message);
    std::this_thread::sleep_for(std::chrono::seconds(62));
  } 

  if (keep_broadcast_ == false){
    INFO("Pink noise stop broadcast");
  }
  return true;
}

void Hardware::MiniLampChange()
{
    // 脸灯 红
    auto face_lamp_request = std::make_shared<::protocol::srv::LedExecute::Request>();
    face_lamp_request->occupation = 1; 
    face_lamp_request->client = "lowpower";
    face_lamp_request->target = 3;                      
    face_lamp_request->mode   = 2;                          
    face_lamp_request->effect = 1;                          
    face_lamp_request-> r_value = 255;                       
    face_lamp_request-> g_value = 0;                        
    face_lamp_request-> b_value = 0;
    while (!led_switch_client_->wait_for_service(5s)) {
      ERROR("Face led test fails, cannot get face led service");
      return;
    }
    auto face_lamp_result = led_switch_client_->async_send_request(face_lamp_request);
    if (face_lamp_result.get()->code != 0){
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // 脸灯 绿
    face_lamp_request-> r_value = 0;                       
    face_lamp_request-> g_value = 255;                        
    face_lamp_request-> b_value = 0;
    while (!led_switch_client_->wait_for_service(5s)) {
      ERROR("Face led test fails, cannot get face led service");
      return;
    }
    face_lamp_result = led_switch_client_->async_send_request(face_lamp_request);
    if (face_lamp_result.get()->code != 0){
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // 脸灯 蓝
    face_lamp_request-> r_value = 0;                       
    face_lamp_request-> g_value = 0;                        
    face_lamp_request-> b_value = 255;
    while (!led_switch_client_->wait_for_service(5s)) {
      ERROR("Face led test fails, cannot get face led service");
      return;
    }
    face_lamp_result = led_switch_client_->async_send_request(face_lamp_request);
    if (face_lamp_result.get()->code != 0){
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

void Hardware::HeadLampChange()
{
    // 头灯 红
    auto head_lamp_request = std::make_shared<::protocol::srv::LedExecute::Request>();
    head_lamp_request->occupation = 1; 
    head_lamp_request->client = "lowpower";
    head_lamp_request->target = 1;                      
    head_lamp_request->mode   = 2;                          
    head_lamp_request->effect = 1;                          
    head_lamp_request-> r_value = 255;                       
    head_lamp_request-> g_value = 0;                        
    head_lamp_request-> b_value = 0;
    while (!led_switch_client_->wait_for_service(5s)) {
      ERROR("Head lamp test fails, Cannot get head led service");
      return;
    }
    auto head_lamp_result = led_switch_client_->async_send_request(head_lamp_request);
    if (head_lamp_result.get()->code != 0){
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // 头灯 绿
    head_lamp_request-> r_value = 0;                       
    head_lamp_request-> g_value = 255;                        
    head_lamp_request-> b_value = 0;
    while (!led_switch_client_->wait_for_service(5s)) {
      ERROR("Head lamp test fails, Cannot get head led service");
      return;
    }
    head_lamp_result = led_switch_client_->async_send_request(head_lamp_request);
    if (head_lamp_result.get()->code != 0){
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // 头灯 蓝
    head_lamp_request-> r_value = 0;                       
    head_lamp_request-> g_value = 0;                        
    head_lamp_request-> b_value = 255;
    while (!led_switch_client_->wait_for_service(5s)) {
      ERROR("Head lamp test fails, Cannot get head led service");
      return;
    }
    head_lamp_result = led_switch_client_->async_send_request(head_lamp_request);
    if (head_lamp_result.get()->code != 0){
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    return;
}

void Hardware::TailLampChange()
{
    // 头灯 红
    auto tail_lamp_request = std::make_shared<::protocol::srv::LedExecute::Request>();
    tail_lamp_request->occupation = 1; 
    tail_lamp_request->client = "lowpower";
    tail_lamp_request->target = 2;                      
    tail_lamp_request->mode   = 2;                          
    tail_lamp_request->effect = 1;
    // 头灯 绿                          
    tail_lamp_request-> r_value = 255;                       
    tail_lamp_request-> g_value = 0;                        
    tail_lamp_request-> b_value = 0;
    while (!led_switch_client_->wait_for_service(5s)) {
      ERROR("Tail lamp test fails, Cannot get tail led service");
      return;
    }
    auto tail_lamp_result = led_switch_client_->async_send_request(tail_lamp_request);
    if (tail_lamp_result.get()->code != 0){
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // 头灯 绿
    tail_lamp_request-> r_value = 0;                       
    tail_lamp_request-> g_value = 255;                        
    tail_lamp_request-> b_value = 0;
    while (!led_switch_client_->wait_for_service(5s)) {
      ERROR("Tail lamp test fails, Cannot get tail led service");
      return;
    }
    tail_lamp_result = led_switch_client_->async_send_request(tail_lamp_request);
    if (tail_lamp_result.get()->code != 0){
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // 头灯 蓝
    tail_lamp_request-> r_value = 0;                       
    tail_lamp_request-> g_value = 0;                        
    tail_lamp_request-> b_value = 255;
    while (!led_switch_client_->wait_for_service(5s)) {
      ERROR("Tail lamp test fails, Cannot get tail led service");
      return;
    }
    tail_lamp_result = led_switch_client_->async_send_request(tail_lamp_request);
    if (tail_lamp_result.get()->code != 0){
      return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    return;
}

bool Hardware::PlayWhiteNoiseOnce(const int& volume)
{
  while (!speech_volume_set_client_->wait_for_service(3s)) {
      ERROR("Specker Error, cannot get specker service");
      kMIC_valid_ = false;
      return false;
  }

 int current_volume = volume;
 auto set_volume_request = std::make_shared<::protocol::srv::AudioVolumeSet::Request>();
 set_volume_request->volume = current_volume;
 auto result = speech_volume_set_client_->async_send_request(set_volume_request);
 std::this_thread::sleep_for(std::chrono::seconds(1));

 auto message = ::protocol::msg::AudioPlayExtend();
 message.module_name = "speech_test";
 message.is_online = false;
 message.speech.module_name = "test";
 message.speech.play_id = 3000;
 message.text = "no";

 auto response = result.get()->success;
 if (!response) {
    ERROR("Specker Error, volume set fail");
    kMIC_valid_ = false;
    return false;
 }

 speech_play_publisher_->publish(message);
 std::this_thread::sleep_for(std::chrono::seconds(65));
 return true;
}

void Hardware::StopPlayWhiteNoise()
{
  keep_broadcast_ = false;
}

void Hardware::StartPlayWhiteNoise()
{
  keep_broadcast_ = true;
}

void Hardware::EmplaceSuccessBatteryTest()
{
   functions_.emplace(std::make_pair("kBattery", std::bind(&Hardware::TestBatterySuccess, this)));
   running_state_.success_results.emplace(std::make_pair("kBattery", "Success"));
}

void Hardware::EmplaceFailedBatteryTest()
{  
   functions_.emplace(std::make_pair("kBattery", std::bind(&Hardware::TestBatteryFailed, this)));
   running_state_.failure_results.emplace(std::make_pair("kBattery", "Failed"));
}

bool Hardware::TestAICamera()
{
  // return true;
  INFO("TestAICamera starts");

  auto open_request = std::make_shared<::protocol::srv::CameraService::Request>();
  open_request->command = 9;
  open_request->width = 640;
  open_request->height = 480;
  open_request->fps = 0;

  auto close_request = std::make_shared<::protocol::srv::CameraService::Request>();
  close_request->command = 10;

  auto take_pic_request = std::make_shared<::protocol::srv::CameraService::Request>();
  take_pic_request->command = 1;

  int test_time = 0;
  while (!ai_camera_switch_client_->wait_for_service(5s)) {
    ERROR("AICamera test fail, cannot get the service");
    cond_wait_.notify_one();
    return false;
  }
  while (test_time < take_picture_time_) {
    // INFO("AICamera test count:%d", test_time);
    // open camera
    auto open_result = ai_camera_switch_client_->async_send_request(open_request);

     while (!ai_camera_switch_client_->wait_for_service(5s)) {
       INFO("AICamera error, cannot get open camera service");
     }
    auto response_open_camera_result = open_result.get()->result;
    auto response_open_camera_code = open_result.get()->code;
    // open camera successfully and send a request of taking a picture
    if (response_open_camera_result == 0 &&  response_open_camera_code == 0) {
      auto take_pic_result = ai_camera_switch_client_->async_send_request(take_pic_request);
      while (!ai_camera_switch_client_->wait_for_service(5s)) {
       ERROR("AICamera error, cannot get take pic service");
      }
      auto response_take_pic_result = take_pic_result.get()->result;
      auto response_take_pic_code = take_pic_result.get()->code;
      // taking the picture successfully and send a request to close the camera
      if (response_take_pic_result == 0 && response_take_pic_code == 0) {
        auto close_result = ai_camera_switch_client_->async_send_request(close_request);
         while (!ai_camera_switch_client_->wait_for_service(5s)) {
          ERROR("AICamera error, cannot get close camera service");
         }
        auto response_close_camera_result = close_result.get()->result;
        auto response_close_camera_code = close_result.get()->code;
        if (response_close_camera_result == 0 && response_close_camera_code == 0) {
          test_time++;
          INFO("Camera_test_time: %d", test_time);
          if (test_time == take_picture_time_) {
            kAICamera_valid_ = true;
            INFO("TestAICamera success");
            // 测试完成后 删除图像的操作
            std::string delete_image = "rm -f /home/mi/Camera/*";
            bool delete_image_success = false;
            RunShellCommand(delete_image.c_str(), delete_image_success);
            cond_wait_.notify_one();
            return true;
          }
        } else {
          ERROR("AICamera test error, close the camera fail");
          cond_wait_.notify_one();
          return false;
        }
      } else {
        ERROR("AICamera test error, take a picture fail");
        cond_wait_.notify_one();
        return false;
      }
    } else {
      ERROR("AICamera test error, open the camera fail");
      cond_wait_.notify_one();
      return false;
    }
  }
  return true;
}

bool Hardware::TestRGBCamera()
{
  return kRGB_valid_;
}

bool Hardware::TestRealSense()
{
  if (kRealSenseImu_valid_ && kRealSenseDepth_valid_ &&
    kRealSenseRaw1_valid_ && kRealSenseRaw2_valid_){
    return true;
  } else {
    INFO("RealSense test failed, kRealSenseImu_valid_ is %d", kRealSenseImu_valid_);
    INFO("RealSense test failed, kRealSenseDepth_valid_ is %d", kRealSenseDepth_valid_);
    INFO("RealSense test failed, kRealSenseRaw1_valid_ is %d", kRealSenseRaw1_valid_);
    INFO("RealSense test failed, kRealSenseRaw2_valid_ is %d", kRealSenseRaw2_valid_);
    return false;
  }
}

bool Hardware::TestLeftFisheyeCamera()
{
  return kLeftFishCamera_valid_;
}

bool Hardware::TestRightFisheyeCamera()
{
  return kRightFishCamera_valid_;
}

void Hardware::SenorStatusIniti()
{
 // sensor flags
   kTouch_valid_ = true;
   kLidar_valid_ = true;
   kHeadLeftTOF_valid_ = true;
   kHeadRightTOF_valid_ = true;
   kRearLeftTOF_valid_ = true;
   kRearRightTOF_valid_ = true;
   kUltrasound_valid_ = true;
   kGPS_valid_ = true;
   kWIFI_valid_ = true;
   kFan_valid_ = true;
   kAICamera_valid_ = true;
   kMIC_valid_ = true;
   
  // RealSense
   active_realsense_success_ = false;

   kRealSenseImu_valid_ = true;
   kRealSenseDepth_valid_ = true;
   kRealSenseRaw1_valid_ = true;
   kRealSenseRaw2_valid_ = true;

  // RGB and fisheye camera
   active_stereocamera_success_ = false;

   kRGB_valid_ = true;
   kLeftFishCamera_valid_ = true;
   kRightFishCamera_valid_ = true;

  // battery
   kBattery_valid_ = false;

   head_left_tof_frame_count = 0;
   head_left_tof_dissatisfied_count = 0;

   head_right_tof_frame_count = 0;
   head_right_tof_dissatisfied_count = 0;

   rear_left_tof_frame_count = 0;
   rear_left_tof_dissatisfied_count = 0;

   rear_right_tof_frame_count = 0;
   rear_right_tof_dissatisfied_count = 0;

   gps_frame_count = 0;
   gps_dissatisfied_count = 0;

   ultrasound_frame_count = 0;
   ultrasound_dissatisfied_count = 0;

   wifi_frame_count = 0;
   wifi_dissatisfied_count = 0;

   realsense_imu_frame_count = 0;
   realsense_imu_dissatisfied_count = 0;

   realsense_depth_frame_count = 0;
   realsense_depth_dissatisfied_count = 0;

   realsense_raw1_frame_count = 0;
   realsense_raw1_dissatisfied_count = 0;

   realsense_raw2_frame_count = 0;
   realsense_raw2_dissatisfied_count = 0;

   rgb_frame_count = 0;
   rgb_dissatisfied_count = 0;

   left_fish_frame_count = 0;
   left_fish_dissatisfied_count = 0;

   right_fish_frame_count = 0;
   right_fish_dissatisfied_count = 0;
}

void Hardware::LedsTestStatusChange()
{
  if (keep_led_){
    keep_led_ = false;
  } else{
    keep_led_ = true;
  }
}

void Hardware::LedsChangeTest()
{
  while(keep_led_){
    MiniLampChange();
    HeadLampChange();
    TailLampChange();
  }
    auto lamp_stop_request = std::make_shared<::protocol::srv::LedExecute::Request>();
    auto lamp_stop_functor = [&lamp_stop_request](int target){
      lamp_stop_request->occupation = 0;
      lamp_stop_request->client = "lowpower";
      lamp_stop_request->target = target;                         
    };
     
    while(!led_switch_client_->wait_for_service(5s)) {
      ERROR("Led close fails, cannot get led service");
      return;
    }
    lamp_stop_functor(1);
    auto head_stop_result = led_switch_client_->async_send_request(lamp_stop_request);
    if (head_stop_result.get()->code != 0){
      ERROR("Head Led stop fails, service get invalid response");
      return;
    }
      
    while(!led_switch_client_->wait_for_service(5s)) {
      ERROR("Led close fails, cannot get led service");
      return;
    }
    lamp_stop_functor(2);
    auto tail_stop_result = led_switch_client_->async_send_request(lamp_stop_request);
    if (tail_stop_result.get()->code != 0){
      ERROR("Tail Led stop fails, service get invalid response");
      return;
    }
      
    while(!led_switch_client_->wait_for_service(5s)) {
      ERROR("Led close fails, cannot get led service");
      return;
    }
    lamp_stop_functor(3);
    auto mini_stop_result = led_switch_client_->async_send_request(lamp_stop_request);
    if (mini_stop_result.get()->code != 0){
      ERROR("Mini Led stop fails, service get invalid response");
      return;
    }
}

void Hardware::InitializeTestModules()
{
  modules_.clear();
  commands_queue_.clear();
  
  // const std::string kEyeLamp = "kEyeLamp";
  // const std::string kEarLamp = "kEarLamp";
  // const std::string kTailLamp = "kTailLamp";
  const std::string kModuleTouch = "kTouch";
  const std::string kModuleLidar = "kLidar";
  const std::string kModuleHeadTOF = "kHeadTOF";
  const std::string kModuleRearTOF = "kRearTOF";
  const std::string kModuleUltrasound = "kUltrasound";
  const std::string kModuleGPS = "kGPS";
  const std::string kModuleWifi = "kWIFI";
  const std::string kModuleSpeech = "kSpeaker";
  const std::string kAiCamera = "kAICamera";
  const std::string kFan = "kFan";
  const std::string kRealSense = "kRealSense";
  const std::string kRGBCamera = "kRGBCamera";
  const std::string kLeftFisheyeCamera = "kLeftFisheyeCamera";
  const std::string kRightFisheyeCamera = "kRightFisheyeCamera";
  const std::string kBattery = "kBattery";

  // modules_.emplace_back(kEyeLamp);
  // modules_.emplace_back(kEarLamp);
  // modules_.emplace_back(kTailLamp);
  modules_.emplace_back(kModuleTouch);
  modules_.emplace_back(kModuleLidar);
  modules_.emplace_back(kModuleHeadTOF);
  modules_.emplace_back(kModuleRearTOF);
  modules_.emplace_back(kModuleUltrasound);
  modules_.emplace_back(kModuleGPS);
  modules_.emplace_back(kModuleWifi);
  modules_.emplace_back(kModuleSpeech);
  modules_.emplace_back(kAiCamera);
  modules_.emplace_back(kFan);
  modules_.emplace_back(kRealSense);
  modules_.emplace_back(kRGBCamera);
  modules_.emplace_back(kLeftFisheyeCamera);
  modules_.emplace_back(kRightFisheyeCamera);
  modules_.emplace_back(kBattery);

  InitializeTotalModuleTests(modules_);
  // activate realsense in a sonthread and activate stereo_camera in a sonthread
  std::thread active_realsense(&Hardware::ActiveRealSense, this); 
  active_realsense.detach();
  // 最多阻塞60s，若60s内没有收到 条件变量的 notify，std::cv_status返回timeout，realsense测试失败。
  std::unique_lock<std::mutex> lock_wait_realsense(mutex_wait_);
  std::cv_status cvsts_realsense = cond_wait_.wait_for(lock_wait_realsense, std::chrono::seconds(60));
  if (cvsts_realsense == std::cv_status::timeout){
    //RealSense测试直接返回失败
    ERROR("Active realsense fail, realsense activate fail");
    kRealSenseImu_valid_ = false;
    kRealSenseDepth_valid_ = false;
    kRealSenseRaw1_valid_ = false;
    kRealSenseRaw2_valid_ = false;
    active_realsense_success_ = false;
  } else{
    INFO("Son thread notified lock_wait_realsense");
  }
  lock_wait_realsense.unlock();

  std::thread active_stereo(&Hardware::ActiveStereoCamera, this);
  active_stereo.detach();
  std::unique_lock<std::mutex> lock_wait_stereo(mutex_wait_);
  // for dev: std::chrono::seconds(60); for debug: std::chrono::seconds(1)
  std::cv_status cvsts_stereo = cond_wait_.wait_for(lock_wait_stereo, std::chrono::seconds(60));
  if (cvsts_stereo == std::cv_status::timeout){
     //StereoCamera测试直接返回失败
     ERROR("Active stereocamera fail, stereocamera activate fail");
     kLeftFishCamera_valid_  = false;
     kRightFishCamera_valid_  = false;
     kRGB_valid_  = false;
     active_stereocamera_success_ = false;
  } else{
    INFO("Son thread notified lock_wait_stereo");
  }
  lock_wait_stereo.unlock();

  INFO("Active realsense and stereocamera lifecycle finished");
}

void Hardware::InitializeSensorSubscriptionTopics()
{
  callbackgroup_hardware_ = node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  options_.callback_group = callbackgroup_hardware_;

  subscribers_.clear();

  const std::string kSensorLidarTopic = "scan";
  const std::string kSensorHeadToFTopic = "head_tof_payload";
  const std::string kSensorRearToFTopic = "rear_tof_payload";
  const std::string kSensorGPSTopic = "gps_payload";
  const std::string kSensorUltrasoundTopic = "ultrasonic_payload";
  const std::string kSensorWifiTopic = "wifi_status";
  const std::string kSensorSpeakerTopic = "speech_play";
  const std::string kRealSenseTopic = "camera/infra1/image_rect_raw";
  const std::string kRealSenseTopic1 = "camera/infra2/image_rect_raw";
  const std::string kRealSenseTopic2 = "camera/depth/image_rect_raw";
  const std::string kRealSenseTopic3 = "camera/imu";
  const std::string kRGBCameraTopic = "image_rgb";
  const std::string kLeftFisheyeCameraTopic = "image_left";
  const std::string kRightFisheyeCameraTopic = "image_right";

  // scan
  subscribers_.push_back(
      {
        SubscribeWithHandler<sensor_msgs::msg::LaserScan>(
          &Hardware::HandleLaserScanMessage, kSensorLidarTopic, node_ptr_, this, options_),
        kSensorLidarTopic
      });

  // head_left_tof
  subscribers_.push_back(
      {
        SubscribeWithHandler<::protocol::msg::HeadTofPayload>(
          &Hardware::HandleHeadLeftToFMessage, kSensorHeadToFTopic, node_ptr_, this, options_),
        kSensorHeadToFTopic
      });
  
  // head_right_tof
  subscribers_.push_back(
      {
        SubscribeWithHandler<::protocol::msg::HeadTofPayload>(
          &Hardware::HandleHeadRightToFMessage, kSensorHeadToFTopic, node_ptr_, this, options_),
        kSensorHeadToFTopic
      });

  // rear_left_tof
  subscribers_.push_back(
      {
        SubscribeWithHandler<::protocol::msg::RearTofPayload>(
          &Hardware::HandleRearLeftToFMessage, kSensorRearToFTopic, node_ptr_, this, options_),
        kSensorRearToFTopic
      });
  
  // rear_right_tof
  subscribers_.push_back(
      {
        SubscribeWithHandler<::protocol::msg::RearTofPayload>(
          &Hardware::HandleRearRightToFMessage, kSensorRearToFTopic, node_ptr_, this, options_),
        kSensorRearToFTopic
      });
      
  // GPS
  subscribers_.push_back(
      {
        SubscribeWithHandler<::protocol::msg::GpsPayload>(
          &Hardware::HandleGPSMessage, kSensorGPSTopic, node_ptr_, this, options_),
        kSensorGPSTopic
      });

  // ultrasonic
  subscribers_.push_back(
      {
        SubscribeWithHandler<sensor_msgs::msg::Range>(
          &Hardware::HandleUltrasoundMessage, kSensorUltrasoundTopic, node_ptr_, this, options_),
        kSensorUltrasoundTopic
      });

  // wifi
  subscribers_.push_back(
      {
        SubscribeWithHandler<::protocol::msg::WifiStatus>(
          &Hardware::HandleWifiMessage, kSensorWifiTopic, node_ptr_, this, options_),
        kSensorWifiTopic
      });

  // speech
  subscribers_.push_back(
      {
        SubscribeWithHandler<::protocol::msg::AudioPlay>(
          &Hardware::HandleSpeechMessage, kSensorSpeakerTopic, node_ptr_, this, options_),
        kSensorSpeakerTopic
      });

  // realsense
  subscribers_.push_back(
      {
        SubscribeWithHandler<sensor_msgs::msg::Image>(
          &Hardware::HandleRealSenseRaw1CameraMessage, kRealSenseTopic, node_ptr_, this, options_),
        kRealSenseTopic
      });

  subscribers_.push_back(
      {
        SubscribeWithHandler<sensor_msgs::msg::Image>(
          &Hardware::HandleRealSenseRaw2CameraMessage, kRealSenseTopic1, node_ptr_, this, options_),
        kRealSenseTopic1
      });

  subscribers_.push_back(
      {
        SubscribeWithHandler<sensor_msgs::msg::Image>(
          &Hardware::HandleRealSenseDepthMessage, kRealSenseTopic2, node_ptr_, this, options_),
        kRealSenseTopic2
      });

  subscribers_.push_back(
      {
        SubscribeWithHandler<sensor_msgs::msg::Imu>(
          &Hardware::HandleRealSenseImuMessage, kRealSenseTopic3, node_ptr_, this, options_),
        kRealSenseTopic3
      });

  // RGB and fisheye camera
  subscribers_.push_back(
      {
        SubscribeWithHandler<sensor_msgs::msg::Image>(
          &Hardware::HandleRGBDCameraMessage, kRGBCameraTopic, node_ptr_, this, options_),
        kRGBCameraTopic
      });

  subscribers_.push_back(
      {
        SubscribeWithHandler<sensor_msgs::msg::Image>(
          &Hardware::HandleLeftFisheyeCameraMessage, kLeftFisheyeCameraTopic, node_ptr_, this, options_),
        kLeftFisheyeCameraTopic
      });
  
  subscribers_.push_back(
      {
        SubscribeWithHandler<sensor_msgs::msg::Image>(
          &Hardware::HandleRightFisheyeCameraMessage, kRightFisheyeCameraTopic, node_ptr_, this, options_),
        kRightFisheyeCameraTopic
      });
}

bool Hardware::InitializeTestParas(const std::string& config_file)
{
  toml::value config;
  if (!common::CyberdogToml::ParseFile(config_file, config)) {
    ERROR("Toml error, Cannot parse %s", config_file.c_str());
    return false;
  }

  toml::value bytopic;
  bool get_bytopic_value = common::CyberdogToml::Get(config, "bytopic", bytopic);
  
  bool get_total_frame = common::CyberdogToml::Get(bytopic, "total_frame", total_test_frame_);
  bool get_AICamera_picture = common::CyberdogToml::Get(bytopic, "AICamera_picture", take_picture_time_);
  bool get_timeout = common::CyberdogToml::Get(bytopic, "timeout", timeout_);

  return get_bytopic_value && get_total_frame && get_AICamera_picture && get_timeout;
}

void Hardware::HandleLaserScanMessage(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  float lidar_center_distance,lidar_left_distance,lidar_right_distance,lidar_angle30_distance,lidar_angle150_distance;
  // 连续50帧能够收到数据且 动态测试中用到的数据不为0 才视为测试通过
   // 若在测试结束时 没有收够50帧数据 则判断 lidar_frame_count与lidar_satify_count 是否相等
  lidar_frame_count++;
  auto lidar_average_distance = [&](int begin, int end)->float{
    int no_zero_count = 0;
    float average_distance = 0.0;
    for (int i = begin; i < end; ++i){
      if(msg->intensities[i] != 0.0){
        no_zero_count++;
        average_distance += msg->ranges[i];
      }
    }
    if(no_zero_count != 0){
      average_distance = average_distance/no_zero_count;
    }else{
      average_distance = 0.0;
    }
    return average_distance;
  };
  
  if(lidar_frame_count < total_test_frame_){
    int laser_points_number = msg->ranges.size();
    // 获取中间5个点激光点强度非零的均值 作为 current_center_distance_
    lidar_center_distance = lidar_average_distance((laser_points_number/2-2), (laser_points_number/2+3));
    // 获取最左边3个点激光点强度非零的均值 作为 current_left_distance_
    lidar_left_distance = lidar_average_distance((laser_points_number-3), (laser_points_number));
    // 获取最右边3个点激光点强度非零的均值 作为 current_right_distance_
    lidar_right_distance = lidar_average_distance(0, 3);
    // 获取30° 5个点激光点强度非零的均值 作为 current_30_distance_
    lidar_angle30_distance = lidar_average_distance((laser_points_number/6-2), (laser_points_number/6+3));
    // 获取150° 5个点激光点强度非零的均值 作为 current_150_distance_
    lidar_angle150_distance = lidar_average_distance((laser_points_number - laser_points_number/6-2), (laser_points_number - laser_points_number/6+3));

    if(!lidar_center_distance || !lidar_left_distance || 
       !lidar_right_distance || !lidar_angle30_distance || !lidar_angle150_distance){
       INFO("Invalid lidar frame is %d", lidar_frame_count);
       lidar_dissatisfied_count++;
       if(lidar_dissatisfied_count >= 2){
         kLidar_valid_ = false;
       }
    } 
  } 
}

void Hardware::HandleHeadLeftToFMessage(::protocol::msg::HeadTofPayload::SharedPtr msg)
{
  // 连续50帧能够收到数据即为测试通过
  head_left_tof_frame_count++;
  if(head_left_tof_frame_count < total_test_frame_){
    if(msg->left_head.data.size() == 0){
      INFO("Head left tof invalid frame is %d", head_left_tof_frame_count);
      head_left_tof_dissatisfied_count++;
      if(head_left_tof_dissatisfied_count >= 2){
         kHeadLeftTOF_valid_ = false;
      }
    } 
  } 
}

void Hardware::HandleHeadRightToFMessage(::protocol::msg::HeadTofPayload::SharedPtr msg)
{
  // 连续50帧能够收到数据即为测试通过
  head_right_tof_frame_count++;
  if(head_right_tof_frame_count < total_test_frame_){
    if(msg->right_head.data.size() == 0){
      INFO("Head right tof invalid frame is %d", head_right_tof_frame_count);
      head_right_tof_dissatisfied_count++;
      if(head_right_tof_dissatisfied_count >= 2){
         kHeadRightTOF_valid_ = false;
      }
    } 
  } 
}

void Hardware::HandleRearLeftToFMessage(const ::protocol::msg::RearTofPayload::SharedPtr msg)
{
  // 连续50帧能够收到数据为测试通过
  rear_left_tof_frame_count++;
  if(rear_left_tof_frame_count < total_test_frame_){
    if(msg->left_rear.data.size() == 0){
      INFO("Rear left tof invalid frame is %d", rear_left_tof_frame_count);
      rear_left_tof_dissatisfied_count++;
      if(rear_left_tof_dissatisfied_count >= 2){
         kRearLeftTOF_valid_ = false;
      }
    } 
  } 
}

void Hardware::HandleRearRightToFMessage(const ::protocol::msg::RearTofPayload::SharedPtr msg)
{
  // 连续50帧能够收到数据为测试通过
  rear_right_tof_frame_count++;
  if(rear_right_tof_frame_count < total_test_frame_){
    if(msg->right_rear.data.size() == 0){
      INFO("Rear right tof invalid frame is %d", rear_right_tof_frame_count);
      rear_right_tof_dissatisfied_count++;
      if(rear_right_tof_dissatisfied_count >= 2){
         kRearRightTOF_valid_ = false;
      }
    } 
  } 
}

void Hardware::HandleUltrasoundMessage(sensor_msgs::msg::Range::SharedPtr msg)
{
  // 连续50帧能够收到距离数据即为测试通过
  ultrasound_frame_count++;
  if(ultrasound_frame_count < total_test_frame_){
    if(!msg->range){
       ultrasound_dissatisfied_count++;
       INFO("Ultrasound invalid frame is %d", ultrasound_frame_count);
       if(ultrasound_dissatisfied_count >= 2){
          kUltrasound_valid_ = false;
       }
    } 
  } 
}

void Hardware::HandleGPSMessage(::protocol::msg::GpsPayload::SharedPtr msg)
{
  // 连续50帧能够收到时间数据即为测试通过
  gps_frame_count++;
  if(gps_frame_count < total_test_frame_){
    if(!msg->sec || !msg->nanosec){
       INFO("GPS invalid frame is %d", gps_frame_count);
       gps_dissatisfied_count++;
       if(gps_dissatisfied_count >= 2){
          kGPS_valid_ = false;
       }
    } 
  } 
}

void Hardware::HandleWifiMessage(::protocol::msg::WifiStatus::SharedPtr msg)
{
  // 连续50帧网络显示已经连接即为测试通过
  wifi_frame_count++;
  if(wifi_frame_count < total_test_frame_){
    if(!msg->is_connected){
       INFO("Wifi invalid frame is %d", wifi_frame_count);
       wifi_dissatisfied_count++;
       if(wifi_dissatisfied_count >= 2){
          kWIFI_valid_ = false;
       }
    } 
  } 
}

void Hardware::HandleRealSenseImuMessage(sensor_msgs::msg::Imu::SharedPtr msg)
{
  realsense_imu_frame_count++;
  if(realsense_imu_frame_count < total_test_frame_){
    if(!msg->linear_acceleration.z){
      INFO("RealSenseImu invalid frame is %d", realsense_imu_frame_count);
      realsense_imu_dissatisfied_count++;
      if(realsense_imu_dissatisfied_count >= 2){
        kRealSenseImu_valid_ = false;
      }
    } 
  }
}

void Hardware::HandleRealSenseDepthMessage(sensor_msgs::msg::Image::SharedPtr msg)
{
  realsense_depth_frame_count++;
  if(realsense_depth_frame_count < total_test_frame_){
    if(!msg->height || !msg->width ){
      INFO("RealSenseDepth invalid frame is %d",realsense_depth_frame_count);
      realsense_depth_dissatisfied_count++;
      if(realsense_depth_dissatisfied_count >= 2){
        kRealSenseDepth_valid_ = false;
      }
    } 
  }
}

void Hardware::HandleRealSenseRaw1CameraMessage(sensor_msgs::msg::Image::SharedPtr msg)
{
  realsense_raw1_frame_count++;
  if(realsense_raw1_frame_count < total_test_frame_){
    if(!msg->height || !msg->width){
      INFO("SenseRaw1 invalid frame is %d",realsense_raw1_frame_count);
      realsense_raw1_dissatisfied_count++;
      if(realsense_raw1_dissatisfied_count >= 2){
         kRealSenseRaw1_valid_ = false;
      }
    } 
  } 
}

void Hardware::HandleRealSenseRaw2CameraMessage(sensor_msgs::msg::Image::SharedPtr msg)
{
  realsense_raw2_frame_count++;
  if(realsense_raw2_frame_count < total_test_frame_){
    if(!msg->height || !msg->width){
      INFO("SenseRaw2 invalid frame is %d",realsense_raw2_frame_count);
      realsense_raw2_dissatisfied_count++;
      if(realsense_raw2_dissatisfied_count >= 2){
        kRealSenseRaw2_valid_ = false;
      }
    } 
  } 
}

void Hardware::HandleRGBDCameraMessage(sensor_msgs::msg::Image::SharedPtr msg)
{
  rgb_frame_count++;
  if(rgb_frame_count < total_test_frame_){
    if(!msg->height && !msg->width){
      rgb_dissatisfied_count++;
      INFO("RGBCamera invalid frame is %d",rgb_frame_count);
      if(rgb_dissatisfied_count >= 2){
        kRGB_valid_ = false;
      }
    } 
  } 
}

void Hardware::HandleLeftFisheyeCameraMessage(sensor_msgs::msg::Image::SharedPtr msg)
{
  left_fish_frame_count++;
  if(left_fish_frame_count < total_test_frame_){
    if(!msg->height || !msg->width){
      INFO("LeftFisheyeCamera invalid frame is %d",left_fish_frame_count);
      left_fish_dissatisfied_count++;
      if(left_fish_dissatisfied_count >= 2){
        kLeftFishCamera_valid_ = false;
      }
    } 
  }
}

void Hardware::HandleRightFisheyeCameraMessage(sensor_msgs::msg::Image::SharedPtr msg)
{
  right_fish_frame_count++;
  if(right_fish_frame_count < total_test_frame_){
    if(!msg->height || msg->width){
      INFO("RightFisheyeCamera invalid frame is %d",right_fish_frame_count);
      right_fish_dissatisfied_count++;
      if(right_fish_dissatisfied_count >= 2){
        kRightFishCamera_valid_ = false;
      }
    } 
  } 
}

void Hardware::HandleSpeechMessage(::protocol::msg::AudioPlay::SharedPtr msg)
{
  if (msg->play_id) {
    kMIC_valid_ = true;
  }
}

void Hardware::ActiveRealSense()
{
  bool success = LifecycleNodeManager::GetSingleton()->Configure(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    INFO("Configure realsense fail");
    return;
  }
  std::this_thread::sleep_for(std::chrono::seconds(3));
  // RealSense camera lifecycle(activate state)
  success = LifecycleNodeManager::GetSingleton()->Startup(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    INFO("Activate realsense fail");
    return;
  }
  active_realsense_success_ = true;
  cond_wait_.notify_one();
}

void Hardware::ShutdownRealSense()
{
  // INFO("Shutdown RealSense");
  bool success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    INFO("Pause RealSense fail");
    return;
  }
  std::this_thread::sleep_for(std::chrono::seconds(3));
  success = LifecycleNodeManager::GetSingleton()->Cleanup(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    INFO("Shutdown RealSense fail");
    return;
  }
}

// activate fisheye camera and rgb
void Hardware::ActiveStereoCamera()
{
  // INFO("Activate StereoCamera");
  // RGB-D camera lifecycle(configure state)
  bool success = LifecycleNodeManager::GetSingleton()->Configure(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
     INFO("Configure StereoCamera fail");
    return;
  }
  std::this_thread::sleep_for(std::chrono::seconds(3));
  // RGB-D camera lifecycle(activate state)
  success = LifecycleNodeManager::GetSingleton()->Startup(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    INFO("Activate StereoCamera fail");
    return;
  }
  active_stereocamera_success_ = true;
  cond_wait_.notify_one();
}

void Hardware::ShutdownStereoCamera()
{
  // INFO("Shutdown StereoCamera");
  // RGB-D camera lifecycle(configure state)
  bool success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    INFO("Pause StereoCamera fail");
    return;
  }
  std::this_thread::sleep_for(std::chrono::seconds(3));
  // RGB-D camera lifecycle(activate state)
  success = LifecycleNodeManager::GetSingleton()->Cleanup(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    INFO("Shutdown StereoCamera fail");
    return;
  }
}
}  // namespace system
}  // namespace cyberdog
