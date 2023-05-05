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

#ifndef CYBERDOG_AFT__SYSTEM__HARDWARE_HPP_
#define CYBERDOG_AFT__SYSTEM__HARDWARE_HPP_

#include <chrono>
#include <memory>
#include <deque>
#include <vector>
#include <string>
#include <functional>
#include <thread>
#include <map>
#include <mutex>
#include <condition_variable>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "protocol/msg/audio_play.hpp"
#include "protocol/msg/gps_payload.hpp"
#include "protocol/msg/rear_tof_payload.hpp"
#include "protocol/msg/head_tof_payload.hpp"
#include "protocol/msg/wifi_status.hpp"
#include "protocol/msg/touch_status.hpp"
#include "protocol/srv/sensor_operation.hpp"
#include "protocol/srv/camera_service.hpp"
#include "protocol/srv/audio_volume_set.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "protocol/msg/bms_status.hpp"

#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_aft/system/report_result.hpp"
#include "cyberdog_aft/utils/lifecycle_node_manager.hpp"

namespace cyberdog
{
namespace system
{

class HardwareInterface
{
public:
  HardwareInterface() {}
  ~HardwareInterface() {}

  HardwareInterface(const HardwareInterface &) = delete;
  HardwareInterface & operator=(const HardwareInterface &) = delete;
};

class Hardware : public HardwareInterface
{
public:
  using LifeCycleNodeType = LifecycleNodeManager::LifeCycleNode;

  explicit Hardware();     // NOLINT
  explicit Hardware(std::shared_ptr<rclcpp::Node> node);
  ~Hardware();

  Hardware(const Hardware &) = delete;
  Hardware & operator=(const Hardware &) = delete;

  // 枚举测试项
  enum Type
  {
    kUnknow,
    kTouch,
    kLidar,
    kHeadLeftTOF,
    kHeadRightTOF,
    kRearLeftTOF,
    kRearRightTOF,
    kUltrasound,
    kGPS,
    kWIFI,
    kFan,
    kAICamera,
    kRealSenseImu,
    kRealSenseDepth,
    kRealSenseRaw1,
    kRealSenseRaw2,
    kRGBCamera,
    kLeftFisheyeCamera,
    kRightFisheyeCamera,
    kMIC,
    kBattery,
  };

  struct Subscriber
  {
    rclcpp::SubscriptionBase::SharedPtr subscriber;
    std::string topic;
  };

  using Callback = std::function<bool ()>;

  bool RunTest(const Type & hardware_type);
  bool RunTest(const std::string & hardware_type);

  // All test for hardware
  bool RunAllTest();

  // Initialize
  bool InitializeTotalModuleTests(const std::vector<std::string> & modules);

  // Report
  bool GetReport(ReportResult & report);

  // Check
  bool CheckAllFinished();

  // Debug
  std::string ToString(const Type & type);
  Type FromString(const std::string & type);

  /**
   * @brief Debug for report message
   *
   * @param report
   */
  void ReportToString(ReportResult & report);
  bool ClearReport(ReportResult & report);

  /**
   * @brief Set test modules
   *
   */
  void InitializeTestModules();

  /**
   * @brief 读取toml中关于静态测试的参数配置
   *
   * @param config_file toml文件路径
   */
  bool InitializeTestParas(const std::string & config_file);

  bool TestTurnOffFan();
  bool TestTurnOnFan();
  bool FanTestSuccess();
  bool PlayWhiteNoise(const int & volume);
  bool PlayWhiteNoiseOnce(const int & volume);
  void StopPlayWhiteNoise();
  void StartPlayWhiteNoise();
  void EmplaceSuccessBatteryTest();
  void EmplaceFailedBatteryTest();
  void LedsChangeTest();
  void LedsTestStatusChange();
  void MiniLampChange();
  void HeadLampChange();
  void TailLampChange();
  void SenorStatusIniti();

private:
  std::mutex mutex_wait_;
  std::condition_variable cond_wait_;
  bool active_realsense_success_{true};
  bool active_stereocamera_success_{true};

  // toml可配置参数
  int total_test_frame_ {};
  int charge_duration_ {};
  int take_picture_time_ {};
  int timeout_ {};

  static int lidar_frame_count;
  static int lidar_dissatisfied_count;

  static int head_left_tof_frame_count;
  static int head_left_tof_dissatisfied_count;

  static int head_right_tof_frame_count;
  static int head_right_tof_dissatisfied_count;

  static int rear_left_tof_frame_count;
  static int rear_left_tof_dissatisfied_count;

  static int rear_right_tof_frame_count;
  static int rear_right_tof_dissatisfied_count;

  static int ultrasound_frame_count;
  static int ultrasound_dissatisfied_count;

  static int gps_frame_count;
  static int gps_dissatisfied_count;

  static int wifi_frame_count;
  static int wifi_dissatisfied_count;

  static int realsense_imu_frame_count;
  static int realsense_imu_dissatisfied_count;

  static int realsense_depth_frame_count;
  static int realsense_depth_dissatisfied_count;

  static int realsense_raw1_frame_count;
  static int realsense_raw1_dissatisfied_count;

  static int realsense_raw2_frame_count;
  static int realsense_raw2_dissatisfied_count;

  static int rgb_frame_count;
  static int rgb_dissatisfied_count;

  static int left_fish_frame_count;
  static int left_fish_dissatisfied_count;

  static int right_fish_frame_count;
  static int right_fish_dissatisfied_count;

  rclcpp::CallbackGroup::SharedPtr callbackgroup_hardware_{nullptr};
  rclcpp::SubscriptionOptions options_;
  /**
   * @brief All test module function
   *
   * @return true
   * @return false
   */
  bool TestTouch();
  bool TestLidar();
  bool TestHeadLeftTOF();
  bool TestHeadRightTOF();
  bool TestRearLeftTOF();
  bool TestRearRightTOF();
  bool TestUltrasound();
  bool TestWIFI();
  bool TestGPS();
  bool TestFan();
  bool TestBatterySuccess();
  bool TestBatteryFailed();
  bool TestAICamera();
  bool TestRGBCamera();
  bool TestRealSense();
  bool TestLeftFisheyeCamera();
  bool TestRightFisheyeCamera();

  /**
   * @brief Subscription sensor topic
   *
   */
  void InitializeSensorSubscriptionTopics();

  /**
   * @brief Receive laserScan sensor message
   *
   * @param msg
   */
  void HandleLaserScanMessage(sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Receive head left TOF sensor message
   *
   * @param msg
   */
  void HandleHeadLeftToFMessage(::protocol::msg::HeadTofPayload::SharedPtr msg);

  /**
   * @brief Receive head right TOF sensor message
   *
   * @param msg
   */
  void HandleHeadRightToFMessage(::protocol::msg::HeadTofPayload::SharedPtr msg);

  /**
   * @brief Receive rear left TOF sensor message
   *
   * @param msg
   */
  void HandleRearLeftToFMessage(::protocol::msg::RearTofPayload::SharedPtr msg);

  /**
   * @brief Receive rear right TOF sensor message
   *
   * @param msg
   */
  void HandleRearRightToFMessage(::protocol::msg::RearTofPayload::SharedPtr msg);

  /**
   * @brief Receive GPS sensor message
   *
   * @param msg
   */
  void HandleGPSMessage(::protocol::msg::GpsPayload::SharedPtr msg);

  /**
   * @brief Receive ultrasound sensor message
   *
   * @param msg
   */
  void HandleUltrasoundMessage(sensor_msgs::msg::Range::SharedPtr msg);

  /**
   * @brief Receive wifi sensor message
   *
   * @param msg
   */
  void HandleWifiMessage(::protocol::msg::WifiStatus::SharedPtr msg);

  /**
   * @brief Receive speech sensor message
   *
   * @param msg
   */
  void HandleSpeechMessage(::protocol::msg::AudioPlay::SharedPtr msg);

  /**
   * @brief realsense imu
   *
   *
   * @param msg
   */
  void HandleRealSenseImuMessage(sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief real sense depth
   *
   * @param msg
   */
  void HandleRealSenseDepthMessage(sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief real sense raw1
   *
   * @param msg
   */
  void HandleRealSenseRaw1CameraMessage(sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief real sense raw2
   *
   * @param msg
   */
  void HandleRealSenseRaw2CameraMessage(sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief RGB-D camera sensor message
   *
   * @param msg
   */
  void HandleRGBDCameraMessage(sensor_msgs::msg::Image::SharedPtr msg);
  void HandleLeftFisheyeCameraMessage(sensor_msgs::msg::Image::SharedPtr msg);
  void HandleRightFisheyeCameraMessage(sensor_msgs::msg::Image::SharedPtr msg);

  void ActiveRealSense();
  void ActiveStereoCamera();
  void ShutdownRealSense();
  void ShutdownStereoCamera();

  std::map<const std::string, Callback> functions_;

  std::deque<std::string> commands_queue_;
  ReportResult running_state_;

  std::shared_ptr<rclcpp::Node> node_ptr_{nullptr};
  std::vector<Subscriber> subscribers_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Client<::protocol::srv::CameraService>::SharedPtr ai_camera_switch_client_{nullptr};
  rclcpp::Client<::protocol::srv::LedExecute>::SharedPtr led_switch_client_{nullptr};
  rclcpp::Client<::protocol::srv::AudioVolumeSet>::SharedPtr speech_volume_set_client_{nullptr};
  rclcpp::Publisher<::protocol::msg::AudioPlayExtend>::SharedPtr speech_play_publisher_{nullptr};

  // Record test modules
  std::vector<std::string> modules_;

  // hardware flags
  bool kTouch_valid_{false};
  bool kLidar_valid_{false};
  bool kHeadLeftTOF_valid_{false};
  bool kHeadRightTOF_valid_{false};
  bool kRearLeftTOF_valid_{false};
  bool kRearRightTOF_valid_{false};
  bool kUltrasound_valid_{false};
  bool kGPS_valid_{false};
  bool kWIFI_valid_{false};
  bool kFan_valid_{false};
  bool kAICamera_valid_{false};

  // RealSense flags
  bool kRealsense_active_{false};

  bool kRealSenseImu_valid_{false};
  bool kRealSenseDepth_valid_{false};
  bool kRealSenseRaw1_valid_{false};
  bool kRealSenseRaw2_valid_{false};

  // RGB and fisheye cameras flags
  bool kStereo_active_{false};

  bool kRGB_valid_{false};
  bool kLeftFishCamera_valid_{false};
  bool kRightFishCamera_valid_{false};

  // mic flags
  bool kMIC_valid_{false};

  // battery flags
  bool kBattery_valid_{false};

  // atimic
  std::atomic_bool stoped_{false};

  // keep broadcast
  bool keep_broadcast_{true};

  // keep led
  bool keep_led_{true};
};
}   // namespace system
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__SYSTEM__HARDWARE_HPP_
