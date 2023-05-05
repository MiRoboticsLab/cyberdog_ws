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

#ifndef CYBERDOG_AFT__PROTOCOL__PROTOCOL_MOTION_HPP_
#define CYBERDOG_AFT__PROTOCOL__PROTOCOL_MOTION_HPP_

#include <chrono>
#include <memory>
#include <deque>
#include <vector>
#include <unordered_map>

#include "cyberdog_aft/protocol/protocol_interface.hpp"
#include "cyberdog_aft/motion/motion.hpp"
#include "cyberdog_aft/system/report_result.hpp"

namespace cyberdog
{
namespace protocol
{

class ProtocolMotion
{
public:
  using DynamicReportResult = system::DynamicReportResult;

  ProtocolMotion();
  ~ProtocolMotion();

  ProtocolMotion(const ProtocolMotion &) = delete;
  ProtocolMotion & operator=(const ProtocolMotion &) = delete;

  enum MotionType
  {
    EmergencyStop,          // 急停
    ForceControlPose,       // 力控姿态
    HighDampingDown,        // 高阻尼趴下
    SelfFrequencyWalking,   // 自变频行走
    WalkTheDog,             // 遛狗（拽狗的时候，狗会跟着行走）
    RunFast,                // 快跑
    QuadrupedJump,          // 四足小跳
    JumpBackAndForth,       // 前后跳
    ReturnStandUp,          // 恢复站立
    RollLeft,               // 向左打滚一圈
    Bow,                    // 作揖
    Backflip,               // 后空翻
    WalkSlowly,             // 慢走
    GoFast,                 // 快走
    Unknow
  };

  struct State
  {
    MotionType current_motion;
    std::deque<MotionType> remain_motions;
    std::vector<MotionType> total_motions;
    std::unordered_map<MotionType, bool> results;

    State()
    : current_motion{MotionType::Unknow}
    {}
  };

  // run what motion type command for test
  bool RunCommand(const MotionType & type);

  // cancel which command for motions
  bool CancelCommand(const MotionType & type);

  // cancel all command for motions
  bool CancelAllCommands();

  // run default motions
  bool RunDefaultActions();

  // get motion running state
  // bool GetRunningState(State & state);

  /**
   * @brief Get the Test Report Result object
   *
   * @param report
   * @return true
   * @return false
   */
  bool GetTestReportResult(DynamicReportResult & report);

private:
  bool RunEmergencyStop();

  bool RunForceControlPose();

  bool RunHighDampingDown();

  bool RunSelfFrequencyWalking();

  bool RunWalkTheDog();

  bool RunRunFast();

  bool RunQuadrupedJump();

  bool RunJumpBackAndForth();

  bool RunReturnStandUp();

  bool RunRollLeft();

  bool RunBow();

  bool RunBackflip();

  bool RunWalkSlowly();

  bool RunGoFast();

  void Spin();

  std::shared_ptr<motion::Motion> motion_ptr_ {nullptr};
  State state_;
};
}  // namespace protocol
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__PROTOCOL__PROTOCOL_MOTION_HPP_
