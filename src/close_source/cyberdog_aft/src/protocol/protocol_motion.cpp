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

#include "cyberdog_aft/protocol/protocol_motion.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace protocol
{

ProtocolMotion::ProtocolMotion()
{
  motion_ptr_ = std::make_shared<motion::Motion>();
}

ProtocolMotion::~ProtocolMotion()
{
}

bool ProtocolMotion::RunCommand(const MotionType & type)
{
  switch (type) {
    case MotionType::EmergencyStop:
      {
        bool ok = RunEmergencyStop();
        if (!ok) {
          return false;
        }
        break;
      }


    case MotionType::ForceControlPose:
      {
        bool ok = RunForceControlPose();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::HighDampingDown:
      {
        bool ok = RunHighDampingDown();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::SelfFrequencyWalking:
      {
        bool ok = RunSelfFrequencyWalking();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::WalkTheDog:
      {
        bool ok = RunWalkTheDog();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::RunFast:
      {
        bool ok = RunRunFast();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::QuadrupedJump:
      {
        bool ok = RunQuadrupedJump();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::JumpBackAndForth:
      {
        bool ok = RunJumpBackAndForth();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::ReturnStandUp:
      {
        bool ok = RunReturnStandUp();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::RollLeft:
      {
        bool ok = RunRollLeft();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::Bow:
      {
        bool ok = RunBow();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::Backflip:
      {
        bool ok = RunBackflip();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::WalkSlowly:
      {
        bool ok = RunWalkSlowly();
        if (!ok) {
          return false;
        }
        break;
      }

    case MotionType::GoFast:
      {
        bool ok = RunGoFast();
        if (!ok) {
          return false;
        }
        break;
      }

    default:
      break;
  }
  return true;
}

bool ProtocolMotion::RunDefaultActions()
{
  return motion_ptr_->RunDefaultTest();
}

// bool ProtocolMotion::GetRunningState(State & state)
// {
//   return true;
// }

bool ProtocolMotion::GetTestReportResult(DynamicReportResult & report)
{
  return motion_ptr_->GetReportResult(report);
}

void ProtocolMotion::Spin()
{
  motion_ptr_->Spin();
}

bool ProtocolMotion::RunEmergencyStop()
{
  bool cmd_ok = motion_ptr_->RunCommandAction(motion::CommmandAction::kCommmandEmergencyStop);
  if (!cmd_ok) {
    INFO("Protocol motion run function RunEmergencyStop() error.");
    return false;
  }
  return true;
}

bool ProtocolMotion::RunForceControlPose()
{
  return true;
}

bool ProtocolMotion::RunHighDampingDown()
{
  bool cmd_ok = motion_ptr_->RunCommandAction(motion::CommmandAction::kCommmandHighDampingDown);
  if (!cmd_ok) {
    INFO("Protocol motion run function RunHighDampingDown() error.");
    return false;
  }
  return true;
}

bool ProtocolMotion::RunSelfFrequencyWalking()
{
  std::array<float, 6> cmd_params{
    0.2,   // vel_x
    0.0,   // vel_y
    0.0,   // yaw
    0.05,   // step_height
    0.05
  };

  bool cmd_ok = motion_ptr_->RunServoCommandAction(
    motion::CommmandAction::kCommmandSelfFrequencyWalking, cmd_params);
  if (!cmd_ok) {
    INFO("Protocol motion run function RunSelfFrequencyWalking() error.");
    return false;
  }
  return true;
}

bool ProtocolMotion::RunWalkTheDog()
{
  bool cmd_ok = motion_ptr_->RunCommandAction(motion::CommmandAction::kCommmandWalkTheDog);
  if (!cmd_ok) {
    INFO("Protocol motion run function RunWalkTheDog() error.");
    return false;
  }
  return true;
}

bool ProtocolMotion::RunRunFast()
{
  return true;
}

bool ProtocolMotion::RunQuadrupedJump()
{
  return true;
}

bool ProtocolMotion::RunJumpBackAndForth()
{
  return true;
}

bool ProtocolMotion::RunReturnStandUp()
{
  bool cmd_ok = motion_ptr_->RunCommandAction(motion::CommmandAction::kCommmandReturnStandUp);
  if (!cmd_ok) {
    INFO("Protocol motion run function RunReturnStandUp() error.");
    return false;
  }
  return true;
}

bool ProtocolMotion::RunRollLeft()
{
  bool cmd_ok = motion_ptr_->RunCommandAction(motion::CommmandAction::kCommmandRollLeft);
  if (!cmd_ok) {
    INFO("Protocol motion run function RunRollLeft() error.");
    return false;
  }
  return true;
}

bool ProtocolMotion::RunBow()
{
  bool cmd_ok = motion_ptr_->RunCommandAction(motion::CommmandAction::kCommmandBow);
  if (!cmd_ok) {
    INFO("Protocol motion run function RunBow() error.");
    return false;
  }
  return true;
}

bool ProtocolMotion::RunBackflip()
{
  return true;
}

bool ProtocolMotion::RunWalkSlowly()
{
  return true;
}

bool ProtocolMotion::RunGoFast()
{
  std::array<float, 6> cmd_params{
    0.2,   // vel_x
    0.0,   // vel_y
    0.0,   // yaw
    0.05,   // step_height
    0.05
  };

  bool cmd_ok = motion_ptr_->RunServoCommandAction(
    motion::CommmandAction::kCommmandRunFast,
    cmd_params);
  if (!cmd_ok) {
    INFO("Protocol motion run function RunSelfFrequencyWalking() error.");
    return false;
  }
  return true;
}

}  // namespace protocol
}  // namespace cyberdog
