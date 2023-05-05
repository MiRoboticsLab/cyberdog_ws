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

#include "cyberdog_aft/system/temp_ctrl.hpp"

namespace cyberdog
{
namespace system
{

TempCtrl::TempCtrl()
{
  functions_.emplace(Type::kCPU, std::bind(&TempCtrl::TestCPU, this));
  functions_.emplace(Type::kGPU, std::bind(&TempCtrl::TestGPU, this));
  functions_.emplace(Type::kFan, std::bind(&TempCtrl::TestFan, this));
  functions_.emplace(Type::kIMU, std::bind(&TempCtrl::TestIMU, this));
}

TempCtrl::~TempCtrl()
{
}

bool TempCtrl::RunTest(const Type & type)
{
  auto it = functions_.find(type);
  if (it == functions_.end()) {
    return false;
  }
  return functions_[type]();
}

bool TempCtrl::TestCPU()
{
  return true;
}

bool TempCtrl::TestGPU()
{
  return true;
}

bool TempCtrl::TestFan()
{
  return true;
}

bool TempCtrl::TestIMU()
{
  return true;
}


}  // namespace system
}  // namespace cyberdog
