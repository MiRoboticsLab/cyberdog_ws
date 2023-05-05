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


#ifndef CYBERDOG_AFT__SLAM__SLAM_INTERFACE_HPP_
#define CYBERDOG_AFT__SLAM__SLAM_INTERFACE_HPP_

#include <chrono>
#include <memory>

namespace cyberdog
{
namespace slam
{

class SLAMInterface
{
public:
  SLAMInterface();
  ~SLAMInterface();

  SLAMInterface(const SLAMInterface &) = delete;
  SLAMInterface & operator=(const SLAMInterface &) = delete;
};
}  // namespace slam
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__SLAM__SLAM_INTERFACE_HPP_
