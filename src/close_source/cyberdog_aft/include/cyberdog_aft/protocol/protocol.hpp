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


#ifndef CYBERDOG_AFT__PROTOCOL__PROTOCOL_HPP_
#define CYBERDOG_AFT__PROTOCOL__PROTOCOL_HPP_

#include "cyberdog_aft/protocol/protocol_interface.hpp"
#include "cyberdog_aft/protocol/protocol_grpc.hpp"

#include <chrono>
#include <memory>
#include <thread>

namespace cyberdog
{
namespace protocol
{

class Protocol : public ProtocolInterface
{
public:
  Protocol();
  ~Protocol();

  Protocol(const Protocol &) = delete;
  Protocol & operator=(const Protocol &) = delete;

private:
  std::shared_ptr<ProtocolGrpc> grpc_ptr_ {nullptr};
  std::shared_ptr<std::thread> grpc_thread_ {nullptr};
};


}  // namespace protocol
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__PROTOCOL__PROTOCOL_HPP_
