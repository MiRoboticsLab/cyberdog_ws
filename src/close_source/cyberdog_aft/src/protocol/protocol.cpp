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

#include "cyberdog_aft/protocol/protocol.hpp"

namespace cyberdog
{
namespace protocol
{

Protocol::Protocol()
{
  grpc_ptr_ = std::make_shared<ProtocolGrpc>();
  grpc_thread_ = std::make_shared<std::thread>(
    [&]() {
      grpc_ptr_->Spin();
    });
  grpc_thread_->join();
}

Protocol::~Protocol()
{
}

}  // namespace protocol
}  // namespace cyberdog
