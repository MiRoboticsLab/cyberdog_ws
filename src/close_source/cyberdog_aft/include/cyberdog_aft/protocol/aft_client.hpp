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


#ifndef CYBERDOG_AFT__PROTOCOL__AFT_CLIENT_HPP_
#define CYBERDOG_AFT__PROTOCOL__AFT_CLIENT_HPP_

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <iostream>
#include <random>
#include <string>
#include <chrono>
#include <memory>
#include <thread>

#include "cyberdog_aft/protocol/protocol_interface.hpp"
#include "./cyberdog_aft.grpc.pb.h"

namespace cyberdog
{
namespace protocol
{

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;


class AFTClient
{
public:
  explicit AFTClient(std::shared_ptr<Channel> channel);
  ~AFTClient();

  AFTClient(const AFTClient &) = delete;
  AFTClient & operator=(const AFTClient &) = delete;

  bool SendRequest(const ::grpcapi::AFTRequest & msg);

private:
  std::unique_ptr<grpcapi::ServerAFT::Stub> stub_ {nullptr};
  std::shared_ptr<grpc::Channel> channel_ {nullptr};
};


}  // namespace protocol
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__PROTOCOL__AFT_CLIENT_HPP_
