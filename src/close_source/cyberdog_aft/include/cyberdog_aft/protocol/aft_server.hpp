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


#ifndef CYBERDOG_AFT__PROTOCOL__AFT_SERVER_HPP_
#define CYBERDOG_AFT__PROTOCOL__AFT_SERVER_HPP_

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

#include <iostream>
#include <random>
#include <string>
#include <chrono>
#include <memory>
#include <thread>

#include "cyberdog_aft/protocol/protocol_interface.hpp"
#include "cyberdog_aft/protocol/protocol_grpc.hpp"
#include "./cyberdog_aft.pb.h"
#include "./cyberdog_aft.grpc.pb.h"


using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;
using grpc::Status;
using std::chrono::system_clock;

namespace cyberdog
{
namespace protocol
{

class ProtocolGrpc;

class AFTServer final : public grpcapi::ServerAFT::Service
{
public:
  explicit AFTServer(const std::string & db);
  ~AFTServer();

  AFTServer(const AFTServer &) = delete;
  AFTServer & operator=(const AFTServer &) = delete;

  void SetRequesProcess(ProtocolGrpc * decision);

  ::grpc::Status SendMessages(
    ::grpc::ServerContext * context,
    const ::grpcapi::AFTRequest * request,
    ::grpc::ServerWriter<::grpcapi::AFTResponse> * writer) override;

  void set_peer_ip(const std::string & peer);

private:
  void Run();
  bool isPeerAvalible(const std::string & peer);
  std::shared_ptr<std::thread> server_thread_ptr_ {nullptr};
  ProtocolGrpc * decision_;
  std::string peer_ip_;
};


}  // namespace protocol
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__PROTOCOL__AFT_SERVER_HPP_
