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
#include <string>

#include "cyberdog_aft/protocol/aft_server.hpp"

namespace cyberdog
{
namespace protocol
{

AFTServer::AFTServer(const std::string & db)
: decision_(nullptr)
{
  server_thread_ptr_ = std::make_shared<std::thread>(std::bind(&AFTServer::Run, this));
}

AFTServer::~AFTServer()
{
}

void AFTServer::SetRequesProcess(ProtocolGrpc * decision)
{
  decision_ = decision;
}

::grpc::Status AFTServer::SendMessages(
  ::grpc::ServerContext * context,
  const ::grpcapi::AFTRequest * request,
  ::grpc::ServerWriter<::grpcapi::AFTResponse> * writer)
{
  if (isPeerAvalible(context->peer())) {
    decision_->HandleMessages(request, writer);
  }
  return ::grpc::Status::OK;
}

bool AFTServer::isPeerAvalible(const std::string & peer)
{
  (void)peer;
  if (decision_ == nullptr) {
    return false;
  }
  // std::cout << "peer:" << peer << "self_ip:" << decision_->getServiceIp() << std::endl;
  return true;
}

void AFTServer::Run()
{
}


}  // namespace protocol
}  // namespace cyberdog
