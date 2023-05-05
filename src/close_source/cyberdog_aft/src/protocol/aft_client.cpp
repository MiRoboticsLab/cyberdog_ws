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

#include "cyberdog_aft/protocol/aft_client.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;

namespace cyberdog
{
namespace protocol
{

AFTClient::AFTClient(std::shared_ptr<Channel> channel)
: stub_(grpcapi::ServerAFT::NewStub(channel))
{
}

AFTClient::~AFTClient()
{
}

bool AFTClient::SendRequest(const ::grpcapi::AFTRequest & msg)
{
  grpc::ClientContext context;
  ::grpcapi::AFTResponse rsp;
  std::unique_ptr<grpc::ClientReader<::grpcapi::AFTResponse>> reader(
    stub_->SendMessages(&context, msg));

  INFO("grpc start");
  while (reader->Read(&rsp)) {}
  Status status = reader->Finish();
  INFO("grpc finish");
  if (!status.ok()) {
    // INFO("Send messages error code:%d", status.error_code());
    return false;
  }
  INFO("Send messages rpc success.");
  return true;
}


}  // namespace protocol
}  // namespace cyberdog
