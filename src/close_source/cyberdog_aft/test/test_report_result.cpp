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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <unordered_map>


#include <rclcpp/rclcpp.hpp>  // NOLINT

#include "cyberdog_aft/protocol/protocol_grpc.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"

namespace cyberdog
{
namespace system
{

class ReportResultTest : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
  }

  static void TearDownTestCase()
  {
  }

  ReportResultTest()
  {
    grpc_demo_ptr_ = std::make_shared<protocol::ProtocolGrpc>();
  }

  ReportResult CreateReportResult();
  StaticStageOneReportResult CreateStaticOneReportResult();
  StaticStageTwoReportResult CreateStaticTwoReportResult();
  DynamicReportResult CreateDynamicReportResult();

  void ReportToString(ReportResult & report);

  void TestBase();
  void TestStaticOne();
  void TestStaticTwo();
  void TestDynamic();

private:
  std::shared_ptr<protocol::ProtocolGrpc> grpc_demo_ptr_ {nullptr};
};

ReportResult ReportResultTest::CreateReportResult()
{
  ReportResult report;

  const std::string kModuleOne = "kOne";
  const std::string kModuleTwo = "kTwo";
  const std::string kModuleThree = "kThree";
  const std::string kModuleFour = "kFour";
  const std::string kModuleFive = "kFive";
  const std::string kModuleSix = "kSix";

  // total
  report.total_modules.emplace_back(kModuleOne);
  report.total_modules.emplace_back(kModuleTwo);
  report.total_modules.emplace_back(kModuleThree);
  report.total_modules.emplace_back(kModuleFour);
  report.total_modules.emplace_back(kModuleFive);
  report.total_modules.emplace_back(kModuleSix);

  // success_results
  report.success_results[kModuleOne] = "OK";
  report.success_results[kModuleTwo] = "OK";

  // failure_results
  report.failure_results[kModuleThree] = "Failed";
  report.failure_results[kModuleFour] = "Failed";
  report.failure_results[kModuleFive] = "Failed";
  report.failure_results[kModuleSix] = "Failed";

  return report;
}

StaticStageOneReportResult ReportResultTest::CreateStaticOneReportResult()
{
  StaticStageOneReportResult report;
  report.hardware = CreateReportResult();
  report.storage = CreateReportResult();
  return report;
}

StaticStageTwoReportResult ReportResultTest::CreateStaticTwoReportResult()
{
  StaticStageTwoReportResult report;
  report.hardware = CreateReportResult();
  report.storage = CreateReportResult();
  return report;
}

DynamicReportResult ReportResultTest::CreateDynamicReportResult()
{
  DynamicReportResult report;
  return report;
}

void ReportResultTest::ReportToString(ReportResult & report)
{
  INFO("[total_modules]: module num = %ld", report.total_modules.size());
  for (auto & module : report.total_modules) {
    INFO("    %s", module.c_str());
  }

  INFO("[success_modules]: module num = %ld", report.success_results.size());
  for (auto & module : report.success_results) {
    INFO("    %s: %s", module.first.c_str(), module.second.c_str());
  }

  INFO("[failure_modules]: module num = %ld", report.failure_results.size());
  for (auto & module : report.failure_results) {
    INFO("    %s: %s", module.first.c_str(), module.second.c_str());
  }
}

void ReportResultTest::TestBase()
{
  auto report = CreateReportResult();
  // print
  ReportToString(report);

  rapidjson::Document document(rapidjson::kObjectType);
  grpc_demo_ptr_->ReportResult2Rapidjson(report, document);

  std::string report_str;
  common::CyberdogJson::Document2String(document, report_str);
  INFO("%s", report_str.c_str());
}

void ReportResultTest::TestStaticOne()
{
  auto report = CreateStaticOneReportResult();

  rapidjson::Document document(rapidjson::kObjectType);
  grpc_demo_ptr_->ReportResult2Rapidjson(report, document);

  std::string report_str;
  common::CyberdogJson::Document2String(document, report_str);
  INFO("%s", report_str.c_str());
}

void ReportResultTest::TestStaticTwo()
{
  auto report = CreateStaticTwoReportResult();

  rapidjson::Document document(rapidjson::kObjectType);
  grpc_demo_ptr_->ReportResult2Rapidjson(report, document);

  std::string report_str;
  common::CyberdogJson::Document2String(document, report_str);
  INFO("%s", report_str.c_str());
}

void ReportResultTest::TestDynamic()
{
  auto report = CreateDynamicReportResult();

  rapidjson::Document document(rapidjson::kObjectType);
  grpc_demo_ptr_->ReportResult2Rapidjson(report, document);

  std::string report_str;
  common::CyberdogJson::Document2String(document, report_str);
  INFO("%s", report_str.c_str());
}

// TEST_F(ReportResultTest, base)
// {
//   TestBase();
// }

// TEST_F(ReportResultTest, stage_one)
// {
//   TestStaticOne();
// }

// TEST_F(ReportResultTest, stage_two)
// {
//   TestStaticTwo();
// }

// TEST_F(ReportResultTest, dynamic)
// {
//   TestDynamic();
// }

}  // namespace system
}  // namespace cyberdog


int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("report_result");
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
