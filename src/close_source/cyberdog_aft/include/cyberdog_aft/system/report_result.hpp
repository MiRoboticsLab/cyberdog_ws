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

#ifndef CYBERDOG_AFT__SYSTEM__REPORT_RESULT_HPP_
#define CYBERDOG_AFT__SYSTEM__REPORT_RESULT_HPP_

#include <chrono>
#include <memory>
#include <deque>
#include <vector>
#include <string>
#include <functional>
#include <map>

/**
 * @brief basic report
 *
 */
namespace cyberdog
{
namespace system
{
struct ReportResult
{
  // module_name success description
  std::map<std::string, std::string> success_results;

  // module_name failure description
  std::map<std::string, std::string> failure_results;
  std::vector<std::string> total_modules;
};

struct DynamicTestReportResult
{
  // motion module response
  std::map<std::string, std::string> motion_response;
  // motor temperature responce
  std::map<std::string, std::string> temperature_response;
};

/**
 * @brief static one
 *
 */
struct StaticStageOneReportResult
{
  ReportResult battery;
  ReportResult hardware;
  ReportResult storage;
};
/**
 * @brief static two
 *
 */
struct StaticStageTwoReportResult
{
  ReportResult hardware;
  ReportResult storage;
};

/**
 * @brief dynamic
 *
 */
struct DynamicReportResult
{
  DynamicTestReportResult motion;
};


std::string ReportResult2String(const ReportResult & report);

}  // namespace system
}  // namespace cyberdog


#endif  // CYBERDOG_AFT__SYSTEM__REPORT_RESULT_HPP_
