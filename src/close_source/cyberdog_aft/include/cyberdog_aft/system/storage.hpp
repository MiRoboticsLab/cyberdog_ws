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

#ifndef CYBERDOG_AFT__SYSTEM__STORAGE_HPP_
#define CYBERDOG_AFT__SYSTEM__STORAGE_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include <deque>
#include <string>
#include <functional>
#include <map>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_aft/system/report_result.hpp"

namespace cyberdog
{
namespace system
{

class Storage
{
public:
  explicit Storage();  // NOLINT
  explicit Storage(std::shared_ptr<rclcpp::Node> node);
  ~Storage();

  Storage(const Storage &) = delete;
  Storage & operator=(const Storage &) = delete;

  enum class Type : u_int32_t
  {
    kMemory,
    kEmmc,
    kSDCard,
    kCPU,
    kFan,
    kDocument1,
    kDocument2,
    kDocument3,
    kDocument4,
    kDocument5,
  };

  using Callback = std::function<bool ()>;

  bool RunTest(const Type & storage_type);
  // All test for hardware
  bool RunAllTest();

  // Initialize
  bool InitializeTotalModuleTests(const std::vector<std::string> & modules);

  // Report
  bool GetReport(ReportResult & report);

  // Debug
  std::string ToString(const Type & type);
  Type FromString(const std::string & type);

  /**
   * @brief Debug for report message
   *
   * @param report
   */
  void ReportToString(ReportResult & report);
  bool ClearReport(ReportResult & report);

  /**
   * @brief Set test modules
   *
   */
  void InitializeTestModules();
  bool TestCPUCase();
  bool TestFanCase();
  bool DeleteDocumentPathInit(std::string & config_file, std::string document_name);
  bool TestDeleteDocumentCase() const;

private:
  bool TestMemoryCase();

  bool TestEmmcCase();

  bool TestSDCardCase();

  bool TestCPUSUCCESS();

  bool TestFanSUCCESS();

  // Record test modules
  std::vector<std::string> modules_;

  std::map<Type, Callback> functions_;
  std::deque<std::string> commands_queue_;
  ReportResult running_state_;

  std::shared_ptr<rclcpp::Node> node_ptr_ {nullptr};

  // atimic
  std::atomic_bool stoped_{false};

  bool is_delete_{true};
  std::string document_path_;
};

}  // namespace system
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__SYSTEM__STORAGE_HPP_
