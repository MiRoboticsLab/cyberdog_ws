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

#include <vector>
#include <string>
#include <memory>
#include <chrono>

#include "cyberdog_aft/system/storage.hpp"
#include "cyberdog_aft/utils/shell_command.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include <boost/filesystem.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
namespace cyberdog
{
namespace system
{

Storage::Storage()
{
  functions_.clear();
  functions_.emplace(
    std::make_pair(
      Type::kMemory,
      std::bind(&Storage::TestMemoryCase, this)));

  functions_.emplace(
    std::make_pair(
      Type::kEmmc,
      std::bind(&Storage::TestEmmcCase, this)));

  functions_.emplace(
    std::make_pair(
      Type::kSDCard,
      std::bind(&Storage::TestSDCardCase, this)));
  
  functions_.emplace(
    std::make_pair(
      Type::kDocument1,
      std::bind(&Storage::TestDeleteDocumentCase, this)));

  functions_.emplace(
    std::make_pair(
      Type::kDocument2,
      std::bind(&Storage::TestDeleteDocumentCase, this)));
  
  functions_.emplace(
    std::make_pair(
      Type::kDocument3,
      std::bind(&Storage::TestDeleteDocumentCase, this)));

  functions_.emplace(
    std::make_pair(
      Type::kDocument4,
      std::bind(&Storage::TestDeleteDocumentCase, this)));

  functions_.emplace(
    std::make_pair(
      Type::kDocument5,
      std::bind(&Storage::TestDeleteDocumentCase, this)));
  // functions_.emplace(
  //   std::make_pair(
  //     Type::kCPU,
  //     std::bind(&Storage::TestCPUCase, this, std::placeholders::_1)));

  InitializeTestModules();
}

Storage::Storage(std::shared_ptr<rclcpp::Node> node):node_ptr_(node)
{
  functions_.clear();

  functions_.emplace(
    std::make_pair(
      Type::kEmmc,
      std::bind(&Storage::TestEmmcCase, this)));

  functions_.emplace(
    std::make_pair(
      Type::kMemory,
      std::bind(&Storage::TestMemoryCase, this)));

  functions_.emplace(
    std::make_pair(
      Type::kSDCard,
      std::bind(&Storage::TestSDCardCase, this)));

functions_.emplace(
    std::make_pair(
      Type::kDocument1,
      std::bind(&Storage::TestDeleteDocumentCase, this)));

  functions_.emplace(
    std::make_pair(
      Type::kDocument2,
      std::bind(&Storage::TestDeleteDocumentCase, this)));
  
  functions_.emplace(
    std::make_pair(
      Type::kDocument3,
      std::bind(&Storage::TestDeleteDocumentCase, this)));

  functions_.emplace(
    std::make_pair(
      Type::kDocument4,
      std::bind(&Storage::TestDeleteDocumentCase, this)));

  functions_.emplace(
    std::make_pair(
      Type::kDocument5,
      std::bind(&Storage::TestDeleteDocumentCase, this)));

  InitializeTestModules();
}

Storage::~Storage()
{
}

bool Storage::RunTest(const Type & storage_type)
{
  auto it = functions_.find(storage_type);
  if (it == functions_.end()) {
    return false;
  }
  bool cmd_ok = functions_[storage_type]();
  running_state_.total_modules.emplace_back(ToString(storage_type));

  if (!cmd_ok) {
    running_state_.failure_results.emplace(ToString(storage_type), "Failed");
  } else {
    running_state_.success_results.emplace(ToString(storage_type), "OK");
  }

  return cmd_ok;
}

bool Storage::RunAllTest()
{
  while (!commands_queue_.empty()) {
    if (stoped_.load()) {
      break;
    }
    // cmd是枚举类测试类型
    auto cmd = FromString(commands_queue_.front());
    commands_queue_.pop_front();
    
    std::string cmd_string = ToString(cmd);
    int position = cmd_string.find("kDocument");

    if(position !=  cmd_string.npos){
      // 读取toml文件路径
      INFO("kDocument........%s:", ToString(cmd).c_str());
      std::string toml_file = ament_index_cpp::get_package_share_directory(
    "cyberdog_aft") + "/config/delete_document.toml";

    INFO("toml_file_:%s", toml_file.c_str());
      DeleteDocumentPathInit(toml_file, ToString(cmd));
      if(is_delete_){
        RunTest(cmd);
      }
    } else{
        RunTest(cmd);
    }
  }
  // get report on contronl stage
  // ReportToString(running_state_);
  return true;
}

bool Storage::InitializeTotalModuleTests(const std::vector<std::string> & modules)
{
  for (const auto & item : modules) {
    commands_queue_.push_back(item);
  }
  return true;
}

bool Storage::GetReport(ReportResult & report)
{
  report = running_state_;
  if(running_state_.failure_results.size()!= 0){
    return false;
  } else{
    return true;
  }
}

std::string Storage::ToString(const Type & type)
{
  std::string type_str = "";

  switch (type) {
    case Type::kMemory:
      type_str = "kMemory";
      break;

    case Type::kEmmc:
      type_str = "kEmmc";
      break;

    case Type::kSDCard:
      type_str = "kSDCard";
      break;

    case Type::kCPU:
      type_str = "kCPU";
      break;

    case Type::kFan:
      type_str = "kFan";
      break;

    case Type::kDocument1:
      type_str = "kDocument1";
      break;

    case Type::kDocument2:
      type_str = "kDocument2";
      break;

    case Type::kDocument3:
      type_str = "kDocument3";
      break;

    case Type::kDocument4:
      type_str = "kDocument4";
      break;

    case Type::kDocument5:
      type_str = "kDocument5";
      break;
  }
  return type_str;
}

Storage::Type Storage::FromString(const std::string & type)
{
  if (type == "kMemory") {
    return Storage::Type::kMemory;
  } else if (type == "kEmmc") {
    return Storage::Type::kEmmc;
  } else if (type == "kSDCard") {
    return Storage::Type::kSDCard;
  } else if (type == "kCPU") {
    return Storage::Type::kCPU;
  } else if (type == "kFan") {
    return Storage::Type::kFan;
  } else if (type == "kDocument1") {
    return Storage::Type::kDocument1;
  } else if (type == "kDocument2") {
    return Storage::Type::kDocument2;
  } else if (type == "kDocument3") {
    return Storage::Type::kDocument3;
  } else if (type == "kDocument4") {
    return Storage::Type::kDocument4;
  } else if (type == "kDocument5") {
    return Storage::Type::kDocument5;
  }
}

void Storage::ReportToString(ReportResult & report)
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

bool Storage::ClearReport(ReportResult & report)
{
  report.total_modules.clear();
  report.success_results.clear();
  report.failure_results.clear();
  return true;
}

void Storage::InitializeTestModules()
{
  modules_.clear();
  commands_queue_.clear();

  const std::string kModuleMemory = "kMemory";
  const std::string kModuleEmmc = "kEmmc";
  const std::string kModuleSDCard = "kSDCard";
  const std::string kModuleCPU = "kCPU";
  const std::string kModuleCFan = "kFan";
  const std::string kDocument1 = "kDocument1";
  const std::string kDocument2 = "kDocument2";
  const std::string kDocument3 = "kDocument3";
  const std::string kDocument4 = "kDocument4";
  const std::string kDocument5 = "kDocument5";

  modules_.emplace_back(kModuleMemory);
  modules_.emplace_back(kModuleEmmc);
  modules_.emplace_back(kModuleSDCard);
  modules_.emplace_back(kModuleCPU);
  modules_.emplace_back(kModuleCFan);
  modules_.emplace_back(kDocument1);
  modules_.emplace_back(kDocument2);
  modules_.emplace_back(kDocument3);
  modules_.emplace_back(kDocument4);
  modules_.emplace_back(kDocument5);

  InitializeTotalModuleTests(modules_);
}

bool Storage::TestMemoryCase()
{
  std::string test_memory_writeRead = "memtester 1024M 1 |grep ok";
  bool memory_writeRead_success = false;

  std::string memory_writeRead_result = RunShellCommand(
    test_memory_writeRead.c_str(), memory_writeRead_success);

  // extract the panel info "ok"(size is 74654) ,then return true
  if (memory_writeRead_result.size() == 74654) {
    INFO("Memory work normally");
    return true;
  }

  return false;
}

bool Storage::TestEmmcCase()
{
  // INFO("Start Storage::TestEmmcCase() function");
  std::string test_emmc_writeRead = "cd $HOME; iozone -a -p -n 0g -g 0.5g -i 0 -i 1 -f tmp -V 5 -w -R | grep complete.";  // NOLINT
  bool emmc_writeRead_success = false;

  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::string emmc_writeRead_result = RunShellCommand(test_emmc_writeRead.c_str(), emmc_writeRead_success);  // NOLINT

  if (!emmc_writeRead_success) {
    ERROR("emmc_writeRead_success : false");
    return false;
  }

  if (emmc_writeRead_result.size() == 22) {
    INFO("Emmc item test successed.");
    std::string delete_save_test = "rm /home/mi/tmp";
    bool delete_saveTest_success = false;
    RunShellCommand(delete_save_test.c_str(), delete_saveTest_success);
    return true;
  } else {
    INFO("Emmc item test failed.");
    return false;
  }
  return true;
}

bool Storage::TestSDCardCase()
{
  // INFO("Start Storage::TestSDCardCase() function");
  std::string enter_sdCard = "cd /SDCARD/";
  bool enter_sdCard_success = false;
  RunShellCommand(enter_sdCard.c_str(), enter_sdCard_success);

  if (enter_sdCard_success) {
    std::string mkdir_saveTest = "mkdir /SDCARD/save_test";
    bool makedir_saveTest_success = false;
    RunShellCommand(mkdir_saveTest.c_str(), makedir_saveTest_success);

    if (makedir_saveTest_success) {
      std::string creat_doc = "touch /SDCARD/save_test/test";
      bool creat_doc_success = false;
      RunShellCommand(creat_doc.c_str(), creat_doc_success);

      if (creat_doc_success) {
        std::string SDCARD_doc_writeRead =
          "iozone -a -n 0g -g 0.5g -i 0 -i 1 -f /SDCARD/save_test/test -Rb ./report.xls| grep complete.";   // NOLINT
        bool SDCARD_writeRead_success = false;

        std::string SDCARD_writeRead_result = RunShellCommand(
          SDCARD_doc_writeRead.c_str(), SDCARD_writeRead_success);

        if (SDCARD_writeRead_result.size() == 22) {
          // INFO("SCCard work normally");
          std::string delete_save_test = "rm -rf /SDCARD/save_test";
          bool delete_saveTest_success = false;
          RunShellCommand(delete_save_test.c_str(), delete_saveTest_success);

          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool Storage::TestCPUSUCCESS()
{
  return true;
}

bool Storage::TestCPUCase()
{
  const std::string test_CPU =
  // debug:20    dev:420
    "stress --cpu 6 --timeout 20|grep successful";
  bool test_CPU_success = false;

  std::string test_CPU_result = RunShellCommand(
    test_CPU.c_str(), test_CPU_success);

  if (test_CPU_result.size() != 0) {
    // INFO("CPU test successed.");
    functions_.insert(
     std::make_pair(
       Type::kCPU,
       std::bind(&Storage::TestCPUSUCCESS, this)));
       // running_state_.success_results.emplace("kCPU", "OK");
    return true;
  } else {
    INFO("CPU test fialed, caculation abort.");
    // running_state_.failure_results.emplace("kCPU", "Failed");
    return false;
  }
}

bool Storage::TestFanSUCCESS()
{
  return true;
}

bool Storage::TestFanCase()
{
  functions_.insert(
    std::make_pair(
      Type::kFan,
      std::bind(&Storage::TestFanSUCCESS, this)));
      running_state_.success_results.emplace(std::make_pair("kFan", "OK"));
  return true;
}

bool Storage::DeleteDocumentPathInit(std::string & config_file, std::string document_name)
{
  toml::value config;
  INFO("config_file:%s", config_file.c_str());
  if (!common::CyberdogToml::ParseFile(config_file, config)) {
    INFO("config_file:%s", config_file.c_str());
    ERROR("Toml error, can not read corresponding toml");
    return false;
  }
  toml::value value;
  INFO("Document name is %s:", document_name.c_str());
  bool get_value = common::CyberdogToml::Get(config, document_name, value);
  bool is_delete = common::CyberdogToml::Get(value, "delete", is_delete_);
  bool document_path = common::CyberdogToml::Get(value, "path", document_path_);
  INFO("Document path is %s:",document_path_.c_str());
  return get_value && is_delete && document_path;
}

// 参数 path 从toml中读取
bool Storage::TestDeleteDocumentCase() const
{
  INFO("document_path_:%s", document_path_.c_str());
  // 检查文件是否存在,该文件不存在 直接返回true
  if (!boost::filesystem::exists(document_path_)) {
    INFO("The document isn't exist");
    return true;
  }

  std::string delete_command = "rm " + document_path_;
  INFO("absolute path: %s", delete_command.c_str());
  bool delete_success = false;
  RunShellCommand(delete_command.c_str(), delete_success);
  if(delete_success){
    return true;
  } else{
    return false;
  }
}

}  // namespace system
}  // namespace cyberdog
