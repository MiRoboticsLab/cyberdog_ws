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


#ifndef CYBERDOG_AFT__CONFIG_MANAGER_HPP_
#define CYBERDOG_AFT__CONFIG_MANAGER_HPP_

#include <vector>
#include <string>
#include <memory>

#include "cyberdog_parameter/cyberdog_parameter.hpp"

namespace cyberdog
{

class ParameterManager
{
public:
  ParameterManager();
  ~ParameterManager();

  ParameterManager(const ParameterManager &) = delete;
  ParameterManager & operator=(const ParameterManager &) = delete;

  // Load toml dynamic library
  bool LoadUnchangeableDefaultConfig(const std::string & filename);

  // Load toml config file
  bool LoadConfigurableDefaultTomlConfig(const std::string & filename);

private:
  std::shared_ptr<parameter::ParameterParser> config_parser_ptr_ {nullptr};
  bool initialized_ {false};
};

}  // namespace cyberdog

#endif  // CYBERDOG_AFT__CONFIG_MANAGER_HPP_
