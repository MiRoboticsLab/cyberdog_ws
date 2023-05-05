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

#ifndef CYBERDOG_AFT__TASK_MANAGER_HPP_
#define CYBERDOG_AFT__TASK_MANAGER_HPP_

#include <memory>

#include "cyberdog_aft/factory_tool.hpp"
#include "cyberdog_aft/config_manager.hpp"
#include "cyberdog_aft/task_scheduler.hpp"

namespace cyberdog
{

class TaskManager
{
public:
  TaskManager();
  ~TaskManager();

  TaskManager(const TaskManager &) = delete;
  TaskManager & operator=(const TaskManager &) = delete;

private:
  std::shared_ptr<FactoryTool> factory_ {nullptr};
  std::shared_ptr<ParameterManager> parameters_ {nullptr};
  std::shared_ptr<TaskScheduler> task_scheduler_ {nullptr};
};
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__TASK_MANAGER_HPP_
