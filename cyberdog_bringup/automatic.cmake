# Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(_DEBUG_ false)

set(_tab_1 "\t")
set(_tab_2 "\t\t")
set(_tab_3 "\t\t\t")
set(_tab_4 "\t\t\t\t")

set(front_bracket "_FRONT_BRACKET_")
set(back_bracket  "_BACK_BRACKET_")

set(include_label  "<include>")
set(include_package_name ${PROJECT_NAME})
set(include_config_path "parameters")

set(yaml_bringup "bringup.yaml")
set(yaml_launch "launch.yaml")
set(yaml_node "node.yaml")

set(yaml_null_type "NoneType")
set(yaml_struct_type "struct")
set(yaml_string_type "str")
set(yaml_int_type "int")
set(yaml_float_type "float")
set(yaml_bool_type "bool")
set(yaml_sequence_type "sequence")

#
# 功能说明: 范端 shyaml 工具能力
#
function(judge_shyaml is_install_)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  set(${is_install_} FALSE PARENT_SCOPE)
  execute_process(
    COMMAND shyaml -V
  OUTPUT_VARIABLE _shyaml_info)
  string(REPLACE "\n" ";" _shyaml_info "${_shyaml_info}")
  if("${_shyaml_info}" STREQUAL "")
    set(_error_info "-bash: shyaml: command not found")
    message("┏━> 检测 shyaml 失败\n┠─> ${_error_info}\n")
    message("┗━> shyaml 尚未安装，请安装(python3 -m pip install shyaml)")
    return()
  endif()
  set(${is_install_} TRUE PARENT_SCOPE)
  if(_ARG_LOG)
    message("┠─>: 当前 shyaml 信息: ${_shyaml_info}")
  endif()
endfunction()

#
# 功能说明: 获取 通用 launch 数据
#
function(get_launch_data file_)
  set(${file_}
  "# Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.\n"
  "#\n"
  "# Licensed under the Apache License, Version 2.0 (the \"License\");\n"
  "# you may not use this file except in compliance with the License.\n"
  "# You may obtain a copy of the License at\n"
  "#\n"
  "#     http://www.apache.org/licenses/LICENSE-2.0\n"
  "#\n"
  "# Unless required by applicable law or agreed to in writing, software\n"
  "# distributed under the License is distributed on an \"AS IS\" BASIS,\n"
  "# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n"
  "# See the License for the specific language governing permissions and\n"
  "# limitations under the License.\n"
  "\n"
  "import os\n"
  "import platform\n"
  "from launch import LaunchDescription\n"
  "from launch.actions import LogInfo\n"
  "from launch.actions import DeclareLaunchArgument\n"
  "from launch.substitutions import EnvironmentVariable\n"
  "from launch.substitutions import LaunchConfiguration\n"
  "from ament_index_python.packages import get_package_share_directory\n"
  "\n"
  PARENT_SCOPE
  )
endfunction()

#
# 功能说明: 获取绝对路径下的 目标launch
#
function(get_launch_file _file_name file_)
  set(${file_}
  ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch/${_file_name}
  PARENT_SCOPE)
endfunction()

#
# 功能说明: 获取 值 类型
#
function(get_value_type _data is_number_ is_bool_ is_string_)
  set(${is_number_} FALSE PARENT_SCOPE)
  set(${is_bool_}   FALSE PARENT_SCOPE)
  set(${is_string_} FALSE PARENT_SCOPE)
  if(${_data} MATCHES "^(-)?[0-9]+(\.)?[0-9]*$")
    set(${is_number_} TRUE PARENT_SCOPE)
  elseif(${_data} MATCHES "^(TRUE|true|True|FALSE|false|False)?$")
    set(${is_bool_} TRUE PARENT_SCOPE)
  else()
    set(${is_string_} TRUE PARENT_SCOPE)
  endif()
endfunction()

#
# 功能说明: 获取 键 类型
#
function(get_key_type _file _key value_)
  execute_process(
    COMMAND cat ${PROJECT_SOURCE_DIR}/config/${_file}
    COMMAND shyaml get-type ${_key}
  OUTPUT_VARIABLE _type)
  set(${value_} ${_type} PARENT_SCOPE)
endfunction()

#
# 功能说明: 获取 键对应的列表长度
#
function(get_length _file _key value_)
  execute_process(
    COMMAND cat ${PROJECT_SOURCE_DIR}/config/${_file}
    COMMAND shyaml get-length ${_key}
  OUTPUT_VARIABLE _length)
  set(${value_} ${_length} PARENT_SCOPE)
endfunction()

#
# 功能说明: 获取 键对应的值
#
function(get_value _file _key value_)
  execute_process(
    COMMAND cat ${PROJECT_SOURCE_DIR}/config/${_file}
    COMMAND shyaml get-value ${_key}
  OUTPUT_VARIABLE _value)
  set(${value_} ${_value} PARENT_SCOPE)
endfunction()

#
# 功能说明: 获取 键 列表
#
function(get_keys _file _key keys_)
  execute_process(
    COMMAND cat ${PROJECT_SOURCE_DIR}/config/${_file}
    COMMAND shyaml get-value ${_key}
    COMMAND shyaml keys
  OUTPUT_VARIABLE _keys)
  if("${_keys}" STREQUAL "")
    message("┏━> 获取 ${_key} 失败\n┗━> ${_file} 下无 ${_key} 参数(${_keys})，请检查")
    return()
  endif()
  string(REPLACE "\n" ";" _keys_list "${_keys}")
  set(${keys_} ${_keys_list} PARENT_SCOPE)
endfunction()

#
# 功能说明: 获取 值 列表
#
function(get_sequence _file _key sequence_)
  execute_process(
    COMMAND cat ${PROJECT_SOURCE_DIR}/config/${_file}
    COMMAND shyaml get-values ${_key}
  OUTPUT_VARIABLE _sequence)
  string(REPLACE "\n" ";" _sequence_list "${_sequence}")
  set(${sequence_} ${_sequence_list} PARENT_SCOPE)
endfunction()

#
# 功能说明: 生成 launch 文件
#
function(generate_launch_file _file_name _file_data)
  if(_DEBUG_)
    message("┠─>: 正在生成 launch 文件 ${_file_name}...")
  endif()
  get_launch_file(${_file_name} _target_launch)
  list(REMOVE_AT ARGV 0)
  string(TIMESTAMP COMPILE_TIME %Y.%m.%d-%H.%M.%S)
  file(WRITE ${_target_launch} "# This is the file generated by the ${PROJECT_NAME} project in ${COMPILE_TIME}.\n")
  foreach(information  IN LISTS  ARGV)
    string(REGEX REPLACE ${front_bracket} [ information ${information})
    string(REGEX REPLACE ${back_bracket} ] information ${information})
    file(APPEND ${_target_launch} ${information})
  endforeach()
endfunction()

#
# 功能说明: 声明 arguments
#
macro(declare_argument _target_file _target_key)
  if(_DEBUG_)
    message("┠─>: 正在声明 launch arguments...")
  endif()
  get_keys(${_target_file} "data.${_target_key}" _target_key_keys)
  list(FIND _target_key_keys "arguments" _arguments_key_index)
  if(${_arguments_key_index} GREATER_EQUAL 0)
    set(_arguments_key "data.${_target_key}.arguments")
    get_key_type(${_target_file} ${_arguments_key} _arguments_type)
    if(${_arguments_type} STREQUAL ${yaml_sequence_type})
      get_length(${_target_file} ${_arguments_key} _arguments_size)
      if(${_arguments_size} GREATER 0)
        math(EXPR _arguments_size "(${_arguments_size}-1)")
        foreach(_arguments_index RANGE 0 ${_arguments_size} 1)
          get_keys(${_target_file} "${_arguments_key}.${_arguments_index}" _target_arguments_keys)
          list(APPEND _launch "${_tab_2}DeclareLaunchArgument(\n")
          set(_key_index 0)
          foreach(_target_arguments_key IN LISTS _target_arguments_keys)
            get_value(${_target_file} "${_arguments_key}.${_arguments_index}.${_target_arguments_key}" _target_key_value)
            string(REGEX REPLACE "'(.*)'" "\\1" _target_key_value "${_target_key_value}")
            if(${_key_index} EQUAL 0)
              list(APPEND _launch "${_tab_3}name='${_target_arguments_key}', default_value='${_target_key_value}',\n")
            else()
              list(APPEND _launch "${_tab_3}description='${_target_key_value}',\n")
            endif()
            math(EXPR _key_index "(${_key_index}+1)")
          endforeach()
          list(APPEND _launch "${_tab_2}),\n")
        endforeach()
      endif()
    endif()
  endif()
endmacro()

#
# 功能说明: 加载 node parameters
#
macro(include_node_parameters _target_key _target_key_size)
  if(_DEBUG_)
    message("┠─>: 正在加载 node parameters...")
  endif()
  foreach(_parameters_index RANGE 0 ${_target_key_size} 1)
    set(_now_parameters_key "${_target_key}.${_parameters_index}")
    get_keys(${yaml_node} "${_now_parameters_key}" _parameters_target_keys)
    list(LENGTH _parameters_target_keys _parameters_target_keys_size)
    if(${_parameters_target_keys_size} LESS 2)
      continue()
    endif()
    list(GET _parameters_target_keys 0 _include_parameters_key)
    if(NOT (${_include_parameters_key} STREQUAL ${include_label}))
      continue()
    endif()
    set(_now_include_parameters_key "${_now_parameters_key}.${_include_parameters_key}")
    get_key_type(${yaml_node} "${_now_include_parameters_key}" _include_key_type)
    if(${_include_key_type} STREQUAL ${yaml_struct_type})
      get_keys(${yaml_node} "${_now_include_parameters_key}" _include_key_keys)
      list(FIND _include_key_keys "yaml_name" _yaml_name_keys_index)
      if(${_yaml_name_keys_index} LESS 0)
        continue()
      endif()
      get_value(${yaml_node} "${_now_include_parameters_key}.yaml_name" _include_yaml_name)
      list(FIND _include_key_keys "package_name" _package_name_keys_index)
      if(${_package_name_keys_index} LESS 0)
        set(_include_package_name ${include_package_name})
      else()
        get_value(${yaml_node} "${_now_include_parameters_key}.package_name" _include_package_name)
      endif()
      list(FIND _include_key_keys "config_path" _config_path_keys_index)
      if(${_config_path_keys_index} LESS 0)
        set(_include_config_path ${include_config_path})
      else()
        get_value(${yaml_node} "${_now_include_parameters_key}.config_path" _include_config_path)
      endif()
    else()
      get_value(${yaml_node} "${_now_include_parameters_key}" _include_yaml_name)
      set(_include_package_name ${include_package_name})
      set(_include_config_path ${include_config_path})
    endif()

    list(GET _parameters_target_keys 1 _description_key)
    get_value(${yaml_node} "${_now_parameters_key}.${_description_key}" _description_value)
    string(REGEX REPLACE "'(.*)'" "\\1" __include_package_name "${_include_package_name}")
    string(REGEX REPLACE "'(.*)'" "\\1" __include_config_path "${_include_config_path}")
    string(REGEX REPLACE "'(.*)'" "\\1" __include_yaml_name "${_include_yaml_name}")
    string(REGEX REPLACE "'(.*)'" "\\1" __description_value "${_description_value}")
    set(__include_package_path "get_package_share_directory('${__include_package_name}')")
    set(__include_yaml "os.path.join(${__include_package_path}, '${__include_config_path}', '${__include_yaml_name}')")
    list(APPEND _launch "${_tab_3}${__include_yaml}, # ${__description_value}\n")
  endforeach()
endmacro()

#
# 功能说明: 添加 node parameters
#
macro(add_node_parameters _target_key _target_key_size)
  if(_DEBUG_)
    message("┠─>: 正在添加 node parameters...")
  endif()
  list(APPEND _launch "${_tab_3}{\n")
  foreach(_parameters_index RANGE 0 ${_target_key_size} 1)
    set(_now_parameters_key "${_target_key}.${_parameters_index}")
    get_keys(${yaml_node} "${_now_parameters_key}" _parameters_target_keys)
    list(LENGTH _parameters_target_keys _parameters_target_keys_size)
    if(${_parameters_target_keys_size} LESS 2)
      continue()
    endif()
    list(GET _parameters_target_keys 0 _parameter_key0_label)
    if(${_parameter_key0_label} STREQUAL ${include_label})
      continue()
    endif()

    set(_parameter_key0 "${_now_parameters_key}.${_parameter_key0_label}")
    get_key_type(${yaml_node} ${_parameter_key0} _parameter_key0_type)
    if(${_parameter_key0_type} STREQUAL ${yaml_sequence_type})
      get_sequence(${yaml_node} ${_parameter_key0} _parameter_key0_vector)
      set(_target_key_parameter "'${_parameter_key0_label}':${front_bracket}")
      set(_meta_key_index 0)
      foreach(_parameter_key0_meta IN LISTS _parameter_key0_vector)
        if(${_meta_key_index} EQUAL 0)
          set(_comma "")
        else()
          set(_comma ", ")
        endif()
        math(EXPR _meta_key_index "(${_meta_key_index}+1)")
        string(REGEX REPLACE "'(.*)'" "\\1" _parameter_key0_meta "${_parameter_key0_meta}")
        get_value_type(${_parameter_key0_meta} _is_number _is_bool _is_string)
        if(_is_string)
          string(CONCAT _target_key_parameter ${_target_key_parameter} "${_comma}'${_parameter_key0_meta}'")
        else()
          string(CONCAT _target_key_parameter ${_target_key_parameter} "${_comma}${_parameter_key0_meta}")
        endif()
      endforeach()
      string(CONCAT _target_key_parameter ${_target_key_parameter} "${back_bracket},  # ")
    else()
      get_value(${yaml_node} ${_parameter_key0} _parameter_key0_value)
      string(REGEX REPLACE "'(.*)'" "\\1" _parameter_key0_value "${_parameter_key0_value}")
      get_value_type(${_parameter_key0_value} _is_number _is_bool _is_string)
      if(_is_string)
        set(_target_key_parameter "'${_parameter_key0_label}':'${_parameter_key0_value}',  # ")
      else()
        set(_target_key_parameter "'${_parameter_key0_label}':${_parameter_key0_value},  # ")
      endif()
    endif()
    list(GET _parameters_target_keys 1 _parameter_key1_label)
    set(_parameter_key1 "${_now_parameters_key}.${_parameter_key1_label}")
    get_value(${yaml_node} ${_parameter_key1} _parameter_key1_value)
    string(REGEX REPLACE "'(.*)'" "\\1" _parameter_key1_value "${_parameter_key1_value}")
    string(CONCAT _target_key_parameter ${_target_key_parameter} "${_parameter_key1_value}")
    list(APPEND _launch "${_tab_4}${_target_key_parameter}\n")
  endforeach()
  list(APPEND _launch "${_tab_3}},\n")
endmacro()

#
# 功能说明: 定义 node parameters
#
macro(definition_node_parameters _target_node)
  if(_DEBUG_)
    message("┠─>: 正在定义 ${_target_node} node parameters...")
  endif()
  set(_parameters_key "data.${_target_node}.parameters")
  get_value(${yaml_node} ${_parameters_key} _parameters_value)
  if(NOT (${_parameters_value} STREQUAL "None"))
    get_length(${yaml_node} ${_parameters_key} _parameters_size)
    if(${_parameters_size})
      math(EXPR _parameters_size "(${_parameters_size}-1)")
      list(APPEND _launch "${_tab_3}parameters=${front_bracket}\n")
      include_node_parameters(${_parameters_key} ${_parameters_size})
      add_node_parameters(${_parameters_key} ${_parameters_size})
      list(APPEND _launch "${_tab_3}${back_bracket},\n")
    endif()
  endif()
endmacro()

#
# 功能说明: 定义 node remappings
#
macro(definition_node_remappings _target_node)
  if(_DEBUG_)
    message("┠─>: 正在定义 node remappings...")
  endif()
  set(_remappings_key "data.${_target_node}.remappings")
  get_value(${yaml_node} ${_remappings_key} _remappings_value)
  if(NOT (${_remappings_value} STREQUAL "None"))
    get_length(${yaml_node} ${_remappings_key} _remappings_size)
    if(${_remappings_size})
      math(EXPR _remappings_size "(${_remappings_size}-1)")
      list(APPEND _launch "${_tab_3}remappings=${front_bracket}\n")
      foreach(remappings_index RANGE 0 ${_remappings_size} 1)
        get_keys(${yaml_node} "${_remappings_key}.${remappings_index}" _target_remappings_keys)
        list(LENGTH _target_remappings_keys _target_remappings_keys_size)
        if(${_target_remappings_keys_size} LESS 2)
          set(_remappings_msg "${yaml_node} 内 ${_target_node} node remappings.${remappings_index}")
          message("┏━> 忽略 ${_remappings_msg}\n┗━> remappings key 非法，请检查")
          continue()
        endif()
        get_value(${yaml_node} "${_remappings_key}.${remappings_index}.from" _target_remappings_from_value)
        get_value(${yaml_node} "${_remappings_key}.${remappings_index}.to" _target_remappings_to_value)
        string(REGEX REPLACE "'(.*)'" "\\1" __target_remappings_from_value "${_target_remappings_from_value}")
        string(REGEX REPLACE "'(.*)'" "\\1" __target_remappings_to_value "${_target_remappings_to_value}")
        list(APPEND _launch "${_tab_4}('${__target_remappings_from_value}', '${__target_remappings_to_value}'),\n")
      endforeach()
      list(APPEND _launch "${_tab_3}${back_bracket},\n")
    endif()
  endif()
endmacro()

#
# 功能说明: 定义 arguments
#
macro(definition_argument _target_file _target_key _target_name)
  if(_DEBUG_)
    message("┠─>: 正在定义 launch arguments...")
  endif()
  get_keys(${_target_file} "data.${_target_key}" _target_key_keys)
  list(FIND _target_key_keys "arguments" _arguments_key_index)
  if(${_arguments_key_index} GREATER_EQUAL 0)
    set(_arguments_key "data.${_target_key}.arguments")
    get_value(${_target_file} "${_arguments_key}" _target_value)
    if(NOT (${_target_value} STREQUAL "None"))
      get_length(${_target_file} "${_arguments_key}" _arguments_key_size)
      if(${_arguments_key_size})
        math(EXPR _arguments_key_size "(${_arguments_key_size}-1)")
        list(APPEND _launch "${_tab_3}${_target_name}={\n")
        foreach(_arguments_index RANGE 0 ${_arguments_key_size} 1)
          get_keys(${_target_file} "${_arguments_key}.${_arguments_index}" _target_arguments_keys)
          foreach(_target_arguments_key IN LISTS _target_arguments_keys)
            list(APPEND _launch "${_tab_4}'${_target_arguments_key}': LaunchConfiguration('${_target_arguments_key}'),\n")
            break()
          endforeach()
        endforeach()
        list(APPEND _launch "${_tab_3}}.items(),\n")
      endif()
    endif()
  endif()
endmacro()

#
# 功能说明: 定义 node
#
macro(definition_node _target_node)
  if(_DEBUG_)
    message("┠─>: 正在定义 ${_target_node} node...")
  endif()
  list(APPEND _launch "${_tab_2}Node(\n")
  get_keys(${yaml_node} "data.${_target_node}" _target_node_keys)
  foreach(_key IN LISTS _target_node_keys)
    get_value(${yaml_node} "data.${_target_node}.${_key}" _key_value)
    if(${_key_value} STREQUAL "None")
      continue()  # 节点属性为空，忽略
    endif()
    if((${_key} STREQUAL "namespace") OR
        (${_key} STREQUAL "package") OR
        (${_key} STREQUAL "executable") OR
        (${_key} STREQUAL "name") OR
        (${_key} STREQUAL "exec_name"))
      list(APPEND _launch "${_tab_3}${_key}='${_key_value}',\n")
    elseif(${_key} STREQUAL "parameters")
      definition_node_parameters(${_target_node})
    elseif(${_key} STREQUAL "remappings")
      definition_node_remappings(${_target_node})
    elseif(${_key} STREQUAL "arguments")
      definition_argument(${yaml_node} ${_target_node} "arguments")
    endif()
  endforeach()
  list(APPEND _launch "${_tab_2}),\n")
endmacro()

#
# 功能说明: 判断 launch
#
function(judge_launch _target_launch is_launch_ launch_name_)
  if(_DEBUG_)
    message("┠─>: 正在判断 launch...")
  endif()
  set(${is_launch_} FALSE PARENT_SCOPE)
  set(${launch_name_} "null" PARENT_SCOPE)
  set(_launch_msg "${yaml_launch} 内 ${_target_launch} launch")
  get_keys(${yaml_launch} "data.${_target_launch}" _launch_keys)
  list(FIND _launch_keys "package_name" _package_name_index)
  if(${_package_name_index} GREATER_EQUAL 0)
    get_key_type(${yaml_launch} "data.${_target_launch}.package_name" _package_name_type)
    if(${_package_name_type} STREQUAL ${yaml_string_type})
      return()  # 手动，无需自动生成
    else()
      message("┏━> ${_launch_msg} 将按生成模式处理 ")
      message("┣━> package_name 非字符串")
      message("┗━> 如果确定自动生成建议删除 package_name 字段")
    endif()
  endif()
  list(FIND _launch_keys "file_name" _file_name_index)
  if(${_file_name_index} LESS 0)
    message("┏━> 忽略 ${_launch_msg}\n┗━> 无 file_name 参数，请检查")
    return()
  endif()
  get_key_type(${yaml_launch} "data.${_target_launch}.file_name" _file_name_type)
  if(NOT ${_file_name_type} STREQUAL ${yaml_string_type})
    message("┏━> 忽略 ${_launch_msg}\n┗━> 无 file_name 非字符串，请检查")
    return()
  endif()
  get_value(${yaml_launch} "data.${_target_launch}.file_name" _file_name_value)
  if(NOT ${_file_name_value} MATCHES ".*(\.py)+$")
    message("┏━> 忽略 ${_launch_msg}\n┗━> file_name 非 python 格式(*.py)，请检查")
    return()
  endif()
  set(${launch_name_} ${_file_name_value} PARENT_SCOPE)
  list(FIND _launch_keys "nodes" _nodes_index)
  if(${_nodes_index} LESS 0)
    message("┏━> 忽略 ${_launch_msg}\n┗━> 无 nodes 键，无自动生成实体，请检查")
    return()
  endif()
  get_key_type(${yaml_launch} "data.${_target_launch}.nodes" _nodes_type)
  if(NOT ${_nodes_type} STREQUAL ${yaml_sequence_type})
    message("┏━> 忽略 ${_launch_msg}\n┗━> nodes 值类型非序列，请检查")
    return()
  endif()
  get_length(${yaml_launch} "data.${_target_launch}.nodes" _nodes_size)
  if(${_nodes_size} LESS_EQUAL 0)
    message("┏━> 忽略 ${_launch_msg}\n┗━> 无 nodes 值，自动生成实体为空，请检查")
    return()
  endif()
  set(${is_launch_} TRUE PARENT_SCOPE)
endfunction()

#
# 功能说明: 判断 node
#
function(judge_node _target_node is_node_)
  if(_DEBUG_)
    message("┠─>: 正在判断 node...")
  endif()
  set(${is_node_} FALSE PARENT_SCOPE)
  set(_node_msg "${yaml_node} 内 ${_target_node} node")
  get_keys(${yaml_node} "data" _node_data_keys)
  list(FIND _node_data_keys "${_target_node}" _target_node_index)
  if(${_target_node_index} LESS 0)
    message("┏━> 忽略 ${_node_msg}\n┗━> 无 ${_target_node} 参数，请检查")
    return()
  endif()
  get_keys(${yaml_node} "data.${_target_node}" _target_node_keys)
  list(FIND _target_node_keys "package" _package_index)
  list(FIND _target_node_keys "executable" _executable_index)
  if(${_package_index} LESS 0)
    message("┏━> 忽略 ${_node_msg}\n┗━> 无 package 参数，请检查")
    return()
  elseif(${_executable_index} LESS 0)
    message("┏━> 忽略 ${_node_msg}\n┗━> 无 executable 参数，请检查")
    return()
  endif()
  set(${is_node_} TRUE PARENT_SCOPE)
endfunction()

#
# 功能说明: 自动生成 launch 模块文件
#
function(automatically_generate_module_launch_files)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  if(_ARG_LOG)
    message("┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━>")
    message("┠─>: 正在生成 模块launch 文件...")
  endif()
  get_keys(${yaml_launch} "data" _launch_data_keys)
  foreach(_now_launch IN LISTS _launch_data_keys)
    judge_launch(${_now_launch} _is_launch _launch_name)
    if(NOT ${_is_launch})
      continue()
    endif()
    if(_ARG_LOG)
      message("┠────────────────────────────>")
      message("┠─>: 正在生成 ${_now_launch} launch 文件...")
    endif()
    get_launch_data(_launch)
    list(APPEND _launch
    "from launch_ros.actions import Node\n"
    "def generate_launch_description():\n"
    "${_tab_1}user_env_var = 'USERNAME' if platform.system() == 'Windows' else 'USER'\n"
    "${_tab_1}return LaunchDescription(${front_bracket}\n"
    "${_tab_2}LogInfo(msg=(EnvironmentVariable(name=user_env_var), ' 正在初始化 ${_now_launch} launch ...')),\n"
    )
    get_sequence(${yaml_launch} "data.${_now_launch}.nodes" _nodes_value)
    foreach(_now_node IN LISTS _nodes_value)
      if(_ARG_LOG)
        message("┠─>: 正在加载 ${_now_node} node 信息...")
      endif()
      judge_node(${_now_node} _is_node)
      if(NOT ${_is_node})
        continue()
      endif()
      declare_argument(${yaml_node} ${_now_node})
      definition_node(${_now_node})
    endforeach()
    list(APPEND _launch "${_tab_1}${back_bracket})\n")
    generate_launch_file(${_launch_name} ${_launch})
  endforeach()
endfunction()

#
# 功能说明: 判断 bringup
#
function(judge_bringup is_bringup_ launch_name_)
  if(_DEBUG_)
    message("┠─>: 正在判断 bringup...")
  endif()
  set(${is_bringup_} FALSE PARENT_SCOPE)
  set(${launch_name_} "null" PARENT_SCOPE)
  get_keys(${yaml_bringup} "data" _bringup_data_keys)
  list(FIND _bringup_data_keys "file_name" _file_name_index)
  if(${_file_name_index} LESS 0)
    message("┏━> 退出当前编译\n┗━> ${yaml_bringup} 无 file_name 参数，请检查")
    return()
  endif()
  get_value(${yaml_bringup} "data.file_name" _file_name_value)
  if(NOT ${_file_name_value} MATCHES ".*(\.py)+$")
    message("┏━> 退出当前编译\n┗━> ${yaml_bringup} file_name 非 python 格式(*.py)，请检查")
    return()
  endif()
  set(${launch_name_} ${_file_name_value} PARENT_SCOPE)
  list(FIND _bringup_data_keys "launchs" _launchs_index)
  if(${_launchs_index} LESS 0)
    message("┏━> 忽略 ${yaml_bringup}\n┗━> 无 launchs 键，无自动生成实体，请检查")
    return()
  endif()
  get_key_type(${yaml_bringup} "data.launchs" _launchs_type)
  if(NOT ${_launchs_type} STREQUAL ${yaml_sequence_type})
    message("┏━> 忽略 ${yaml_bringup}\n┗━> launchs 值类型非序列，请检查")
    return()
  endif()
  set(${is_bringup_} TRUE PARENT_SCOPE)
endfunction()

#
# 功能说明: 添加 launch 元
#
macro(add_launch_meta _target_launch)
  if(_DEBUG_)
    message("┠─>: 正在添加 ${_target_launch} launch 元...")
  endif()
  set(_launch_msg "${yaml_launch} 内 ${_target_launch} launch")
  get_keys(${yaml_launch} "data.${_target_launch}" _launch_keys)
  list(FIND _launch_keys "package_name" _package_name_index)
  if(${_package_name_index} GREATER_EQUAL 0)
    get_key_type(${yaml_launch} "data.${_target_launch}.package_name" _package_name_type)
    if(${_package_name_type} STREQUAL ${yaml_string_type})
      get_value(${yaml_launch} "data.${_target_launch}.package_name" _target_package)
    else()
      message("┏━> ${_launch_msg} 将按生成模式处理 ")
      message("┣━> package_name 非字符串")
      message("┗━> 如果确定自动生成建议删除 package_name 字段")
      set(_target_package ${PROJECT_NAME})
    endif()
  endif()
  list(FIND _launch_keys "file_name" _file_name_index)
  if(${_file_name_index} GREATER_EQUAL 0)
    get_key_type(${yaml_launch} "data.${_target_launch}.file_name" _file_name_type)
    if(${_file_name_type} STREQUAL ${yaml_string_type})
      get_value(${yaml_launch} "data.${_target_launch}.file_name" _target_file)
      if(${_target_file} MATCHES ".*(\.py)+$")
        set(_launch_file_source "os.path.join(get_package_share_directory('${_target_package}'), 'launch', '${_target_file}')")
        list(APPEND _launch "${_tab_2}LogInfo(msg=('正在加载 launch 文件:', ${_launch_file_source})),\n")
        declare_argument(${yaml_launch} ${_target_launch})
        list(APPEND _launch
        "${_tab_2}IncludeLaunchDescription(\n"
        "${_tab_3}PythonLaunchDescriptionSource(${_launch_file_source}),\n"
        )
        definition_argument(${yaml_launch} ${_target_launch} "launch_arguments")
        list(APPEND _launch "${_tab_2}),\n")
      else()
        message("┏━> 忽略 ${_launch_msg}\n┗━> file_name 非 python 格式(*.py)，请检查")
      endif()
    else()
      message("┏━> 忽略 ${_launch_msg}\n┗━> file_name 非字符串，请检查")
    endif()
  else()
    message("┏━> 忽略 ${_launch_msg}\n┗━> 无 file_name 参数，请检查")
  endif()
endmacro()

#
# 功能说明: 自动生成 launch 集合文件
#
function(automatically_generate_launch_gather_files)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  if(_ARG_LOG)
    message("┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━>")
    message("┠─>: 正在生成 集合launch 文件...")
  endif()
  judge_bringup(_is_bringup _launch_name)
  if(NOT ${_is_bringup})
    return()
  endif()
  get_launch_data(_launch)
  list(APPEND _launch
  "from launch.actions import IncludeLaunchDescription\n"
  "from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
  "\n"
  "def generate_launch_description():\n"
  "${_tab_1}user_env_var = 'USERNAME' if platform.system() == 'Windows' else 'USER'\n"
  "${_tab_1}return LaunchDescription(${front_bracket}\n"
  "${_tab_2}LogInfo(msg=(EnvironmentVariable(name=user_env_var), ' 正在初始化 launch ...')),\n"
  )
  get_sequence(${yaml_bringup} "data.launchs" _launchs_value)
  foreach(_now_launch IN LISTS _launchs_value)
    if(_ARG_LOG)
      message("┠─>: 正在加载 ${_now_launch} launch 信息...")
    endif()
    add_launch_meta(${_now_launch})
  endforeach()
  list(APPEND _launch "${_tab_1}${back_bracket})\n")
  generate_launch_file(${_launch_name} ${_launch})
endfunction()

#
# 功能说明: 自动生成 节点launch 文件
#
function(automatically_generate_node_launch_files)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  if(_ARG_LOG)
    message("┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━>")
    message("┠─>: 正在生成 节点launch 文件...")
  endif()
  get_keys(${yaml_node} "data" _node_data_keys)
  foreach(_now_node IN LISTS _node_data_keys)
    set(_launch_name "node.${_now_node}.launch.py")
    if(_ARG_LOG)
      message("┠─>: 正在生成 ${_launch_name} launch 文件...")
    endif()
    get_launch_data(_launch)
    list(APPEND _launch
    "from launch_ros.actions import Node\n"
    "def generate_launch_description():\n"
    "${_tab_1}user_env_var = 'USERNAME' if platform.system() == 'Windows' else 'USER'\n"
    "${_tab_1}return LaunchDescription(${front_bracket}\n"
    "${_tab_2}LogInfo(msg=(EnvironmentVariable(name=user_env_var), ' 正在初始化 ${_launch_name} launch ...')),\n"
    )
    judge_node(${_now_node} _is_node)
    if(NOT ${_is_node})
      continue()
    endif()
    declare_argument(${yaml_node} ${_now_node})
    definition_node(${_now_node})
    list(APPEND _launch "${_tab_1}${back_bracket})\n")
    generate_launch_file(${_launch_name} ${_launch})
  endforeach()
endfunction()

#
# 功能说明: 自动生成 launch 文件
#
# :param LOG: 如果设置 LOG 则打印日志
# :type LOG: 选项
# :param NODE: 如果设置 NODE 则为每个节点生成一个启动文件
# :type NODE: 选项
# 举例:
# 1. automatically_generate_launch_files()                        # 生成 bringup 的启动文件
# 2. automatically_generate_launch_files(NODE)                    # 生成 bringup 和 每个节点的启动文件
# 3. automatically_generate_launch_files(NODE LOG) # 生成 bringup 和 每个节点的启动文件并输出日志
#
function(automatically_generate_launch_files)
  cmake_parse_arguments(_ARG "LOG;NODE" "" "" ${ARGN})
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR "lcmidl_generate_interfaces() 必须在 ament_package() 之前调用")
  endif()
  if(_ARG_LOG)
    message("\n┏━>: 开始依据 ${PROJECT_SOURCE_DIR}/config/*.yaml 文件生成 launch 文件...")
    set(_ARG_LOG_ "LOG")
  endif()
  judge_shyaml(_is_install ${_ARG_LOG_})
  if(NOT ${_is_install})
    return()
  endif()
  if(_ARG_NODE)
    automatically_generate_node_launch_files(${_ARG_LOG_})
  endif()
  automatically_generate_module_launch_files(${_ARG_LOG_})
  automatically_generate_launch_gather_files(${_ARG_LOG_})
  if(_ARG_LOG)
    message("┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━>")
    message("┗━>: 生成 launch 文件结束，已安装到 ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch 路径下。")
  endif()
endfunction()
