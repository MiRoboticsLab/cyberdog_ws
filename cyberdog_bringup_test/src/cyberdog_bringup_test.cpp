// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "cyberdog_common/cyberdog_log.hpp"

using namespace std::chrono_literals;

void print_usage()
{
  std::cout <<
    "Usage: message_lost_talker [-h] [-s SIZE]\n\n"
    "optional arguments:\n"
    "\t-h:                           Print this help message.\n"
    "\t-s <message_size>:            Message size in KiB, default to 8192 KiB" <<
    std::endl;
}

class TestNode : public rclcpp::Node
{
public:
  TestNode()
  : Node("cyberdog_bringup_test")
  {
    INFO_STREAM("当前 node 信息:" <<
      "\nname = " << this->get_name() <<
      "\nnamespace = " << this->get_namespace() <<
      "\nfully qualified name = " << this->get_fully_qualified_name()
    );

    auto options = rclcpp::NodeOptions();
    INFO_STREAM("当前 options 信息:" <<
      "\nuse_global_arguments = " << options.use_global_arguments() <<
      "\nget_rcl_node_options()->use_global_arguments = " << options.get_rcl_node_options()->use_global_arguments
    );
    
    const std::vector<std::string> & args = this->get_node_options().arguments();
    INFO_STREAM("+++>" << args.size());
    for (size_t i = 0; i < args.size(); i++)
    {
      INFO_STREAM("+++>" << args[i]);
    }
    
    // if (args.size()) {
    //   // Argument 用法
    //   if (args.cend() != std::find(args.cbegin(), args.cend(), "-h")) {
    //     // print_usage();
    //     // TODO(ivanpauno)：更新 rclcpp_components 模板以能够处理异常。 在这里举一个，所以堆栈展开优雅地发生。
    //     std::exit(0);
    //   }
    //   // Argument: message size
    //   auto opt_it = std::find(args.cbegin(), args.cend(), "-s");
    //   if (opt_it != args.cend()) {
    //     ++opt_it;
    //     if (opt_it == args.cend()) {
    //       print_usage();
    //       std::cout << "\n-s must be followed by a possitive integer" << std::endl;
    //       // TODO(ivanpauno): Update the rclcpp_components template to be able to handle
    //       // exceptions. Raise one here, so stack unwinding happens gracefully.
    //       std::exit(0);
    //     }
    //     std::istringstream input_stream(*opt_it);
    //     input_stream >> message_size_;
    //     if (!input_stream) {
    //       print_usage();
    //       std::cout << "\n-s must be followed by a possitive integer, got: '" <<
    //         *opt_it << "'" << std::endl;
    //       // TODO(ivanpauno): Update the rclcpp_components template to be able to handle
    //       // exceptions. Raise one here, so stack unwinding happens gracefully.
    //       std::exit(0);
    //     }
    //     message_size_ *= 1024uL;
    //   }
    // }
    // 申明 node.yaml 配置参数
    this->declare_parameter("simulator", false);
    this->declare_parameter("parameters_bool", false);
    this->declare_parameter("parameters_int", 0);
    this->declare_parameter("parameters_double", 0.0);
    this->declare_parameter("parameters_string0", "null");
    this->declare_parameter("parameters_string1", "null");
    this->declare_parameter("parameters_string2", "null");
    this->declare_parameter("parameters_yaml", "null");
    this->declare_parameter("parameters_bool_vector", std::vector<bool>{});
    this->declare_parameter("parameters_int_vector", std::vector<int64_t>{});
    this->declare_parameter("parameters_double_vector", std::vector<double>{});
    this->declare_parameter("parameters_string_vector", std::vector<std::string>{});
    // 申明 cyberdog_bringup_test 包下 配置参数
    this->declare_parameter("cyberdog_bringup_test_parameter_int0", 0);
    this->declare_parameter("cyberdog_bringup_test_parameter_int1", 0);
    this->declare_parameter("cyberdog_bringup_test_parameter_int2", 0);
    this->declare_parameter("cyberdog_bringup_test_parameter_int3", 0);
    // 申明 cyberdog_bringup 包下 配置参数
    this->declare_parameter("cyberdog_bringup_parameter_int0", 0);
    this->declare_parameter("cyberdog_bringup_parameter_int1", 0);
    this->declare_parameter("cyberdog_bringup_parameter_int2", 0);
    this->declare_parameter("cyberdog_bringup_parameter_int3", 0);
    this->declare_parameter("parameter.parameters_bool", false);
    this->declare_parameter("parameter.parameters_int", 0);
    this->declare_parameter("parameter.parameters_double", 0.0);
    this->declare_parameter("parameter.parameters_string0", "null");
    this->declare_parameter("parameter.parameters_string1", "null");
    this->declare_parameter("parameter.parameters_string2", "null");
    this->declare_parameter("parameter.vector.parameters_bool_vector", std::vector<bool>{});
    this->declare_parameter("parameter.vector.parameters_int_vector", std::vector<int64_t>{});
    this->declare_parameter("parameter.vector.parameters_double_vector", std::vector<double>{});
    this->declare_parameter("parameter.vector.parameters_string_vector", std::vector<std::string>{});

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "等待服务时中断。 退出");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "服务不可用，再次等待...");
    }

    // // 设置几种不同类型的参数。
    // auto set_parameters_results = parameters_client->set_parameters(
    //   {
    //     rclcpp::Parameter("parameters_bool", true),
    //     rclcpp::Parameter("parameters_int", 100),
    //     rclcpp::Parameter("parameters_double", 0.123),
    //     rclcpp::Parameter("parameters_bool_vector", std::vector<bool>({true, false})),
    //     rclcpp::Parameter("parameters_int_vector", std::vector<int64_t>({1, 2})),
    //   });
    // // 检查它们是否已设置。
    // for (auto & result : set_parameters_results) {
    //   if (!result.successful) {
    //     INFO("设置参数失败: %s", result.reason.c_str());
    //   }
    // }

    std::stringstream msg;
    for (
      auto & parameter : parameters_client->get_parameters(
      { "simulator",
        "parameters_bool", 
        "parameters_int", 
        "parameters_double", 
        "parameters_string0",
        "parameters_string1",
        "parameters_string2",
        "parameters_yaml",
        "parameters_bool_vector",
        "parameters_int_vector",
        "parameters_double_vector",
        "parameters_string_vector",
        "cyberdog_bringup_test_parameter_int0",
        "cyberdog_bringup_test_parameter_int1",
        "cyberdog_bringup_test_parameter_int2",
        "cyberdog_bringup_test_parameter_int3",
        "cyberdog_bringup_parameter_int0",
        "cyberdog_bringup_parameter_int1",
        "cyberdog_bringup_parameter_int2",
        "cyberdog_bringup_parameter_int3",
        "parameter.parameters_bool", 
        "parameter.parameters_int", 
        "parameter.parameters_double", 
        "parameter.parameters_string0",
        "parameter.parameters_string1",
        "parameter.parameters_string2",
        "parameter.vector.parameters_bool_vector",
        "parameter.vector.parameters_int_vector",
        "parameter.vector.parameters_double_vector",
        "parameter.vector.parameters_string_vector"
      }))
    {
      msg << "\nParameter name: " << parameter.get_name();
      msg << "\nParameter value (" << parameter.get_type_name() << "): " <<
        parameter.value_to_string();
    }
    INFO_STREAM("参数属性:" << msg.str().c_str());
    // 加载 node.yaml 配置参数
    this->get_parameter("simulator",                this->_simulator);
    this->get_parameter("parameters_bool",          this->_parameters_bool);
    this->get_parameter("parameters_int",           this->_parameters_int);
    this->get_parameter("parameters_double",        this->_parameters_double);
    this->get_parameter("parameters_string0",       this->_parameters_string0);
    this->get_parameter("parameters_string1",       this->_parameters_string1);
    this->get_parameter("parameters_string2",       this->_parameters_string2);
    this->get_parameter("parameters_yaml",          this->_parameters_yaml);
    this->get_parameter("parameters_bool_vector",   this->_parameters_bool_vector);
    this->get_parameter("parameters_int_vector",    this->_parameters_int_vector);
    this->get_parameter("parameters_double_vector", this->_parameters_double_vector);
    this->get_parameter("parameters_string_vector", this->_parameters_string_vector);
    // 加载 cyberdog_bringup_test 包下 配置参数
    this->get_parameter("cyberdog_bringup_test_parameter_int0", this->_include_cyberdog_bringup_test_parameter_int0);
    this->get_parameter("cyberdog_bringup_test_parameter_int1", this->_include_cyberdog_bringup_test_parameter_int1);
    this->get_parameter("cyberdog_bringup_test_parameter_int2", this->_include_cyberdog_bringup_test_parameter_int2);
    this->get_parameter("cyberdog_bringup_test_parameter_int3", this->_include_cyberdog_bringup_test_parameter_int3);
    // 加载 cyberdog_bringup 包下 配置参数
    this->get_parameter("cyberdog_bringup_parameter_int0", this->_include_cyberdog_bringup_parameter_int0);
    this->get_parameter("cyberdog_bringup_parameter_int1", this->_include_cyberdog_bringup_parameter_int1);
    this->get_parameter("cyberdog_bringup_parameter_int2", this->_include_cyberdog_bringup_parameter_int2);
    this->get_parameter("cyberdog_bringup_parameter_int3", this->_include_cyberdog_bringup_parameter_int3);
    this->get_parameter("parameter.parameters_bool", this->_include_cyberdog_bringup_parameters_bool);
    this->get_parameter("parameter.parameters_int", this->_include_cyberdog_bringup_parameters_int);
    this->get_parameter("parameter.parameters_double", this->_include_cyberdog_bringup_parameters_double);
    this->get_parameter("parameter.parameters_string0", this->_include_cyberdog_bringup_parameters_string0);
    this->get_parameter("parameter.parameters_string1", this->_include_cyberdog_bringup_parameters_string1);
    this->get_parameter("parameter.parameters_string2", this->_include_cyberdog_bringup_parameters_string2);
    this->get_parameter("parameter.vector.parameters_bool_vector", this->_include_cyberdog_bringup_parameters_bool_vector);
    this->get_parameter("parameter.vector.parameters_int_vector", this->_include_cyberdog_bringup_parameters_int_vector);
    this->get_parameter("parameter.vector.parameters_double_vector", this->_include_cyberdog_bringup_parameters_double_vector);
    this->get_parameter("parameter.vector.parameters_string_vector", this->_include_cyberdog_bringup_parameters_string_vector);

    INFO_EXPRESSION(this->_simulator, "simulator = TRUE(开启模拟器模式)");
    INFO_EXPRESSION(!this->_simulator, "simulator = FALSE(关闭模拟器模式)");
    INFO_STREAM("node.yaml 配置参数" <<
      "\nparameters_bool = " << this->_parameters_bool <<
      "\nparameters_int = " << this->_parameters_int <<
      "\nparameters_double = " << this->_parameters_double <<
      "\nparameters_string0 = " << this->_parameters_string0 <<
      "\nparameters_string1 = " << this->_parameters_string1 <<
      "\nparameters_string2 = " << this->_parameters_string2 <<
      "\nparameters_yaml = " << this->_parameters_yaml
    );
    this->info_vector("parameters_bool_vector", this->_parameters_bool_vector);
    this->info_vector("parameters_int_vector", this->_parameters_int_vector);
    this->info_vector("parameters_double_vector", this->_parameters_double_vector);
    this->info_vector("parameters_string_vector", this->_parameters_string_vector);

    INFO_STREAM("cyberdog_bringup_test 包下 配置参数" <<
      "\ncyberdog_bringup_test_parameter_int0 = " << this->_include_cyberdog_bringup_test_parameter_int0 <<
      "\ncyberdog_bringup_test_parameter_int1 = " << this->_include_cyberdog_bringup_test_parameter_int1 <<
      "\ncyberdog_bringup_test_parameter_int2 = " << this->_include_cyberdog_bringup_test_parameter_int2 <<
      "\ncyberdog_bringup_test_parameter_int3 = " << this->_include_cyberdog_bringup_test_parameter_int3
    );

    INFO_STREAM("cyberdog_bringup 包下 配置参数" <<
      "\ncyberdog_bringup_parameter_int0 = " << this->_include_cyberdog_bringup_parameter_int0 <<
      "\ncyberdog_bringup_parameter_int1 = " << this->_include_cyberdog_bringup_parameter_int1 <<
      "\ncyberdog_bringup_parameter_int2 = " << this->_include_cyberdog_bringup_parameter_int2 <<
      "\ncyberdog_bringup_parameter_int3 = " << this->_include_cyberdog_bringup_parameter_int3 <<
      "\nparameter.parameters_bool = " << this->_include_cyberdog_bringup_parameters_bool <<
      "\nparameter.parameters_int = " << this->_include_cyberdog_bringup_parameters_int <<
      "\nparameter.parameters_double = " << this->_include_cyberdog_bringup_parameters_double <<
      "\nparameter.parameters_string0 = " << this->_include_cyberdog_bringup_parameters_string0 <<
      "\nparameter.parameters_string1 = " << this->_include_cyberdog_bringup_parameters_string1 <<
      "\nparameter.parameters_string2 = " << this->_include_cyberdog_bringup_parameters_string2
    );

    this->info_vector("parameter.vector.parameters_bool_vector", this->_include_cyberdog_bringup_parameters_bool_vector);
    this->info_vector("parameter.vector.parameters_int_vector", this->_include_cyberdog_bringup_parameters_int_vector);
    this->info_vector("parameter.vector.parameters_double_vector", this->_include_cyberdog_bringup_parameters_double_vector);
    this->info_vector("parameter.vector.parameters_string_vector", this->_include_cyberdog_bringup_parameters_string_vector);
  }

private:
  bool        _simulator {false};       // 是否开启模拟器模式？
  bool        _parameters_bool {false}; // bool 参数
  int         _parameters_int {0};      // int 参数
  double      _parameters_double {0.0}; // float 参数
  std::string _parameters_string0 {""}; // string 键值对(单string):无引号方式约束
  std::string _parameters_string1 {""}; // string 键值对(多string):单引号方式约束
  std::string _parameters_string2 {""}; // string 键值对(多string):双引号方式约束
  std::string _parameters_yaml {""};    // 传入参数文件名称，由程序内部自行解析
  std::vector<bool>         _parameters_bool_vector;    // bool 键值对(数组)
  std::vector<int64_t>      _parameters_int_vector;     // int 键值对(数组)
  std::vector<double>       _parameters_double_vector;   // double 键值对(数组)
  std::vector<std::string>  _parameters_string_vector;  // string 键值对(数组)
  int         _include_cyberdog_bringup_test_parameter_int0 {0}; // int 参数
  int         _include_cyberdog_bringup_test_parameter_int1 {0}; // int 参数
  int         _include_cyberdog_bringup_test_parameter_int2 {0}; // int 参数
  int         _include_cyberdog_bringup_test_parameter_int3 {0}; // int 参数
  int         _include_cyberdog_bringup_parameter_int0 {0};      // int 参数
  int         _include_cyberdog_bringup_parameter_int1 {0};      // int 参数
  int         _include_cyberdog_bringup_parameter_int2 {0};      // int 参数
  int         _include_cyberdog_bringup_parameter_int3 {0};      // int 参数
  bool        _include_cyberdog_bringup_parameters_bool {false}; // bool 参数
  int         _include_cyberdog_bringup_parameters_int {0};      // int 参数
  double      _include_cyberdog_bringup_parameters_double {0.0}; // float 参数
  std::string _include_cyberdog_bringup_parameters_string0 {""}; // string 键值对(单string):无引号方式约束
  std::string _include_cyberdog_bringup_parameters_string1 {""}; // string 键值对(多string):单引号方式约束
  std::string _include_cyberdog_bringup_parameters_string2 {""}; // string 键值对(多string):双引号方式约束
  std::vector<bool>         _include_cyberdog_bringup_parameters_bool_vector;    // bool 键值对(数组)
  std::vector<int64_t>      _include_cyberdog_bringup_parameters_int_vector;     // int 键值对(数组)
  std::vector<double>       _include_cyberdog_bringup_parameters_double_vector;   // double 键值对(数组)
  std::vector<std::string>  _include_cyberdog_bringup_parameters_string_vector;  // string 键值对(数组)

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  template<typename T>
  void info_vector(std::string _msg, const std::vector<T> & _vector) {
    if (!_vector.empty())
    {
      std::stringstream vector_msg;
      vector_msg << "[" << _vector[0];
      for (size_t i = 1; i < _vector.size(); i++)
      {
        vector_msg << ", "<< _vector[i];
      }
      vector_msg << "]";
      INFO_STREAM(_msg << " = " << vector_msg.str());
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}
