cmake_minimum_required(VERSION 3.8)
project(cyberdog_aft)

# add_compile_options("-g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
add_compile_options(-g)

include_directories(include)
include_directories(SYSTEM ${QGLVIEWER_INCLUDE_DIR})
include("${CMAKE_SOURCE_DIR}/cmake/common.cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(cyberdog_parameter REQUIRED)
find_package(protocol REQUIRED)
find_package(filesystem REQUIRED)
find_package(toml REQUIRED)
find_package(Protobuf REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(cyberdog_debug REQUIRED)

ament_register_extension("ament_package" "cyberdog_parameter"
  "functions.cmake"
)
project_initialize_config_parameter()

if(Qt5_FOUND)
  qt5_wrap_ui(UI_HDRS ${PROJECT_SOURCE_DIR}/include/ui/cuberdog_aft.ui)
  # QT5_WRAP_CPP(UI_SOURCES main_window.h properties_widget.h)
  include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
  set(AFT_QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Xml_LIBRARIES} ${Qt5OpenGL_LIBRARIES})
endif()

# Proto file
get_filename_component(RG_PROTO "./proto/cyberdog_aft.proto" ABSOLUTE)
get_filename_component(RG_PROTO_PATH "${RG_PROTO}" PATH)

# Generated sources
set(RG_PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}/cyberdog_aft.pb.cc")
set(RG_PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/cyberdog_aft.pb.h")
set(RG_GRPC_SRCS "${CMAKE_CURRENT_BINARY_DIR}/cyberdog_aft.grpc.pb.cc")
set(RG_GRPC_HDRS "${CMAKE_CURRENT_BINARY_DIR}/cyberdog_aft.grpc.pb.h")

add_custom_command(
      OUTPUT "${RG_PROTO_SRCS}" "${RG_PROTO_HDRS}" "${RG_GRPC_SRCS}" "${RG_GRPC_HDRS}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${RG_PROTO_PATH}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${RG_PROTO}"
      DEPENDS "${RG_PROTO}"
)

set(DEPS
  std_msgs
  rclcpp
  nav2_msgs
  sensor_msgs
  protocol
  cyberdog_common
  cyberdog_parameter
  filesystem
  ament_index_cpp
  toml
  PythonInterp
  PythonLibs
  nav2_util
  nav2_lifecycle_manager
  nav_msgs
  Boost
  cyberdog_debug
)

# Proto
set(PROTO_LIBS
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF}
)

set(SRCS
  src/motion/motion.cpp
  src/motion/motion_interface.cpp
  src/navigation/navigation.cpp
  src/navigation/navigation_interface.cpp
  src/protocol/aft_client.cpp
  src/protocol/aft_server.cpp
  src/protocol/protocol.cpp
  src/protocol/protocol_grpc.cpp
  src/protocol/protocol_motion.cpp
  src/protocol/protocol_system.cpp
  src/protocol/protocol_interface.cpp
  src/slam/slam.cpp
  src/slam/slam_interface.cpp
  src/utils/utils.cpp
  src/utils/shell_command.cpp
  src/utils/string_util.cpp
  src/utils/lifecycle_controller.cpp
  src/utils/lifecycle_node_manager.cpp
  src/system/hardware.cpp
  src/system/storage.cpp
  src/system/system_interface.cpp
  src/system/system.cpp
  src/system/temp_ctrl.cpp
  src/visualization/visualization.cpp
  src/factory_tool.cpp
  src/config_manager.cpp
  ${RG_GRPC_SRCS}
  ${RG_GRPC_HDRS}
  ${RG_PROTO_SRCS}
  ${RG_PROTO_HDRS}
)

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

add_library(${PROJECT_NAME} ${SRCS})
ament_target_dependencies(${PROJECT_NAME} ${DEPS})
target_link_libraries(${PROJECT_NAME}
  ${QGLVIEWER_LIBRARY}
  ${AFT_QT_LIBRARIES}
  ${PROTO_LIBS})

add_executable(cyberdog_aft_node src/cyberdog_aft_node.cpp ${SRCS})
ament_target_dependencies(cyberdog_aft_node ${DEPS})
target_link_libraries(cyberdog_aft_node
  ${QGLVIEWER_LIBRARY}
  ${AFT_QT_LIBRARIES}
  ${PROTO_LIBS})

add_executable(cyberdog_aft_client_node src/cyberdog_aft_client_node.cpp ${SRCS})
ament_target_dependencies(cyberdog_aft_client_node ${DEPS})
target_link_libraries(cyberdog_aft_client_node
  ${QGLVIEWER_LIBRARY}
  ${AFT_QT_LIBRARIES}
  ${PROTO_LIBS})

target_link_libraries(cyberdog_aft_client_node ${Boost_FILESYSTEM_LIBRARY})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS cyberdog_aft_client_node
  DESTINATION lib/cyberdog_aft)

install(TARGETS cyberdog_aft_node
  DESTINATION lib/cyberdog_aft)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
      scripts/connect_by_id.py
      scripts/connect_by_ip.py
      DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

if(BUILD_TESTING)
  include_directories(include)
  include_directories(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)

  # test_report_result
  ament_add_gtest(${PROJECT_NAME}-test_report_result test/test_report_result.cpp)
  if(TARGET ${PROJECT_NAME}-test_report_result)
    target_link_libraries(${PROJECT_NAME}-test_report_result ${PROJECT_NAME})
  endif()

  # ament_target_dependencies(${PROJECT_NAME}-test_report_result rclcpp ${})
  # if(TARGET ${PROJECT_NAME}-test_report_result)
  #   add_executable(${PROJECT_NAME}-test_report_result test/test_report_result.cpp)
  #   ament_target_dependencies(${PROJECT_NAME}-test_report_result rclcpp)
  # endif()
endif()

ament_package()














# cmake_minimum_required(VERSION 3.5)

# project(message_filters)

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake_python REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rcutils REQUIRED)
# find_package(builtin_interfaces REQUIRED)

# add_library(${PROJECT_NAME} src/connection.cpp)
# if(WIN32)
#   target_compile_definitions(${PROJECT_NAME}
#     PRIVATE "MESSAGE_FILTERS_BUILDING_DLL")
# endif()
# target_include_directories(${PROJECT_NAME} PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")

# ament_target_dependencies(${PROJECT_NAME}
#   "rclcpp"
#   "rcutils"
#   "builtin_interfaces"
#   )

# install(
#   TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(
#   DIRECTORY "include/"
#   DESTINATION include
# )

# find_package(python_cmake_module REQUIRED)
# _ament_cmake_python_register_environment_hook()
# install(
#   DIRECTORY "src/${PROJECT_NAME}/"
#   DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
#   PATTERN "*.pyc" EXCLUDE
#   PATTERN "__pycache__" EXCLUDE
# )

# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_targets(${PROJECT_NAME})
# ament_export_dependencies(rclcpp rcutils builtin_interfaces)

# if(BUILD_TESTING)
#   find_package(sensor_msgs REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest)

#   ament_add_gtest(${PROJECT_NAME}-test_simple test/test_simple.cpp)
#   if(TARGET ${PROJECT_NAME}-test_simple)
#     target_link_libraries(${PROJECT_NAME}-test_simple ${PROJECT_NAME})
#   endif()

#   ament_add_gtest(${PROJECT_NAME}-msg_cache_unittest test/msg_cache_unittest.cpp)
#   if(TARGET ${PROJECT_NAME}-msg_cache_unittest)
#     target_link_libraries(${PROJECT_NAME}-msg_cache_unittest ${PROJECT_NAME})
#   endif()

#   ament_add_gtest(${PROJECT_NAME}-test_chain test/test_chain.cpp)
#   if(TARGET ${PROJECT_NAME}-test_chain)
#     target_link_libraries(${PROJECT_NAME}-test_chain ${PROJECT_NAME})
#   endif()

#   ament_add_gtest(${PROJECT_NAME}-test_time_sequencer test/time_sequencer_unittest.cpp)
#   if(TARGET ${PROJECT_NAME}-test_time_sequencer)
#     target_link_libraries(${PROJECT_NAME}-test_time_sequencer ${PROJECT_NAME})
#   endif()

#   ament_add_gtest(${PROJECT_NAME}-test_subscriber test/test_subscriber.cpp)
#   if(TARGET ${PROJECT_NAME}-test_subscriber)
#     target_link_libraries(${PROJECT_NAME}-test_subscriber ${PROJECT_NAME})
#     target_include_directories(${PROJECT_NAME}-test_subscriber PUBLIC include)
#     ament_target_dependencies(${PROJECT_NAME}-test_subscriber "rclcpp" "sensor_msgs")
#   endif()

#   ament_add_gtest(${PROJECT_NAME}-test_synchronizer test/test_synchronizer.cpp)
#   if(TARGET ${PROJECT_NAME}-test_synchronizer)
#     target_link_libraries(${PROJECT_NAME}-test_synchronizer ${PROJECT_NAME})
#   endif()

#   ament_add_gtest(${PROJECT_NAME}-time_synchronizer_unittest test/time_synchronizer_unittest.cpp)
#   if(TARGET ${PROJECT_NAME}-time_synchronizer_unittest)
#     target_link_libraries(${PROJECT_NAME}-time_synchronizer_unittest ${PROJECT_NAME})
#   endif()

#   ament_add_gtest(${PROJECT_NAME}-test_exact_time_policy test/test_exact_time_policy.cpp)
#   if(TARGET ${PROJECT_NAME}-test_exact_time_policy)
#     target_link_libraries(${PROJECT_NAME}-test_exact_time_policy ${PROJECT_NAME})
#   endif()

#   ament_add_gtest(${PROJECT_NAME}-test_approximate_time_policy test/test_approximate_time_policy.cpp)
#   if(TARGET ${PROJECT_NAME}-test_approximate_time_policy)
#     target_link_libraries(${PROJECT_NAME}-test_approximate_time_policy ${PROJECT_NAME})
#   endif()

#   ament_add_gtest(${PROJECT_NAME}-test_fuzz test/test_fuzz.cpp SKIP_TEST)
#   if(TARGET ${PROJECT_NAME}-test_fuzz)
#     target_link_libraries(${PROJECT_NAME}-test_fuzz ${PROJECT_NAME})
#     target_include_directories(${PROJECT_NAME}-test_fuzz PUBLIC include)
#     ament_target_dependencies(${PROJECT_NAME}-test_fuzz "rclcpp" "sensor_msgs")
#   endif()

#   # Provides PYTHON_EXECUTABLE_DEBUG
#   find_package(PythonExtra REQUIRED)
#   set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
#   if(WIN32)
#     if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#       set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
#     endif()
#   endif()

#   # python tests with python interfaces of message filters
#   find_package(ament_cmake_pytest REQUIRED)
#   ament_add_pytest_test(directed.py "test/directed.py"
#     PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")
#   ament_add_pytest_test(test_approxsync.py "test/test_approxsync.py"
#     PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")
#   ament_add_pytest_test(test_message_filters_cache.py "test/test_message_filters_cache.py"
#     PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")
# endif()

# ament_package()
