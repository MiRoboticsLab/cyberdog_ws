cmake_minimum_required(VERSION 3.8)
project(mqttc)

# MQTT-C build options
#option(MQTT_C_OpenSSL_SUPPORT "Build MQTT-C with OpenSSL support?" OFF)
#option(MQTT_C_MbedTLS_SUPPORT "Build MQTT-C with mbed TLS support?" OFF)
#option(MQTT_C_BearSSL_SUPPORT "Build MQTT-C with Bear SSL support?" OFF)
#option(MQTT_C_EXAMPLES "Build MQTT-C examples?" ON)
#option(MQTT_C_INSTALL_EXAMPLES "Install MQTT-C examples?" OFF)
#option(MQTT_C_TESTS "Build MQTT-C tests?" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)

# MQTT-C library
add_library(mqttc SHARED
    src/mqtt_pal.c
    src/mqtt.c
)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "MQTTC_DLL")
  
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()  
  
# Install includes and library
install(
	DIRECTORY include/
	DESTINATION include)
install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

	
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()	
	
ament_export_include_directories(include)
ament_export_targets(mqttc HAS_LIBRARY_TARGET)
ament_package()
