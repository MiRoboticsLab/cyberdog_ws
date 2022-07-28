cmake_minimum_required(VERSION 3.8)
project(cpp_httplib)

# MQTT-C build options
#option(MQTT_C_OpenSSL_SUPPORT "Build MQTT-C with OpenSSL support?" OFF)
#option(MQTT_C_MbedTLS_SUPPORT "Build MQTT-C with mbed TLS support?" OFF)
#option(MQTT_C_BearSSL_SUPPORT "Build MQTT-C with Bear SSL support?" OFF)
#option(MQTT_C_EXAMPLES "Build MQTT-C examples?" ON)
#option(MQTT_C_INSTALL_EXAMPLES "Install MQTT-C examples?" OFF)
#option(MQTT_C_TESTS "Build MQTT-C tests?" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)

  
# Install includes and library
install(
	DIRECTORY include/
	DESTINATION include)

	
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()	
	
ament_export_include_directories(include)
ament_package()
