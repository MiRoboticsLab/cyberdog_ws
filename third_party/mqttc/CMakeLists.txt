cmake_minimum_required(VERSION 3.8)
project(mqttc)

# MQTT-C build options
option(MQTT_C_OpenSSL_SUPPORT "Build MQTT-C with OpenSSL support?" OFF)
option(MQTT_C_MbedTLS_SUPPORT "Build MQTT-C with mbed TLS support?" ON)
#option(MQTT_C_BearSSL_SUPPORT "Build MQTT-C with Bear SSL support?"FF OFF)
#option(MQTT_C_EXAMPLES "Build MQTT-C examples?" ON)
#option(MQTT_C_INSTALL_EXAMPLES "Install MQTT-C examples?" OFF)
#option(MQTT_C_TESTS "Build MQTT-C tests?" OFF)
list (APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_LIBRARY_PATH "/usr/lib")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(MBEDTLS_INCLUDE_DIRS /usr/include)
set(MBEDX509_LIBRARY /usr/lib/aarch64-linux-gnu)
set(MBEDTLS_LIBRARY /usr/lib/aarch64-linux-gnu)
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
if(MQTT_C_OpenSSL_SUPPORT)
    find_package(OpenSSL REQUIRED)
    target_link_libraries(mqttc INTERFACE OpenSSL::SSL)
    target_include_directories(mqttc PUBLIC ${OPENSSL_INCLUDE_DIR})
    target_compile_definitions(mqttc PUBLIC MQTT_USE_BIO)
endif()
if(MQTT_C_MbedTLS_SUPPORT)
    find_package(MbedTLS REQUIRED)
    target_include_directories(mqttc PUBLIC ${MBEDTLS_INCLUDE_DIRS})
    # target_include_directories(mqttc PUBLIC /usr/include/mbedtls/)
    target_link_libraries(mqttc INTERFACE ${MBEDTLS_LIBRARY})
    # target_link_libraries(mqttc INTERFACE mbedtls)
    target_compile_definitions(mqttc PUBLIC MQTT_USE_MBEDTLS)
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
