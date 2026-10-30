cmake_minimum_required(VERSION 3.10)
project(pm_tests)

set(CMAKE_CXX_STANDARD 20)

# Find Google Test on your host system
find_package(GTest REQUIRED)

# Headers moved into subfolders
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/Power_Management_System
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/RTOS_Command_based_thread_system
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/Smart_Battery_System
)

# Sources moved into subfolders: collect them into variables
file(GLOB PMS_SRCS      ${CMAKE_CURRENT_SOURCE_DIR}/../src/Power_Management_System/*.cpp)
file(GLOB BATT_SRCS     ${CMAKE_CURRENT_SOURCE_DIR}/../src/Smart_Battery_System/*.cpp)
file(GLOB RTOS_CMD_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/../src/RTOS_Command_based_thread_system/*.cpp)
# Split gtest sources. List order = link order = order of tests across files, so keep the mocks file first.
set(PM_TEST_SRCS
    power/test_power_mocks.cpp
    power/test_Power_Management_System.cpp
    power/test_Power_ActiveIdleStates.cpp
    power/test_Power_StopState.cpp
    power/test_Power_Integration.cpp
    power/test_Power_Hardware.cpp
    power/test_Power_Singleton.cpp
)
set(BATT_TEST_SRCS
    battery/test_battery_mocks.cpp
    battery/test_Smart_Battery_System.cpp
    battery/test_Smart_Battery_Fetch.cpp
    battery/test_Smart_Battery_Poll.cpp
    battery/test_Smart_Battery_Soc.cpp
    battery/test_Smart_Battery_FSM.cpp
    battery/test_Sensor_Drivers.cpp
    battery/test_Bms_Comm_Thread.cpp
    battery/test_Battery_Monitor_Thread.cpp
    battery/test_Battery_Runtime.cpp
)
set(RTOS_CMD_TEST_SRCS
    rtos/test_rtos_mocks.cpp
    rtos/test_Command_Core.cpp
    rtos/test_System_Objects.cpp
    rtos/test_Sensor_Read_Cmd.cpp
    rtos/test_Compute_Cmd.cpp
    rtos/test_Sensor_Math.cpp
    rtos/test_Print_Commands.cpp
    rtos/test_Producer_Thread.cpp
    rtos/test_Consumer_Threads.cpp
    rtos/test_Thread_Trace.cpp
)

set(MOCK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mocks/include)
set(MOCK_GLOBALS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/mocks/mock_globals.cpp)

# 4. Define the executable
add_executable(run_pm_tests 
    ${PM_TEST_SRCS} 
    ${PMS_SRCS}
    ../src/Device_State_Machine+Watchdog.cpp
    ${MOCK_GLOBALS_SRC}
)

add_executable(run_usb_shell_tests 
    test_usb_shell.cpp 
    ../src/USB_CDC_Virtual_COM_Shell_Interface.cpp
    ${PMS_SRCS}
    ../src/Device_State_Machine+Watchdog.cpp
    ../src/Fault_Tolerant_I2C_Communication_Layer.cpp
    ../src/Persistent_Configuration_System.cpp
    ${BATT_SRCS}
    ${MOCK_GLOBALS_SRC}
)

add_executable(run_battery_tests
    ${BATT_TEST_SRCS}
    ${BATT_SRCS}
    ../src/Fault_Tolerant_I2C_Communication_Layer.cpp
    ../src/Device_State_Machine+Watchdog.cpp
    ${PMS_SRCS}
    ../src/Persistent_Configuration_System.cpp
    ${MOCK_GLOBALS_SRC}
)

add_executable(run_static_memory_tests
    test_static_memory.cpp
    ../src/Static_Memory+MISRA_Compliance_Layer.cpp
    ../src/Device_State_Machine+Watchdog.cpp
    ${PMS_SRCS}
    ${MOCK_GLOBALS_SRC}
)

add_executable(run_config_tests
     test_config_store.cpp
     ../src/Persistent_Configuration_System.cpp
     ../src/Device_State_Machine+Watchdog.cpp
     ${PMS_SRCS}
     ${MOCK_GLOBALS_SRC}
)

add_executable(run_i2c_faults_tests
      test_I2C_Fault_Recovery.cpp
      ../src/Fault_Tolerant_I2C_Communication_Layer.cpp
      ${PMS_SRCS}
      ../src/Device_State_Machine+Watchdog.cpp
      ${MOCK_GLOBALS_SRC}
)

add_executable(run_state_machine_tests
       test_state_machine.cpp
       ../src/Device_State_Machine+Watchdog.cpp
       ${PMS_SRCS}
       ${MOCK_GLOBALS_SRC}
)

add_executable(run_sync_tests
    test_synchronization.cpp
    ../src/RTOS_Synchronization_Layer.cpp
    ../src/Device_State_Machine+Watchdog.cpp
    ${PMS_SRCS}
    ${MOCK_GLOBALS_SRC}
)

add_executable(run_command_tests
    ${RTOS_CMD_TEST_SRCS}
    ${RTOS_CMD_SRCS}
    ../src/Device_State_Machine+Watchdog.cpp
    ${PMS_SRCS}
    ../src/Fault_Tolerant_I2C_Communication_Layer.cpp
    ${MOCK_GLOBALS_SRC}
)

# 5. Include the checked-in Zephyr mocks and your real headers
target_include_directories(run_pm_tests PRIVATE 
    ${MOCK_DIR}
    ../include
)

target_include_directories(run_usb_shell_tests PRIVATE 
    ${MOCK_DIR}
    ../include
)

target_include_directories(run_battery_tests PRIVATE
    ${MOCK_DIR}
    ../include
)

target_include_directories(run_static_memory_tests PRIVATE
   ${MOCK_DIR}
   ../include
)

target_include_directories(run_config_tests PRIVATE
   ${MOCK_DIR}      
   ../include
)

target_include_directories(run_i2c_faults_tests PRIVATE
   ${MOCK_DIR}
   ../include
)

target_include_directories(run_state_machine_tests PRIVATE
   ${MOCK_DIR}
   ../include
)

target_include_directories(run_sync_tests PRIVATE
   ${MOCK_DIR}
   ../include
)

target_include_directories(run_command_tests PRIVATE
   ${MOCK_DIR}
   ../include
)

# 6. Link GTest and threading libraries
target_link_libraries(run_pm_tests PRIVATE GTest::gtest_main pthread)
target_link_libraries(run_usb_shell_tests PRIVATE GTest::gtest_main pthread)
target_link_libraries(run_battery_tests PRIVATE GTest::gtest_main pthread)
target_link_libraries(run_static_memory_tests PRIVATE GTest::gtest_main pthread)
target_link_libraries(run_config_tests PRIVATE GTest::gtest_main pthread)
target_link_libraries(run_i2c_faults_tests PRIVATE GTest::gtest_main pthread)
target_link_libraries(run_state_machine_tests PRIVATE GTest::gtest_main pthread)
target_link_libraries(run_sync_tests PRIVATE GTest::gtest_main pthread)
target_link_libraries(run_command_tests PRIVATE GTest::gtest_main pthread)

# Enable GCOV profiling flags for compilation and linking
target_compile_options(run_pm_tests PRIVATE --coverage)
target_link_options(run_pm_tests PRIVATE --coverage)

target_compile_options(run_usb_shell_tests PRIVATE --coverage -fno-builtin-snprintf -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0)
target_link_options(run_usb_shell_tests PRIVATE --coverage -Wl,--wrap=snprintf)

target_compile_options(run_battery_tests PRIVATE --coverage)
target_link_options(run_battery_tests PRIVATE --coverage)

target_compile_options(run_static_memory_tests PRIVATE --coverage)
target_link_options(run_static_memory_tests PRIVATE --coverage -Wl,--wrap=k_thread_stack_space_get)

target_compile_options(run_config_tests PRIVATE --coverage)
target_link_options(run_config_tests PRIVATE --coverage)

target_compile_options(run_i2c_faults_tests PRIVATE --coverage)
target_link_options(run_i2c_faults_tests PRIVATE --coverage)

target_compile_options(run_state_machine_tests PRIVATE --coverage)
target_link_options(run_state_machine_tests PRIVATE --coverage)

target_compile_options(run_sync_tests PRIVATE --coverage)
target_link_options(run_sync_tests PRIVATE --coverage)

target_compile_options(run_command_tests PRIVATE --coverage)
target_link_options(run_command_tests PRIVATE --coverage)

target_compile_definitions(run_static_memory_tests PRIVATE IS_TEST_ENVIRONMENT)
target_compile_definitions(run_pm_tests PRIVATE IS_TEST_ENVIRONMENT CONFIG_BOARD_MPS2_AN386)
target_compile_definitions(run_usb_shell_tests PRIVATE IS_TEST_ENVIRONMENT INA226_TEST_BYPASS_STATIC_ASSERT)
target_compile_definitions(run_battery_tests PRIVATE IS_TEST_ENVIRONMENT INA226_TEST_BYPASS_STATIC_ASSERT)
target_compile_definitions(run_config_tests PRIVATE IS_TEST_ENVIRONMENT)
target_compile_definitions(run_sync_tests PRIVATE IS_TEST_ENVIRONMENT)
target_compile_definitions(run_command_tests PRIVATE IS_TEST_ENVIRONMENT)
target_compile_definitions(run_i2c_faults_tests PRIVATE IS_TEST_ENVIRONMENT)
target_compile_definitions(run_state_machine_tests PRIVATE IS_TEST_ENVIRONMENT)
