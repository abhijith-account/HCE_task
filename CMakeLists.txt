cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(medical_firmware)

zephyr_cc_option(-Wno-deprecated-declarations -Wno-deprecated)

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE
	src/main.cpp
    	src/RTOS_Command_based_thread_system.cpp
    	src/RTOS_Synchronization_Layer.cpp
        src/Device_State_Machine+Watchdog.cpp
        src/Fault_Tolerant_I2C_Communication_Layer.cpp
        src/Persistent_Configuration_System.cpp
        src/Power_Management_System.cpp
        src/Smart_Battery_System.cpp
        src/Static_Memory+MISRA_Compliance_Layer.cpp
        src/USB_CDC_Virtual_COM_Shell_Interface.cpp
        src/qemu_sensor_mocks.c
)

target_compile_options(app PRIVATE
    -Wall
    -Wextra
    -Werror
    -Wno-pedantic
    -Wno-variadic-macros
    -Wno-vla
    -Wno-missing-field-initializers
    -Wno-unused-parameter
)

if(CONFIG_COVERAGE)
    target_compile_options(app PRIVATE --coverage -O0)
    target_link_options(app PRIVATE --coverage)
endif()
