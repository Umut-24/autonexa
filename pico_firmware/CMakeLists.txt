cmake_minimum_required(VERSION 3.13)

# Pull in Pico SDK (must be before project())
include(pico_sdk_import.cmake)

project(autonexa_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# ── Build option ─────────────────────────────────────────────
option(USE_MICRO_ROS "Build with micro-ROS support" OFF)

# ── Common sources ───────────────────────────────────────────
set(COMMON_SOURCES
    src/main.c
    src/servo.c
    src/hiwonder_driver.c
    src/safety.c
    src/ackermann.c
    src/motor_control.c
)

set(COMMON_LIBS
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
    hardware_timer
)

# ── ASCII serial target (always built) ──────────────────────
add_executable(autonexa_pico ${COMMON_SOURCES})

target_include_directories(autonexa_pico PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(autonexa_pico ${COMMON_LIBS})

# Enable USB serial output (disable UART)
pico_enable_stdio_usb(autonexa_pico 1)
pico_enable_stdio_uart(autonexa_pico 0)

# Generate UF2 file for drag-and-drop flashing
pico_add_extra_outputs(autonexa_pico)

# ── micro-ROS target (optional) ─────────────────────────────
if(USE_MICRO_ROS)
    include(micro_ros_sdk/micro_ros_raspberrypi_pico_sdk.cmake)

    add_executable(autonexa_pico_uros
        ${COMMON_SOURCES}
        src/uros_transport.c
    )

    target_compile_definitions(autonexa_pico_uros PRIVATE USE_MICRO_ROS)

    target_include_directories(autonexa_pico_uros PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(autonexa_pico_uros
        ${COMMON_LIBS}
        microros
    )

    # micro-ROS takes over USB — disable pico stdio USB
    pico_enable_stdio_usb(autonexa_pico_uros 0)
    pico_enable_stdio_uart(autonexa_pico_uros 0)

    pico_add_extra_outputs(autonexa_pico_uros)

    message(STATUS "micro-ROS target enabled: autonexa_pico_uros")
endif()

# Print build info
message(STATUS "Autonexa Pico firmware v3.0 (Hiwonder Chassis)")
message(STATUS "  Motor driver: I2C (Hiwonder 4-ch board)")
message(STATUS "  Servo: LD-1501MG on GPIO 12")
message(STATUS "  Control frequency: 50 Hz")
message(STATUS "  Targets: autonexa_pico (ASCII serial)")
if(USE_MICRO_ROS)
    message(STATUS "           autonexa_pico_uros (micro-ROS)")
endif()
