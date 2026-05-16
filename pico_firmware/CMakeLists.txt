cmake_minimum_required(VERSION 3.13)

# Pull in Pico SDK (must be before project())
include(pico_sdk_import.cmake)

project(autonexa_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# ── Build option ─────────────────────────────────────────────
# Auto-detect the micro-ROS static lib so a plain `cmake ..` on a fully
# provisioned checkout produces the micro-ROS UF2 (autonexa_pico_uros.uf2),
# which is what the RPi5 micro-ROS agent needs. Falls back to ASCII-serial
# only when the SDK is missing so bench-test builds still work out of the box.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_sdk/libmicroros/libmicroros.a")
    set(USE_MICRO_ROS_DEFAULT ON)
else()
    set(USE_MICRO_ROS_DEFAULT OFF)
endif()
option(USE_MICRO_ROS "Build with micro-ROS support" ${USE_MICRO_ROS_DEFAULT})

# ── Common sources ───────────────────────────────────────────
set(COMMON_SOURCES
    src/main.c
    src/servo.c
    src/l298n_driver.c
    src/safety.c
    src/ackermann.c
    src/motor_control.c
    src/encoder.c
)

set(COMMON_LIBS
    pico_stdlib
    hardware_pwm
    hardware_gpio
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

# ── Encoder bench-test target (standalone, optional) ─────────
# Self-contained: does not drive motors or use micro-ROS/CLI. Build it
# explicitly with `make encoder_test`; flash encoder_test.uf2 to verify
# the two quadrature encoders before integrating them into the firmware.
add_executable(encoder_test test/encoder_test.c)
target_include_directories(encoder_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(encoder_test
    pico_stdlib
    hardware_gpio
)
pico_enable_stdio_usb(encoder_test 1)
pico_enable_stdio_uart(encoder_test 0)
pico_add_extra_outputs(encoder_test)

# ── micro-ROS target (optional) ─────────────────────────────
if(USE_MICRO_ROS)
    # Prefer bundled static lib setup from micro_ros_sdk to avoid relying on
    # an external helper cmake file that may not exist in this repo snapshot.
    set(MICROROS_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_sdk/libmicroros)
    set(MICROROS_LIB ${MICROROS_LIB_DIR}/libmicroros.a)
    set(MICROROS_INCLUDE_DIR ${MICROROS_LIB_DIR}/include)

    if(NOT EXISTS ${MICROROS_LIB})
        message(FATAL_ERROR
            "micro-ROS static library not found: ${MICROROS_LIB}\n"
            "  Either install the micro-ROS SDK into micro_ros_sdk/libmicroros/\n"
            "  or configure with -DUSE_MICRO_ROS=OFF to build the ASCII serial target only.")
    endif()

    add_executable(autonexa_pico_uros
        ${COMMON_SOURCES}
        src/uros_transport.c
        src/uros_time_shim.c
        micro_ros_sdk/pico_uart_transport.c
    )

    target_compile_definitions(autonexa_pico_uros PRIVATE
        USE_MICRO_ROS
        PICO_UROS_SKIP_TIME_SHIMS
    )

    target_include_directories(autonexa_pico_uros PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_sdk
        ${MICROROS_INCLUDE_DIR}
    )

    target_link_directories(autonexa_pico_uros PRIVATE
        ${MICROROS_LIB_DIR}
    )

    target_link_libraries(autonexa_pico_uros
        ${COMMON_LIBS}
        microros
    )

    # micro-ROS transport in this project uses Pico stdio over USB CDC.
    pico_enable_stdio_usb(autonexa_pico_uros 1)
    pico_enable_stdio_uart(autonexa_pico_uros 0)

    pico_add_extra_outputs(autonexa_pico_uros)

    message(STATUS "micro-ROS target enabled: autonexa_pico_uros")
endif()

# Print build info
message(STATUS "Autonexa Pico firmware v3.0 (L298N Chassis)")
message(STATUS "  Motor driver: L298N H-bridge (open-loop PWM, no encoders)")
message(STATUS "  Servo: hobby servo on configured GPIO")
message(STATUS "  Control frequency: 50 Hz")
message(STATUS "  Targets: autonexa_pico (ASCII serial — bench testing)")
if(USE_MICRO_ROS)
    message(STATUS "           autonexa_pico_uros (micro-ROS — FLASH THIS FOR THE ROBOT)")
else()
    message(STATUS "  NOTE: micro-ROS target disabled. The robot needs autonexa_pico_uros.uf2")
    message(STATUS "        to talk to the RPi5. Install micro_ros_sdk/libmicroros/ to enable.")
endif()
