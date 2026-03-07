cmake_minimum_required(VERSION 3.13)

# Pull in Pico SDK (must be before project())
include(pico_sdk_import.cmake)

project(autonexa_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# ── Firmware executable ──────────────────────────────────────
add_executable(autonexa_pico
    src/main.c
    src/servo.c
    src/hiwonder_driver.c
    src/safety.c
    src/ackermann.c
)

target_include_directories(autonexa_pico PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link Pico SDK libraries
target_link_libraries(autonexa_pico
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
    hardware_timer
)

# Enable USB serial output (disable UART if using USB)
pico_enable_stdio_usb(autonexa_pico 1)
pico_enable_stdio_uart(autonexa_pico 0)

# Generate UF2 file for drag-and-drop flashing
pico_add_extra_outputs(autonexa_pico)

# Print build info
message(STATUS "Autonexa Pico firmware v2.0 (Hiwonder Chassis)")
message(STATUS "  Motor driver: I2C (Hiwonder 4-ch board)")
message(STATUS "  Servo: LD-1501MG on GPIO 12")
message(STATUS "  Control frequency: 50 Hz")
message(STATUS "  Serial: USB @ 115200")
