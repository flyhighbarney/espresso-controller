cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Cross-compilation toolchain
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(ARM_CC arm-none-eabi-gcc)
find_program(ARM_CXX arm-none-eabi-g++)
find_program(ARM_OBJCOPY arm-none-eabi-objcopy)
find_program(ARM_SIZE arm-none-eabi-size)

if(ARM_CC)
    set(CMAKE_C_COMPILER ${ARM_CC})
    set(CMAKE_CXX_COMPILER ${ARM_CXX})
endif()

project(espresso-controller C ASM)

# MCU flags for STM32G474RE (Cortex-M4F with FPU)
set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS} -Wall -Wextra -Og -g3 -ffunction-sections -fdata-sections")
set(CMAKE_C_FLAGS_RELEASE "${MCU_FLAGS} -Wall -Wextra -O2 -DNDEBUG -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS} -Wl,--gc-sections -specs=nosys.specs -specs=nano.specs")

# Linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32G474RETX_FLASH.ld)

# Build option: use Simulink-generated controller
option(USE_SIMULINK_CONTROLLER "Use Simulink Coder generated controller instead of hand-written" OFF)

# Sources
file(GLOB_RECURSE CORE_SOURCES "Core/Src/*.c")
file(GLOB_RECURSE APP_SOURCES
    "App/control/*.c"
    "App/sensors/*.c"
    "App/actuators/*.c"
    "App/telemetry/*.c"
    "App/supervisor/*.c"
)

if(USE_SIMULINK_CONTROLLER)
    file(GLOB SIMULINK_SOURCES "${CMAKE_SOURCE_DIR}/../simulink/generated/*.c")
    add_definitions(-DUSE_SIMULINK_CONTROLLER)
else()
    set(SIMULINK_SOURCES "")
endif()

# ML sources (CMSIS-NN inference)
file(GLOB ML_SOURCES "App/ml/*.c")

set(ALL_SOURCES
    ${CORE_SOURCES}
    ${APP_SOURCES}
    ${ML_SOURCES}
    ${SIMULINK_SOURCES}
)

add_executable(${PROJECT_NAME}.elf ${ALL_SOURCES})

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    Core/Inc
    App/control
    App/sensors
    App/actuators
    App/ml
    App/telemetry
    App/supervisor
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

if(EXISTS ${LINKER_SCRIPT})
    set_target_properties(${PROJECT_NAME}.elf PROPERTIES
        LINK_FLAGS "-T${LINKER_SCRIPT}"
    )
endif()

# Post-build: generate .bin and .hex, print size
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${ARM_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${ARM_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${ARM_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Building .hex and .bin, printing size"
)

# Host-side unit test build (native compilation)
option(BUILD_TESTS "Build host-side unit tests" OFF)
if(BUILD_TESTS)
    # Reset to native compiler for tests
    enable_testing()
    add_subdirectory(Tests)
endif()
