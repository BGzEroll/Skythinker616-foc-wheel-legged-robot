cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

project(stm32_can_bootloader C ASM)

set(APP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(HAL_ROOT ${APP_ROOT}/Drivers/STM32F1xx_HAL_Driver)

add_executable(${PROJECT_NAME}
    src/main.c
    ${APP_ROOT}/startup_stm32f103x6.s
    ${APP_ROOT}/Core/Src/system_stm32f1xx.c
    ${HAL_ROOT}/Src/stm32f1xx_hal.c
    ${HAL_ROOT}/Src/stm32f1xx_hal_can.c
    ${HAL_ROOT}/Src/stm32f1xx_hal_cortex.c
    ${HAL_ROOT}/Src/stm32f1xx_hal_flash.c
    ${HAL_ROOT}/Src/stm32f1xx_hal_flash_ex.c
    ${HAL_ROOT}/Src/stm32f1xx_hal_gpio.c
    ${HAL_ROOT}/Src/stm32f1xx_hal_gpio_ex.c
    ${HAL_ROOT}/Src/stm32f1xx_hal_pwr.c
    ${HAL_ROOT}/Src/stm32f1xx_hal_rcc.c
    ${HAL_ROOT}/Src/stm32f1xx_hal_rcc_ex.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    include
    ${APP_ROOT}/shared
    ${HAL_ROOT}/Inc
    ${HAL_ROOT}/Inc/Legacy
    ${APP_ROOT}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${APP_ROOT}/Drivers/CMSIS/Include
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER STM32F103x6 CAN_BOOTLOADER_BUILD
)
target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m3 -mthumb -Wall -Wextra -Werror
    -ffunction-sections -fdata-sections
    $<$<CONFIG:Release>:-Os>
)
target_link_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m3 -mthumb
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C6_BOOT.ld
    --specs=nano.specs
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}>
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}>
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating bootloader HEX/BIN and checking the fixed 8 KB FLASH region"
    VERBATIM
)
