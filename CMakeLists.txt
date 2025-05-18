# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
set(FREERTOS_KERNEL_PATH "C:/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(main C CXX ASM)
pico_sdk_init()


include_directories(${CMAKE_SOURCE_DIR}/libs)

add_executable(main  
        main.c
        libs/ssd1306.c # Biblioteca para o display OLED
        libs/display.c # Biblioteca para controle do display
        libs/pio.c     # Biblioteca pra controle da matriz 5x5
        libs/buzzer.c  # Biblioteca para buzzer
        )

target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR})

pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_link_libraries(main 
        pico_stdlib 
        hardware_gpio
        hardware_i2c
        FreeRTOS-Kernel 
        FreeRTOS-Kernel-Heap4
        hardware_pio
        hardware_pwm
        hardware_adc
        )

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

pico_add_extra_outputs(main)