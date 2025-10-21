cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms
        USE_HAL_DRIVER
        STM32F405xx
        $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
        ../../Core/Inc
        ../../Drivers/STM32F4xx_HAL_Driver/Inc
        ../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        ../../Middlewares/Third_Party/FreeRTOS/Source/include
        ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
        ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        ../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
        ../../Drivers/CMSIS/Include
        ../../USB_DEVICE/App
        ../../USB_DEVICE/Target
        ../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
)

# STM32CubeMX generated application sources
set(MX_Application_Src
        ../../USB_DEVICE/Target/usbd_conf.c
        ../../USB_DEVICE/App/usb_device.c
        ../../USB_DEVICE/App/usbd_desc.c
        ../../USB_DEVICE/App/usbd_cdc_if.c
        ../../Core/Src/system_stm32f4xx.c
        ../../Core/Src/main.c
        ../../Core/Src/gpio.c
        ../../Core/Src/freertos.c
        ../../Core/Src/adc.c
        ../../Core/Src/dma.c
        ../../Core/Src/spi.c
        ../../Core/Src/tim.c
        ../../Core/Src/usart.c
        ../../Core/Src/stm32f4xx_it.c
        ../../Core/Src/stm32f4xx_hal_msp.c
        ../../Core/Src/stm32f4xx_hal_timebase_tim.c
        ../../Core/Src/sysmem.c
        ../../Core/Src/syscalls.c
        ../../startup_stm32f405xx.s

)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src

        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c

        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
        ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# Drivers Midllewares

set(USB_Device_Library_Src
        ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
        ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
        ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
        ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)
set(FreeRTOS_Src
        ../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
        ../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
        ../../Middlewares/Third_Party/FreeRTOS/Source/list.c
        ../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
        ../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
        ../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
        ../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
        ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
        ../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
        ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS
        STM32_Drivers
        FreeRTOS
        USB_Device_Library

)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)

# Create USB_Device_Library static library
add_library(USB_Device_Library OBJECT)
target_sources(USB_Device_Library PRIVATE ${USB_Device_Library_Src})
target_link_libraries(USB_Device_Library PUBLIC stm32cubemx)

# Create FreeRTOS static library
add_library(FreeRTOS OBJECT)
target_sources(FreeRTOS PRIVATE ${FreeRTOS_Src})
target_link_libraries(FreeRTOS PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if ((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif ()
