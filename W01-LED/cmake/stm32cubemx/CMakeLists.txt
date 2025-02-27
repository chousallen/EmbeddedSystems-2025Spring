cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32L475xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/include
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/freertos.c
    ../../Core/Src/stm32l4xx_it.c
    ../../Core/Src/stm32l4xx_hal_msp.c
    ../../Core/Src/stm32l4xx_hal_timebase_tim.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c
    ../../Core/Src/system_stm32l4xx.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/list.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/queue.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/timers.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    /home/sallen/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32l475xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


