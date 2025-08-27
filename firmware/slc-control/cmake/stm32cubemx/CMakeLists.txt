cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32G431xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../USB_Device/App
    ../../USB_Device/Target
    ../../Core/Inc
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Inc
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ../../stm32cube-g4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ../../stm32cube-g4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ../../stm32cube-g4/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ../../stm32cube-g4/Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/stm32g4xx_it.c
    ../../Core/Src/stm32g4xx_hal_msp.c
    ../../USB_Device/App/usb_device.c
    ../../USB_Device/App/usbd_desc.c
    ../../USB_Device/App/usbd_cdc_if.c
    ../../USB_Device/Target/usbd_conf.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_comp.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac_ex.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    ../../stm32cube-g4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
    ../../Core/Src/system_stm32g4xx.c
    ../../stm32cube-g4/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ../../stm32cube-g4/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ../../stm32cube-g4/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ../../stm32cube-g4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32g431xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


