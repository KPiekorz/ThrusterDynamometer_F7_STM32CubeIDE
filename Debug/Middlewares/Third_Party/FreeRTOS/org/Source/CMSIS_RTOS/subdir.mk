################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS/cmsis_os.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS/cmsis_os.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS/cmsis_os.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS/cmsis_os.o: ../Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS/cmsis_os.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F767xx -DDEBUG -c -I"E:/ThrusterDynamometer/ThrusterDynamometer_F7_STM32CubeIDE/Middlewares/Third_Party/FreeRTOS/org/Source/include" -I"E:/ThrusterDynamometer/ThrusterDynamometer_F7_STM32CubeIDE/Middlewares/Third_Party/SEGGER/Config" -I"E:/ThrusterDynamometer/ThrusterDynamometer_F7_STM32CubeIDE/Middlewares/Third_Party/SEGGER/OS" -I"E:/ThrusterDynamometer/ThrusterDynamometer_F7_STM32CubeIDE/Middlewares/Third_Party/SEGGER/SEGGER" -I"E:/ThrusterDynamometer/ThrusterDynamometer_F7_STM32CubeIDE/Middlewares/Third_Party/FreeRTOS/org/Source/portable/GCC/ARM_CM7/r0p1" -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I"E:/ThrusterDynamometer/ThrusterDynamometer_F7_STM32CubeIDE/Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS" -I../Middlewares/Third_Party/LwIP/src/include/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/posix -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/Third_Party/LwIP/system/arch -I../Middlewares/Third_Party/LwIP/src/include -I../LWIP/App -I../Core/Inc -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../USB_DEVICE/Target -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../USB_DEVICE/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include/netif -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS/cmsis_os.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

