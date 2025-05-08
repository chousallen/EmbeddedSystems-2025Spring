################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/stm32l475e_iot01.c \
../Core/Inc/stm32l475e_iot01_accelero.c 

OBJS += \
./Core/Inc/stm32l475e_iot01.o \
./Core/Inc/stm32l475e_iot01_accelero.o 

C_DEPS += \
./Core/Inc/stm32l475e_iot01.d \
./Core/Inc/stm32l475e_iot01_accelero.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/stm32l475e_iot01.cyclo ./Core/Inc/stm32l475e_iot01.d ./Core/Inc/stm32l475e_iot01.o ./Core/Inc/stm32l475e_iot01.su ./Core/Inc/stm32l475e_iot01_accelero.cyclo ./Core/Inc/stm32l475e_iot01_accelero.d ./Core/Inc/stm32l475e_iot01_accelero.o ./Core/Inc/stm32l475e_iot01_accelero.su

.PHONY: clean-Core-2f-Inc

