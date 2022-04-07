################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -DDEBUG -c -I../Core/Inc -IC:/Users/Stasy/STM32Cube/Repository/STM32Cube_FW_G0_V1.4.0/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/Users/Stasy/STM32Cube/Repository/STM32Cube_FW_G0_V1.4.0/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/Users/Stasy/STM32Cube/Repository/STM32Cube_FW_G0_V1.4.0/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/Users/Stasy/STM32Cube/Repository/STM32Cube_FW_G0_V1.4.0/Drivers/CMSIS/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -IC:/Users/andre/STM32Cube/Repository/STM32Cube_FW_G0_V1.4.0/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/Users/andre/STM32Cube/Repository/STM32Cube_FW_G0_V1.4.0/Drivers/CMSIS/Include -IC:/Users/andre/STM32Cube/Repository/STM32Cube_FW_G0_V1.4.0/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/Users/andre/STM32Cube/Repository/STM32Cube_FW_G0_V1.4.0/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su

.PHONY: clean-Core-2f-Src

