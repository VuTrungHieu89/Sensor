################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middleware/Src/GPS.c 

OBJS += \
./Middleware/Src/GPS.o 

C_DEPS += \
./Middleware/Src/GPS.d 


# Each subdirectory must supply rules for building sources it contributes
Middleware/Src/%.o Middleware/Src/%.su Middleware/Src/%.cyclo: ../Middleware/Src/%.c Middleware/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Middleware/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middleware-2f-Src

clean-Middleware-2f-Src:
	-$(RM) ./Middleware/Src/GPS.cyclo ./Middleware/Src/GPS.d ./Middleware/Src/GPS.o ./Middleware/Src/GPS.su

.PHONY: clean-Middleware-2f-Src

