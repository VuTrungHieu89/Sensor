################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middleware/Scr/BMP180.c 

OBJS += \
./Middleware/Scr/BMP180.o 

C_DEPS += \
./Middleware/Scr/BMP180.d 


# Each subdirectory must supply rules for building sources it contributes
Middleware/Scr/%.o Middleware/Scr/%.su Middleware/Scr/%.cyclo: ../Middleware/Scr/%.c Middleware/Scr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/Embedded STM32F1/BMP180/Middleware/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middleware-2f-Scr

clean-Middleware-2f-Scr:
	-$(RM) ./Middleware/Scr/BMP180.cyclo ./Middleware/Scr/BMP180.d ./Middleware/Scr/BMP180.o ./Middleware/Scr/BMP180.su

.PHONY: clean-Middleware-2f-Scr

