################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/TiBoostXLSensors/sensors.c 

OBJS += \
./Core/TiBoostXLSensors/sensors.o 

C_DEPS += \
./Core/TiBoostXLSensors/sensors.d 


# Each subdirectory must supply rules for building sources it contributes
Core/TiBoostXLSensors/%.o Core/TiBoostXLSensors/%.su Core/TiBoostXLSensors/%.cyclo: ../Core/TiBoostXLSensors/%.c Core/TiBoostXLSensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I"D:/BITSILICA/SALMAN/Learning/STM32/STM32_Projects/STM32L4xx_TI_Sensors/STM32L4xx_Ti_Sensors/Core/TiBoostXLSensors" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-TiBoostXLSensors

clean-Core-2f-TiBoostXLSensors:
	-$(RM) ./Core/TiBoostXLSensors/sensors.cyclo ./Core/TiBoostXLSensors/sensors.d ./Core/TiBoostXLSensors/sensors.o ./Core/TiBoostXLSensors/sensors.su

.PHONY: clean-Core-2f-TiBoostXLSensors

