################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/mcp23008_driver/mcp23008_driver.c \
../Core/mcp23008_driver/mcp23008_driver_basic.c \
../Core/mcp23008_driver/mcp23008_driver_interface.c 

OBJS += \
./Core/mcp23008_driver/mcp23008_driver.o \
./Core/mcp23008_driver/mcp23008_driver_basic.o \
./Core/mcp23008_driver/mcp23008_driver_interface.o 

C_DEPS += \
./Core/mcp23008_driver/mcp23008_driver.d \
./Core/mcp23008_driver/mcp23008_driver_basic.d \
./Core/mcp23008_driver/mcp23008_driver_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/mcp23008_driver/%.o Core/mcp23008_driver/%.su: ../Core/mcp23008_driver/%.c Core/mcp23008_driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I"E:/Library Master/mcp23x08 IO Expander/Project Example/STM32L434KCU6/Core/mcp23008_driver" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-mcp23008_driver

clean-Core-2f-mcp23008_driver:
	-$(RM) ./Core/mcp23008_driver/mcp23008_driver.d ./Core/mcp23008_driver/mcp23008_driver.o ./Core/mcp23008_driver/mcp23008_driver.su ./Core/mcp23008_driver/mcp23008_driver_basic.d ./Core/mcp23008_driver/mcp23008_driver_basic.o ./Core/mcp23008_driver/mcp23008_driver_basic.su ./Core/mcp23008_driver/mcp23008_driver_interface.d ./Core/mcp23008_driver/mcp23008_driver_interface.o ./Core/mcp23008_driver/mcp23008_driver_interface.su

.PHONY: clean-Core-2f-mcp23008_driver

