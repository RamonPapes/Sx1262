################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Libs/dev_sx1262/sx1262.c \
../Core/Libs/dev_sx1262/sx1262_hal.c 

OBJS += \
./Core/Libs/dev_sx1262/sx1262.o \
./Core/Libs/dev_sx1262/sx1262_hal.o 

C_DEPS += \
./Core/Libs/dev_sx1262/sx1262.d \
./Core/Libs/dev_sx1262/sx1262_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Libs/dev_sx1262/%.o Core/Libs/dev_sx1262/%.su Core/Libs/dev_sx1262/%.cyclo: ../Core/Libs/dev_sx1262/%.c Core/Libs/dev_sx1262/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I"C:/Users/ramon.papes/STM32CubeIDE/Lora1262_lib/Lib1262" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Libs-2f-dev_sx1262

clean-Core-2f-Libs-2f-dev_sx1262:
	-$(RM) ./Core/Libs/dev_sx1262/sx1262.cyclo ./Core/Libs/dev_sx1262/sx1262.d ./Core/Libs/dev_sx1262/sx1262.o ./Core/Libs/dev_sx1262/sx1262.su ./Core/Libs/dev_sx1262/sx1262_hal.cyclo ./Core/Libs/dev_sx1262/sx1262_hal.d ./Core/Libs/dev_sx1262/sx1262_hal.o ./Core/Libs/dev_sx1262/sx1262_hal.su

.PHONY: clean-Core-2f-Libs-2f-dev_sx1262

