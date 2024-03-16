################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/NHK2024_Library/example/pid/i_pd.c \
../Core/NHK2024_Library/example/pid/pid.c 

OBJS += \
./Core/NHK2024_Library/example/pid/i_pd.o \
./Core/NHK2024_Library/example/pid/pid.o 

C_DEPS += \
./Core/NHK2024_Library/example/pid/i_pd.d \
./Core/NHK2024_Library/example/pid/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/NHK2024_Library/example/pid/%.o Core/NHK2024_Library/example/pid/%.su Core/NHK2024_Library/example/pid/%.cyclo: ../Core/NHK2024_Library/example/pid/%.c Core/NHK2024_Library/example/pid/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-NHK2024_Library-2f-example-2f-pid

clean-Core-2f-NHK2024_Library-2f-example-2f-pid:
	-$(RM) ./Core/NHK2024_Library/example/pid/i_pd.cyclo ./Core/NHK2024_Library/example/pid/i_pd.d ./Core/NHK2024_Library/example/pid/i_pd.o ./Core/NHK2024_Library/example/pid/i_pd.su ./Core/NHK2024_Library/example/pid/pid.cyclo ./Core/NHK2024_Library/example/pid/pid.d ./Core/NHK2024_Library/example/pid/pid.o ./Core/NHK2024_Library/example/pid/pid.su

.PHONY: clean-Core-2f-NHK2024_Library-2f-example-2f-pid

