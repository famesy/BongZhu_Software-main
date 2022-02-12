################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AMT21.c \
../Core/Src/ARMsProtocol.c \
../Core/Src/KalmanFilter.c \
../Core/Src/PID.c \
../Core/Src/adc.c \
../Core/Src/crc.c \
../Core/Src/gpio.c \
../Core/Src/kinematic.c \
../Core/Src/main.c \
../Core/Src/motor.c \
../Core/Src/spi.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/AMT21.o \
./Core/Src/ARMsProtocol.o \
./Core/Src/KalmanFilter.o \
./Core/Src/PID.o \
./Core/Src/adc.o \
./Core/Src/crc.o \
./Core/Src/gpio.o \
./Core/Src/kinematic.o \
./Core/Src/main.o \
./Core/Src/motor.o \
./Core/Src/spi.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/AMT21.d \
./Core/Src/ARMsProtocol.d \
./Core/Src/KalmanFilter.d \
./Core/Src/PID.d \
./Core/Src/adc.d \
./Core/Src/crc.d \
./Core/Src/gpio.d \
./Core/Src/kinematic.d \
./Core/Src/main.d \
./Core/Src/motor.d \
./Core/Src/spi.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H733xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AMT21.d ./Core/Src/AMT21.o ./Core/Src/ARMsProtocol.d ./Core/Src/ARMsProtocol.o ./Core/Src/KalmanFilter.d ./Core/Src/KalmanFilter.o ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/crc.d ./Core/Src/crc.o ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/kinematic.d ./Core/Src/kinematic.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/usart.d ./Core/Src/usart.o

.PHONY: clean-Core-2f-Src

