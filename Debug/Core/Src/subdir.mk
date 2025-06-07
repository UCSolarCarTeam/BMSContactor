################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Gatekeeper.c \
../Core/Src/MessageReader.c \
../Core/Src/SendingCANMessage.c \
../Core/Src/changeSwitch.c \
../Core/Src/main.c \
../Core/Src/makingCANMessage.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/Gatekeeper.o \
./Core/Src/MessageReader.o \
./Core/Src/SendingCANMessage.o \
./Core/Src/changeSwitch.o \
./Core/Src/main.o \
./Core/Src/makingCANMessage.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/Gatekeeper.d \
./Core/Src/MessageReader.d \
./Core/Src/SendingCANMessage.d \
./Core/Src/changeSwitch.d \
./Core/Src/main.d \
./Core/Src/makingCANMessage.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Gatekeeper.cyclo ./Core/Src/Gatekeeper.d ./Core/Src/Gatekeeper.o ./Core/Src/Gatekeeper.su ./Core/Src/MessageReader.cyclo ./Core/Src/MessageReader.d ./Core/Src/MessageReader.o ./Core/Src/MessageReader.su ./Core/Src/SendingCANMessage.cyclo ./Core/Src/SendingCANMessage.d ./Core/Src/SendingCANMessage.o ./Core/Src/SendingCANMessage.su ./Core/Src/changeSwitch.cyclo ./Core/Src/changeSwitch.d ./Core/Src/changeSwitch.o ./Core/Src/changeSwitch.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/makingCANMessage.cyclo ./Core/Src/makingCANMessage.d ./Core/Src/makingCANMessage.o ./Core/Src/makingCANMessage.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src

