################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Ethernet/src/ethernet.c \
../Ethernet/src/socket.c \
../Ethernet/src/w5500.c \
../Ethernet/src/w5500_port.c 

C_DEPS += \
./Ethernet/src/ethernet.d \
./Ethernet/src/socket.d \
./Ethernet/src/w5500.d \
./Ethernet/src/w5500_port.d 

OBJS += \
./Ethernet/src/ethernet.o \
./Ethernet/src/socket.o \
./Ethernet/src/w5500.o \
./Ethernet/src/w5500_port.o 


# Each subdirectory must supply rules for building sources it contributes
Ethernet/src/%.o Ethernet/src/%.su Ethernet/src/%.cyclo: ../Ethernet/src/%.c Ethernet/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FatFs/src -I../Core/ThreadSafe -I"../Ethernet/inc" -I"D:/Project/STM32CubeIDE/STM32F746Z_SAKHY/Heap_pool/Inc" -I../IO_Control/Inc -I../FIFO_Queue/Inc -I../Audio_Mixing/Inc -I../Device/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Ethernet-2f-src

clean-Ethernet-2f-src:
	-$(RM) ./Ethernet/src/ethernet.cyclo ./Ethernet/src/ethernet.d ./Ethernet/src/ethernet.o ./Ethernet/src/ethernet.su ./Ethernet/src/socket.cyclo ./Ethernet/src/socket.d ./Ethernet/src/socket.o ./Ethernet/src/socket.su ./Ethernet/src/w5500.cyclo ./Ethernet/src/w5500.d ./Ethernet/src/w5500.o ./Ethernet/src/w5500.su ./Ethernet/src/w5500_port.cyclo ./Ethernet/src/w5500_port.d ./Ethernet/src/w5500_port.o ./Ethernet/src/w5500_port.su

.PHONY: clean-Ethernet-2f-src

