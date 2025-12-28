################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Heap_pool/Src/heap_pool.c 

C_DEPS += \
./Heap_pool/Src/heap_pool.d 

OBJS += \
./Heap_pool/Src/heap_pool.o 


# Each subdirectory must supply rules for building sources it contributes
Heap_pool/Src/%.o Heap_pool/Src/%.su Heap_pool/Src/%.cyclo: ../Heap_pool/Src/%.c Heap_pool/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FatFs/src -I../Core/ThreadSafe -I"../Ethernet/inc" -I"D:/Project/STM32CubeIDE/STM32F746Z_SAKHY/Heap_pool/Inc" -I../IO_Control/Inc -I../FIFO_Queue/Inc -I../Audio_Mixing/Inc -I../Device/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Heap_pool-2f-Src

clean-Heap_pool-2f-Src:
	-$(RM) ./Heap_pool/Src/heap_pool.cyclo ./Heap_pool/Src/heap_pool.d ./Heap_pool/Src/heap_pool.o ./Heap_pool/Src/heap_pool.su

.PHONY: clean-Heap_pool-2f-Src

