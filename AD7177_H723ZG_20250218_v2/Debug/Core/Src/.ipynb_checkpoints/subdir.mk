################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/.ipynb_checkpoints/main-checkpoint.c 

OBJS += \
./Core/Src/.ipynb_checkpoints/main-checkpoint.o 

C_DEPS += \
./Core/Src/.ipynb_checkpoints/main-checkpoint.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/.ipynb_checkpoints/%.o Core/Src/.ipynb_checkpoints/%.su Core/Src/.ipynb_checkpoints/%.cyclo: ../Core/Src/.ipynb_checkpoints/%.c Core/Src/.ipynb_checkpoints/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDATA_IN_D2_SRAM -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_PWR_LDO_SUPPLY -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/BSP/Components/lan8742 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f--2e-ipynb_checkpoints

clean-Core-2f-Src-2f--2e-ipynb_checkpoints:
	-$(RM) ./Core/Src/.ipynb_checkpoints/main-checkpoint.cyclo ./Core/Src/.ipynb_checkpoints/main-checkpoint.d ./Core/Src/.ipynb_checkpoints/main-checkpoint.o ./Core/Src/.ipynb_checkpoints/main-checkpoint.su

.PHONY: clean-Core-2f-Src-2f--2e-ipynb_checkpoints

