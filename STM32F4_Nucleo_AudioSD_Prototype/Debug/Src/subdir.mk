################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/bsp_driver_sd.c \
../Src/fatfs.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c 

OBJS += \
./Src/bsp_driver_sd.o \
./Src/fatfs.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o 

C_DEPS += \
./Src/bsp_driver_sd.d \
./Src/fatfs.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F411xE -I"C:/cygwin64/home/Chris/Nucleo/NucleoTest/Inc" -I"C:/cygwin64/home/Chris/Nucleo/NucleoTest/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/cygwin64/home/Chris/Nucleo/NucleoTest/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/cygwin64/home/Chris/Nucleo/NucleoTest/Middlewares/Third_Party/FatFs/src/drivers" -I"C:/cygwin64/home/Chris/Nucleo/NucleoTest/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/cygwin64/home/Chris/Nucleo/NucleoTest/Middlewares/Third_Party/FatFs/src" -I"C:/cygwin64/home/Chris/Nucleo/NucleoTest/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


