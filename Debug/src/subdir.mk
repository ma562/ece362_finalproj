################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/background.c \
../src/ball.c \
../src/cat.c \
../src/fifo.c \
../src/lcd.c \
../src/main.c \
../src/map_bank.c \
../src/midi.c \
../src/step.c \
../src/super.c \
../src/support.c \
../src/syscalls.c \
../src/system_stm32f0xx.c \
../src/tty.c \
../src/wall.c \
../src/wavetable.c 

OBJS += \
./src/background.o \
./src/ball.o \
./src/cat.o \
./src/fifo.o \
./src/lcd.o \
./src/main.o \
./src/map_bank.o \
./src/midi.o \
./src/step.o \
./src/super.o \
./src/support.o \
./src/syscalls.o \
./src/system_stm32f0xx.o \
./src/tty.o \
./src/wall.o \
./src/wavetable.o 

C_DEPS += \
./src/background.d \
./src/ball.d \
./src/cat.d \
./src/fifo.d \
./src/lcd.d \
./src/main.d \
./src/map_bank.d \
./src/midi.d \
./src/step.d \
./src/super.d \
./src/support.d \
./src/syscalls.d \
./src/system_stm32f0xx.d \
./src/tty.d \
./src/wall.d \
./src/wavetable.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F0 -DSTM32F091VCTx -DDEBUG -DSTM32F091 -DUSE_STDPERIPH_DRIVER -I"/home/shay/a/ma562/Desktop/First 6 mazes added/G15/StdPeriph_Driver/inc" -I"/home/shay/a/ma562/Desktop/First 6 mazes added/G15/inc" -I"/home/shay/a/ma562/Desktop/First 6 mazes added/G15/CMSIS/device" -I"/home/shay/a/ma562/Desktop/First 6 mazes added/G15/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


