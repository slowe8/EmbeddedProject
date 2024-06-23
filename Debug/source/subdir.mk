################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/I2C.c \
../source/LineSensors.c \
../source/Project\ 4-fallback.c \
../source/Utilities.c \
../source/mtb.c \
../source/semihost_hardfault.c 

C_DEPS += \
./source/I2C.d \
./source/LineSensors.d \
./source/Project\ 4-fallback.d \
./source/Utilities.d \
./source/mtb.d \
./source/semihost_hardfault.d 

OBJS += \
./source/I2C.o \
./source/LineSensors.o \
./source/Project\ 4-fallback.o \
./source/Utilities.o \
./source/mtb.o \
./source/semihost_hardfault.o 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DCPU_MKL46Z256VLL4 -DCPU_MKL46Z256VLL4_cm0plus -DSDK_OS_BAREMETAL -DFSL_RTOS_BM -DPRINTF_FLOAT_ENABLE=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_DEBUGCONSOLE=0 -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\board" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\source" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\drivers" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\CMSIS" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\utilities" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\startup" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/Project\ 4-fallback.o: ../source/Project\ 4-fallback.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DCPU_MKL46Z256VLL4 -DCPU_MKL46Z256VLL4_cm0plus -DSDK_OS_BAREMETAL -DFSL_RTOS_BM -DPRINTF_FLOAT_ENABLE=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_DEBUGCONSOLE=0 -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\board" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\source" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\drivers" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\CMSIS" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\utilities" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\startup" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"source/Project 4-fallback.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source

clean-source:
	-$(RM) ./source/I2C.d ./source/I2C.o ./source/LineSensors.d ./source/LineSensors.o ./source/Project\ 4-fallback.d ./source/Project\ 4-fallback.o ./source/Utilities.d ./source/Utilities.o ./source/mtb.d ./source/mtb.o ./source/semihost_hardfault.d ./source/semihost_hardfault.o

.PHONY: clean-source

