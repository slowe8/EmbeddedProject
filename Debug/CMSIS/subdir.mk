################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS/system_MKL46Z4.c 

C_DEPS += \
./CMSIS/system_MKL46Z4.d 

OBJS += \
./CMSIS/system_MKL46Z4.o 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/%.o: ../CMSIS/%.c CMSIS/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DCPU_MKL46Z256VLL4 -DCPU_MKL46Z256VLL4_cm0plus -DSDK_OS_BAREMETAL -DFSL_RTOS_BM -DPRINTF_FLOAT_ENABLE=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_DEBUGCONSOLE=0 -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\board" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\source" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\drivers" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\CMSIS" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\utilities" -I"C:\Users\seanm\Documents\MCUXpressoIDE_11.6.1_8255\workspace\Project 4\startup" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-CMSIS

clean-CMSIS:
	-$(RM) ./CMSIS/system_MKL46Z4.d ./CMSIS/system_MKL46Z4.o

.PHONY: clean-CMSIS

