################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../utilities/fsl_assert.c 

C_DEPS += \
./utilities/fsl_assert.d 

OBJS += \
./utilities/fsl_assert.o 


# Each subdirectory must supply rules for building sources it contributes
utilities/%.o: ../utilities/%.c utilities/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_MCXC444VLH -DCPU_MCXC444VLH_cm0plus -DMCUXPRESSO_SDK -DPRINTF_ADVANCED_ENABLE=1 -DFRDM_MCXC444 -DFREEDOM -DSDK_DEBUGCONSOLE=1 -DMCUX_META_BUILD -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\source" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\drivers" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\CMSIS" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\CMSIS\m-profile" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\device" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\device\periph2" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\utilities" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\utilities\str" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\utilities\debug_console_lite" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\component\uart" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\board" -O0 -fno-common -g3 -gdwarf-4 -c -ffunction-sections -fdata-sections -fno-builtin -imacros "C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\frdmmcxc444_i2c_read_fxls897x_accel_value_transfer\source\mcux_config.h" -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-utilities

clean-utilities:
	-$(RM) ./utilities/fsl_assert.d ./utilities/fsl_assert.o

.PHONY: clean-utilities

