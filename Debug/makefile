################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utilities/str/subdir.mk
-include utilities/debug_console_lite/subdir.mk
-include utilities/subdir.mk
-include startup/subdir.mk
-include source/subdir.mk
-include drivers/subdir.mk
-include device/subdir.mk
-include component/uart/subdir.mk
-include board/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := frdmmcxc444_i2c_read_fxls897x_accel_value_transfer
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: frdmmcxc444_i2c_read_fxls897x_accel_value_transfer.axf

# Tool invocations
frdmmcxc444_i2c_read_fxls897x_accel_value_transfer.axf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -Xlinker -no-warn-rwx-segments -Xlinker -Map="frdmmcxc444_i2c_read_fxls897x_accel_value_transfer.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -Xlinker --cref -mcpu=cortex-m0plus -mthumb -T frdmmcxc444_i2c_read_fxls897x_accel_value_transfer_Debug.ld -o "frdmmcxc444_i2c_read_fxls897x_accel_value_transfer.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) frdmmcxc444_i2c_read_fxls897x_accel_value_transfer.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "frdmmcxc444_i2c_read_fxls897x_accel_value_transfer.axf"; # arm-none-eabi-objcopy -v -O binary "frdmmcxc444_i2c_read_fxls897x_accel_value_transfer.axf" "frdmmcxc444_i2c_read_fxls897x_accel_value_transfer.bin" ; # checksum -p MCXC444 -d "frdmmcxc444_i2c_read_fxls897x_accel_value_transfer.bin";
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
