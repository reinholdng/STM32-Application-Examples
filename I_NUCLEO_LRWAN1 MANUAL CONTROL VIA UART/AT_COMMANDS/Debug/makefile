################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
AT_COMMANDS.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
AT_COMMANDS.list \


# All Target
all: AT_COMMANDS.elf secondary-outputs

# Tool invocations
AT_COMMANDS.elf: $(OBJS) $(USER_OBJS) /home/reinhold/STM32CubeIDE/workspace_1.0.2/AT_COMMANDS/STM32L476RGTX_FLASH.ld
	arm-none-eabi-gcc -o "AT_COMMANDS.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/reinhold/STM32CubeIDE/workspace_1.0.2/AT_COMMANDS/STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="AT_COMMANDS.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

AT_COMMANDS.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "AT_COMMANDS.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
