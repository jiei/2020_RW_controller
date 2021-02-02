################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
2020_RW_controller.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
2020_RW_controller.list \

OBJCOPY_BIN += \
2020_RW_controller.bin \


# All Target
all: 2020_RW_controller.elf secondary-outputs

# Tool invocations
2020_RW_controller.elf: $(OBJS) $(USER_OBJS) /home/jack/STM32CubeIDE/workspace_1.4.0/2020_RW_controller/STM32F303K8TX_FLASH.ld
	arm-none-eabi-gcc -o "2020_RW_controller.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/jack/STM32CubeIDE/workspace_1.4.0/2020_RW_controller/STM32F303K8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="2020_RW_controller.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

2020_RW_controller.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "2020_RW_controller.list"
	@echo 'Finished building: $@'
	@echo ' '

2020_RW_controller.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "2020_RW_controller.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
