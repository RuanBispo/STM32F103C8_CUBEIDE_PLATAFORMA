################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
EX6_PWM_MOTOR_DC.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
EX6_PWM_MOTOR_DC.list \

OBJCOPY_HEX += \
EX6_PWM_MOTOR_DC.hex \

OBJCOPY_BIN += \
EX6_PWM_MOTOR_DC.bin \


# All Target
all: EX6_PWM_MOTOR_DC.elf secondary-outputs

# Tool invocations
EX6_PWM_MOTOR_DC.elf: $(OBJS) $(USER_OBJS) C:\Users\ruanr\Desktop\STM32F103C8\ _CUBEIDE_PLATAFORMA\EXEMPLOS\EX6_PWM_MOTOR_DC\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "EX6_PWM_MOTOR_DC.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\ruanr\Desktop\STM32F103C8 _CUBEIDE_PLATAFORMA\EXEMPLOS\EX6_PWM_MOTOR_DC\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="EX6_PWM_MOTOR_DC.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

EX6_PWM_MOTOR_DC.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "EX6_PWM_MOTOR_DC.list"
	@echo 'Finished building: $@'
	@echo ' '

EX6_PWM_MOTOR_DC.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "EX6_PWM_MOTOR_DC.hex"
	@echo 'Finished building: $@'
	@echo ' '

EX6_PWM_MOTOR_DC.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "EX6_PWM_MOTOR_DC.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
