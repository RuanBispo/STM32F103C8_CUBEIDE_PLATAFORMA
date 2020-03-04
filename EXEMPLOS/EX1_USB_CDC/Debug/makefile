################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
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
EX1_USB_CDC.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
EX1_USB_CDC.list \

OBJCOPY_HEX += \
EX1_USB_CDC.hex \

OBJCOPY_BIN += \
EX1_USB_CDC.bin \


# All Target
all: EX1_USB_CDC.elf secondary-outputs

# Tool invocations
EX1_USB_CDC.elf: $(OBJS) $(USER_OBJS) C:\Users\ruanr\Documents\GitHub\STM32F103C8_CUBEIDE_PLATAFORMA\EXEMPLOS\EX1_USB_CDC\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "EX1_USB_CDC.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\ruanr\Documents\GitHub\STM32F103C8_CUBEIDE_PLATAFORMA\EXEMPLOS\EX1_USB_CDC\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="EX1_USB_CDC.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

EX1_USB_CDC.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "EX1_USB_CDC.list"
	@echo 'Finished building: $@'
	@echo ' '

EX1_USB_CDC.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "EX1_USB_CDC.hex"
	@echo 'Finished building: $@'
	@echo ' '

EX1_USB_CDC.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "EX1_USB_CDC.bin"
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
