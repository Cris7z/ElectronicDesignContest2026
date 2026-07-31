TOOLCHAIN ?= arm-none-eabi-
CC := $(TOOLCHAIN)gcc
OBJCOPY := $(TOOLCHAIN)objcopy
SIZE := $(TOOLCHAIN)size

BUILD := build
TARGET := $(BUILD)/ball_beam_rct6

CFLAGS := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -Os -g3 \
  -ffunction-sections -fdata-sections -fno-common -Wall -Wextra \
  -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER \
  -IUSER -ICORE -ISTM32F10x_FWLib/inc -ISYSTEM/delay -ISYSTEM/sys \
  -ISYSTEM/usart -IHARDWARE
LDFLAGS := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -specs=nano.specs \
  -Wl,--gc-sections -Wl,-Tstm32f103rct6.ld -Wl,-Map,$(TARGET).map

SOURCES := \
  USER/main_d36a_test.c USER/system_stm32f10x.c USER/stm32f10x_it.c \
  SYSTEM/delay/delay.c SYSTEM/sys/sys_gcc.c SYSTEM/sys/syscalls_gcc.c SYSTEM/usart/usart_k230.c \
  HARDWARE/d36a_motor.c HARDWARE/ms42_encoder.c HARDWARE/adc.c \
  STM32F10x_FWLib/src/misc.c STM32F10x_FWLib/src/stm32f10x_adc.c \
  STM32F10x_FWLib/src/stm32f10x_exti.c STM32F10x_FWLib/src/stm32f10x_gpio.c \
  STM32F10x_FWLib/src/stm32f10x_rcc.c STM32F10x_FWLib/src/stm32f10x_tim.c \
  STM32F10x_FWLib/src/stm32f10x_usart.c
OBJECTS := $(patsubst %.c,$(BUILD)/%.o,$(SOURCES))

.PHONY: all clean flash
all: $(TARGET).bin

$(BUILD)/%.o: %.c
	@if not exist "$(@D)" mkdir "$(@D)"
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS) CORE/startup_stm32f103rct6_gcc.s stm32f103rct6.ld
	@if not exist "$(BUILD)" mkdir "$(BUILD)"
	$(CC) $(CFLAGS) -c CORE/startup_stm32f103rct6_gcc.s -o $(BUILD)/startup_stm32f103rct6_gcc.o
	$(CC) $(BUILD)/startup_stm32f103rct6_gcc.o $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

flash: $(TARGET).bin
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program $(TARGET).elf verify reset exit"

clean:
	@if exist "$(BUILD)" rmdir /S /Q "$(BUILD)"
