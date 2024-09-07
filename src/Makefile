PROJECT_NAME = fiubasat

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

INCLUDES = \
	-I../include \
	-I../lib/rtos \
	-I../lib/libopencm3/include

SOURCES = \
	main.c \
	blink.c \
	test.c \
	uart.c \
	spi_driver.c\
	../lib/rtos/heap_4.c \
	../lib/rtos/list.c \
	../lib/rtos/port.c \
	../lib/rtos/tasks.c \
	../lib/rtos/queue.c \
	../lib/rtos/timers.c
#	../lib/rtos/opencm3.c

CFLAGS = -std=c99 -Wall -Wextra -Wshadow -Wimplicit-function-declaration -Wredundant-decls \
	-Wmissing-prototypes -Wstrict-prototypes -fno-common -ffunction-sections -fdata-sections \
	-Os -g -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd -DSTM32F1 -flto $(INCLUDES)

LDFLAGS = -T./stm32f103c8t6.ld -nostartfiles -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs -Wl,--undefined=vTaskSwitchContext

LDLIBS = -L../lib/libopencm3/lib -lopencm3_stm32f1

OBJS = $(SOURCES:.c=.o)

all: check_libopencm3 $(PROJECT_NAME).elf clean

check_libopencm3:
	@if [ ! -d "../lib/libopencm3" ] || [ -z "$$(ls -A ../lib/libopencm3 2>/dev/null)" ]; then \
		echo "Clonando libopencm3..."; \
		git clone https://github.com/libopencm3/libopencm3.git ../lib/libopencm3; \
	else \
		echo "libopencm3 ya está presente."; \
	fi
	@echo "Editando el target del Makefile de libopencm3..."; \
	sed -i 's/^TARGETS = .*/TARGETS = stm32\/f1/' ../lib/libopencm3/Makefile; \
	echo "Construyendo libopencm3..."; \
	cd ../lib/libopencm3 && make TARGETS=stm32/f1

$(PROJECT_NAME).elf: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@
	$(OBJCOPY) -O binary $@ $(PROJECT_NAME).bin
	$(SIZE) $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(PROJECT_NAME).elf

delete:
	rm -f $(OBJS) $(PROJECT_NAME).elf $(PROJECT_NAME).bin

flash:
	openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c "program $(PROJECT_NAME).bin 0x08000000" -c "shutdown"

.PHONY: all clean delete flash check_libopencm3