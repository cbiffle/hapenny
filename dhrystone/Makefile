UARTADDR ?= 0x18000
STACK ?= 0x8000
OBJS = dhry_1.o dhry_2.o stdlib.o start.o
CFLAGS = -MD -O3 -mabi=ilp32 -march=rv32i -DTIME -DRISCV -g3
TOOLCHAIN_PREFIX = riscv64-unknown-elf-

CFLAGS += -DUSE_MYSTDLIB -ffreestanding -nostdlib -DUARTADDR=$(UARTADDR) -DSTACK=$(STACK)

dhry.bin: dhry.elf
	$(TOOLCHAIN_PREFIX)objcopy -Obinary $^ $@

dhry.elf: $(OBJS) sections.lds
	$(TOOLCHAIN_PREFIX)gcc $(CFLAGS) -Wl,-Bstatic,-T,sections.lds,-Map,dhry.map,--strip-debug -o $@ $(OBJS) -lgcc
	chmod -x $@

%.o: %.c
	$(TOOLCHAIN_PREFIX)gcc -c $(CFLAGS) $<

%.o: %.S
	$(TOOLCHAIN_PREFIX)gcc -c $(CFLAGS) $<

dhry_1.o dhry_2.o: CFLAGS += -Wno-implicit-int -Wno-implicit-function-declaration

clean:
	rm -rf *.o *.d dhry.elf dhry.map dhry.bin dhry.hex

.PHONY: test clean

-include *.d

