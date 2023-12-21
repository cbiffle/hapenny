import itertools
import argparse
import struct
from pathlib import Path

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.build import ResourceError, Resource, Pins, Attrs
from amaranth_boards.test.blinky import Blinky
from amaranth_boards.resources.interface import UARTResource
from amaranth_boards.tinyfpga_bx import TinyFPGABXPlatform
import amaranth.lib.cdc

from hapenny import StreamSig
from hapenny.cpu import Cpu
from hapenny.bus import BusPort, SimpleFabric, partial_decode
from hapenny.serial import BidiUart
from hapenny.mem import BasicMemory

# tiny-bootloader is written in a high-level language and needs to have a stack,
RAM_WORDS = 256
RAM_ADDR_BITS = (RAM_WORDS - 1).bit_length()
BUS_ADDR_BITS = RAM_ADDR_BITS + 1

bootloader = Path("tinybx8k.bin").read_bytes()
boot_image = struct.unpack("<" + "h" * (len(bootloader) // 2), bootloader)


class Test(Elaboratable):
    def __init__(self):
        super().__init__()

    def elaborate(self, platform):
        m = Module()
        F = 16e6  # Hz

        m.submodules.cpu = cpu = Cpu(
            # execute from the bootloader dump.
            reset_vector=4096,
            addr_width=14,
        )
        m.submodules.mainmem = mainmem = BasicMemory(depth=256 * 16)
        m.submodules.mem = bootmem = BasicMemory(depth=RAM_WORDS, contents=boot_image)
        m.submodules.uart = uart = BidiUart(
            baud_rate=115_200, oversample=2, clock_freq=F
        )

        m.submodules.iofabric = iofabric = SimpleFabric(
            [
                partial_decode(m, bootmem.bus, 11),  # 0x____0000
                partial_decode(m, uart.bus, 11),  # 0x____1000
            ]
        )
        m.submodules.fabric = fabric = SimpleFabric(
            [
                mainmem.bus,
                partial_decode(m, iofabric.bus, 12), 
            ]
        )

        connect(m, cpu.bus, fabric.bus)

        uartpins = platform.request("uart", 0)

        rx_post_sync = Signal()
        m.submodules.rxsync = amaranth.lib.cdc.FFSynchronizer(
            i=uartpins.rx.i,
            o=rx_post_sync,
            o_domain="sync",
            reset=1,
            stages=2,
        )
        m.d.comb += [
            uartpins.tx.o.eq(uart.tx),
            uart.rx.eq(rx_post_sync),
        ]

        return m


p = TinyFPGABXPlatform()
# 3.3V FTDI connected to the tinybx.
# pico running micro python to run
p.add_resources(
    [
        UARTResource(
            0, rx="A8", tx="B8", attrs=Attrs(IO_STANDARD="SB_LVCMOS", PULLUP=1)
        ),
        Resource("reset_pin", 0, Pins("A9", dir="i"), Attrs(IO_STANDARD="SB_LVCMOS")),
        Resource("warmboot", 0, Pins("H2", dir="i"), Attrs(IO_STANDARD="SB_LVCMOS")),
    ]
)

p.build(Test(), do_program=True)

# TINYBOOT_UART_ADDR=12288 cargo objcopy --release -- -O binary ../tinybx8k.bin
# MEMORY {
#    PROGMEM (rwx): ORIGIN = 0x2000, LENGTH = 512
#    RAM (rw): ORIGIN = 0x0000, LENGTH = 8192
# }