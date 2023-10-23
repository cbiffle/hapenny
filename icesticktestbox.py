import itertools
import argparse
import struct
from pathlib import Path

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.build import ResourceError, Resource, Pins, Attrs
from amaranth_boards.test.blinky import Blinky
from amaranth_boards.icestick import ICEStickPlatform

from hapenny import StreamSig
from hapenny.boxcpu import Cpu
from hapenny.bus import BusPort, SimpleFabric, partial_decode
from hapenny.gpio import OutputPort
from hapenny.serial import BidiUart
from hapenny.mem import BasicMemory

RAM_WORDS = 256 * 2
RAM_ADDR_BITS = (RAM_WORDS - 1).bit_length()

BUS_ADDR_BITS = RAM_ADDR_BITS + 1

bootloader = Path("tiny-bootloader.bin").read_bytes()
boot_image = struct.unpack("<" + "h" * (len(bootloader) // 2), bootloader)

class Test(Elaboratable):
    def __init__(self):
        super().__init__()

    def elaborate(self, platform):
        m = Module()
        m.submodules.cpu = cpu = Cpu(
            # +1 to adjust from bus halfword addressing to CPU byte addressing.
            addr_width = BUS_ADDR_BITS + 1,
            # Program addresses only need to be able to address program memory,
            # so configure the PC and fetch port to be narrower. (+1 because,
            # again, our RAM is halfword addressed but this parameter is in
            # bytes.)
            prog_addr_width = RAM_ADDR_BITS + 1,
        )
        m.submodules.mem = mem = BasicMemory(depth = RAM_WORDS,
                                             contents = boot_image)
        m.submodules.uart = uart = BidiUart(baud_rate = 115_200)
        m.submodules.fabric = fabric = SimpleFabric([
            mem.bus,
            partial_decode(m, uart.bus, RAM_ADDR_BITS),
        ])

        connect(m, cpu.bus, fabric.bus)

        uartport = platform.request("uart", 0)
        m.d.comb += [
            uartport.tx.o.eq(uart.tx),
            uart.rx.eq(uartport.rx.i),
        ]

        return m

p = ICEStickPlatform()
p.build(Test())
