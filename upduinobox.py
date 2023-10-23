import itertools
import argparse
import struct
from pathlib import Path

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.build import ResourceError, Resource, Pins, Attrs
from amaranth_boards.upduino_v3 import UpduinoV3Platform

from hapenny import StreamSig
from hapenny.boxcpu import Cpu
from hapenny.bus import BusPort, SimpleFabric, partial_decode
from hapenny.serial import BidiUart
from hapenny.mem import BasicMemory

RAM_WORDS = 256 * 2
RAM_ADDR_BITS = (RAM_WORDS - 1).bit_length()

BUS_ADDR_BITS = RAM_ADDR_BITS + 1

print(f"configuring for {RAM_ADDR_BITS}-bit RAM and {BUS_ADDR_BITS}-bit bus")

bootloader = Path("tiny-bootloader.bin").read_bytes()
boot_image = struct.unpack("<" + "h" * (len(bootloader) // 2), bootloader)

class Test(Elaboratable):
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
        m.submodules.uart = uart = BidiUart(baud_rate = 115200)
        m.submodules.fabric = fabric = SimpleFabric([
            mem.bus,
            # Extend the GPIO port's address bus to match the RAM.
            partial_decode(m, uart.bus, RAM_ADDR_BITS),
        ])

        connect(m, cpu.bus, fabric.bus)
        platform.add_resources([
            Resource("tx", 0, Pins("7", dir="o", conn=("j", 0))),
            Resource("rx", 0, Pins("8", dir="i", conn=("j", 0))),
        ])
        def get_all_resources(name):
            resources = []
            for number in itertools.count():
                try:
                    resources.append(platform.request(name, number))
                except ResourceError:
                    break
            return resources

        tx = platform.request("tx", 0)
        rx = platform.request("rx", 0)
        rgb_led = platform.request("rgb_led", 0)

        m.d.comb += [
            rgb_led.r.o.eq(cpu.halted),
            tx.o[0].eq(uart.tx),
            uart.rx.eq(rx.i[0]),
        ]

        return m

p = UpduinoV3Platform()
p.hfosc_div = 1
p.build(Test(), do_program = True)
