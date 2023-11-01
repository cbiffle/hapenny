import itertools
import argparse
import struct
from pathlib import Path

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.build import ResourceError, Resource, Pins, Attrs
from amaranth_boards.upduino_v3 import UpduinoV3Platform

from hapenny import StreamSig
import hapenny.chonk.cpu
from hapenny.bus import BusPort, SimpleFabric, partial_decode, narrow_addr
from hapenny.chonk.gpio32 import OutputPort32
from hapenny.chonk.serial32 import BidiUart
from hapenny.chonk.mem32 import BasicMemory, SpramMemory

RAM_WORDS = 256 * 1
RAM_ADDR_BITS = (RAM_WORDS - 1).bit_length()

print(f"boot memory will use {RAM_ADDR_BITS}-bit addressing")

bootloader = Path("tinyboot-upduino-chonk.bin").read_bytes()
boot_image = struct.unpack("<" + "I" * (len(bootloader) // 4), bootloader)

class Test(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        m.submodules.cpu = cpu = hapenny.chonk.cpu.Cpu(
            reset_vector = 0x1_0000,
            # 2 for the width of the fabric's port select
            # 14 for the width of the devices attached to the ports
            # +2 because the fabric is word-addressed but this is in bytes
            addr_width = 2 + 14 + 2,
            # Program addresses only need to be able to address RAM, not
            # I/O, so configure the PC and fetch port to be narrower. (+2
            # because, again, our RAM is halfword addressed but this
            # parameter is in bytes.)
            prog_addr_width = 1 + 14 + 2,
            counters = True,
        )
        m.submodules.bootmem = bootmem = BasicMemory(depth = RAM_WORDS,
                                                     contents = boot_image)
        m.submodules.bulkmem0 = bulkmem0 = SpramMemory()
        m.submodules.port = port = OutputPort32(1)
        m.submodules.uart = uart = BidiUart(baud_rate = 115200, oversample=8)
        m.submodules.fabric = fabric = SimpleFabric([
            # Put all the potentially executable RAM in the bottom portion of
            # the address space, to allow PC and fetch circuitry to be slightly
            # narrower. This helps with timing.
            bulkmem0.bus,                       # 0x0000_0000
            partial_decode(m, bootmem.bus, 14), # 0x0001_0000
            partial_decode(m, port.bus, 14),    # 0x0002_0000
            partial_decode(m, uart.bus, 14),    # 0x0003_0000
        ])

        connect(m, cpu.bus, fabric.bus)
        platform.add_resources([
            Resource("tx", 0, Pins("7", dir="o", conn=("j", 0))),
            Resource("rx", 0, Pins("8", dir="i", conn=("j", 0))),
        ])

        tx = platform.request("tx", 0)
        rx = platform.request("rx", 0)

        rgb_led = platform.request("rgb_led", 0)
        m.d.comb += [
            rgb_led.r.o.eq(cpu.halted),
            rgb_led.g.o.eq(port.pins[0]),
            tx.o[0].eq(uart.tx),
            uart.rx.eq(rx.i[0]),
        ]

        return m

parser = argparse.ArgumentParser(
    prog = "upduino-chonk",
    description = "Script for synthesizing a larger UPduino SoC using chonk",
)
args = parser.parse_args()


p = UpduinoV3Platform()
p.hfosc_div = 2 # divide 48MHz by 2**1 = 24 MHz
p.build(Test(), do_program = True)
