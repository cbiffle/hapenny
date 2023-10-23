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

RAM_WORDS = 256 * 2
RAM_ADDR_BITS = (RAM_WORDS - 1).bit_length()

BUS_ADDR_BITS = RAM_ADDR_BITS + 1

bootloader = Path("tiny-bootloader.bin").read_bytes()
boot_image = struct.unpack("<" + "h" * (len(bootloader) // 2), bootloader)

class TestMemory(Component):
    bus: In(BusPort(addr = RAM_ADDR_BITS, data = 16))

    def __init__(self, contents):
        super().__init__()

        self.m = Memory(
            width = 16,
            depth = RAM_WORDS,
            name = "testram",
            init = contents,
        )

    def elaborate(self, platform):
        m = Module()

        m.submodules.m = self.m

        rp = self.m.read_port(transparent = False)
        wp = self.m.write_port(granularity = 8)

        m.d.comb += [
            rp.addr.eq(self.bus.cmd.payload.addr[1:]),
            rp.en.eq(self.bus.cmd.valid & (self.bus.cmd.payload.lanes == 0)),

            wp.addr.eq(self.bus.cmd.payload.addr[1:]),
            wp.data.eq(self.bus.cmd.payload.data),
            wp.en[0].eq(self.bus.cmd.valid & self.bus.cmd.payload.lanes[0]),
            wp.en[1].eq(self.bus.cmd.valid & self.bus.cmd.payload.lanes[1]),
        ]

        m.d.comb += [
            self.bus.resp.eq(rp.data),
        ]

        return m


class Test(Elaboratable):
    def __init__(self, has_interrupt = None):
        super().__init__()

        self.has_interrupt = has_interrupt

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
        m.submodules.mem = mem = TestMemory(boot_image)
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
        if self.has_interrupt is not None:
            irq = platform.request("irq", 0)
            m.d.comb += [
                cpu.irq.eq(irq.i),
            ]

        return m

parser = argparse.ArgumentParser(
    prog = "icesticktest16",
    description = "Script for synthesizing image for HX1K",
)
parser.add_argument('-i', '--interrupt-model', help = 'which interrupt model to use', required = False, choices = ['m', 'fast'])
args = parser.parse_args()

p = ICEStickPlatform()
p.resources["irq", 0] = Resource("irq", 0, Pins("78", dir="i"), Attrs(IO_STANDARD="SB_LVCMOS"))
p.build(Test(has_interrupt = args.interrupt_model))
