import itertools
import argparse

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.build import ResourceError, Resource, Pins, Attrs
from amaranth_boards.test.blinky import Blinky
from amaranth_boards.icestick import ICEStickPlatform

from hapenny import StreamSig
from hapenny.boxcpu import Cpu
from hapenny.bus import BusPort, SimpleFabric, partial_decode
from hapenny.gpio import OutputPort

RAM_WORDS = 256 * 1
BUS_ADDR_BITS = (RAM_WORDS - 1).bit_length()

class TestMemory(Component):
    bus: In(BusPort(addr = BUS_ADDR_BITS, data = 16))

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
            # +1 for byte addressing, +1 more for the mem/I/O bit.
            addr_width = BUS_ADDR_BITS + 1 + 1,
        )
        m.submodules.mem = mem = TestMemory([
            # 00000000 <reset>:
            #    0:   20000093                li      ra,512
            0x0093,
            0x2000,
            #    4:   00100113                li      sp,1
            0x0113,
            0x0010,
            #    8:   000f51b7                li      gp,0xf5
            0x000f,
            0x51b7,
            # 
            # 0000000c <loop>:
            #    c:   00209023                sh      sp,0(ra)
            0x9023,
            0x0020,
            #   10:   00018213                mv      tp,gp
            0x8213,
            0x0001,
            # 
            # 00000014 <loop2>:
            #   14:   fff20213                addi    tp,tp,-1
            0x0213,
            0xfff2,
            #   18:   fe020ee3                beqz    tp,14 <loop2>
            0x0ee3,
            0xfe02,
            #   1c:   00009023                sh      zero,0(ra)
            0x9023,
            0x0000,
            #   20:   00018213                mv      tp,gp
            0x8213,
            0x0001,
            # 
            # 00000024 <loop3>:
            #   24:   fff20213                addi    tp,tp,-1
            0x0213,
            0xfff2,
            #   28:   fe0206e3                beqz    tp,14 <loop2>
            0x06e3,
            0xfe02,
            #   2c:   fe1ff06f                j       c <loop>
            0xf06f,
            0xfe1f,
        ])
        m.submodules.port = port = OutputPort(1)
        m.submodules.fabric = fabric = SimpleFabric([
            mem.bus,
            partial_decode(m, port.bus, BUS_ADDR_BITS),
        ])

        connect(m, cpu.bus, fabric.bus)

        def get_all_resources(name):
            resources = []
            for number in itertools.count():
                try:
                    resources.append(platform.request(name, number))
                except ResourceError:
                    break
            return resources

        leds     = [res.o for res in get_all_resources("led")]
        m.d.comb += [
            leds[0].eq(port.pins),
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
