import itertools
import random

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.build import ResourceError
from amaranth_boards.test.blinky import Blinky
from amaranth_boards.icestick import ICEStickPlatform

from rv32 import StreamSig
from rv32.cpu16 import Cpu
from rv32.bus import BusPort

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

            # Nothing causes this memory to stop being available.
            self.bus.cmd.ready.eq(1),
        ]

        # Delay the read enable signal by one cycle to use as output valid.
        # TODO this isn't really correct and ignores ready.
        delayed_read = Signal(1)
        m.d.sync += delayed_read.eq(rp.en)

        m.d.comb += [
            self.bus.resp.valid.eq(delayed_read),
            self.bus.resp.payload.eq(rp.data),
        ]

        return m


class Test(Elaboratable):
    def elaborate(self, platform):
        m = Module()
        m.submodules.cpu = cpu = Cpu(
            addr_width = BUS_ADDR_BITS + 1,
        )
        random.seed("omglol")
        m.submodules.mem = mem = TestMemory([
            random.getrandbits(16) for _ in range(RAM_WORDS)
            #0b00000000000000000000000000010011,
            #0b00000000000000000000000001100111,
            #0b10000000000000000000000000100011,
            #0b000010101100_00000_010_00001_0000011, # LW x1, (x0 + 0xAC)
            #0b10101010101010101010_00001_0010111, # AUIPC x1, ...
            #0b10101010101010101010_00001_0110111, # LUI x1, 0xAAAAA
            #0b0_100000_00010_00001_000_0000_0_1100011, # BEQ something
            #0b00000000000000000000_00000_1101111, # JAL x0, .
        ])

        connect(m, cpu.bus, mem.bus)

        def get_all_resources(name):
            resources = []
            for number in itertools.count():
                try:
                    resources.append(platform.request(name, number))
                except ResourceError:
                    break
            return resources

        leds     = [res.o for res in get_all_resources("led")]

        m.d.comb += leds[0].eq(cpu.bus.cmd.valid)

        return m

ICEStickPlatform().build(Test())
