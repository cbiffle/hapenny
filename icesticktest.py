import itertools
import random

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.build import ResourceError
from amaranth_boards.test.blinky import Blinky
from amaranth_boards.icestick import ICEStickPlatform

from rv32 import StreamSig
from rv32.cpu import Cpu, MemCmd

class TestMemory(Component):
    command: In(StreamSig(MemCmd))
    response: Out(StreamSig(32))

    def __init__(self, contents):
        super().__init__()

        self.m = Memory(
            width = 32,
            depth = 256,
            name = "testram",
            init = contents,
        )

    def elaborate(self, platform):
        m = Module()

        m.submodules.m = self.m

        rp = self.m.read_port(transparent = False)
        wp = self.m.write_port(granularity = 8)

        m.d.comb += [
            rp.addr.eq(self.command.payload.addr[2:]),
            rp.en.eq(self.command.valid & (self.command.payload.lanes == 0)),

            wp.addr.eq(self.command.payload.addr[2:]),
            wp.data.eq(self.command.payload.data),
            wp.en[0].eq(self.command.valid & self.command.payload.lanes[0]),
            wp.en[1].eq(self.command.valid & self.command.payload.lanes[1]),
            wp.en[2].eq(self.command.valid & self.command.payload.lanes[2]),
            wp.en[3].eq(self.command.valid & self.command.payload.lanes[3]),

            # Nothing causes this memory to stop being available.
            self.command.ready.eq(1),
        ]

        # Delay the read enable signal by one cycle to use as output valid.
        # TODO this isn't really correct and ignores ready.
        delayed_read = Signal(1)
        m.d.sync += delayed_read.eq(rp.en)

        m.d.comb += [
            self.response.valid.eq(delayed_read),
            self.response.payload.eq(rp.data),
        ]

        return m


class Test(Elaboratable):
    def elaborate(self, platform):
        m = Module()
        m.submodules.cpu = cpu = Cpu(
            #check_alignment = False,
            #wait = False,
            pc_width = 10,
            allow_halt_request = False,
        )
        random.seed("omglol")
        m.submodules.mem = mem = TestMemory([
            random.getrandbits(32) for _ in range(256)
            #0b00000000000000000000000000010011,
            #0b00000000000000000000000001100111,
            #0b10000000000000000000000000100011,
            #0b000010101100_00000_010_00001_0000011, # LW x1, (x0 + 0xAC)
            #0b10101010101010101010_00001_0010111, # AUIPC x1, ...
            #0b10101010101010101010_00001_0110111, # LUI x1, 0xAAAAA
            #0b0_100000_00010_00001_000_0000_0_1100011, # BEQ something
            #0b00000000000000000000_00000_1101111, # JAL x0, .
        ])

        connect(m, cpu.mem_out, mem.command)
        connect(m, cpu.mem_in, mem.response)

        def get_all_resources(name):
            resources = []
            for number in itertools.count():
                try:
                    resources.append(platform.request(name, number))
                except ResourceError:
                    break
            return resources

        leds     = [res.o for res in get_all_resources("led")]

        m.d.comb += leds[0].eq(cpu.mem_out.valid)

        return m

ICEStickPlatform().build(Test())
