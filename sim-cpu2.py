from amaranth import *
from amaranth.sim import Simulator, Delay, Settle
from amaranth.back import verilog
from amaranth.lib.wiring import *

from rv32.cpu import Cpu, MemCmd
from rv32 import *

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

        rp = self.m.read_port()
        wp = self.m.write_port(granularity = 8)

        m.d.comb += [
            rp.addr.eq(self.command.payload.addr[1:]),
            rp.en.eq(self.command.valid & (self.command.payload.lanes == 0b0000)),

            wp.addr.eq(self.command.payload.addr[1:]),
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


if __name__ == "__main__":
    m = Module()
    m.submodules.uut = uut = Cpu()
    m.submodules.mem = mem = TestMemory([
        0b0000000000000000000_00000_1101111, # JAL x0, .
    ])

    connect(m, uut.mem_out, mem.command)
    connect(m, uut.mem_in, mem.response)

    ports = [
        uut.ustate,
        uut.mem_out.valid,
        uut.mem_out.payload.addr,
        uut.mem_out.payload.data,
        uut.mem_out.payload.lanes,
        uut.mem_in.valid,
        uut.mem_in.payload,
    ]

    sim = Simulator(m)
    sim.add_clock(1e-6)

    def process():
        for i in range(10):
            yield

    sim.add_sync_process(process)

    with sim.write_vcd(vcd_file="test.vcd", gtkw_file="test.gtkw", traces=ports):
        sim.run()
