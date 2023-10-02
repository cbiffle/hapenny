from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *

from rv32 import StreamSig, AlwaysReady

class RegFile(Component):
    read_cmd: In(StreamSig(5))
    read_resp: Out(32)

    write_cmd: In(StreamSig(Signature({
        'reg': Out(5),
        'value': Out(32),
    })))

    def elaborate(self, platform):
        m = Module()

        m.submodules.mem = mem = Memory(
            width = 32,
            depth = 32,
            name = "regfile",
        )

        rp = mem.read_port(transparent = False)
        wp = mem.write_port()

        m.d.comb += [
            rp.addr.eq(self.read_cmd.payload),
            rp.en.eq(self.read_cmd.valid),

            self.read_resp.eq(rp.data),

            wp.addr.eq(self.write_cmd.payload.reg),
            wp.data.eq(self.write_cmd.payload.value),
            # Block writes to x0.
            wp.en.eq((self.write_cmd.payload.reg != 0) & self.write_cmd.valid),
        ]

        return m
