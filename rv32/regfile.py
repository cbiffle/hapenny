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

    def __init__(self, *, rv32e = False):
        super().__init__()

        self.rv32e = rv32e

    def elaborate(self, platform):
        m = Module()

        nregs = 16 if self.rv32e else 32

        m.submodules.mem_hi = mem_hi = Memory(
            width = 16,
            depth = nregs,
            name = "regfile_hi",
        )

        rp_hi = mem_hi.read_port(transparent = False)
        wp_hi = mem_hi.write_port()

        m.submodules.mem_lo = mem_lo = Memory(
            width = 16,
            depth = nregs,
            name = "regfile_lo",
        )

        rp_lo = mem_lo.read_port(transparent = False)
        wp_lo = mem_lo.write_port()

        m.d.comb += [
            rp_lo.addr.eq(self.read_cmd.payload),
            rp_hi.addr.eq(self.read_cmd.payload),
            rp_lo.en.eq(self.read_cmd.valid),
            rp_hi.en.eq(self.read_cmd.valid),

            self.read_resp.eq(Cat(rp_lo.data, rp_hi.data)),

            wp_lo.addr.eq(self.write_cmd.payload.reg),
            wp_hi.addr.eq(self.write_cmd.payload.reg),
            wp_lo.data.eq(self.write_cmd.payload.value[:16]),
            wp_hi.data.eq(self.write_cmd.payload.value[16:]),
            # Block writes to x0.
            wp_lo.en.eq((self.write_cmd.payload.reg != 0) & self.write_cmd.valid),
            wp_hi.en.eq((self.write_cmd.payload.reg != 0) & self.write_cmd.valid),
        ]

        return m
