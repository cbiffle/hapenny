# 16-bit x 64 register file for narrow datapath RV32 implementation.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *

from hapenny import StreamSig, AlwaysReady

RegWrite = Signature({
    'reg': Out(7),
    'value': Out(16),
})

class RegFile16(Component):
    read_cmd: In(AlwaysReady(7))
    read_resp: Out(16)

    write_cmd: In(AlwaysReady(RegWrite))

    def __init__(self, *, rv32e = False):
        super().__init__()

        self.rv32e = rv32e

    def elaborate(self, platform):
        m = Module()

        contexts = 2
        nregs = contexts * (16 if self.rv32e else 32)

        m.submodules.mem = mem = Memory(
            width = 16,
            depth = 2 * nregs,
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
            # Block writes to both halves of x0.
            wp.en.eq((self.write_cmd.payload.reg[:5] != 0) & self.write_cmd.valid),
        ]

        return m
