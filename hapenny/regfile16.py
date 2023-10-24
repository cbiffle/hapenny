# 16-bit x 64 register file for narrow datapath RV32 implementation.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *

from hapenny import StreamSig, AlwaysReady

def RegWrite(addrbits = 5):
    return Signature({
        'reg': Out(addrbits),
        'value': Out(16),
    })

class RegFile16(Component):
    read_resp: Out(16)

    def __init__(self, *, 
                 banks = 1):
        super().__init__()

        self.banks = banks

        # 5 bits for x0..x31, 1 bit for top vs bottom half, then bank bits
        select_bits = 5 + 1 + (banks - 1).bit_length()

        self.read_cmd = AlwaysReady(select_bits).flip().create()
        self.write_cmd = AlwaysReady(RegWrite(select_bits)).flip().create()

    def elaborate(self, platform):
        m = Module()

        nregs = 32 * self.banks

        m.submodules.mem = mem = Memory(
            width = 16,
            depth = 2 * nregs,
            name = "regfile",
            attrs = {
                'ram_style': 'block',
            },
        )

        rp = mem.read_port(transparent = False)
        wp = mem.write_port()

        m.d.comb += [
            rp.addr.eq(self.read_cmd.payload),
            rp.en.eq(self.read_cmd.valid),

            self.read_resp.eq(rp.data),

            wp.addr.eq(self.write_cmd.payload.reg),
            wp.data.eq(self.write_cmd.payload.value),
            # Block writes to both halves of x0 in all banks.
            wp.en.eq((self.write_cmd.payload.reg[:5] != 0) & self.write_cmd.valid),
        ]

        return m
