# 32-bit x 32 register file for a full-width RV32 implementation.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *

from hapenny import StreamSig, AlwaysReady

def RegWrite(addrbits = 5):
    return Signature({
        'reg': Out(addrbits),
        'value': Out(32),
    })

class RegFile(Component):
    read_resp: Out(32)

    def __init__(self, *, 
                 banks = 1):
        super().__init__()

        self.banks = banks

        # 5 bits for x0..x31, then bank bits
        select_bits = 5 + (banks - 1).bit_length()

        self.read_cmd = AlwaysReady(select_bits).flip().create()
        self.write_cmd = AlwaysReady(RegWrite(select_bits)).flip().create()

    def elaborate(self, platform):
        m = Module()

        nregs = 32 * self.banks
        contents = [0xDEAD_0000 | n | (b << 8) for n in range(32) for b in range(self.banks)]
        contents[0] = 0

        m.submodules.mem = mem = Memory(
            width = 32,
            depth = nregs,
            name = "regfile",
            #init = contents,
        )

        # The 32-bit core can read a register at the same time that it's
        # writing it, so we have to make this transparent to bypass.
        rp = mem.read_port(transparent = True)
        wp = mem.write_port()

        m.d.comb += [
            rp.addr.eq(self.read_cmd.payload),
            rp.en.eq(self.read_cmd.valid),

            self.read_resp.eq(rp.data),

            wp.addr.eq(self.write_cmd.payload.reg),
            wp.data.eq(self.write_cmd.payload.value),
            # Block writes to both halves of x0 in all banks.
            wp.en.eq((self.write_cmd.payload.reg != 0) & self.write_cmd.valid),
        ]

        return m
