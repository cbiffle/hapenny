# Combinational decode logic.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *

class ImmediateDecoder(Component):
    """The ImmediateDecoder decodes an instruction word into its various
    immediate formats. It's used by the larger components.

    Attributes
    ----------
    inst (input): instruction word.
    imm_i (output): I-format immediate.
    imm_s (output): S-format immediate.
    imm_b (output): B-format immediate.
    imm_u (output): U-format immediate.
    imm_j (output): J-format immediate.
    """
    inst: In(32)

    i: Out(32)
    s: Out(32)
    b: Out(32)
    u: Out(32)
    j: Out(32)

    def elaborate(self, platform):
        m = Module()

        m.d.comb += [
            self.i.eq(Cat(self.inst[20:31], self.inst[31].replicate(21))),
            self.s.eq(Cat(self.inst[7:12], self.inst[25:31],
                     self.inst[31].replicate(21))),
            self.b.eq(Cat(0, self.inst[8:12], self.inst[25:31], self.inst[7],
                 self.inst[31].replicate(20))),
            self.u.eq(self.inst & 0xFFFFF000),
            self.j.eq(Cat(0, self.inst[21:31], self.inst[20],
                         self.inst[12:20], self.inst[31].replicate(12))),
        ]

        return m

