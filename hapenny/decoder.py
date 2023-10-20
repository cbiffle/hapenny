# Combinational decode logic.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.data import *
import amaranth.lib.coding

class Opcode(Enum):
    LUI = 0b01101
    AUIPC = 0b00101
    JAL = 0b11011
    JALR = 0b11001
    Bxx = 0b11000
    Lxx = 0b00000
    Sxx = 0b01000
    ALUIMM = 0b00100
    ALUREG = 0b01100
    SYSTEM = 0b11100
    CUSTOM0 = 0b00001

class DecodeSignals(Struct):
    inst: unsigned(32)

    opcode: unsigned(5)
    funct3: unsigned(3)
    rs1: unsigned(5)
    rs2: unsigned(5)
    rd: unsigned(5)

    is_auipc: unsigned(1)
    is_lui: unsigned(1)
    is_jal: unsigned(1)
    is_jalr: unsigned(1)
    is_b: unsigned(1)
    is_load: unsigned(1)
    is_store: unsigned(1)
    is_alu: unsigned(1)
    is_alu_rr: unsigned(1)
    is_alu_ri: unsigned(1)
    is_system: unsigned(1)
    is_custom0: unsigned(1)

    # derived signals to make it easier to move the functions before a register.
    is_auipc_or_lui: unsigned(1)
    is_auipc_or_jal: unsigned(1)
    is_auipc_or_lui_or_jal: unsigned(1)
    is_jal_or_jalr: unsigned(1)
    is_load_or_jalr: unsigned(1)
    is_csr: unsigned(1)
    writes_rd_normally: unsigned(1)
    is_imm_i: unsigned(1)
    is_neg_imm_i: unsigned(1)
    is_any_imm_i: unsigned(1)
    is_neg_reg_to_adder: unsigned(1)
    is_reg_to_adder: unsigned(1)
    is_any_reg_to_adder: unsigned(1)
    is_shift: unsigned(1)
    is_slt: unsigned(1)
    is_sw: unsigned(1)
    is_adder_rhs_complemented: unsigned(1)
    writes_adder_to_reg: unsigned(1)

    # one-hot decode of funct3
    funct3_is: unsigned(8)

class Decoder(Component):
    """The Decoder is a circuit that breaks an instruction into the various
    control signals. It's used by the larger components.

    Attributes
    ----------
    inst (input): instruction word.
    out (output): group of decode signals, see DecodeSignals struct.
    """
    inst: In(32)

    out: Out(DecodeSignals)

    def __init__(self, 
                 ):
        super().__init__()

    def elaborate(self, platform):
        m = Module()

        m.submodules.funct3_decode = f3d = amaranth.lib.coding.Decoder(8)

        m.d.comb += f3d.i.eq(self.inst[12:15])

        opcode = Signal(5)
        m.d.comb += opcode.eq(self.inst[2:7])

        m.d.comb += [
            self.out.inst.eq(self.inst),
            self.out.opcode.eq(opcode),
            self.out.funct3.eq(self.inst[12:15]),
            self.out.rs1.eq(self.inst[15:20]),
            self.out.rs2.eq(self.inst[20:25]),
            self.out.rd.eq(self.inst[7:12]),
            self.out.is_auipc.eq(opcode == Opcode.AUIPC),
            self.out.is_lui.eq(opcode == Opcode.LUI),
            self.out.is_jal.eq(opcode == Opcode.JAL),
            self.out.is_jalr.eq(opcode == Opcode.JALR),
            self.out.is_b.eq(opcode == Opcode.Bxx),
            self.out.is_load.eq(opcode == Opcode.Lxx),
            self.out.is_store.eq(opcode == Opcode.Sxx),
            self.out.is_alu_rr.eq(opcode == Opcode.ALUREG),
            self.out.is_alu_ri.eq(opcode == Opcode.ALUIMM),
            self.out.is_system.eq(opcode == Opcode.SYSTEM),
            self.out.is_custom0.eq(opcode == Opcode.CUSTOM0),
            self.out.funct3_is.eq(f3d.o),
        ]

        # derived signals
        m.d.comb += [
            self.out.is_alu.eq(
                self.out.is_alu_rr | self.out.is_alu_ri
            ),
            self.out.is_auipc_or_lui.eq(
                self.out.is_auipc | self.out.is_lui
            ),
            self.out.is_auipc_or_jal.eq(
                self.out.is_auipc | self.out.is_jal
            ),
            self.out.is_auipc_or_lui_or_jal.eq(
                self.out.is_auipc | self.out.is_lui | self.out.is_jal
            ),
            self.out.is_jal_or_jalr.eq(
                self.out.is_jal | self.out.is_jalr
            ),
            self.out.is_load_or_jalr.eq(
                self.out.is_load | self.out.is_jalr
            ),
            self.out.is_csr.eq((opcode == Opcode.SYSTEM) & ~self.out.funct3_is[0b000]),

            self.out.writes_rd_normally.eq(
                self.out.is_jal
                | self.out.is_jalr
                | self.out.is_lui
                | self.out.is_auipc
                | self.out.is_alu
            ),
            self.out.is_imm_i.eq(
                (opcode == Opcode.Lxx) | (opcode == Opcode.JALR)
                | ((opcode == Opcode.ALUIMM) & ~(self.out.funct3_is[0b010] |
                                                 self.out.funct3_is[0b011]))
                | (opcode == Opcode.CUSTOM0)
            ),
            self.out.is_neg_imm_i.eq(
                (opcode == Opcode.ALUIMM) & (self.out.funct3_is[0b010] |
                                             self.out.funct3_is[0b011])
            ),
            self.out.is_any_imm_i.eq(
                (opcode == Opcode.Lxx) | (opcode == Opcode.JALR)
                | (opcode == Opcode.ALUIMM)
                | (opcode == Opcode.CUSTOM0)
            ),
            self.out.is_neg_reg_to_adder.eq(
                (opcode == Opcode.Bxx)
                | ((opcode == Opcode.ALUREG) & (self.out.funct3_is[0b010] |
                                                self.out.funct3_is[0b011]))
            ),
            self.out.is_reg_to_adder.eq(
                ((opcode == Opcode.ALUREG) & ~(self.out.funct3_is[0b010] |
                                               self.out.funct3_is[0b011]))
            ),
            self.out.is_any_reg_to_adder.eq(
                (opcode == Opcode.Bxx)
                | (opcode == Opcode.ALUREG)
            ),
            self.out.is_shift.eq(
                self.out.is_alu & (self.out.funct3_is[0b001] |
                                   self.out.funct3_is[0b101])
            ),
            self.out.is_slt.eq(
                self.out.is_alu & (self.out.funct3_is[0b010] |
                                   self.out.funct3_is[0b011])
            ),
            self.out.is_sw.eq(
                self.out.is_store & self.out.funct3_is[0b010]
            ),
            self.out.is_adder_rhs_complemented.eq(
                self.out.is_neg_reg_to_adder
                | self.out.is_neg_imm_i
                | (self.out.is_reg_to_adder & self.out.inst[30])
            ),
            self.out.writes_adder_to_reg.eq(
                self.out.is_auipc_or_lui | (self.out.is_alu &
                                            self.out.funct3_is[0b000])
            ),
        ]

        return m

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

