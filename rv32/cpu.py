from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *

from rv32 import StreamSig, AlwaysReady
from rv32.regfile import RegFile

MemCmd = Signature({
    'addr': Out(32 - 2), # word-addressed
    'lanes': Out(4),
    'data': Out(32),
})

DebugPort = Signature({
    'reg_read': Out(StreamSig(5)),
    'reg_value': In(AlwaysReady(32)),
    'reg_write': Out(StreamSig(Signature({
        'addr': Out(5),
        'value': Out(32),
    }))),
    'pc': Out(32),
    'pc_write': Out(StreamSig(32)),
})


class UState(Enum):
    FETCH      = 0b000
    INST_RS2   = 0b001
    DECODE_RS1 = 0b010
    OPERATE    = 0b011
    LOAD       = 0b100
    SHIFT      = 0b101
    DEAD       = 0b110
    HALTED     = 0b111

class Cpu(Component):
    mem_out: Out(StreamSig(MemCmd))
    mem_in: In(StreamSig(32))

    halt_request: In(1)
    halted: Out(1)

    debug: In(DebugPort)

    def __init__(self, *,
                 check_alignment = True,
                 wait = True,
                 pc_width = 32,
                 allow_halt_request = True):
        super().__init__()

        self.check_alignment = check_alignment
        self.wait = wait
        self.allow_halt_request = allow_halt_request

        self.ustate = Signal(UState)
        self.pc = Signal(pc_width)
        self.inst = Signal(32)
        self.rs2 = Signal(32)
        self.comp_rhs = Signal(32)
        self.load_lsbs = Signal(2)
        self.shift_fill = Signal(1)
        self.shift_amt = Signal(5)

    def elaborate(self, platform):
        m = Module()

        # Fields of the instruction
        inst_rd = Signal(5)
        inst_rs1 = Signal(5)
        inst_funct3 = Signal(3)
        inst_funct7 = Signal(7)
        opcode = Signal(7)
        m.d.comb += [
            opcode.eq(self.inst[0:7]),
            inst_rd.eq(self.inst[7:12]),
            inst_rs1.eq(self.inst[15:20]),
            inst_funct3.eq(self.inst[12:15]),
            inst_funct7.eq(self.inst[25:]),
        ]

        # Immediate encodings
        imm_i = Signal(32)
        imm_s = Signal(32)
        imm_b = Signal(32)
        imm_u = Signal(32)
        imm_j = Signal(32)
        m.d.comb += [
            imm_i.eq(Cat(self.inst[20:31], self.inst[31].replicate(21))),
            imm_s.eq(Cat(self.inst[7:12], self.inst[25:31],
                         self.inst[31].replicate(21))),
            imm_b.eq(Cat(0, self.inst[8:12], self.inst[25:31], self.inst[7],
                     self.inst[31].replicate(20))),
            imm_u.eq(self.inst & 0xFFFFF000),
            imm_j.eq(Cat(0, self.inst[21:31], self.inst[20],
                         self.inst[12:20], self.inst[31].replicate(12))),
        ]

        # Register file and basic wiring
        m.submodules.regfile = rf = RegFile()
        # Permanently drive the regfile write port using the rd field of the
        # instruction. We'll only override this in one case (debug probe).
        m.d.comb += rf.write_cmd.payload.reg.eq(inst_rd)

        # Debug and status port wiring
        m.d.comb += [
            self.halted.eq(self.ustate == UState.HALTED),
            self.debug.pc.eq(self.pc),
        ]

        # Internal controls
        do_fetch = Signal(1)
        next_pc = Signal(32)
        pc_plus_4 = Signal(32)
        m.d.comb += [
            pc_plus_4.eq(self.pc + 4),
            next_pc.eq(pc_plus_4), # may change below
        ]

        m.d.comb += self.mem_out.payload.addr.eq(next_pc[2:])

        with m.If(do_fetch):
            m.d.comb += [
                self.mem_out.payload.addr.eq(next_pc[2:]),
                self.mem_out.valid.eq(1),
            ]
            m.d.sync += self.pc.eq(next_pc)

            with m.If(self.halt_request if self.allow_halt_request else 0):
                m.d.sync += self.ustate.eq(UState.HALTED)
            with m.Else():
                with m.If(self.mem_out.ready if self.wait else 1):
                    m.d.sync += self.ustate.eq(UState.INST_RS2)
                with m.Else():
                    m.d.sync += self.ustate.eq(UState.FETCH)

        # The Big Switch State
        with m.Switch(self.ustate):
            with m.Case(UState.FETCH):
                m.d.comb += [
                    self.mem_out.payload.addr.eq(self.pc[2:]),
                ]
                with m.If(self.halt_request if self.allow_halt_request else 0):
                    m.d.sync += self.ustate.eq(UState.HALTED)
                with m.Else():
                    # Set up the bus to fetch the instruction.
                    m.d.comb += self.mem_out.valid.eq(1)
                    # But don't act as though it's actually happening unless the bus
                    # is ready for us.
                    with m.If(self.mem_out.ready if self.wait else 1):
                        m.d.sync += self.ustate.eq(UState.INST_RS2)

            with m.Case(UState.INST_RS2):
                m.d.comb += [
                    # Indicate to the bus that we're ready for the instruction.
                    self.mem_in.ready.eq(1),
                    # Also, go ahead and route the rs2 subfield of any incoming
                    # instruction to the register file's read port.
                    rf.read_cmd.payload.eq(self.mem_in.payload[20:25]),
                    rf.read_cmd.valid.eq(1),
                ]

                # We may not receive the instruction right away. Wait for it.
                with m.If(self.mem_in.valid if self.wait else 1):
                    m.d.sync += [
                        # Latch the instruction.
                        self.inst.eq(self.mem_in.payload),
                        # Start running it.
                        self.ustate.eq(UState.DECODE_RS1),
                    ]

            with m.Case(UState.DECODE_RS1):
                # Start a'loadin rs1.
                m.d.comb += [
                    rf.read_cmd.payload.eq(inst_rs1),
                    rf.read_cmd.valid.eq(1),
                ]

                # Do a bit of early decode. This is hand-optimized because I was
                # having a hard time getting the tools to infer it.
                #  opcode    mnemonic    comp_rhs value
                #  1100011   Bxx         rs2
                #  0110011   ALU r-r     rs2
                #  0010011   ALU r-i     I-format immediate
                #      all others        don't care!
                #
                # So, squinting at the table above, you can see that opcode[5]
                # reliably discriminates these cases.
                with m.If(opcode[5]):
                    # Take rs2, which is coming out of the register file still.
                    m.d.sync += self.comp_rhs.eq(rf.read_resp)
                with m.Else():
                    # Take the immediate.
                    m.d.sync += self.comp_rhs.eq(imm_i)

                m.d.sync += [
                    # rs2 is now available in the register file's output
                    # register. Latch it for use in subsequent states.
                    self.rs2.eq(rf.read_resp),
                    # Transition
                    self.ustate.eq(UState.OPERATE),
                ]

            with m.Case(UState.OPERATE):
                rs1 = Signal(32)
                m.d.comb += rs1.eq(rf.read_resp)

                dying = Signal(1)
                stalling = Signal(1)
                loading = Signal(1)
                storing = Signal(1)
                shifting = Signal(1)

                # Force structural sharing between the subtractor and
                # comparator(s).
                difference = Signal(33)
                signed_less_than = Signal(1)
                unsigned_less_than = Signal(1)
                m.d.comb += [
                    difference.eq(rs1 + Cat(~self.comp_rhs, 1) + 1),
                    unsigned_less_than.eq(difference[32]),
                ]
                with m.If(rs1[31] ^ self.comp_rhs[31]):
                    m.d.comb += signed_less_than.eq(rs1[31])
                with m.Else():
                    m.d.comb += signed_less_than.eq(difference[32])

                with m.Switch(opcode):
                    with m.Case(0b01101_11):  # LUI
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(imm_u),
                            rf.write_cmd.valid.eq(1),
                        ]
                    with m.Case(0b00101_11):  # AUIPC
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(self.pc + imm_u),
                            rf.write_cmd.valid.eq(1),
                        ]
                    with m.Case(0b11011_11):  # JAL
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(pc_plus_4),
                            rf.write_cmd.valid.eq(1),
                            next_pc.eq(self.pc + imm_j),
                        ]
                    with m.Case(0b11001_11):  # JALR
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(pc_plus_4),
                            rf.write_cmd.valid.eq(1),
                            next_pc.eq((rs1 + imm_i) & 0xFFFF_FFFE),
                        ]
                    with m.Case(0b11000_11):  # Bxx
                        taken = Signal(1)
                        with m.Switch(inst_funct3):
                            with m.Case("00-"): # EQ/NE
                                m.d.comb += taken.eq(rs1 == self.rs2)
                            with m.Case("10-"): # LT/GE
                                m.d.comb += taken.eq(signed_less_than)
                            with m.Case("11-"): # LTU/GEU
                                m.d.comb += taken.eq(unsigned_less_than)
                            with m.Default(): # undefined comparisons
                                # will function as a nop, i.e. branch never
                                # taken
                                pass

                        with m.If(taken ^ inst_funct3[0]):
                            m.d.comb += next_pc.eq(self.pc + imm_b)

                    with m.Case(0b00000_11):  # Lx
                        ea = Signal(32)
                        aligned = Signal(1)
                        m.d.comb += [
                            ea.eq(rs1 + imm_i),
                            self.mem_out.payload.addr.eq(ea[2:]),
                            self.mem_out.valid.eq(aligned),
                        ]
                        if self.check_alignment:
                            with m.Switch(inst_funct3):
                                with m.Case(0b010): # LW
                                    m.d.comb += aligned.eq(ea[:2] == 0)
                                with m.Case("-00"): # LB/LBU
                                    m.d.comb += aligned.eq(1)
                                with m.Case("-01"): # LW/LWU
                                    m.d.comb += aligned.eq(ea[0] == 0)
                                with m.Default(): # 64-bit, etc
                                    # this will appear unaligned.
                                    pass
                        else:
                            m.d.comb += aligned.eq(1)

                        with m.If(~aligned):
                            m.d.comb += dying.eq(1)

                        with m.If(self.mem_out.ready if self.wait else 1):
                            m.d.comb += loading.eq(1)
                        with m.Else():
                            m.d.comb += stalling.eq(1)

                        m.d.sync += [
                            self.load_lsbs.eq(ea[:2]),
                        ]

                    with m.Case(0b01000_11):  # Sx
                        ea = Signal(32)
                        m.d.comb += ea.eq(rs1 + imm_s)

                        aligned = Signal(1)
                        val = Signal(32)
                        mask = Signal(4)
                        with m.Switch(inst_funct3):
                            with m.Case(0b010): # SW
                                m.d.comb += [
                                    aligned.eq(ea[:2] == 0),
                                    val.eq(self.rs2),
                                    mask.eq(Const(0b1111, 4) << ea[:2]),
                                ]
                            with m.Case("-00"): # SB/LBU
                                m.d.comb += [
                                    aligned.eq(1),
                                    val.eq(self.rs2[:8].replicate(4)),
                                    mask.eq(Const(1, 4) << ea[:2]),
                                ]
                            with m.Case("-01"): # SW/LWU
                                m.d.comb += [
                                    aligned.eq(ea[0] == 0),
                                    val.eq(self.rs2[:16].replicate(2)),
                                    mask.eq(Const(0b11, 4) << ea[:2]),
                                ]
                            with m.Default(): # 64 bit, etc
                                # when alignment checks are on, this will appear
                                # unaligned and fault.
                                pass

                        if not self.check_alignment:
                            m.d.comb += aligned.eq(1)

                        m.d.comb += [
                            self.mem_out.payload.data.eq(val),
                            self.mem_out.payload.lanes.eq(mask),
                            self.mem_out.payload.addr.eq(ea[2:]),
                            self.mem_out.valid.eq(aligned),
                        ]
                        with m.If(~aligned):
                            m.d.comb += dying.eq(1)

                        with m.If(self.mem_out.ready if self.wait else 1):
                            m.d.comb += [
                                storing.eq(1),
                            ]
                        with m.Else():
                            m.d.comb += stalling.eq(1)

                    with m.Case("0-10011"):  # ALU reg or immediate
                        is_reg = Signal(1)
                        m.d.comb += is_reg.eq(opcode[5])

                        # Not every case below puts a defined value on this
                        # signal; so far that hasn't affected synthesis results.
                        alu_result = Signal(32)

                        with m.Switch(inst_funct3):
                            with m.Case(0b000): # ADDI/ADD/SUB
                                with m.If(opcode[5] & inst_funct7[5]):
                                    m.d.comb += alu_result.eq(difference[:32])
                                with m.Else():
                                    m.d.comb += alu_result.eq(rs1 +
                                                              self.comp_rhs)

                            with m.Case(0b001): # SLLI/SLL
                                m.d.comb += shifting.eq(1)
                                m.d.sync += [
                                    self.shift_fill.eq(0),
                                    self.shift_amt.eq(self.comp_rhs[:5]),
                                    self.rs2.eq(rs1),
                                ]
                            with m.Case(0b010): # SLTI/SLT
                                m.d.comb += alu_result.eq(signed_less_than)
                            with m.Case(0b011): # SLTIU/SLTU
                                m.d.comb += alu_result.eq(unsigned_less_than)
                            with m.Case(0b100): # XORI/XOR
                                m.d.comb += alu_result.eq(rs1 ^ self.comp_rhs)
                            with m.Case(0b101): # SRLI/SRL/SRAI/SRA
                                m.d.comb += shifting.eq(1)
                                m.d.sync += [
                                    self.shift_fill.eq(rs1[31] &
                                                       inst_funct7[5]),
                                    self.shift_amt.eq(self.comp_rhs[:5]),
                                    self.rs2.eq(rs1),
                                ]
                            with m.Case(0b110): # ORI/OR
                                m.d.comb += alu_result.eq(rs1 | self.comp_rhs)
                            with m.Case(0b111): # ANDI/AND
                                m.d.comb += alu_result.eq(rs1 & self.comp_rhs)

                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(alu_result),
                            rf.write_cmd.valid.eq(1),
                        ]
                    with m.Default():
                        # Catch undefined instructions
                        m.d.comb += dying.eq(1)

                with m.If(dying):
                    m.d.sync += self.ustate.eq(UState.DEAD)
                with m.Elif(stalling):
                    pass
                with m.Elif(shifting):
                    m.d.sync += [
                        self.ustate.eq(UState.SHIFT),
                    ]
                with m.Elif(loading):
                    m.d.sync += [
                        self.ustate.eq(UState.LOAD),
                    ]
                with m.Elif(storing):
                    m.d.sync += [
                        self.ustate.eq(UState.FETCH),
                        self.pc.eq(next_pc),
                    ]
                with m.Else():
                    m.d.comb += do_fetch.eq(1)

                # end of OPERATE case

            with m.Case(UState.SHIFT):
                m.d.comb += rf.write_cmd.payload.value.eq(self.rs2)

                with m.If(self.shift_amt == 0): # done shifting?
                    m.d.comb += [
                        rf.write_cmd.valid.eq(1),
                        do_fetch.eq(1),
                    ]
                with m.Elif(inst_funct3[2]): # shifting right?
                    m.d.sync += self.rs2.eq(Cat(self.rs2[1:], self.shift_fill))
                with m.Else():
                    m.d.sync += self.rs2.eq(Cat(self.shift_fill, self.rs2[:-1]))

                # This will leave the shift amount at 32 when we're not
                # shifting, but have shifted at some point. That's fine -- we'll
                # overwrite it before the next shift -- and ensures that the
                # downcounter doesn't have to depend on the zero-comparator.
                m.d.sync += [
                    self.shift_amt.eq(self.shift_amt - 1),
                ]

            with m.Case(UState.LOAD):
                size = Signal(2)
                zext = Signal(1)
                shifted = Signal(32)
                m.d.comb += [
                    self.mem_in.ready.eq(1),

                    Cat(size, zext).eq(inst_funct3),
                    shifted.eq(self.mem_in.payload >> Cat(0, 0, 0,
                                                          self.load_lsbs)),
                ]
                result = Signal(32)
                with m.Switch(size):
                    with m.Case(0b00): # byte
                        m.d.comb += result.eq(Cat(
                            shifted[:8],
                            (~zext & shifted[7]).replicate(24),
                        ))
                    with m.Case(0b01): # half
                        m.d.comb += result.eq(Cat(
                            shifted[:16],
                            (~zext & shifted[15]).replicate(16),
                        ))
                    with m.Case(0b10): # word
                        m.d.comb += result.eq(shifted)

                m.d.comb += [
                    rf.write_cmd.payload.value.eq(result),
                    self.mem_out.payload.addr.eq(next_pc[2:]),
                ]

                with m.If(self.mem_in.valid if self.wait else 1):
                    m.d.comb += [
                        rf.write_cmd.valid.eq(1),
                        do_fetch.eq(1),
                    ]

            if self.allow_halt_request:
                with m.Case(UState.HALTED):
                    # Indicate debug port availability.
                    m.d.comb += [
                        self.debug.reg_read.ready.eq(1),
                        self.debug.reg_write.ready.eq(1),
                        self.debug.pc_write.ready.eq(1),
                    ]

                    # Register read wiring
                    delay = Signal(1)
                    m.d.comb += [
                        rf.read_cmd.payload.eq(self.debug.reg_read.payload),
                        rf.read_cmd.valid.eq(self.debug.reg_read.valid),
                        self.debug.reg_value.valid.eq(delay),
                        self.debug.reg_value.payload.eq(rf.read_resp),
                    ]
                    m.d.sync += delay.eq(0)

                    with m.If(self.debug.reg_read.valid):
                        m.d.sync += [
                            delay.eq(1),
                        ]

                    # Register write wiring
                    m.d.comb += [
                        rf.write_cmd.payload.reg.eq(self.debug.reg_write.payload.addr),
                        rf.write_cmd.payload.value.eq(self.debug.reg_write.payload.value),
                        rf.write_cmd.valid.eq(self.debug.reg_write.valid),
                    ]

                    # PC update wiring
                    with m.If(self.debug.pc_write.valid):
                        m.d.sync += [
                            self.pc.eq(self.debug.pc_write.payload),
                        ]

                    with m.If(self.halt_request == 0):
                        m.d.sync += self.ustate.eq(UState.FETCH)
            # endof allow_halt_request

        return m
