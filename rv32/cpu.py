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
    FETCH = 0
    INST = 1
    BEGIN = 2
    HALTED = 3
    LOAD = 4
    STORE = 5
    BRANCH = 6
    ALU_RR = 7
    SHIFTING = 8
    SHIFTING_IMM = 9

class Cpu(Component):
    mem_out: Out(StreamSig(MemCmd))
    mem_in: In(StreamSig(32))

    halt_request: In(1)
    halted: Out(1)

    debug: In(DebugPort)

    def __init__(self):
        super().__init__()

        self.ustate = Signal(UState)
        self.pc = Signal(32)
        self.inst = Signal(32)
        self.rs1 = Signal(32)
        self.shift_start = Signal(1)
        self.shift_amt = Signal(5)

    def elaborate(self, platform):
        m = Module()

        m.submodules.regfile = rf = RegFile()

        m.d.comb += [
            rf.write_cmd.payload.reg.eq(self.inst[7:12]),
        ]

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
            imm_j.eq(Cat(0, self.inst[21:25], self.inst[25:31], self.inst[20],
                         self.inst[12:20], self.inst[31].replicate(12))),
        ]

        opcode = Signal(7)
        m.d.comb += opcode.eq(self.inst[0:7])

        do_fetch = Signal(1)
        fetch_addr = Signal(32)
        m.d.comb += fetch_addr.eq(self.pc + 4) # overridden where needed

        m.d.comb += self.halted.eq(self.ustate == UState.HALTED)

        m.d.comb += [
            self.debug.pc.eq(self.pc),
        ]
        slti_compare_result = Signal(33)
        signed_less_than_i = Signal(1)
        unsigned_less_than_i = Signal(1)
        m.d.comb += [
            slti_compare_result.eq(
                rf.read_resp + Cat(~imm_i, 1) + 1
            ),
            unsigned_less_than_i.eq(slti_compare_result[32]),
        ]
        with m.If((rf.read_resp[31] ^ imm_i[31]) != 0):
            m.d.comb += signed_less_than_i.eq(rf.read_resp[31])
        with m.Else():
            m.d.comb += signed_less_than_i.eq(unsigned_less_than_i)

        last_lsbs = Signal(2)

        with m.Switch(self.ustate):
            with m.Case(UState.FETCH):
                m.d.comb += [
                    do_fetch.eq(1),
                    # Override default fetch address to avoid the +4
                    fetch_addr.eq(self.pc),
                ]

            with m.Case(UState.INST):
                m.d.comb += [
                    # Indicate to the bus that we're ready for the instruction.
                    self.mem_in.ready.eq(1),
                    # Also, go ahead and route the rs1 subfield of any incoming
                    # instruction to the register file's read port.
                    rf.read_cmd.payload.eq(self.mem_in.payload[15:20]),
                    rf.read_cmd.valid.eq(1),
                ]

                # We may not receive the instruction right away. Wait for it.
                with m.If(self.mem_in.valid):
                    m.d.sync += [
                        # Latch the instruction.
                        self.inst.eq(self.mem_in.payload),
                        # Start running it.
                        self.ustate.eq(UState.BEGIN),
                    ]

            with m.Case(UState.BEGIN):
                # We've already loaded rs1 just in case.
                m.d.sync += [
                    self.rs1.eq(rf.read_resp),
                ]
                # Start a'loadin rs2 -- again, just in case.
                m.d.comb += [
                    rf.read_cmd.payload.eq(self.inst[20:25]),
                    rf.read_cmd.valid.eq(1),
                ]

                with m.Switch(opcode):
                    with m.Case(0b11011_11): # JAL
                        # ha, we didn't actually need rs1. Oh well!
                        jump_adder = Signal(32)
                        m.d.comb += [
                            jump_adder.eq(self.pc + imm_j),
                            fetch_addr.eq(jump_adder),
                            do_fetch.eq(1),

                            # rd write set by default
                            rf.write_cmd.payload.value.eq(self.pc + 4),
                            rf.write_cmd.valid.eq(1),
                        ]
                    with m.Case(0b11001_11): # JALR
                        jump_adder = Signal(32)
                        m.d.comb += [
                            # rd write set by default
                            rf.write_cmd.payload.value.eq(self.pc + 4),
                            rf.write_cmd.valid.eq(1),

                            jump_adder.eq(Cat(0, (rf.read_resp + imm_i)[1:])),
                            fetch_addr.eq(jump_adder),
                            do_fetch.eq(1),
                        ]

                    with m.Case(0b01101_11): # LUI
                        # ha, we didn't actually need rs1. Oh well!
                        m.d.comb += [
                            # rd write set by default
                            rf.write_cmd.payload.value.eq(imm_u),
                            rf.write_cmd.valid.eq(1),
                            do_fetch.eq(1),
                        ]

                    with m.Case(0b00101_11): # AUIPC
                        # ha, we didn't actually need rs1. Oh well!
                        auipc_adder = Signal(32)
                        m.d.comb += [
                            auipc_adder.eq(self.pc + imm_u),
                            # rd write set by default
                            rf.write_cmd.payload.value.eq(auipc_adder),
                            rf.write_cmd.valid.eq(1),
                            do_fetch.eq(1),
                        ]

                    with m.Case(0b00000_11): # Lx
                        ea_adder = Signal(32)
                        m.d.comb += [
                            ea_adder.eq(rf.read_resp + imm_i),
                            self.mem_out.payload.addr.eq(ea_adder[2:]),
                            self.mem_out.valid.eq(1),
                        ]
                        m.d.sync += last_lsbs.eq(ea_adder[:2])

                        with m.If(self.mem_out.ready == 1):
                            m.d.sync += self.ustate.eq(UState.LOAD)

                    with m.Case(0b01000_11): # Sx
                        ea_adder = Signal(32)
                        m.d.comb += [
                            ea_adder.eq(rf.read_resp + imm_s),
                        ]
                        m.d.sync += [
                            self.rs1.eq(ea_adder),
                        ]

                        with m.If(self.mem_out.ready == 1):
                            m.d.sync += self.ustate.eq(UState.STORE)

                    with m.Case(0b11000_11): # Bxx
                        # Gotta wait for the other register to become available.
                        m.d.sync += self.ustate.eq(UState.BRANCH)

                    with m.Case(0b01100_11): # ALU register-immediate
                        # Waiting on the second register.
                        # We handle shifts separately.
                        with m.If(self.inst[12:15] == 0b001):
                            # SLL
                            m.d.sync += [
                                self.ustate.eq(UState.SHIFTING),
                                self.shift_start.eq(1),
                            ]
                        with m.Elif(self.inst[12:15] == 0b101):
                            # SRL/SRA
                            m.d.sync += [
                                self.ustate.eq(UState.SHIFTING),
                                self.shift_start.eq(1),
                            ]
                        with m.Else():
                            m.d.sync += self.ustate.eq(UState.ALU_RR)

                    with m.Case(0b00100_11): # ALU register-immediate
                        # We handle shifts separately.
                        with m.If(self.inst[12:15] == 0b001):
                            # SLLI
                            m.d.sync += [
                                self.ustate.eq(UState.SHIFTING_IMM),
                                self.shift_start.eq(1),
                            ]
                        with m.Elif(self.inst[12:15] == 0b101):
                            # SRLI/SRAI
                            m.d.sync += [
                                self.ustate.eq(UState.SHIFTING_IMM),
                                self.shift_start.eq(1),
                            ]
                        with m.Else():
                            result = Signal(32)
                            write_result = Signal(1)

                            with m.Switch(self.inst[12:15]):
                                with m.Case(0b000): # ADDI
                                    m.d.comb += result.eq(rf.read_resp + imm_i)
                                with m.Case(0b010): # SLTI
                                    m.d.comb += result.eq(signed_less_than_i)
                                with m.Case(0b011): # SLTIU
                                    m.d.comb += result.eq(unsigned_less_than_i)
                                with m.Case(0b100): # XORI
                                    m.d.comb += result.eq(rf.read_resp ^ imm_i)
                                with m.Case(0b110): # ORI
                                    m.d.comb += result.eq(rf.read_resp | imm_i)
                                with m.Case(0b111): # ANDI
                                    m.d.comb += result.eq(rf.read_resp & imm_i)

                            m.d.comb += [
                                # rd write set by default
                                rf.write_cmd.payload.value.eq(result),
                                rf.write_cmd.valid.eq(1),
                                do_fetch.eq(1),
                            ]

            with m.Case(UState.LOAD):
                m.d.comb += self.mem_in.ready.eq(1)

                with m.If(self.mem_in.valid == 1):
                    with m.Switch(self.inst[12:15]):
                        with m.Case(0b000, 0b100): # LB, LBU
                            byte = Signal(8)
                            with m.Switch(last_lsbs):
                                with m.Case(0b00):
                                    m.d.comb += byte.eq(self.mem_in.payload[:8])
                                with m.Case(0b01):
                                    m.d.comb += byte.eq(self.mem_in.payload[8:16])
                                with m.Case(0b10):
                                    m.d.comb += byte.eq(self.mem_in.payload[16:24])
                                with m.Case(0b11):
                                    m.d.comb += byte.eq(self.mem_in.payload[24:])

                            ext = Signal(32)
                            with m.If(self.inst[14]):
                                m.d.comb += ext.eq(byte)
                            with m.Else():
                                m.d.comb += ext.eq(byte.as_signed())
                            m.d.comb += [
                                # rd write set by default
                                rf.write_cmd.payload.value.eq(ext),
                                rf.write_cmd.valid.eq(1),

                                do_fetch.eq(1),
                            ]
                        with m.Case(0b001, 0b101): # LH, LHU
                            half = Signal(16)
                            with m.Switch(last_lsbs):
                                with m.Case("0-"):
                                    m.d.comb += half.eq(self.mem_in.payload[:16])
                                with m.Case("1-"):
                                    m.d.comb += half.eq(self.mem_in.payload[16:])

                            ext = Signal(32)
                            with m.If(self.inst[14]):
                                m.d.comb += ext.eq(half)
                            with m.Else():
                                m.d.comb += ext.eq(half.as_signed())

                            m.d.comb += [
                                # rd write set by default
                                rf.write_cmd.payload.value.eq(ext),
                                rf.write_cmd.valid.eq(1),

                                do_fetch.eq(1),
                            ]
                        with m.Case("-1-"): # LW
                            m.d.comb += [
                                # rd write set by default
                                rf.write_cmd.payload.value.eq(self.mem_in.payload),
                                rf.write_cmd.valid.eq(1),

                                do_fetch.eq(1),
                            ]

            with m.Case(UState.STORE):
                with m.If(self.mem_out.ready == 1):
                    store_data = Signal(32)
                    with m.Switch(self.inst[12:14]):
                        with m.Case(0b00): # byte
                            m.d.comb += store_data.eq(rf.read_resp[:8].replicate(4))
                        with m.Case(0b01): # half
                            m.d.comb += store_data.eq(rf.read_resp[:16].replicate(2))
                        with m.Case("1-"): # word
                            m.d.comb += store_data.eq(rf.read_resp)

                    lanes = Signal(4)
                    with m.Switch(self.inst[12:14]):
                        with m.Case(0b00): # byte
                            with m.Switch(self.rs1[:2]):
                                with m.Case(0b00):
                                    m.d.comb += lanes.eq(0b0001)
                                with m.Case(0b01):
                                    m.d.comb += lanes.eq(0b0010)
                                with m.Case(0b10):
                                    m.d.comb += lanes.eq(0b0100)
                                with m.Case(0b11):
                                    m.d.comb += lanes.eq(0b1000)
                        with m.Case(0b01): # half
                            with m.Switch(self.rs1[1]):
                                with m.Case(0):
                                    m.d.comb += lanes.eq(0b0011)
                                with m.Case(1):
                                    m.d.comb += lanes.eq(0b1100)
                        with m.Case("1-"): # word
                            m.d.comb += lanes.eq(0b1111)

                    m.d.comb += [
                        self.mem_out.payload.addr.eq(self.rs1[2:]),
                        self.mem_out.payload.data.eq(store_data),
                        self.mem_out.payload.lanes.eq(lanes),
                        self.mem_out.valid.eq(1),
                    ]
                    m.d.sync += self.pc.eq(fetch_addr)
                    m.d.sync += self.ustate.eq(UState.FETCH)

            with m.Case(UState.ALU_RR):
                result = Signal(32)
                write_result = Signal(1)
                with m.Switch(Cat(self.inst[12:15], self.inst[30])):
                    with m.Case(0b0_000): # ADD
                        m.d.comb += result.eq(self.rs1 + rf.read_resp)
                    with m.Case(0b1_000): # SUB
                        m.d.comb += result.eq(self.rs1 - rf.read_resp)
                    with m.Case(0b0_010): # SLT
                        m.d.comb += result.eq(self.rs1.as_signed() <
                                              rf.read_resp.as_signed())
                    with m.Case(0b0_011): # SLTU
                        m.d.comb += result.eq(self.rs1 < rf.read_resp)
                    with m.Case(0b0_100): # XOR
                        m.d.comb += result.eq(self.rs1 ^ rf.read_resp)
                    with m.Case(0b0_110): # OR
                        m.d.comb += result.eq(self.rs1 | rf.read_resp)
                    with m.Case(0b0_111): # AND
                        m.d.comb += result.eq(self.rs1 & rf.read_resp)

                m.d.comb += [
                    # rd write set by default
                    rf.write_cmd.payload.value.eq(result),
                    rf.write_cmd.valid.eq(1),
                    do_fetch.eq(1),
                ]

            with m.Case(UState.SHIFTING, UState.SHIFTING_IMM):
                shift_right = Signal(1)
                m.d.comb += shift_right.eq(self.inst[14])

                shifted1 = Signal(32)
                with m.If(shift_right):
                    new_msb = Signal(1)
                    m.d.comb += [
                        new_msb.eq(self.rs1[31] & self.inst[30]),
                        shifted1.eq(Cat(self.rs1[1:], new_msb)),
                    ]
                with m.Else():
                    m.d.comb += shifted1.eq(Cat(0, self.rs1[0:31]))

                shift_amt = Signal(5)
                with m.If(self.shift_start):
                    with m.If(self.ustate == UState.SHIFTING):
                        m.d.comb += shift_amt.eq(rf.read_resp[0:6])
                    with m.Else():
                        m.d.comb += shift_amt.eq(self.inst[20:25])
                with m.Else():
                    m.d.comb += shift_amt.eq(self.shift_amt)

                m.d.sync += [
                    self.shift_start.eq(0),
                    self.shift_amt.eq(shift_amt - 1),
                ]

                with m.If(shift_amt == 0):
                    # All done!
                    m.d.comb += [
                        # rd write set by default
                        rf.write_cmd.payload.value.eq(self.rs1),
                        rf.write_cmd.valid.eq(1),
                        #do_fetch.eq(1),
                    ]
                    m.d.sync += self.ustate.eq(UState.FETCH)
                with m.Else():
                    # Still shifting!
                    m.d.sync += [
                        self.rs1.eq(shifted1),
                    ]

            with m.Case(UState.BRANCH):
                # We were waiting for rs2 to become available. It's now
                # available.
                br_adder = Signal(32)
                m.d.comb += br_adder.eq(self.pc + imm_b)

                is_eq = Signal(1)
                is_lt = Signal(1)
                is_ltu = Signal(1)
                m.d.comb += [
                    is_eq.eq(rf.read_resp == self.rs1),
                    is_lt.eq(self.rs1.as_signed() <
                             rf.read_resp.as_signed()),
                    is_ltu.eq(self.rs1 < rf.read_resp),

                    do_fetch.eq(1),
                ]

                taken = Signal(1)
                with m.Switch(self.inst[12:15]):
                    with m.Case(0b000):  # EQ
                        m.d.comb += taken.eq(is_eq)
                    with m.Case(0b001):  # NE
                        m.d.comb += taken.eq(~is_eq)
                    with m.Case(0b100):  # LT
                        m.d.comb += taken.eq(is_lt)
                    with m.Case(0b101):  # GE
                        m.d.comb += taken.eq(~is_lt)
                    with m.Case(0b110):  # LTU
                        m.d.comb += taken.eq(is_ltu)
                    with m.Case(0b111):  # GEU
                        m.d.comb += taken.eq(~is_ltu)

                with m.If(taken):
                    m.d.comb += fetch_addr.eq(br_adder)

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

        with m.If(do_fetch):
            m.d.sync += [
                self.pc.eq(fetch_addr),
            ]
            with m.If(self.halt_request):
                m.d.sync += [
                    self.ustate.eq(UState.HALTED),
                ]

            with m.Else():
                m.d.comb += [
                    # Attempt fetch of next instruction
                    self.mem_out.payload.addr.eq(fetch_addr[2:]),
                    self.mem_out.valid.eq(1),
                ]

                # Only treat that as successful if the interface is ready.
                with m.If(self.mem_out.ready):
                    m.d.sync += self.ustate.eq(UState.INST)
                with m.Else():
                    m.d.sync += self.ustate.eq(UState.FETCH)

        return m
