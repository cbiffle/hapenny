from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from rv32 import StreamSig, AlwaysReady
from rv32.regfile16 import RegFile16

MemCmd = Signature({
    'addr': Out(32 - 1), # half-addressed
    'lanes': Out(2),
    'data': Out(16),
})

DebugPort = Signature({
    'reg_read': Out(StreamSig(6)),
    'reg_value': In(AlwaysReady(16)),
    'reg_write': Out(StreamSig(Signature({
        'addr': Out(6),
        'value': Out(16),
    }))),
    'pc': Out(32),
    'pc_write': Out(StreamSig(32)),
})

def mux(select, one, zero):
    if isinstance(one, Enum):
        one = one.value
    if isinstance(one, int):
        one = Const(one)
    if isinstance(zero, Enum):
        zero = zero.value
    if isinstance(zero, int):
        zero = Const(zero)
    n = max(one.shape().width, zero.shape().width)
    select = select.any() # force to 1 bit
    return (
        (select.replicate(n) & one) | (~select.replicate(n) & zero)
    )

def onehot_choice(onehot_sig, options):
    assert len(options) > 0
    output = None
    for (choice, result) in options.items():
        if isinstance(choice, Enum):
            choice = choice.value

        case = onehot_sig[choice].replicate(result.shape().width) & result
        if output is not None:
            output = output | case
        else:
            output = case

    return output

def oneof(options):
    assert len(options) > 0
    output = None
    for (condition, result) in options:
        case = condition.any().replicate(result.shape().width) & result
        if output is not None:
            output = output | case
        else:
            output = case

    return output

class UState(Enum):
    RESET = 0 # TODO remove me
    FETCH = 1
    INST_REG1_LO = 2
    FETCH_HI_WAIT = 3
    REG2 = 4
    OPERATE = 5
    BRANCH = 6
    LOAD = 7
    LOAD_WAIT = 8
    FILL_MSBS = 9
    STORE = 10
    SHIFT = 11
    FINISH_SHIFT = 12
    SLT = 13
    HALTED = 14

class Cpu(Component):
    mem_out: Out(StreamSig(MemCmd))
    mem_in: In(StreamSig(16))

    halt_request: In(1)
    halted: Out(1)

    debug: In(DebugPort)

    def __init__(self, *, addr_width = 32, relax_instruction_alignment = False):
        super().__init__()

        self.addr_width = addr_width
        self.relax_instruction_alignment = relax_instruction_alignment

        self.ustate = Signal(UState, reset = UState.RESET) # TODO
        self.hi = Signal(1)
        self.pc = Signal(addr_width)
        self.inst = Signal(32)

        self.accum = Signal(16)
        self.shadow_pc = Signal(16)
        self.mar_lo = Signal(16)
        self.shift_lo = Signal(16)
        self.shift_amt = Signal(5)
        self.last_unsigned_less_than = Signal(1)
        self.last_signed_less_than = Signal(1)

    def elaborate(self, platform):
        m = Module()

        m.submodules.regfile = rf = RegFile16()

        opcode = Signal(5)
        inst_rd = Signal(5)
        m.d.comb += [
            opcode.eq(self.inst[2:7]),
            inst_rd.eq(self.inst[7:12]),
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

        m.d.comb += [
            self.halted.eq(self.ustate == UState.HALTED),
            self.debug.pc.eq(self.pc),
        ]

        adder_carry_in = Signal(1)
        adder_result = Signal(16)
        adder_rhs = Signal(16)
        adder_carry_out = Signal(1)
        saved_carry = Signal(1)
        saved_zero = Signal(1)
        zero_out = Signal(1)
        
        m.d.comb += [
            Cat(adder_result, adder_carry_out).eq(self.accum + adder_rhs +
                                                  adder_carry_in),
            zero_out.eq(saved_zero & (adder_result == 0)),
        ]

        signed_less_than = Signal(1)
        unsigned_less_than = Signal(1)
        m.d.comb += [
            unsigned_less_than.eq(~adder_carry_out),
            signed_less_than.eq(mux(
                self.accum[15] ^ adder_rhs[15],
                self.accum[15],
                adder_carry_out,
            )),
        ]

        branch_condition = Signal(1)
        with m.Switch(self.inst[13:15]):
            with m.Case(0b00):
                m.d.comb += branch_condition.eq(zero_out)
            with m.Case(0b10):
                m.d.comb += branch_condition.eq(signed_less_than)
            with m.Case(0b11):
                m.d.comb += branch_condition.eq(unsigned_less_than)

        branch_taken = Signal(1)
        m.d.comb += branch_taken.eq(branch_condition ^ self.inst[12])

        load_result = Signal(16)
        with m.Switch(self.inst[12:15]):
            with m.Case(0b000): # LB
                m.d.comb += [
                    load_result[:8].eq(mux(
                        self.mar_lo[0],
                        self.mem_in.payload[8:],
                        self.mem_in.payload[:8],
                    )),
                    load_result[8:].eq(load_result[7].replicate(8)),
                ]
            with m.Case(0b001, 0b010, 0b101): # LH, LW, LHU
                m.d.comb += load_result.eq(self.mem_in.payload)
            with m.Case(0b100): # LBU
                m.d.comb += [
                    load_result[:8].eq(mux(
                        self.mar_lo[0],
                        self.mem_in.payload[8:],
                        self.mem_in.payload[:8],
                    )),
                    load_result[8:].eq(0),
                ]
        store_data = Signal(16)
        with m.Switch(self.inst[12:15]):
            with m.Case(0b000): # SB
                m.d.comb += store_data.eq(rf.read_resp[:8].replicate(2))
            with m.Default(): # SW, SH
                m.d.comb += store_data.eq(rf.read_resp)
        store_mask = Signal(2)
        with m.Switch(self.inst[12:15]):
            with m.Case(0b00): # SB
                m.d.comb += [
                    store_mask[0].eq(~self.mar_lo[0]),
                    store_mask[1].eq(self.mar_lo[0]),
                ]
            with m.Default(): # SW, SH
                m.d.comb += store_mask.eq(0b11)

        pc31_plus_hi = Signal(31)
        if self.relax_instruction_alignment:
            # have to use an adder here as we may go from 2-mod-4 to 0-mod-4
            m.d.comb += pc31_plus_hi.eq(self.pc[1:] + self.hi)
        else:
            m.d.comb += pc31_plus_hi.eq(self.pc[1:] | self.hi)

        with m.Switch(self.ustate):
            with m.Case(UState.RESET):
                m.d.sync += self.ustate.eq(UState.FETCH)

            with m.Case(UState.FETCH):
                m.d.comb += [
                    self.mem_out.payload.addr.eq(pc31_plus_hi),

                    self.mem_in.ready.eq(self.hi),
                ]
                m.d.sync += saved_zero.eq(1)
                m.d.sync += saved_carry.eq(0)
                m.d.sync += self.inst[:16].eq(self.mem_in.payload)
                with m.If(self.mem_out.valid & self.mem_out.ready):
                    m.d.sync += self.hi.eq(~self.hi)

                with m.If(~self.hi):
                    with m.If(self.halt_request):
                        m.d.sync += self.ustate.eq(UState.HALTED)
                    with m.Else():
                        m.d.comb += self.mem_out.valid.eq(1)
                with m.Else():
                    m.d.comb += [
                        # Only issue a new request if our last one is completing.
                        self.mem_out.valid.eq(self.mem_in.valid),
                    ]

                    with m.If(self.mem_in.valid):
                        with m.If(self.mem_out.ready):
                            # fetch-lo completed and fetch-hi has issued,
                            # we can move on to...
                            m.d.sync += self.ustate.eq(UState.INST_REG1_LO)
                        with m.Else():
                            # fetch-lo completed but fetch-hi has not been able to
                            # issue. we need to hang out waiting for fetch-hi.
                            m.d.sync += self.ustate.eq(UState.FETCH_HI_WAIT)

            with m.Case(UState.FETCH_HI_WAIT):
                m.d.comb += [
                    self.mem_out.payload.addr.eq(pc31_plus_hi),
                    self.mem_out.valid.eq(1),
                ]

                with m.If(self.mem_out.ready):
                    # fetch-hi has issued, we can move on to...
                    m.d.sync += self.ustate.eq(UState.INST_REG1_LO)
                    m.d.sync += self.hi.eq(~self.hi)

            with m.Case(UState.INST_REG1_LO):
                with m.If(self.mem_in.valid):
                    # Hi half fetch is completing!
                    m.d.sync += self.inst[16:].eq(self.mem_in.payload)
                    # Set up a read of the low half of the first operand
                    # register, optimistically, using the not-yet-latched rs1
                    # select bits coming in from the bus.
                    m.d.comb += [
                        rf.read_cmd.valid.eq(1),
                        rf.read_cmd.payload.eq(
                            Cat(self.inst[15], self.mem_in.payload[0:4]),
                        ),
                    ]

                    m.d.sync += self.ustate.eq(UState.REG2)
                with m.Else():
                    # We're waiting on the bus to finish the fetch before we can
                    # do anything.
                    pass

            with m.Case(UState.REG2):
                # rs1 is now available on the register file output port.
                # Latch it into the accumulator.
                with m.Switch(opcode):
                    with m.Case(0b00101, 0b11011): # AUIPC, JAL
                        # Load program counter instead.
                        m.d.sync += self.accum.eq(mux(
                            self.hi,
                            self.pc[16:],
                            self.pc[:16],
                        ))
                    with m.Default():
                        m.d.sync += self.accum.eq(rf.read_resp)

                # Set up a read of the second operand register.
                m.d.comb += [
                    rf.read_cmd.valid.eq(1),
                    rf.read_cmd.payload.eq(Cat(
                        self.inst[20:25],
                        self.hi & (opcode != 0b01000), # overriden for store TODO
                    )),
                ]

                m.d.sync += self.ustate.eq(UState.OPERATE)

            with m.Case(UState.OPERATE):
                # Overridden assignments:
                m.d.comb += [
                    adder_carry_in.eq(saved_carry),
                ]
                m.d.sync += [
                    self.hi.eq(~self.hi),
                ]
                # Common assignments are at end of case to make it clear
                # they're not being overridden.

                with m.Switch(opcode):
                    with m.Case(0b01101): # LUI
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(mux(
                                self.hi,
                                imm_u[16:],
                                imm_u[:16],
                            )),
                            rf.write_cmd.valid.eq(1),
                        ]
                        # We can jump directly to operate hi, no additional
                        # register loads are required.
                        with m.If(self.hi):
                            m.d.sync += self.ustate.eq(UState.FETCH)
                            m.d.sync += self.pc.eq(self.pc + 4)
                        with m.Else():
                            m.d.sync += self.ustate.eq(UState.OPERATE)
                    with m.Case(0b00101): # AUIPC
                        m.d.comb += [
                            adder_rhs.eq(mux(
                                self.hi,
                                imm_u[16:],
                                imm_u[:16],
                            )),
                            rf.write_cmd.payload.value.eq(adder_result),
                            rf.write_cmd.valid.eq(1),
                        ]
                        with m.If(self.hi):
                            m.d.sync += self.ustate.eq(UState.FETCH)
                            m.d.sync += self.pc.eq(self.pc + 4)
                        with m.Else():
                            m.d.sync += self.ustate.eq(UState.OPERATE)
                            m.d.sync += self.accum.eq(self.pc[16:])

                    with m.Case(0b11011): # JAL
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(mux(
                                self.hi,
                                (self.pc + 4)[16:],
                                self.pc + 4,
                            )),
                            rf.write_cmd.valid.eq(1),

                            adder_rhs.eq(mux(
                                self.hi,
                                imm_j[16:],
                                imm_j[:16],
                            )),
                        ]
                        with m.If(self.hi):
                            m.d.sync += [
                                self.pc.eq(Cat(self.shadow_pc, adder_result)),
                                self.ustate.eq(UState.FETCH),
                            ]
                        with m.Else():
                            m.d.sync += [
                                self.shadow_pc.eq(adder_result),
                                self.accum.eq(self.pc[16:]),
                                self.ustate.eq(UState.OPERATE),
                            ]

                    with m.Case(0b11001): # JALR
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(mux(
                                self.hi,
                                (self.pc + 4)[16:],
                                self.pc + 4,
                            )),
                            rf.write_cmd.valid.eq(1),

                            adder_rhs.eq(mux(
                                self.hi,
                                imm_i[16:],
                                imm_i[:16],
                            )),
                        ]

                        m.d.comb += [
                            # Start read of high half of rs1
                            rf.read_cmd.valid.eq(1),
                            rf.read_cmd.payload.eq(Cat(self.inst[15:20], 1)),
                        ]
                        with m.If(self.hi):
                            m.d.sync += [
                                self.pc.eq(Cat(self.shadow_pc, adder_result)),
                                self.ustate.eq(UState.FETCH),
                            ]
                        with m.Else():
                            m.d.sync += [
                                self.shadow_pc.eq(adder_result),
                                self.ustate.eq(UState.REG2),
                            ]

                    with m.Case(0b11000): # Bxx
                        m.d.comb += [
                            # Configure the adder to do a subtraction.
                            adder_rhs.eq(~rf.read_resp),
                            adder_carry_in.eq(saved_carry | ~self.hi),

                            # Start read of high half of rs1
                            rf.read_cmd.valid.eq(1),
                            rf.read_cmd.payload.eq(Cat(self.inst[15:20], 1)),
                        ]
                        with m.If(self.hi):
                            with m.If(branch_taken):
                                m.d.sync += [
                                    self.accum.eq(self.pc[:16]),
                                    self.ustate.eq(UState.BRANCH),
                                ]
                            with m.Else():
                                m.d.sync += [
                                    self.pc.eq(self.pc + 4),
                                    self.ustate.eq(UState.FETCH)
                                ]
                        with m.Else():
                            m.d.comb += adder_carry_in.eq(1)
                            m.d.sync += [
                                saved_zero.eq(zero_out),
                                self.ustate.eq(UState.REG2),
                            ]

                    with m.Case(0b00000): # Lx
                        m.d.comb += [
                            adder_rhs.eq(mux(
                                self.hi,
                                imm_i[16:],
                                imm_i[:16],
                            )),
                        ]
                        # Start read of high half of rs1
                        m.d.comb += [
                            rf.read_cmd.valid.eq(~self.hi),
                            rf.read_cmd.payload.eq(Cat(self.inst[15:20], 1)),

                            self.mem_out.payload.addr.eq(
                                Cat(self.mar_lo, adder_result)[1:],
                            ),
                        ]
                        with m.If(self.hi):
                            # Generate our memory transaction from our various bits:
                            m.d.comb += self.mem_out.valid.eq(1)
                            with m.If(self.mem_out.ready):
                                m.d.sync += self.ustate.eq(UState.LOAD)
                            with m.Else():
                                # wait for the bus
                                pass
                        with m.Else():
                            m.d.sync += [
                                self.mar_lo.eq(adder_result),
                                self.ustate.eq(UState.REG2),
                            ]

                    with m.Case(0b01000): # Sx
                        m.d.comb += [
                            adder_rhs.eq(mux(
                                self.hi,
                                imm_s[16:],
                                imm_s[:16],
                            )),
                        ]
                        with m.If(self.hi):
                            # Generate our memory transaction from our various bits.
                            m.d.comb += [
                                self.mem_out.valid.eq(1),
                                self.mem_out.payload.addr.eq(
                                    Cat(self.mar_lo, adder_result)[1:],
                                ),
                                self.mem_out.payload.data.eq(store_data),
                                self.mem_out.payload.lanes.eq(store_mask),
                            ]
                            # Set up a read of hi(rs2). We'll ignore it if we're not
                            # doing a word store.
                            m.d.comb += [
                                rf.read_cmd.payload.eq(Cat(self.inst[20:25], 1)),
                                rf.read_cmd.valid.eq(1),
                            ]
                            with m.If(self.mem_out.ready):
                                with m.If(self.inst[12:15] == 0b010):
                                    # Word store!
                                    m.d.sync += self.ustate.eq(UState.STORE)
                                with m.Else():
                                    # Smaller store -- we're done
                                    m.d.sync += [
                                        self.pc.eq(self.pc + 4),
                                        self.ustate.eq(UState.FETCH),
                                    ]
                            with m.Else():
                                # wait for the bus
                                pass

                        with m.Else():
                            # Start read of high half of rs1
                            m.d.comb += [
                                rf.read_cmd.valid.eq(1),
                                rf.read_cmd.payload.eq(Cat(self.inst[15:20], 1)),
                            ]
                            m.d.sync += [
                                self.mar_lo.eq(adder_result),
                                self.ustate.eq(UState.REG2),
                            ]

                    with m.Case("0-100"): # ALU register-register / register-imm
                        # Note: opcode[3] is 1 for RR, 0 for RI.
                        m.d.comb += [
                            adder_rhs.eq(mux(
                                opcode[3],
                                rf.read_resp ^ self.inst[30].replicate(16),
                                mux(
                                    self.hi,
                                    imm_i[16:],
                                    imm_i[:16],
                                ),
                            )),
                            adder_carry_in.eq(mux(
                                self.hi | ~opcode[3],
                                saved_carry,
                                self.inst[30],
                            )),

                            rf.write_cmd.valid.eq(1),

                            # Start read of other half of rs1. this is only
                            # useful on the lo phase but whatever.
                            rf.read_cmd.valid.eq(1),
                            rf.read_cmd.payload.eq(Cat(self.inst[15:20], ~self.hi)),
                        ]
                        with m.Switch(self.inst[12:15]):
                            with m.Case(0b000): # ADD/SUB
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(adder_result),
                                ]
                            with m.Case(0b001, 0b101): # SLL, SRL, SRA
                                pass # handled in SHIFT state
                            with m.Case(0b010, 0b011): # SLT, SLTU
                                m.d.comb += [
                                    adder_rhs.eq(~rf.read_resp),
                                    adder_carry_in.eq(saved_carry | ~self.hi),
                                ]
                            with m.Case(0b100): # XOR
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum ^
                                                                  adder_rhs),
                                ]
                            with m.Case(0b110): # OR
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum |
                                                                  adder_rhs),
                                ]
                            with m.Case(0b111): # AND
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum &
                                                                  adder_rhs),
                                ]
                        with m.If(self.hi):
                            with m.If(self.inst[12:15].matches("-01")):
                                m.d.sync += self.hi.eq(self.hi)
                                m.d.sync += self.ustate.eq(UState.SHIFT)
                            with m.Elif(self.inst[12:15].matches("01-")):
                                m.d.sync += self.ustate.eq(UState.SLT)
                            with m.Else():
                                m.d.sync += self.ustate.eq(UState.FETCH)
                        with m.Else():
                            m.d.sync += [
                                # Record shift amount
                                self.shift_amt.eq(mux(
                                    opcode[3],
                                    rf.read_resp[:5],
                                    self.inst[20:25],
                                )),
                                # Back up LSBs of rs1 into shifter
                                self.shift_lo.eq(self.accum),

                                self.ustate.eq(UState.REG2),
                            ]
                # OPERATE common assignments:
                m.d.comb += [
                    rf.write_cmd.payload.reg.eq(Cat(inst_rd, self.hi)),
                ]
                m.d.sync += [
                    saved_carry.eq(adder_carry_out),
                    self.last_unsigned_less_than.eq(unsigned_less_than),
                    self.last_signed_less_than.eq(signed_less_than),
                ]

            with m.Case(UState.BRANCH):
                m.d.comb += [
                    adder_rhs.eq(mux(
                        self.hi,
                        imm_b[16:],
                        imm_b[:16],
                    )),
                    adder_carry_in.eq(saved_carry & self.hi),
                ]
                m.d.sync += [
                    self.hi.eq(~self.hi),
                    saved_carry.eq(adder_carry_out),
                ]
                with m.If(self.hi):
                    m.d.sync += [
                        self.pc.eq(Cat(self.shadow_pc, adder_result)),
                        self.ustate.eq(UState.FETCH)
                    ]
                with m.Else():
                    m.d.sync += [
                        self.shadow_pc.eq(adder_result),
                        self.accum.eq(self.pc[16:]),
                        self.ustate.eq(UState.BRANCH)
                    ]

            with m.Case(UState.LOAD):
                m.d.comb += [
                    self.mem_in.ready.eq(1),

                    # Set up the write of the halfword...
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], self.hi)),
                    rf.write_cmd.payload.value.eq(load_result),
                    # ...but only commit it if the data's actually here.
                    rf.write_cmd.valid.eq(self.mem_in.valid),
                    self.mem_out.payload.addr.eq(
                        Cat(self.mar_lo, adder_result)[1:] | 1,
                    ),
                ]

                with m.If(self.mem_in.valid):
                    m.d.sync += self.hi.eq(~self.hi)

                    with m.If(self.inst[12:15].matches("00-")):  # LB, LH
                        m.d.sync += self.accum.eq(load_result[15].replicate(16))
                    with m.Else():
                        m.d.sync += self.accum.eq(0)

                    with m.If(~self.hi):
                        with m.Switch(self.inst[12:15]):
                            with m.Case(0b000, 0b001): # LB, LH
                                m.d.sync += self.ustate.eq(UState.FILL_MSBS)
                            with m.Case(0b010): # LW
                                # Address the next halfword
                                m.d.comb += self.mem_out.valid.eq(1)
                                with m.If(self.mem_out.ready):
                                    m.d.sync += self.ustate.eq(UState.LOAD)
                                with m.Else():
                                    m.d.sync += self.ustate.eq(UState.LOAD_WAIT)
                            with m.Case(0b100, 0b101): # LBU, LHU
                                m.d.sync += self.ustate.eq(UState.FILL_MSBS)
                    with m.Else():
                        m.d.sync += [
                            self.pc.eq(self.pc + 4),
                            self.ustate.eq(UState.FETCH),
                        ]

            with m.Case(UState.LOAD_WAIT):
                # Address the next halfword
                m.d.comb += [
                    self.mem_out.valid.eq(1),
                    self.mem_out.payload.addr.eq(
                        Cat(self.mar_lo, adder_result)[1:] | 1,
                    ),
                ]
                with m.If(self.mem_out.ready):
                    m.d.sync += self.ustate.eq(UState.LOAD_WAIT)
                with m.Else():
                    m.d.sync += self.ustate.eq(UState.LOAD)

            with m.Case(UState.STORE):
                # Write the high half of rs2 (which has just arrived on the
                # register file output port) to memory.
                m.d.comb += [
                    self.mem_out.valid.eq(1),
                    self.mem_out.payload.addr.eq(
                        Cat(self.mar_lo, adder_result)[1:] | 1,
                    ),
                    self.mem_out.payload.data.eq(store_data),
                    self.mem_out.payload.lanes.eq(store_mask),
                ]
                with m.If(self.mem_out.ready):
                    m.d.sync += [
                        self.pc.eq(self.pc + 4),
                        self.ustate.eq(UState.FETCH),
                    ]

            with m.Case(UState.FILL_MSBS):
                m.d.comb += [
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], 1)),
                    rf.write_cmd.payload.value.eq(self.accum),
                    rf.write_cmd.valid.eq(1),
                ]
                m.d.sync += [
                    self.pc.eq(self.pc + 4),
                    self.ustate.eq(UState.FETCH),
                    self.hi.eq(~self.hi),
                ]

            with m.Case(UState.SHIFT):
                m.d.comb += [
                    # Set up write unconditionally but don't set VALID.
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], self.hi)),
                    rf.write_cmd.payload.value.eq(mux(
                        self.hi,
                        self.accum,
                        self.shift_lo,
                    )),
                ]
                with m.If(self.shift_amt == 0):
                    m.d.comb += [
                        rf.write_cmd.valid.eq(1),
                    ]
                    m.d.sync += self.hi.eq(0)
                    with m.If(self.hi):
                        pass
                    with m.Else():
                        m.d.sync += self.ustate.eq(UState.FETCH)
                with m.Else():
                    m.d.sync += self.shift_amt.eq(self.shift_amt - 1)
                    with m.If(self.inst[14] == 0): # left shift
                        m.d.sync += [
                            Cat(self.shift_lo, self.accum).eq(
                                Cat(0, self.shift_lo, self.accum[:15]),
                            ),
                        ]
                    with m.Else(): # right shift
                        m.d.sync += [
                            Cat(self.shift_lo, self.accum).eq(
                                Cat(self.shift_lo[1:], self.accum, self.inst[30] & self.accum[15]),
                            ),
                        ]


            with m.Case(UState.FINISH_SHIFT):
                m.d.comb += [
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], 0)),
                    rf.write_cmd.payload.value.eq(self.shift_lo),
                    rf.write_cmd.valid.eq(1),
                ]
                m.d.sync += [
                    self.pc.eq(self.pc + 4),
                    self.ustate.eq(UState.FETCH),
                ]

            with m.Case(UState.SLT):
                m.d.comb += [
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], self.hi)),
                    rf.write_cmd.payload.value.eq(mux(
                        self.hi,
                        0,
                        mux(
                            self.inst[12],
                            self.last_unsigned_less_than,
                            self.last_signed_less_than,
                        ),
                    )),
                    rf.write_cmd.valid.eq(1),
                ]
                m.d.sync += [
                    self.hi.eq(~self.hi)
                ]
                with m.If(self.hi):
                    m.d.sync += self.ustate.eq(UState.FETCH)
                with m.Else():
                    pass

            with m.Case(UState.HALTED):
                with m.If(self.debug.pc_write.valid):
                    m.d.sync += self.pc.eq(self.debug.pc_write.payload)

                m.d.comb += [
                    self.debug.reg_read.ready.eq(1),
                    rf.read_cmd.valid.eq(self.debug.reg_read.valid),
                    rf.read_cmd.payload.eq(self.debug.reg_read.payload),

                    self.debug.reg_value.valid.eq(1),
                    self.debug.reg_value.payload.eq(rf.read_resp),

                    self.debug.reg_write.ready.eq(1),
                    rf.write_cmd.valid.eq(self.debug.reg_write.valid),
                    rf.write_cmd.payload.reg.eq(self.debug.reg_write.payload.addr),
                    rf.write_cmd.payload.value.eq(self.debug.reg_write.payload.value),
                ]

                with m.If(~self.halt_request):
                    m.d.sync += self.ustate.eq(UState.FETCH)

        #

        return m
