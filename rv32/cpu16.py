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
    FETCH_LO = 1
    FETCH_HI = 2
    INST_REG1_LO = 3
    FETCH_HI_WAIT = 4
    REG2_LO = 5
    OPERATE_LO = 6
    HALTED = 7
    OPERATE_HI = 8
    REG2_HI = 9
    BRANCH_LO = 10
    BRANCH_HI = 11
    LOAD = 12
    LOAD_HI = 13
    LOAD_HI_WAIT = 14
    FILL_MSBS = 15
    STORE_HI = 16
    FINISH_SHIFT = 17

class Cpu(Component):
    mem_out: Out(StreamSig(MemCmd))
    mem_in: In(StreamSig(16))

    halt_request: In(1)
    halted: Out(1)

    debug: In(DebugPort)

    def __init__(self, *, addr_width = 32):
        super().__init__()

        self.addr_width = addr_width

        self.ustate = Signal(UState, reset = UState.RESET) # TODO
        self.pc = Signal(addr_width)
        self.inst = Signal(32)

        self.accum = Signal(16)
        self.shadow_pc = Signal(16)
        self.mar_lo = Signal(16)
        self.shift_lo = Signal(16)
        self.shift_amt = Signal(5)

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

        with m.Switch(self.ustate):
            with m.Case(UState.RESET):
                m.d.sync += self.ustate.eq(UState.FETCH_LO)

            with m.Case(UState.FETCH_LO):
                m.d.comb += [
                    self.mem_out.payload.addr.eq(self.pc[1:]),
                ]

                with m.If(self.halt_request):
                    m.d.sync += self.ustate.eq(UState.HALTED)
                with m.Else():
                    m.d.comb += self.mem_out.valid.eq(1)
                    with m.If(self.mem_out.ready):
                        m.d.sync += self.ustate.eq(UState.FETCH_HI)

            with m.Case(UState.FETCH_HI):
                m.d.comb += [
                    self.mem_in.ready.eq(1),
                    self.mem_out.payload.addr.eq(self.pc[1:] + 1),
                    # Only issue a new request if our last one is completing.
                    self.mem_out.valid.eq(self.mem_in.valid),
                ]

                with m.If(self.mem_in.valid):
                    m.d.sync += self.inst[:16].eq(self.mem_in.payload)
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
                    self.mem_out.payload.addr.eq(self.pc[1:] + 1),
                    self.mem_out.valid.eq(1),
                ]

                with m.If(self.mem_out.ready):
                    # fetch-hi has issued, we can move on to...
                    m.d.sync += self.ustate.eq(UState.INST_REG1_LO)

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

                    m.d.sync += self.ustate.eq(UState.REG2_LO)
                with m.Else():
                    # We're waiting on the bus to finish the fetch before we can
                    # do anything.
                    pass

            with m.Case(UState.REG2_LO):
                # lo(rs1) is now available on the register file output port.
                # Latch it into the accumulator.
                with m.Switch(opcode):
                    with m.Case(0b00101): # AUIPC
                        # Load low half of program counter to prepare for
                        # addition to immediate.
                        m.d.sync += self.accum.eq(self.pc[:16])
                    with m.Case(0b11011): # JAL
                        # Load low half of program counter to prepare for
                        # addition to immediate.
                        m.d.sync += self.accum.eq(self.pc[:16])
                    with m.Default():
                        m.d.sync += self.accum.eq(rf.read_resp)

                # Set up a read of the low half of the second operand
                # register, optimistically.
                m.d.comb += [
                    rf.read_cmd.valid.eq(1),
                    rf.read_cmd.payload.eq(self.inst[20:25]),
                ]

                m.d.sync += saved_zero.eq(1)
                m.d.sync += saved_carry.eq(0)
                m.d.sync += self.ustate.eq(UState.OPERATE_LO)

            with m.Case(UState.OPERATE_LO):
                m.d.comb += [
                    rf.write_cmd.payload.reg.eq(Cat(inst_rd, 0)),
                ]
                m.d.sync += [
                    saved_carry.eq(adder_carry_out),
                ]
                with m.Switch(opcode):
                    with m.Case(0b01101): # LUI
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(imm_u[:16]),
                            rf.write_cmd.valid.eq(1),
                        ]
                        # We can jump directly to operate hi, no additional
                        # register loads are required.
                        m.d.sync += self.ustate.eq(UState.OPERATE_HI)
                    with m.Case(0b00101): # AUIPC
                        m.d.comb += [
                            adder_rhs.eq(imm_u[:16]),
                            rf.write_cmd.payload.value.eq(adder_result),
                            rf.write_cmd.valid.eq(1),
                        ]
                        m.d.sync += [
                            self.accum.eq(self.pc[16:]),
                            self.ustate.eq(UState.OPERATE_HI),
                        ]

                    with m.Case(0b11011): # JAL
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(self.pc + 4),
                            rf.write_cmd.valid.eq(1),

                            adder_rhs.eq(imm_j[:16]),
                        ]
                        m.d.sync += [
                            self.accum.eq(self.pc[16:]),
                            self.shadow_pc.eq(adder_result),
                            self.ustate.eq(UState.OPERATE_HI),
                        ]

                    with m.Case(0b11001): # JALR
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(self.pc + 4),
                            rf.write_cmd.valid.eq(1),

                            adder_rhs.eq(imm_i[:16]),

                            # Start read of high half of rs1
                            rf.read_cmd.valid.eq(1),
                            rf.read_cmd.payload.eq(Cat(self.inst[15:20], 1)),
                        ]
                        m.d.sync += [
                            self.shadow_pc.eq(adder_result),
                            self.ustate.eq(UState.REG2_HI),
                        ]

                    with m.Case(0b11000): # Bxx
                        m.d.comb += [
                            # Configure the adder to do a subtraction.
                            adder_rhs.eq(~rf.read_resp[:16]),
                            adder_carry_in.eq(1),

                            # Start read of high half of rs1
                            rf.read_cmd.valid.eq(1),
                            rf.read_cmd.payload.eq(Cat(self.inst[15:20], 1)),
                        ]
                        m.d.sync += [
                            saved_zero.eq(zero_out),
                            self.ustate.eq(UState.REG2_HI),
                        ]

                    with m.Case(0b00000): # Lx
                        # We have the low half of rs1 in the accumulator, we
                        # can begin computing the effective address.
                        m.d.comb += [
                            adder_rhs.eq(imm_i[:16]),
                            adder_carry_in.eq(saved_carry),  # known to be 0
                        ]
                        # Start read of high half of rs1
                        m.d.comb += [
                            rf.read_cmd.valid.eq(1),
                            rf.read_cmd.payload.eq(Cat(self.inst[15:20], 1)),
                        ]
                        m.d.sync += [
                            self.mar_lo.eq(adder_result),
                            self.ustate.eq(UState.REG2_HI),
                        ]

                    with m.Case(0b01000): # Sx
                        # We have the low half of rs1 in the accumulator, we
                        # can begin computing the effective address.
                        m.d.comb += [
                            adder_rhs.eq(imm_s[:16]),
                            adder_carry_in.eq(saved_carry),  # known to be 0
                        ]
                        # Start read of high half of rs1
                        m.d.comb += [
                            rf.read_cmd.valid.eq(1),
                            rf.read_cmd.payload.eq(Cat(self.inst[15:20], 1)),
                        ]
                        m.d.sync += [
                            self.mar_lo.eq(adder_result),
                            self.ustate.eq(UState.REG2_HI),
                        ]

                    with m.Case(0b01100): # ALU register-register
                        # Compute the low half of the operation
                        with m.Switch(self.inst[12:15]):
                            with m.Case(0b000): # ADD/SUB
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(adder_result),
                                ]
                            with m.Case(0b001, 0b101): # SLL, SRL, SRA
                                m.d.sync += [
                                    # Back up LSBs of rs1 into shifter
                                    self.shift_lo.eq(self.accum),
                                    # Record shift amount
                                    self.shift_amt.eq(rf.read_resp[:5]),
                                ]
                            with m.Case(0b100): # XOR
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum ^
                                                                  rf.read_resp),
                                ]
                            with m.Case(0b110): # OR
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum |
                                                                  rf.read_resp),
                                ]
                            with m.Case(0b111): # AND
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum &
                                                                  rf.read_resp),
                                    rf.write_cmd.valid.eq(1),
                                ]
                        m.d.comb += [
                            adder_rhs.eq(rf.read_resp ^ self.inst[30].replicate(16)),
                            adder_carry_in.eq(self.inst[30]),

                            rf.write_cmd.valid.eq(1),

                            # Start read of high half of rs1
                            rf.read_cmd.valid.eq(1),
                            rf.read_cmd.payload.eq(Cat(self.inst[15:20], 1)),
                        ]
                        m.d.sync += [
                            self.ustate.eq(UState.REG2_HI),
                        ]

                    with m.Case(0b00100): # ALU register-immediate
                        # Compute the low half of the operation
                        with m.Switch(self.inst[12:15]):
                            with m.Case(0b000): # ADD
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(adder_result),
                                ]
                            with m.Case(0b001, 0b101): # SLL, SRL, SRA
                                m.d.sync += [
                                    # Back up LSBs of rs1 into shifter
                                    self.shift_lo.eq(self.accum),
                                    # Record shift amount
                                    self.shift_amt.eq(imm_i[:5]),
                                ]
                            with m.Case(0b100): # XOR
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum ^
                                                                  imm_i),
                                ]
                            with m.Case(0b110): # OR
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum |
                                                                  imm_i),
                                ]
                            with m.Case(0b111): # AND
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum &
                                                                  imm_i),
                                ]
                        m.d.comb += [
                            adder_rhs.eq(imm_i),
                            adder_carry_in.eq(saved_carry),  # known to be 0

                            rf.write_cmd.valid.eq(1),

                            # Start read of high half of rs1
                            rf.read_cmd.valid.eq(1),
                            rf.read_cmd.payload.eq(Cat(self.inst[15:20], 1)),
                        ]
                        m.d.sync += [
                            self.ustate.eq(UState.REG2_HI),
                        ]

            with m.Case(UState.REG2_HI):
                # hi(rs1) is now available on the register file output port.
                # Latch it into the accumulator.
                m.d.sync += self.accum.eq(rf.read_resp)

                # Set up a read of the high half of the second operand
                # register. Except if we're storing. Then do the low half.
                # Technically redundant.
                m.d.comb += [
                    rf.read_cmd.valid.eq(1),
                    rf.read_cmd.payload.eq(Cat(self.inst[20:25], opcode !=
                                               0b01000)),
                ]

                m.d.sync += self.ustate.eq(UState.OPERATE_HI)

            with m.Case(UState.OPERATE_HI):
                m.d.comb += [
                    adder_carry_in.eq(saved_carry),
                    rf.write_cmd.payload.reg.eq(Cat(inst_rd, 1)),
                ]
                m.d.sync += [
                    saved_carry.eq(adder_carry_out),
                ]
                with m.Switch(opcode):
                    with m.Case(0b01101): # LUI
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq(imm_u[16:]),
                            rf.write_cmd.valid.eq(1),
                        ]
                        m.d.sync += self.ustate.eq(UState.FETCH_LO)
                        m.d.sync += self.pc.eq(self.pc + 4)

                    with m.Case(0b00101): # AUIPC
                        m.d.comb += [
                            adder_rhs.eq(imm_u[16:]),
                            rf.write_cmd.payload.value.eq(adder_result),
                            rf.write_cmd.valid.eq(1),
                        ]
                        m.d.sync += self.ustate.eq(UState.FETCH_LO)
                        m.d.sync += self.pc.eq(self.pc + 4)

                    with m.Case(0b11011): # JAL
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq((self.pc + 4)[16:]),
                            rf.write_cmd.valid.eq(1),

                            adder_rhs.eq(imm_j[16:]),
                        ]
                        m.d.sync += [
                            self.pc.eq(Cat(self.shadow_pc, adder_result)),
                            self.ustate.eq(UState.FETCH_LO),
                        ]

                    with m.Case(0b11001): # JALR
                        m.d.comb += [
                            rf.write_cmd.payload.value.eq((self.pc + 4)[16:]),
                            rf.write_cmd.valid.eq(1),

                            adder_rhs.eq(imm_i[16:]),
                        ]
                        m.d.sync += [
                            self.pc.eq(Cat(self.shadow_pc, adder_result)),
                            self.ustate.eq(UState.FETCH_LO),
                        ]

                    with m.Case(0b11000): # Bxx
                        m.d.comb += [
                            # Configure the adder to finish the subtraction.
                            adder_rhs.eq(~rf.read_resp),
                        ]
                        with m.If(branch_taken):
                            m.d.sync += [
                                self.accum.eq(self.pc[:16]),
                                self.ustate.eq(UState.BRANCH_LO),
                            ]
                        with m.Else():
                            m.d.sync += [
                                self.pc.eq(self.pc + 4),
                                self.ustate.eq(UState.FETCH_LO)
                            ]
                    with m.Case(0b00000): # Lx
                        # We have the high half of rs1 in the accumulator, we
                        # can finish computing the effective address.
                        m.d.comb += [
                            adder_rhs.eq(imm_i[16:]),
                        ]
                        # Generate our memory transaction from our various bits:
                        m.d.comb += [
                            self.mem_out.valid.eq(1),
                            self.mem_out.payload.addr.eq(
                                Cat(self.mar_lo, adder_result)[1:],
                            ),
                        ]
                        with m.If(self.mem_out.ready):
                            m.d.sync += self.ustate.eq(UState.LOAD)
                        with m.Else():
                            # wait for the bus
                            pass

                    with m.Case(0b01000): # Sx
                        # lo(rs2) is now available.

                        # We have the high half of rs1 in the accumulator, we
                        # can finish computing the effective address.
                        m.d.comb += [
                            adder_rhs.eq(imm_s[16:]),
                        ]
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
                                m.d.sync += self.ustate.eq(UState.STORE_HI)
                            with m.Else():
                                # Smaller store -- we're done
                                m.d.sync += [
                                    self.pc.eq(self.pc + 4),
                                    self.ustate.eq(UState.FETCH_LO),
                                ]
                        with m.Else():
                            # wait for the bus
                            pass

                    with m.Case(0b01100): # ALU register-register
                        # Compute the high half of the operation
                        m.d.comb += [
                            adder_rhs.eq(rf.read_resp ^ self.inst[30].replicate(16)),

                            rf.write_cmd.valid.eq(1),
                        ]
                        m.d.sync += [
                            self.ustate.eq(UState.FETCH_LO),
                        ]

                        with m.Switch(self.inst[12:15]):
                            with m.Case(0b000): # ADD/SUB
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(adder_result),
                                ]
                            with m.Case(0b001): # SLL
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum),
                                ]
                                with m.If(self.shift_amt != 0):
                                    m.d.sync += [
                                        # Shift left one position.
                                        Cat(self.shift_lo, self.accum).eq(
                                            Cat(0, self.shift_lo, self.accum[:15]),
                                        ),
                                        # Decrement shift count
                                        self.shift_amt.eq(self.shift_amt - 1),

                                        # Don't leave this state.
                                        self.ustate.eq(UState.OPERATE_HI),
                                    ]
                                with m.Else():
                                    m.d.sync += [
                                        self.ustate.eq(UState.FINISH_SHIFT),
                                    ]
                            with m.Case(0b101): # SRL, SRA
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum),
                                ]
                                with m.If(self.shift_amt != 0):
                                    m.d.sync += [
                                        # Shift right one position.
                                        Cat(self.shift_lo, self.accum).eq(
                                            Cat(self.shift_lo[1:], self.accum,
                                                self.accum[15] & self.inst[30]),
                                        ),
                                        # Decrement shift count
                                        self.shift_amt.eq(self.shift_amt - 1),

                                        # Don't leave this state.
                                        self.ustate.eq(UState.OPERATE_HI),
                                    ]
                                with m.Else():
                                    m.d.sync += [
                                        self.ustate.eq(UState.FINISH_SHIFT),
                                    ]
                            with m.Case(0b100): # XOR
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum ^
                                                                  rf.read_resp),
                                ]
                            with m.Case(0b110): # OR
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum |
                                                                  rf.read_resp),
                                ]
                            with m.Case(0b111): # AND
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum &
                                                                  rf.read_resp),
                                ]

                    with m.Case(0b00100): # ALU register-immediate
                        # Compute the high half of the operation
                        m.d.sync += [
                            self.ustate.eq(UState.FETCH_LO),
                        ]
                        with m.Switch(self.inst[12:15]):
                            with m.Case(0b000):
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(adder_result),
                                ]
                            with m.Case(0b001): # SLL
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum),
                                ]
                                with m.If(self.shift_amt != 0):
                                    m.d.sync += [
                                        # Shift left one position.
                                        Cat(self.shift_lo, self.accum).eq(
                                            Cat(0, self.shift_lo, self.accum[:15]),
                                        ),
                                        # Decrement shift count
                                        self.shift_amt.eq(self.shift_amt - 1),

                                        # Don't leave this state.
                                        self.ustate.eq(UState.OPERATE_HI),
                                    ]
                                with m.Else():
                                    m.d.sync += [
                                        self.ustate.eq(UState.FINISH_SHIFT),
                                    ]
                            with m.Case(0b101): # SRL, SRA
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum),
                                ]
                                with m.If(self.shift_amt != 0):
                                    m.d.sync += [
                                        # Shift right one position.
                                        Cat(self.shift_lo, self.accum).eq(
                                            Cat(self.shift_lo[1:], self.accum,
                                                self.accum[15] & self.inst[30]),
                                        ),
                                        # Decrement shift count
                                        self.shift_amt.eq(self.shift_amt - 1),

                                        # Don't leave this state.
                                        self.ustate.eq(UState.OPERATE_HI),
                                    ]
                                with m.Else():
                                    m.d.sync += [
                                        self.ustate.eq(UState.FINISH_SHIFT),
                                    ]
                            with m.Case(0b100): # XOR
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum ^
                                                                  imm_i[16:]),
                                ]
                            with m.Case(0b110): # OR
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum |
                                                                  imm_i[16:]),
                                ]
                            with m.Case(0b111): # AND
                                m.d.comb += [
                                    rf.write_cmd.payload.value.eq(self.accum &
                                                                  imm_i[16:]),
                                ]
                        m.d.comb += [
                            adder_rhs.eq(imm_i[16:]),

                            rf.write_cmd.valid.eq(1),
                        ]

            with m.Case(UState.BRANCH_LO):
                m.d.comb += [
                    adder_rhs.eq(imm_b[:16]),
                    adder_carry_in.eq(0),
                ]
                m.d.sync += [
                    self.shadow_pc.eq(adder_result),
                    saved_carry.eq(adder_carry_out),
                    self.accum.eq(self.pc[16:]),
                    self.ustate.eq(UState.BRANCH_HI)
                ]
            with m.Case(UState.BRANCH_HI):
                m.d.comb += [
                    adder_rhs.eq(imm_b[16:]),
                    adder_carry_in.eq(saved_carry),
                ]
                m.d.sync += [
                    self.pc.eq(Cat(self.shadow_pc, adder_result)),
                    self.ustate.eq(UState.FETCH_LO)
                ]

            with m.Case(UState.LOAD):
                m.d.comb += [
                    self.mem_in.ready.eq(1),

                    # Set up the write of the low halfword...
                    rf.write_cmd.payload.reg.eq(self.inst[7:12]),
                    rf.write_cmd.payload.value.eq(load_result),
                    # ...but only commit it if the data's actually here.
                    rf.write_cmd.valid.eq(self.mem_in.valid),
                ]

                with m.If(self.mem_in.valid):
                    with m.Switch(self.inst[12:15]):
                        with m.Case(0b000, 0b001): # LB, LH
                            m.d.sync += [
                                self.accum.eq(load_result[15].replicate(16)),
                                self.ustate.eq(UState.FILL_MSBS),
                            ]
                        with m.Case(0b010): # LW
                            # Address the next halfword
                            m.d.comb += [
                                self.mem_out.valid.eq(1),
                                self.mem_out.payload.addr.eq(
                                    Cat(self.mar_lo, adder_result)[1:] | 1,
                                ),
                            ]
                            with m.If(self.mem_out.ready):
                                m.d.sync += self.ustate.eq(UState.LOAD_HI)
                            with m.Else():
                                m.d.sync += self.ustate.eq(UState.LOAD_HI_WAIT)
                        with m.Case(0b100, 0b101): # LBU, LHU
                            m.d.sync += [
                                self.accum.eq(0),
                                self.ustate.eq(UState.FILL_MSBS),
                            ]

            with m.Case(UState.LOAD_HI_WAIT):
                # Address the next halfword
                m.d.comb += [
                    self.mem_out.valid.eq(1),
                    self.mem_out.payload.addr.eq(
                        Cat(self.mar_lo, adder_result)[1:] | 1,
                    ),
                ]
                with m.If(self.mem_out.ready):
                    m.d.sync += self.ustate.eq(UState.LOAD_HI_WAIT)
                with m.Else():
                    m.d.sync += self.ustate.eq(UState.LOAD_HI)

            with m.Case(UState.LOAD_HI):
                # Receive the high halfword from the bus.
                m.d.comb += [
                    self.mem_in.ready.eq(1),

                    # Set up the write of the high halfword...
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], 1)),
                    rf.write_cmd.payload.value.eq(load_result),
                    # ...but only commit it if the data's actually here.
                    rf.write_cmd.valid.eq(self.mem_in.valid),
                ]
                with m.If(self.mem_in.valid):
                    m.d.sync += [
                        self.pc.eq(self.pc + 4),
                        self.ustate.eq(UState.FETCH_LO),
                    ]

            with m.Case(UState.STORE_HI):
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
                        self.ustate.eq(UState.FETCH_LO),
                    ]

            with m.Case(UState.FILL_MSBS):
                m.d.comb += [
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], 1)),
                    rf.write_cmd.payload.value.eq(self.accum),
                    rf.write_cmd.valid.eq(1),
                ]
                m.d.sync += [
                    self.pc.eq(self.pc + 4),
                    self.ustate.eq(UState.FETCH_LO),
                ]

            with m.Case(UState.FINISH_SHIFT):
                m.d.comb += [
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], 0)),
                    rf.write_cmd.payload.value.eq(self.shift_lo),
                    rf.write_cmd.valid.eq(1),
                ]
                m.d.sync += [
                    self.pc.eq(self.pc + 4),
                    self.ustate.eq(UState.FETCH_LO),
                ]

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
                    m.d.sync += self.ustate.eq(UState.FETCH_LO)

        #

        return m
