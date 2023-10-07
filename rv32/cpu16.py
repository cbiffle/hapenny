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
        if isinstance(result, Enum):
            result = result.value
        if isinstance(result, int):
            result = Const(result)

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
        if isinstance(result, Enum):
            result = result.value
        if isinstance(result, int):
            result = Const(result)
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
        self.pc = Signal(addr_width - 1)
        self.inst = Signal(32)

        self.accum = Signal(16)
        self.shadow_pc = Signal(15)
        self.mar_lo = Signal(16)
        self.shift_lo = Signal(16)
        self.shift_amt = Signal(5)
        self.last_unsigned_less_than = Signal(1)
        self.last_signed_less_than = Signal(1)

    def elaborate(self, platform):
        m = Module()

        m.submodules.regfile = rf = RegFile16()

        pc_plus_4 = Signal(self.addr_width - 1)
        m.d.comb += pc_plus_4.eq(self.pc + 2)

        opcode = Signal(5)
        inst_rd = Signal(5)
        is_auipc = Signal(1)
        is_lui = Signal(1)
        is_jal = Signal(1)
        is_jalr = Signal(1)
        is_b = Signal(1)
        is_load = Signal(1)
        is_store = Signal(1)
        is_alu_rr = Signal(1)
        is_alu_ri = Signal(1)
        m.d.comb += [
            opcode.eq(self.inst[2:7]),
            inst_rd.eq(self.inst[7:12]),
        ]
        m.d.sync += [
            is_auipc.eq(opcode == 0b00101),
            is_lui.eq(opcode == 0b01101),
            is_jal.eq(opcode == 0b11011),
            is_store.eq(opcode == 0b01000),
            is_jalr.eq(opcode == 0b11001),
            is_b.eq(opcode == 0b11000),
            is_load.eq(opcode == 0b00000),
            is_alu_rr.eq(opcode == 0b01100),
            is_alu_ri.eq(opcode == 0b00100),
        ]

        m.submodules.funct3_is = Decoder(8)
        funct3_is = Signal(8)
        m.d.comb += [
            m.submodules.funct3_is.i.eq(self.inst[12:15]),
            funct3_is.eq(m.submodules.funct3_is.o),
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
            self.debug.pc.eq(Cat(0, self.pc)),
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
            with m.Case(0b100): # LBU
                m.d.comb += [
                    load_result[:8].eq(mux(
                        self.mar_lo[0],
                        self.mem_in.payload[8:],
                        self.mem_in.payload[:8],
                    )),
                    load_result[8:].eq(0),
                ]
            with m.Default(): # LH, LW, LHU
                m.d.comb += load_result.eq(self.mem_in.payload)
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

        pc31_plus_hi = Signal(self.addr_width - 1)
        if self.relax_instruction_alignment:
            # have to use an adder here as we may go from 2-mod-4 to 0-mod-4
            m.d.comb += pc31_plus_hi.eq(self.pc + self.hi)
        else:
            m.d.comb += pc31_plus_hi.eq(self.pc | self.hi)

        with m.Switch(self.ustate):
            with m.Case(UState.RESET):
                m.d.sync += self.ustate.eq(UState.FETCH)

            # State in which we issue bus requests for the low and high halves
            # of the instruction, and latch the low half.
            with m.Case(UState.FETCH):
                m.d.comb += [
                    self.mem_out.payload.addr.eq(pc31_plus_hi),

                    self.mem_in.ready.eq(self.hi),
                ]
                m.d.sync += saved_zero.eq(1)
                m.d.sync += saved_carry.eq(0)
                m.d.sync += self.inst[:16].eq(self.mem_in.payload)
                # This logic is a little subtle, but
                # - We flip this flag on the low half if the bus is ready
                #   because mem_out.valid is fixed high.
                # - We flip this on the high half if the bus is ready and an
                #   incoming transaction has appeared, because the latter gates our
                #   assertion of mem_out.valid.
                with m.If(self.mem_out.valid & self.mem_out.ready):
                    m.d.sync += self.hi.eq(~self.hi)

                with m.If(~self.hi):
                    # During the FETCH+lo state we will honor a halt request,
                    # if present.
                    with m.If(self.halt_request):
                        m.d.sync += self.ustate.eq(UState.HALTED)
                    with m.Else():
                        # Otherwise, we can generate a memory transaction
                        # unconditionally, because we know we have somewhere to
                        # put it.
                        m.d.comb += self.mem_out.valid.eq(1)
                with m.Else():
                    # Forward the fetch completion to the valid signal.
                    m.d.comb += self.mem_out.valid.eq(self.mem_in.valid)

                    # If the low half of the fetch has completed...
                    with m.If(self.mem_in.valid):
                        # And the bus is ready to accept our request for the
                        # high half...
                        with m.If(self.mem_out.ready):
                            # ...then we can move on
                            m.d.sync += self.ustate.eq(UState.INST_REG1_LO)
                        with m.Else():
                            # fetch-lo completed but fetch-hi has not been able
                            # to issue. We've written the result to inst[:16]
                            # above but can't transition to INST_REG1_LO until
                            # we issue fetch-hi.
                            m.d.sync += self.ustate.eq(UState.FETCH_HI_WAIT)

            # State entered when we were able to complete the low half of a
            # fetch, but were not able to issue the high half.
            #
            # Only entered with hi==1
            with m.Case(UState.FETCH_HI_WAIT):
                # Keep trying to issue the high half
                m.d.comb += [
                    self.mem_out.payload.addr.eq(pc31_plus_hi),
                    self.mem_out.valid.eq(1),
                ]

                with m.If(self.mem_out.ready):
                    # fetch-hi has issued, we can move on to...
                    m.d.sync += self.ustate.eq(UState.INST_REG1_LO)
                    m.d.sync += self.hi.eq(~self.hi)

            # Latching the high half of the instruction and starting the read
            # for rs1 low half.
            #
            # Only entered with hi==0
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

            # Latching half of rs1 and starting the read of the corresponding
            # half of rs2, with some overrides below.
            #
            # This preserves self.hi and proceeds to OPERATE on the same half.
            with m.Case(UState.REG2):
                # rs1 (hi/lo) is now available on the register file output
                # port. Latch it into the accumulator.
                with m.If(is_auipc | is_jal):
                    # Load program counter instead.
                    m.d.sync += self.accum.eq(mux(
                        self.hi,
                        self.pc[15:],
                        Cat(0, self.pc[:15]),
                    ))
                with m.Elif(is_lui):
                    m.d.sync += self.accum.eq(0)
                with m.Else():
                    m.d.sync += self.accum.eq(rf.read_resp)

                # Set up a read of the second operand register.
                # TODO: store actually reads the opposing half of the second
                # operand register, because reasons.
                m.d.comb += [
                    rf.read_cmd.valid.eq(1),
                    rf.read_cmd.payload.eq(Cat(
                        self.inst[20:25],
                        self.hi & ~is_store,
                    )),
                ]

                # We never have to stall in this state.
                m.d.sync += self.ustate.eq(UState.OPERATE)

            # Both operand halves are available and we can do some stuff.
            #
            # This generally toggles self.hi but there is an override below.
            with m.Case(UState.OPERATE):
                # This case is a giant box of assignments instead of a switch
                # because it saves significant area.

                # Adder configuration

                # Value used on right hand side:
                m.d.comb += adder_rhs.eq(oneof([
                    (is_auipc | is_lui, mux(
                        self.hi,
                        imm_u[16:],
                        imm_u[:16],
                    )),
                    (is_jal, mux(
                        self.hi,
                        imm_j[16:],
                        imm_j[:16],
                    )),
                    (is_b, ~rf.read_resp),
                    (is_load | is_jalr, mux(
                        self.hi,
                        imm_i[16:],
                        imm_i[:16],
                    )),
                    (is_store, mux(
                        self.hi,
                        imm_s[16:],
                        imm_s[:16],
                    )),
                    (is_alu_rr, mux(
                        funct3_is[0b010] | funct3_is[0b011], # SLTs
                        ~rf.read_resp,
                        rf.read_resp ^ self.inst[30].replicate(16),
                    )),
                    (is_alu_ri, mux(
                        funct3_is[0b010] | funct3_is[0b011], # SLTs
                        mux(
                            self.hi,
                            ~imm_i[16:],
                            ~imm_i[:16],
                        ),
                        mux(
                            self.hi,
                            imm_i[16:],
                            imm_i[:16],
                        ),
                    )),
                ]))
                m.d.comb += adder_carry_in.eq(oneof([
                    (is_b, saved_carry | ~self.hi),
                    (is_alu_ri, mux(
                        funct3_is[0b010] | funct3_is[0b011], # SLTs
                        saved_carry | ~self.hi,
                        saved_carry,
                    )),
                    (is_alu_rr, mux(
                        funct3_is[0b010] | funct3_is[0b011], # SLTs
                        saved_carry | ~self.hi,
                        mux(
                            self.hi,
                            saved_carry,
                            self.inst[30],
                        ),
                    )),
                    (is_auipc | is_lui | is_jal | is_jalr | is_load | is_store,
                     saved_carry),
                ]))

                # Register write behavior

                # Instructions that write a register do so unconditionally:
                m.d.comb += rf.write_cmd.valid.eq(
                    is_auipc | is_lui | is_jal | is_jalr | is_alu_rr | is_alu_ri
                )
                # Register being written
                m.d.comb += rf.write_cmd.payload.reg.eq(Cat(inst_rd, self.hi))
                # Value written to a register:
                m.d.comb += rf.write_cmd.payload.value.eq(oneof([
                    (is_auipc | is_lui, adder_result),
                    (is_jal | is_jalr, mux(
                        self.hi,
                        pc_plus_4[15:],
                        Cat(0, pc_plus_4[:15]),
                    )),
                    (is_alu_rr | is_alu_ri, onehot_choice(funct3_is, {
                        0b000: adder_result,
                        0b010: 0, # important for SLT step
                        0b011: 0, # important for SLT step
                        0b100: self.accum ^ adder_rhs,
                        0b110: self.accum | adder_rhs,
                        0b111: self.accum & adder_rhs,
                    })),
                ]))

                # Register read for next cycle

                m.d.comb += rf.read_cmd.valid.eq(oneof([
                    (is_jalr | is_b | is_alu_rr | is_alu_ri | is_store, 1),
                    (is_load, ~self.hi),
                ]))
                m.d.comb += rf.read_cmd.payload.eq(mux(
                    is_store & self.hi,
                    Cat(self.inst[20:25], 1),
                    mux(
                        is_alu_rr | is_alu_ri,
                        Cat(self.inst[15:20], ~self.hi),
                        Cat(self.inst[15:20], 1),
                    ),
                ))

                # Memory transaction generation

                m.d.comb += self.mem_out.payload.addr.eq(
                    Cat(self.mar_lo, adder_result)[1:]
                )
                m.d.comb += self.mem_out.payload.data.eq(store_data)
                m.d.comb += self.mem_out.payload.lanes.eq(oneof([
                    (is_store & self.hi, store_mask),
                ]))
                m.d.comb += self.mem_out.valid.eq(oneof([
                    (is_load | is_store, self.hi),
                ]))

                # Assorted register update rules

                m.d.sync += self.accum.eq(oneof([
                    (is_auipc | is_jal, self.pc[15:]),
                    (is_lui, 0),
                    (is_b, Cat(0, self.pc[:15])),
                    (is_alu_ri | is_alu_rr, self.accum),
                ]))

                m.d.sync += [
                    self.shadow_pc.eq(adder_result[1:]),

                    saved_carry.eq(adder_carry_out),
                    saved_zero.eq(zero_out),
                    self.last_unsigned_less_than.eq(unsigned_less_than),
                    self.last_signed_less_than.eq(signed_less_than),
                ]

                # Updates that only occur during hi phase:
                with m.If(self.hi):
                    m.d.sync += self.pc.eq(oneof([
                        (is_auipc | is_lui, pc_plus_4),
                        (is_jal | is_jalr, Cat(self.shadow_pc, adder_result)),
                        (is_b, mux(
                            branch_taken,
                            self.pc,
                            pc_plus_4,
                        )),
                        (is_store, mux(
                            self.mem_out.ready & funct3_is[0b010],
                            self.pc,
                            pc_plus_4,
                        )),
                        (is_load | is_alu_rr | is_alu_ri, self.pc),
                    ]))

                # Updates that only occur during low phase:
                with m.If(~self.hi):
                    m.d.sync += self.mar_lo.eq(adder_result)
                    m.d.sync += self.shift_amt.eq(oneof([
                        (is_alu_ri, self.inst[20:25]),
                        (is_alu_rr, rf.read_resp[:5]),
                    ]))
                    m.d.sync += self.shift_lo.eq(self.accum)

                # State transition logic

                m.d.sync += self.ustate.eq(mux(
                    ~self.hi,
                    oneof([
                        (is_auipc | is_lui | is_jal, UState.OPERATE),
                        (is_jalr | is_b | is_load | is_store | is_alu_ri | is_alu_rr, UState.REG2),
                    ]),
                    oneof([
                        (is_auipc | is_lui | is_jal | is_jalr, UState.FETCH),
                        (is_b, mux(
                            branch_taken,
                            UState.BRANCH,
                            UState.FETCH,
                        )),
                        (is_load, mux(
                            self.mem_out.ready,
                            UState.LOAD,
                            self.ustate,
                        )),
                        (is_store, mux(
                            self.mem_out.ready,
                            mux(
                                funct3_is[0b010],
                                UState.STORE,
                                UState.FETCH,
                            ),
                            self.ustate,
                        )),
                        (is_alu_ri | is_alu_rr, onehot_choice(funct3_is, {
                            0b000: UState.FETCH,
                            0b001: UState.SHIFT,
                            0b010: UState.SLT,
                            0b011: UState.SLT,
                            0b100: UState.FETCH,
                            0b101: UState.SHIFT,
                            0b110: UState.FETCH,
                            0b111: UState.FETCH,
                        })),
                    ]),
                ))
                m.d.sync += self.hi.eq(mux(
                    (is_alu_rr | is_alu_ri) & self.hi & (funct3_is[0b001] | funct3_is[0b101]),
                    self.hi,
                    ~self.hi,
                ))


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
                        self.shadow_pc.eq(adder_result[1:]),
                        self.accum.eq(self.pc[15:]),
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
                    m.d.sync += [
                        self.hi.eq(~self.hi),

                        self.accum.eq(mux(
                            funct3_is[0b000] | funct3_is[0b001],
                            load_result[15].replicate(16),
                            0,
                        )),
                    ]

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
                            self.pc.eq(pc_plus_4),
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
                        self.pc.eq(pc_plus_4),
                        self.ustate.eq(UState.FETCH),
                    ]

            with m.Case(UState.FILL_MSBS):
                m.d.comb += [
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], 1)),
                    rf.write_cmd.payload.value.eq(self.accum),
                    rf.write_cmd.valid.eq(1),
                ]
                m.d.sync += [
                    self.pc.eq(pc_plus_4),
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
                    m.d.comb += rf.write_cmd.valid.eq(1)
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
                    self.pc.eq(pc_plus_4),
                    self.ustate.eq(UState.FETCH),
                ]

            with m.Case(UState.SLT):
                m.d.comb += [
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], self.hi)),
                    rf.write_cmd.payload.value.eq(
                        mux(
                            self.inst[12],
                            self.last_unsigned_less_than,
                            self.last_signed_less_than,
                        ),
                    ),
                    rf.write_cmd.valid.eq(1),
                ]
                m.d.sync += self.ustate.eq(UState.FETCH)

            with m.Case(UState.HALTED):
                with m.If(self.debug.pc_write.valid):
                    m.d.sync += self.pc.eq(self.debug.pc_write.payload[1:])

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
