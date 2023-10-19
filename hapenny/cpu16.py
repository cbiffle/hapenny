# An RV32I implementation using a 16-bit datapath to save space.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
import amaranth.lib.coding

from hapenny import StreamSig, AlwaysReady, csr16, mux, oneof, onehot_choice
from hapenny.decoder import ImmediateDecoder, Decoder, DecodeSignals
from hapenny.regfile16 import RegFile16
from hapenny.bus import BusPort
from hapenny.csr16 import CsrFile16, CsrReg

DebugPort = Signature({
    'reg_read': Out(StreamSig(7)),
    'reg_value': In(AlwaysReady(16)),
    'reg_write': Out(StreamSig(Signature({
        'addr': Out(7),
        'value': Out(16),
    }))),
    'pc': In(32),
    'pc_write': Out(StreamSig(32)),
    'ctx': Out(1),
    'ctx_write': In(StreamSig(1)),
})

class UState(Enum):
    RESET = 0 # TODO remove me
    FETCH = 1
    INST_REG1_LO = 2
    REG2 = 3
    OPERATE = 4
    BRANCH = 5
    LOAD = 6
    FILL_MSBS = 7
    STORE = 8
    SHIFT = 9
    FINISH_SHIFT = 10
    SLT = 11
    HALTED = 12
    CSR = 13

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

class Cpu(Component):
    halt_request: In(1)
    halted: Out(1)

    debug: In(DebugPort)

    def __init__(self, *,
                 addr_width = 32,
                 has_interrupt = None,
                 relax_instruction_alignment = False):
        super().__init__()

        self.bus = BusPort(addr = addr_width - 1, data = 16).create()

        self.has_interrupt = has_interrupt
        if has_interrupt is not None:
            assert has_interrupt in ['m', 'fast'], \
                    f"unknown value for has_interrupt: {has_interrupt}"
            self.irq = Signal(1)

        self.addr_width = addr_width
        self.relax_instruction_alignment = relax_instruction_alignment

        if self.has_interrupt == "fast":
            self.npcs = 2
            self.ctx = Signal(1)
        else:
            self.npcs = 1
            self.ctx = Signal(0)

        self.ustate = Signal(UState, reset = UState.RESET) # TODO
        self.hi = Signal(1)
        self.pc = Array(Signal(addr_width - 1) for _ in range(self.npcs))
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

        m.submodules.imm = imm = ImmediateDecoder()
        m.d.comb += imm.inst.eq(self.inst)

        m.submodules.dec = Decoder()
        m.d.comb += m.submodules.dec.inst.eq(self.inst)
        # Register the decode signals. This breaks the critical path through the
        # decoder to various control signals.
        dec = Signal(DecodeSignals)
        m.d.sync += dec.eq(m.submodules.dec.out)

        if self.has_interrupt == "m":
            m.submodules.mstatus = mstatus = CsrReg(
                name = "mstatus",
                width = 8,
                mask = 0b1000_1000,
            )
            m.submodules.mepc = mepc = CsrReg(
                name = "mepc",
                width = self.addr_width,
                mask = ((1 << self.addr_width) - 1) & ~1,
            )
            m.submodules.mtvec = mtvec = CsrReg(
                name = "mtvec",
                width = self.addr_width,
                reset = 4,
                mask = 0,
            )
            m.submodules.mscratch = mscratch = CsrReg(
                name = "mscratch",
                width = 32,
            )
            m.submodules.csr_file = csr_file = CsrFile16({
                0x300: mstatus.cmd,
                0x305: mtvec.cmd,
                0x340: mscratch.cmd,
                0x341: mepc.cmd,
            })

            irq_enable = mstatus.read[3]

        pc_plus_4 = Signal(self.addr_width - 1)
        m.d.comb += pc_plus_4.eq(self.pc[self.ctx] + 2)

        opcode = Signal(5)
        inst_rd = Signal(5)
        inst_rs1 = Signal(5)
        inst_rs2 = Signal(5)

        # Instruction decode strobes. These are all registered and are available
        # one cycle after the inst contents are latched.
        is_mret = Signal(1)
        is_flip = Signal(1)
        is_xpc = Signal(1)

        m.d.comb += [
            opcode.eq(self.inst[2:7]),
            inst_rd.eq(self.inst[7:12]),
            inst_rs1.eq(self.inst[15:20]),
            inst_rs2.eq(self.inst[20:25]),
        ]

        m.submodules.funct3_is = amaranth.lib.coding.Decoder(8)
        funct3_is = Signal(8)
        m.d.comb += [
            m.submodules.funct3_is.i.eq(self.inst[12:15]),
            funct3_is.eq(m.submodules.funct3_is.o),
        ]

        # Only implement decode logic for these instructions if necessary, to
        # save area in the baseline implementation.
        if self.has_interrupt == "m":
            m.d.sync += [
                is_mret.eq((opcode == Opcode.SYSTEM) & funct3_is[0b000]
                           & (self.inst[25:] == 0b0011000)),
            ]
        if self.has_interrupt == "fast":
            m.d.sync += [
                is_flip.eq((opcode == Opcode.CUSTOM0) & funct3_is[0b000]),
                is_xpc.eq((opcode == Opcode.CUSTOM0) & funct3_is[0b001]),
            ]

        inst_rd_ctx = Signal(1)
        inst_rs2_ctx = Signal(1)
        if self.has_interrupt == 'fast':
            m.d.comb += [
                inst_rd_ctx.eq((dec.is_alu_rr & self.inst[29]) ^ self.ctx),
                inst_rs2_ctx.eq((dec.is_alu_rr & self.inst[27]) ^ self.ctx),
            ]
        else:
            m.d.comb += inst_rd_ctx.eq(self.ctx)
            m.d.comb += inst_rs2_ctx.eq(self.ctx)


        m.d.comb += [
            self.halted.eq(self.ustate == UState.HALTED),
            self.debug.pc.eq(Cat(0, self.pc[self.ctx])),
            self.debug.ctx.eq(self.ctx),
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
        m.d.comb += adder_rhs.eq(oneof([
            (dec.is_auipc_or_lui, mux(
                self.hi,
                imm.u[16:],
                imm.u[:16],
            )),
            (dec.is_jal, mux(
                self.hi,
                imm.j[16:],
                imm.j[:16],
            )),
            (dec.is_neg_reg_to_adder, ~rf.read_resp),
            (dec.is_imm_i, mux(
                self.hi,
                imm.i[16:],
                imm.i[:16],
            )),
            (dec.is_store, mux(
                self.hi,
                imm.s[16:],
                imm.s[:16],
            )),
            (dec.is_reg_to_adder, rf.read_resp ^ self.inst[30].replicate(16)),
            (dec.is_neg_imm_i, mux(
                self.hi,
                ~imm.i[16:],
                ~imm.i[:16],
            )),
        ]))

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
                m.d.comb += branch_condition.eq(saved_zero)
            with m.Case(0b10):
                m.d.comb += branch_condition.eq(self.last_signed_less_than)
            with m.Case(0b11):
                m.d.comb += branch_condition.eq(self.last_unsigned_less_than)

        branch_taken = Signal(1)
        m.d.comb += branch_taken.eq(branch_condition ^ self.inst[12])

        load_result = Signal(16)
        with m.Switch(self.inst[12:15]):
            with m.Case(0b000): # LB
                m.d.comb += [
                    load_result[:8].eq(mux(
                        self.mar_lo[0],
                        self.bus.resp[8:],
                        self.bus.resp[:8],
                    )),
                    load_result[8:].eq(load_result[7].replicate(8)),
                ]
            with m.Case(0b100): # LBU
                m.d.comb += [
                    load_result[:8].eq(mux(
                        self.mar_lo[0],
                        self.bus.resp[8:],
                        self.bus.resp[:8],
                    )),
                    load_result[8:].eq(0),
                ]
            with m.Default(): # LH, LW, LHU
                m.d.comb += load_result.eq(self.bus.resp)
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
            m.d.comb += pc31_plus_hi.eq(self.pc[self.ctx] + self.hi)
        else:
            m.d.comb += pc31_plus_hi.eq(self.pc[self.ctx] | self.hi)

        if self.has_interrupt == "m":
            # CSR port driving
            m.d.comb += [
                # We DO NOT set valid here. That happens in OPERATE et al.
                csr_file.cmd.payload.write.eq(oneof([
                    # Immediate versions have inst14 set. We only pass the
                    # immediate through on the low phase.
                    (self.inst[14] & ~self.hi, inst_rs1),
                    # Register versions write the accumulator on both phases.
                    (~self.inst[14], self.accum),
                ])),
                csr_file.cmd.payload.high.eq(self.hi),
                csr_file.cmd.payload.addr.eq(imm.i),
                csr_file.cmd.payload.mode.eq(
                    mux(
                        # Write phase occurs for CSRRW(I) or any RS/RC
                        # instruction with a source that is not static
                        # zero (x0 or a 0 immediate).
                        funct3_is[0b001] | (inst_rs1 != 0),
                        self.inst[12:14], # write phase taken from inst
                        csr16.WriteMode.NONE,
                    ),
                ),
            ]

        with m.Switch(self.ustate):
            with m.Case(UState.RESET):
                m.d.sync += self.ustate.eq(UState.FETCH)

            # State in which we issue bus requests for the low and high halves
            # of the instruction, and latch the low half.
            with m.Case(UState.FETCH):
                m.d.comb += [
                    self.bus.cmd.payload.addr.eq(pc31_plus_hi),
                ]
                m.d.sync += saved_zero.eq(1)
                m.d.sync += saved_carry.eq(0)
                m.d.sync += self.inst[:16].eq(self.bus.resp)
                # This logic is a little subtle, but
                # - We flip this flag on the low half because bus.cmd.valid is
                #   fixed high.
                # - We flip this on the high half if an incoming transaction has
                #   appeared, because the latter gates our assertion of
                #   bus.cmd.valid.
                with m.If(self.bus.cmd.valid):
                    m.d.sync += self.hi.eq(~self.hi)

                with m.If(~self.hi):
                    # By default, we can generate a memory transaction
                    # unconditionally, because we know we have somewhere to put
                    # it.
                    m.d.comb += self.bus.cmd.valid.eq(1)
                    if self.has_interrupt == "m":
                        # Override this by handling an interrupt, if necessary.
                        with m.If(self.irq & irq_enable):
                            # Stop the fetch.
                            m.d.comb += self.bus.cmd.valid.eq(0)
                            m.d.comb += [
                                mstatus.update.valid.eq(1),
                                # Clear interrupt enable
                                mstatus.update.payload[3].eq(0),
                                # Transfer previous setting.
                                mstatus.update.payload[7].eq(mstatus.read[3]),

                                mepc.update.valid.eq(1),
                                # Save interrupted PC into EPC.
                                mepc.update.payload.eq(Cat(0, self.pc[self.ctx])),
                            ]
                            # Transfer state to enter interrupt.
                            m.d.sync += [
                                # Jump to the vector.
                                self.pc[self.ctx].eq(mtvec.read[1:]),

                                # Remain in FETCH state but repeat LOW phase.
                                self.hi.eq(0),
                            ]
                    if self.has_interrupt == "fast":
                        # Override this by handling an interrupt, if necessary.
                        with m.If(self.irq & self.ctx):
                            # Stop the fetch.
                            m.d.comb += self.bus.cmd.valid.eq(0)
                            m.d.sync += [
                                # Flip to the zero context.
                                self.ctx.eq(0),
                                # Remain in FETCH state but repeat LOW phase.
                                self.hi.eq(0),
                            ]
                    # Override this by honoring a halt request, if present.
                    with m.If(self.halt_request):
                        m.d.comb += self.bus.cmd.valid.eq(0)
                        m.d.sync += self.ustate.eq(UState.HALTED)
                with m.Else():
                    # Forward the fetch completion to the valid signal.
                    m.d.comb += self.bus.cmd.valid.eq(1)

                    # Move on to the high half.
                    m.d.sync += self.ustate.eq(UState.INST_REG1_LO)

            # Latching the high half of the instruction and starting the read
            # for rs1 low half.
            #
            # Only entered with hi==0
            with m.Case(UState.INST_REG1_LO):
                # Hi half fetch is completing!
                m.d.sync += self.inst[16:].eq(self.bus.resp)
                # Set up a read of the low half of the first operand
                # register, optimistically, using the not-yet-latched rs1
                # select bits coming in from the bus.
                m.d.comb += [
                    rf.read_cmd.valid.eq(1),
                    rf.read_cmd.payload.eq(
                        Cat(self.inst[15], self.bus.resp[0:4], self.ctx),
                    ),
                ]

                m.d.sync += self.ustate.eq(UState.REG2)

            # Latching half of rs1 and starting the read of the corresponding
            # half of rs2, with some overrides below.
            #
            # This preserves self.hi and proceeds to OPERATE on the same half.
            with m.Case(UState.REG2):
                # rs1 (hi/lo) is now available on the register file output
                # port. Latch it into the accumulator.
                with m.If(dec.is_auipc_or_jal):
                    # Load program counter instead.
                    m.d.sync += self.accum.eq(mux(
                        self.hi,
                        self.pc[self.ctx][15:],
                        Cat(0, self.pc[self.ctx][:15]),
                    ))
                with m.Elif(dec.is_lui):
                    m.d.sync += self.accum.eq(0)
                with m.Else():
                    m.d.sync += self.accum.eq(rf.read_resp)

                # Set up a read of the second operand register.
                # TODO: store actually reads the opposing half of the second
                # operand register, because reasons.
                m.d.comb += [
                    rf.read_cmd.valid.eq(1),
                    rf.read_cmd.payload.eq(Cat(
                        inst_rs2,
                        self.hi & ~dec.is_store,
                        inst_rs2_ctx,
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

                # Carry input:
                m.d.comb += adder_carry_in.eq(oneof([
                    (dec.is_b, saved_carry | ~self.hi),
                    (dec.is_alu_ri, mux(
                        funct3_is[0b010] | funct3_is[0b011], # SLTs
                        saved_carry | ~self.hi,
                        saved_carry,
                    )),
                    (dec.is_alu_rr, mux(
                        funct3_is[0b010] | funct3_is[0b011], # SLTs
                        saved_carry | ~self.hi,
                        mux(
                            self.hi,
                            saved_carry,
                            self.inst[30],
                        ),
                    )),
                    (dec.is_auipc_or_lui_or_jal | dec.is_load_or_jalr | dec.is_store | is_xpc,
                     saved_carry),
                ]))

                # Register write behavior

                # Instructions that write a register:
                m.d.comb += rf.write_cmd.valid.eq(
                    dec.is_auipc_or_lui_or_jal | dec.is_jalr | dec.is_alu
                    | (dec.is_csr & self.hi)  # CSR writes high side first
                    | is_xpc
                )
                # Register being written
                m.d.comb += rf.write_cmd.payload.reg.eq(Cat(inst_rd, self.hi, inst_rd_ctx))
                # Value written to a register:
                m.d.comb += rf.write_cmd.payload.value.eq(oneof([
                    (dec.is_auipc_or_lui, adder_result),
                    (dec.is_jal_or_jalr, mux(
                        self.hi,
                        pc_plus_4[15:],
                        Cat(0, pc_plus_4[:15]),
                    )),
                    (dec.is_alu, onehot_choice(funct3_is, {
                        0b000: adder_result,
                        0b010: 0, # important for SLT step
                        0b011: 0, # important for SLT step
                        0b100: self.accum ^ adder_rhs,
                        0b110: self.accum | adder_rhs,
                        0b111: self.accum & adder_rhs,
                    })),
                    (dec.is_csr,
                     csr_file.cmd.payload.read \
                             if self.has_interrupt == "m" else 0),
                    (is_xpc, mux(
                        self.hi,
                        self.pc[~self.ctx][15:],
                        self.pc[~self.ctx][:15],
                    )),
                ]))

                # Register read for next cycle

                m.d.comb += rf.read_cmd.valid.eq(oneof([
                    (dec.is_jalr | dec.is_b | dec.is_alu | dec.is_store | is_xpc, 1),
                    (dec.is_load, ~self.hi),
                ]))
                m.d.comb += rf.read_cmd.payload.eq(mux(
                    dec.is_store & self.hi,
                    Cat(inst_rs2, 1, inst_rs2_ctx),
                    mux(
                        dec.is_alu,
                        Cat(inst_rs1, ~self.hi, self.ctx),
                        Cat(inst_rs1, 1, self.ctx),
                    ),
                ))

                # Memory transaction generation

                m.d.comb += self.bus.cmd.payload.addr.eq(
                    Cat(self.mar_lo, adder_result)[1:]
                )
                m.d.comb += self.bus.cmd.payload.data.eq(store_data)
                m.d.comb += self.bus.cmd.payload.lanes.eq(oneof([
                    (dec.is_store & self.hi, store_mask),
                ]))
                m.d.comb += self.bus.cmd.valid.eq(oneof([
                    (dec.is_load | dec.is_store, self.hi),
                ]))

                if self.has_interrupt == "m":
                    # CSR port driving
                    m.d.comb += csr_file.cmd.valid.eq(dec.is_csr)

                # Assorted register update rules

                m.d.sync += self.accum.eq(oneof([
                    (dec.is_auipc_or_jal, self.pc[self.ctx][15:]),
                    (dec.is_lui, 0),
                    (dec.is_b, Cat(0, self.pc[self.ctx][:15])),
                    (dec.is_alu, self.accum),
                ]))

                m.d.sync += [
                    self.shadow_pc.eq(adder_result[1:]),

                    saved_carry.eq(adder_carry_out),
                    saved_zero.eq(zero_out),
                    self.last_unsigned_less_than.eq(unsigned_less_than),
                    self.last_signed_less_than.eq(signed_less_than),
                ]

                if self.has_interrupt == 'fast':
                    with m.If(is_xpc & self.hi):
                        m.d.sync += self.pc[~self.ctx].eq(
                            Cat(self.shadow_pc, adder_result)
                        )

                # Updates that only occur during hi phase:
                with m.If(self.hi):
                    m.d.sync += self.pc[self.ctx].eq(oneof([
                        (dec.is_auipc_or_lui | dec.is_alu | is_xpc | is_flip, pc_plus_4),
                        (dec.is_jal_or_jalr, Cat(self.shadow_pc, adder_result)),
                        (dec.is_store, mux(
                            funct3_is[0b010],
                            self.pc[self.ctx],
                            pc_plus_4,
                        )),
                        (dec.is_load | dec.is_b | dec.is_csr, self.pc[self.ctx]),
                    ]))
                    m.d.sync += self.ctx.eq(self.ctx ^ is_flip)

                # Updates that only occur during low phase:
                with m.If(~self.hi):
                    m.d.sync += self.mar_lo.eq(adder_result)
                    m.d.sync += self.shift_amt.eq(oneof([
                        (dec.is_alu_ri, inst_rs2),
                        (dec.is_alu_rr, rf.read_resp[:5]),
                    ]))
                    m.d.sync += self.shift_lo.eq(self.accum)

                # State transition logic

                m.d.sync += self.ustate.eq(mux(
                    ~self.hi,
                    oneof([
                        (dec.is_auipc_or_lui_or_jal | is_xpc | is_flip, UState.OPERATE),
                        (dec.is_b | dec.is_load_or_jalr | dec.is_store |
                         dec.is_alu | dec.is_csr |
                         is_mret, UState.REG2),
                    ]),
                    oneof([
                        (dec.is_auipc_or_lui_or_jal | dec.is_jalr | is_mret, UState.FETCH),
                        (dec.is_b, UState.BRANCH),
                        (dec.is_csr, UState.CSR),
                        (dec.is_load, UState.LOAD),
                        (dec.is_store, mux(
                            funct3_is[0b010],
                            UState.STORE,
                            UState.FETCH,
                        )),
                        (dec.is_alu, onehot_choice(funct3_is, {
                            0b000: UState.FETCH,
                            0b001: UState.SHIFT,
                            0b010: UState.SLT,
                            0b011: UState.SLT,
                            0b100: UState.FETCH,
                            0b101: UState.SHIFT,
                            0b110: UState.FETCH,
                            0b111: UState.FETCH,
                        })),
                        (is_flip | is_xpc, UState.FETCH),
                    ]),
                ))
                m.d.sync += self.hi.eq(mux(
                    dec.is_alu & self.hi & (funct3_is[0b001] | funct3_is[0b101]),
                    self.hi,
                    ~self.hi,
                ))

                if self.has_interrupt == "m":
                    # Implement MRET.
                    with m.If(is_mret & self.hi):
                        m.d.comb += [
                            mstatus.update.valid.eq(1),
                            mstatus.update.payload[3].eq(
                                mstatus.read[7],
                            ),
                            mstatus.update.payload[7].eq(1),

                        ]
                        m.d.sync += self.pc[self.ctx].eq(mepc.read[1:])

            with m.Case(UState.BRANCH):
                m.d.comb += [
                    adder_rhs.eq(mux(
                        self.hi,
                        imm.b[16:],
                        imm.b[:16],
                    )),
                    adder_carry_in.eq(saved_carry & self.hi),
                ]
                m.d.sync += [
                    saved_carry.eq(adder_carry_out),
                    self.shadow_pc.eq(adder_result[1:]),
                ]
                with m.If(~branch_taken):
                    m.d.sync += [
                        self.pc[self.ctx].eq(pc_plus_4),
                        self.ustate.eq(UState.FETCH),
                    ]
                with m.Else():
                    m.d.sync += [
                        self.hi.eq(~self.hi),
                    ]
                    with m.If(self.hi):
                        m.d.sync += [
                            self.pc[self.ctx].eq(Cat(self.shadow_pc, adder_result)),
                            self.ustate.eq(UState.FETCH)
                        ]
                    with m.Else():
                        m.d.sync += [
                            self.accum.eq(self.pc[self.ctx][15:]),
                            self.ustate.eq(UState.BRANCH)
                        ]

            with m.Case(UState.LOAD):
                m.d.comb += [
                    # Set up the write of the halfword...
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], self.hi)),
                    rf.write_cmd.payload.value.eq(load_result),
                    rf.write_cmd.valid.eq(1),
                    self.bus.cmd.payload.addr.eq(
                        Cat(self.mar_lo, adder_result)[1:] | 1,
                    ),
                ]

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
                            m.d.comb += self.bus.cmd.valid.eq(1)
                            m.d.sync += self.ustate.eq(UState.LOAD)
                        with m.Case(0b100, 0b101): # LBU, LHU
                            m.d.sync += self.ustate.eq(UState.FILL_MSBS)
                with m.Else():
                    m.d.sync += [
                        self.pc[self.ctx].eq(pc_plus_4),
                        self.ustate.eq(UState.FETCH),
                    ]

            with m.Case(UState.STORE):
                # Write the high half of rs2 (which has just arrived on the
                # register file output port) to memory.
                m.d.comb += [
                    self.bus.cmd.valid.eq(1),
                    self.bus.cmd.payload.addr.eq(
                        Cat(self.mar_lo, adder_result)[1:] | 1,
                    ),
                    self.bus.cmd.payload.data.eq(store_data),
                    self.bus.cmd.payload.lanes.eq(store_mask),
                ]
                m.d.sync += [
                    self.pc[self.ctx].eq(pc_plus_4),
                    self.ustate.eq(UState.FETCH),
                ]

            with m.Case(UState.FILL_MSBS):
                m.d.comb += [
                    rf.write_cmd.payload.reg.eq(Cat(self.inst[7:12], 1)),
                    rf.write_cmd.payload.value.eq(self.accum),
                    rf.write_cmd.valid.eq(1),
                ]
                m.d.sync += [
                    self.pc[self.ctx].eq(pc_plus_4),
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
                    self.pc[self.ctx].eq(pc_plus_4),
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

            if self.has_interrupt == "m":
                with m.Case(UState.CSR):
                    m.d.comb += [
                        # note: self.hi is always low here
                        rf.write_cmd.payload.reg.eq(Cat(inst_rd, self.hi, inst_rd_ctx)),
                        rf.write_cmd.payload.value.eq(
                            csr_file.cmd.payload.read,
                        ),
                        rf.write_cmd.valid.eq(1),

                        csr_file.cmd.valid.eq(dec.is_csr),
                    ]
                    m.d.sync += [
                        self.pc[self.ctx].eq(pc_plus_4),
                        self.ustate.eq(UState.FETCH),
                    ]

            with m.Case(UState.HALTED):
                with m.If(self.debug.pc_write.valid):
                    m.d.sync += self.pc[self.ctx].eq(self.debug.pc_write.payload[1:])
                with m.If(self.debug.ctx_write.valid):
                    m.d.sync += self.ctx.eq(self.debug.ctx_write.payload)

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
