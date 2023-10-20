# The EW-Box, responsible for execute and writeback.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *

from hapenny import StreamSig, AlwaysReady, onehot_choice, oneof, mux, hihalf, lohalf, choosehalf
from hapenny.sbox import STATE_COUNT
from hapenny.regfile16 import RegFile16, RegWrite
from hapenny.bus import BusPort
from hapenny.decoder import Decoder, ImmediateDecoder, DecodeSignals

class EWBox(Component):
    """The EW-Box does execute and writeback.

    Parameters
    ----------
    prog_addr_width (integer): number of bits in a program address, 32 by default
        but can be shrunk to save logic.

    Attributes
    ----------
    onehot_state (input): state input from the S-Box
    rf_read_cmd (output): read command to the register file, intended to be OR'd.
    rf_resp (input): value most recently read from the register file.
    inst_next (input): instruction word from FD.
    bus (port): our connection to the memory fabric.
    pc_next (output): PC to FD for fetch of next instruction. We need to provide
        a useful next-PC value here by the end of state 1 (ideally, earlier) to
        get a useful fetch done. We then need to hold that value through states
        1 and 2.
    ctx_next (output): Context select to FD for fetch of next rs1
    from_the_top (output): signals S-Box to rewind state when asserted.
    rf_write_cmd (port): output to register file to write registers.
    full (output): indicates whether we're doing useful work. When we have a
        real instruction from FD, and we're honoring it (i.e. it is not a
        speculative fetch that became irrelevant due to a change in control
        flow), we set this. It is clear at reset to create an initial pipeline
        bubble until the first fetch completes, and we clear it on instruction
        stream changes.
    pc (output): Current contents of PC register, mostly for debug port.
    debug_pc_write (port): while halted, can overwrite the PC.
    hold (output): Indicates that we're going to repeat this state, signals
        s-box to maintain it.
    """
    onehot_state: In(STATE_COUNT)
    rf_read_cmd: Out(AlwaysReady(7))
    rf_resp: In(16)
    inst_next: In(32)
    ctx_next: Out(1)
    from_the_top: Out(1)
    rf_write_cmd: Out(AlwaysReady(RegWrite))
    full: Out(1)
    hold: Out(1)

    debug_pc_write: In(StreamSig(32))

    def __init__(self, *,
                 prog_addr_width = 32,
                 ):
        super().__init__()

        # Create a bus port of sufficient width to fetch instructions only.
        # (Width is -1 because we're addressing halfwords.)
        self.bus = BusPort(addr = prog_addr_width - 1, data = 16).create()

        # The PC width is -2 because it's addressing words.
        self.pc_next = Signal(prog_addr_width - 2)

        self.accum = Signal(16)
        self.pc = Signal(prog_addr_width - 2)

    def elaborate(self, platform):
        m = Module()

        # Nested decoder bits

        m.submodules.dec = Decoder()
        m.submodules.imm = imm = ImmediateDecoder()
        dec = Signal(DecodeSignals)
        m.d.comb += [
            m.submodules.dec.inst.eq(self.inst_next),
            imm.inst.eq(dec.inst),
        ]

        # The adder
        saved_carry = Signal(1) # register
        saved_zero = Signal(1) # register

        adder_rhs_pos = Signal(16)
        adder_rhs = Signal(16)

        adder_carry_out = Signal(1)
        adder_result = Signal(16)
        zero_out = Signal(1)

        m.d.comb += [
            # Adder RHS operand selection:
            adder_rhs_pos.eq(onehot_choice(self.onehot_state, {
                (1, 3): oneof([
                    (dec.is_auipc_or_lui, choosehalf(self.onehot_state[3],
                                                         imm.u)),
                    (dec.is_jal, choosehalf(self.onehot_state[3], imm.j)),
                    (dec.is_any_imm_i, choosehalf(self.onehot_state[3], imm.i)),
                    (dec.is_any_reg_to_adder, self.rf_resp),
                    (dec.is_store, choosehalf(self.onehot_state[3], imm.s)),
                ]),
                # In states 4/5 we only use the adder to branch.
                4: lohalf(imm.b),
                5: hihalf(imm.b),
            })),
            adder_rhs.eq(mux(
                dec.is_adder_rhs_complemented & (self.onehot_state[1] | self.onehot_state[3]),
                ~adder_rhs_pos,
                adder_rhs_pos,
            )),
            # Adder implementation:
            Cat(adder_result, adder_carry_out).eq(
                self.accum + adder_rhs + saved_carry,
            ),
            zero_out.eq(saved_zero & (adder_result == 0)),
        ]

        # Comparator
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

        # Branch comparator condition select
        # We delay the comparator signals by one cycle to cut the critical path
        # through the carry chain.
        zero_out_d = Signal(1)
        signed_less_than_d = Signal(1)
        unsigned_less_than_d = Signal(1)
        m.d.sync += [
            zero_out_d.eq(zero_out),
            signed_less_than_d.eq(signed_less_than),
            unsigned_less_than_d.eq(unsigned_less_than),
        ]
        # Thus, this indicates whether a branch condition would have succeeded
        # using the adder result from _last cycle._
        branch_taken_d = Signal(1)
        m.d.comb += branch_taken_d.eq(onehot_choice(dec.funct3_is, {
            (0b000, 0b001): zero_out_d,
            (0b100, 0b101): signed_less_than_d,
            (0b110, 0b111): unsigned_less_than_d,
        }) ^ dec.funct3[0])

        # Shifter state
        shift_amt = Signal(5)
        shift_lo = Signal(16)

        m.d.comb += self.hold.eq(
            dec.is_shift & self.onehot_state[4] & (shift_amt != 0)
        )

        m.d.sync += [
            shift_amt.eq(onehot_choice(self.onehot_state, {
                1: mux(
                    dec.is_alu_ri,
                    dec.rs2,
                    self.rf_resp[:5],
                ),
                4: shift_amt - 1,
            }, default = shift_amt)),
            shift_lo.eq(oneof([
                (self.onehot_state[1], self.accum),
                (self.onehot_state[4] & (shift_amt != 0), mux(
                     dec.funct3[2],
                     Cat(shift_lo[1:], self.accum[0]), # right shift
                     Cat(0, shift_lo), # left shift
                 )),
            ], default = shift_lo)),
        ]

        # Effective address generation and bus transactions
        # mar_lo is used for loads/stores, but also as the low half of the new
        # PC value during a computed branch.
        mar_lo = Signal(16)

        m.d.comb += [
            self.bus.cmd.valid.eq(
                self.full & (
                    # Both loads and stores generate traffic in state 3.
                    ((dec.is_load | dec.is_store) & self.onehot_state[3])
                    # Stores unconditionally generate traffic in state 4.
                    | (dec.is_store & self.onehot_state[4])
                    # Loads only generate traffic in state 4 if we're loading a
                    # word.
                    | (dec.is_load & self.onehot_state[4] & dec.funct3_is[2])
                )
            ),
            self.bus.cmd.payload.addr.eq(mux(
                self.onehot_state[1] | self.onehot_state[2],
                0,
                Cat(mar_lo[1:] | self.onehot_state[4], adder_result),
            )),
            self.bus.cmd.payload.lanes.eq(onehot_choice(self.onehot_state, {
                (3, 4): mux(
                    self.full & dec.is_store,
                    mux(
                        dec.funct3_is[0b000], 
                        Cat(~mar_lo[0], mar_lo[0]),
                        0b11,
                    ),
                    0,
                ),
            })),
            self.bus.cmd.payload.data.eq(onehot_choice(self.onehot_state, {
                (3, 4): mux(
                    dec.funct3_is[0b000],
                    self.rf_resp[:8].replicate(2),
                    self.rf_resp,
                ),
            })),
        ]
        m.d.sync += mar_lo.eq(mux(
            self.onehot_state[1] | self.onehot_state[4],
            adder_result,
            mar_lo,
        ))

        # Program Counter management
        end_of_instruction = Signal(1)
        start_bubble = Signal(1)
        pc_inc = Signal(self.pc.shape().width)
        m.d.comb += [
            # Dedicated program counter incrementer.
            pc_inc.eq(self.pc + 1),
            # Address we send to FD / load into PC
            self.pc_next.eq(mux(
                self.full,
                # When full, the PC we send is either...
                mux(
                    # ... when taking a branch,
                    dec.is_jal_or_jalr
                        | (dec.is_b & self.onehot_state[5]),
                    # the computed PC,
                    Cat(mar_lo[2:], adder_result),
                    # Otherwise, PC+1
                    pc_inc,
                ),
                # When not full, send current PC to fetch current instruction
                # rather than next.
                self.pc,
            )),
        ]
        m.d.sync += [
            self.pc.eq(oneof([
                (self.onehot_state[STATE_COUNT - 1] & self.debug_pc_write.valid,
                 self.debug_pc_write.payload[2:]),
                (end_of_instruction & self.full, self.pc_next),
            ], default = self.pc)),
            self.full.eq(oneof([
                # If halted, we are not full, to ensure a fetch and refill when
                # we resume.
                (self.onehot_state[STATE_COUNT - 1], 0),
                # Otherwise, we become empty only at the end of an instruction
                # when a bubble is required.
                (end_of_instruction, ~start_bubble),
            ], default = self.full)),
        ]

        # Load lane mixer
        load_result = Signal(16)
        m.d.comb += [
            # Note: expressing both of these in the same assignment generates a
            # cycle that produces invalid RTL and a warning, but still completes
            # "successfully." This is a notable difference from Verilog where it
            # would be legal.

            # Bottom byte of load data: select high byte of bus response for
            # byte loads aligned 1-mod-2.
            load_result[:8].eq(oneof([
                # LB/LBU (bottom byte, signedness doesn't matter)
                (dec.funct3_is[0b000] | dec.funct3_is[0b100],
                 mux(mar_lo[0], self.bus.resp[8:], self.bus.resp[:8])),
                # LW/LH/LHU
                (~dec.funct3_is[0b000] & ~dec.funct3_is[0b100],
                 self.bus.resp[:8]),
            ])),
            # High byte of load data: extend byte if required.
            load_result[8:].eq(oneof([
                # LB (signed): sign extend
                (dec.funct3_is[0b000], load_result[7].replicate(8)),
                # LBU (unsigned): zero extend
                (dec.funct3_is[0b100], 0),
                # LW/LH/LHU: copy data from bus
                (~dec.funct3_is[0b000] & ~dec.funct3_is[0b100],
                 self.bus.resp[8:]),
            ])),
        ]

        # Register file read port
        m.d.comb += [
            self.rf_read_cmd.valid.eq(onehot_choice(self.onehot_state, {
                # We always use the register file read port during states 0-2
                # inclusive.
                (0, 1, 2): 1,
                # We also use it during word stores in state 3.
                3: dec.is_store & dec.funct3_is[2],
            })),
            self.rf_read_cmd.payload.eq(onehot_choice(self.onehot_state, {
                # rs2.lo
                0: Cat(dec.rs2, 0),
                # rs1.hi
                1: Cat(dec.rs1, 1),
                # rs2.hi, except for stores which read lo
                2: Cat(dec.rs2, ~dec.is_store),
                # duplicate rs2.hi for word stores
                3: oneof([(dec.is_store & self.full, Cat(dec.rs2, 1))]),
            })),
        ]

        # Register file write port
        m.d.comb += [
            self.rf_write_cmd.valid.eq(self.full & onehot_choice(self.onehot_state, {
                1: dec.writes_rd_normally,
                3: dec.writes_rd_normally,
                4: dec.is_load | dec.is_alu,
                5: dec.is_load | dec.is_shift,
            })),
            self.rf_write_cmd.payload.reg.eq(
                Cat(dec.rd, self.onehot_state[3] | self.onehot_state[5])
            ),
            self.rf_write_cmd.payload.value.eq(oneof([
                # Hoisted add out of the ALU select mux below because it tends
                # to be the critical path to the register file.
                (dec.writes_adder_to_reg, adder_result),
                (dec.is_jal_or_jalr, choosehalf(
                    self.onehot_state[3],
                    Cat(0, 0, pc_inc),
                )),
                (dec.is_load & self.onehot_state[4], load_result),
                (dec.is_load & self.onehot_state[5], mux(
                    dec.funct3_is[2],
                    load_result,
                    self.accum,
                )),
                (dec.is_alu, onehot_choice(dec.funct3_is, {
                    # Shifts
                    (0b001, 0b101): mux(
                        self.onehot_state[5],
                        self.accum,
                        shift_lo,
                    ),
                    # SLT
                    0b010: mux(
                        self.onehot_state[3],
                        0,
                        signed_less_than_d,
                    ),
                    # SLTU
                    0b011: mux(
                        self.onehot_state[3],
                        0,
                        unsigned_less_than_d,
                    ),
                    # XOR
                    0b100: self.accum ^ adder_rhs,
                    # OR
                    0b110: self.accum | adder_rhs,
                    # AND
                    0b111: self.accum & adder_rhs,
                })),
            ])),
        ]

        # Generating end of instruction and bubble control signals

        m.d.comb += [
            end_of_instruction.eq(onehot_choice(self.onehot_state, {
                3: oneof([
                    # Unconditionally end the instruction at cycle 3 if we're in
                    # a bubble.
                    (~self.full, 1),
                    # instructions that always end in state 3:
                    (dec.is_jal_or_jalr, 1),
                    (dec.is_auipc_or_lui, 1),
                    # Branches do not end here.
                    # ALU operations generally end here, but shifts and SLTs
                    # continue.
                    (dec.is_alu, ~dec.funct3_is[0b011] &
                     ~dec.funct3_is[0b010] &
                     ~dec.is_shift
                     ),
                    # Stores end here except for word stores.
                    (dec.is_store, ~dec.funct3_is[0b010]),
                ]),
                # Word stores and not-taken branches end in state 4.
                4: dec.is_store | (dec.is_b & ~branch_taken_d),
                # All other instructions end in state 5.
                5: 1,
            })),
            # We signal state restart as a side effect of the EOI signal.
            self.from_the_top.eq(end_of_instruction),

            start_bubble.eq(self.full & onehot_choice(self.onehot_state, {
                # Because bubbles end in state 3, it's important to include
                # self.full here to make forward progress.
                3: dec.is_jal_or_jalr,
                # Any branch that makes it to state 5 is being taken, and so is
                # creating a bubble.
                5: dec.is_b,
            })),
        ]
        # Next instruction capture logic
        m.d.sync += dec.eq(mux(
            end_of_instruction,
            m.submodules.dec.out,
            dec,
        ))

        # Accumulator update
        m.d.sync += self.accum.eq(onehot_choice(self.onehot_state, {
            # Load the accumulator with a new LHS in states 0 and 2. The LHS is
            # usually a value read from a register, but is occasionally the PC.
            (0, 2): oneof([
                # Load program counter instead.
                (dec.is_auipc_or_jal, choosehalf(
                    self.onehot_state[2],
                    Cat(0, 0, self.pc),
                )),
                # Make the adder pass through the immediate without changes.
                (dec.is_lui, 0),
                # Otherwise, regfile:
            ], default = self.rf_resp),
            # Load the accumulator in state 3 with various values expected by
            # instructions that continue into state 4.
            3: oneof([
                # For branches, load the PC to begin the target calculation.
                (dec.is_b, lohalf(Cat(0, 0, self.pc))),
                # Shifts rely on preserving the accumulator contents
                (dec.is_shift, self.accum),
            ]),
            4: oneof([
                # Prepare for a taken branch. We'll do this even if it's not
                # taken.
                (dec.is_b, hihalf(Cat(0, 0, self.pc))),

                (dec.is_load, mux(
                    dec.funct3[2], # unsigned?
                    0, # zero extend
                    load_result[15].replicate(16), # sign extend
                )),
                (dec.is_shift, mux(
                    shift_amt != 0,
                    mux(
                        dec.funct3[2] == 0, # left shift
                        Cat(shift_lo[15], self.accum[:15]),
                        Cat(self.accum[1:], dec.inst[30] & self.accum[15]),
                    ),
                    self.accum,
                )),
            ]),
        }, default = self.accum))

        # Chain registers (saved carry and zero) update rules
        m.d.sync += [
            saved_carry.eq(onehot_choice(self.onehot_state, {
                # Set saved carry for instructions that subtract.
                0: dec.is_b
                    | dec.is_neg_reg_to_adder
                    | dec.is_neg_imm_i
                    | (dec.is_alu_rr & dec.inst[30]),
                1: adder_carry_out,
                # We need to clear saved carry for the branch target computation
                # on a taken branch -- clearing it unconditionally doesn't hurt
                # and saves some logic.
                3: 0,
            }, default = saved_carry)),

            saved_zero.eq(oneof([
                # Set saved zero unconditionally.
                (self.onehot_state[0], 1),
                # Capture zero out for chaining.
                (self.onehot_state[1], zero_out),
                # Otherwise, preserve.
            ], default = saved_zero)),
        ]

        # Debug port wiring
        m.d.comb += [
            self.debug_pc_write.ready.eq(self.onehot_state[STATE_COUNT - 1]),
        ]

        # State logic

        # State 0: latch accumulator, start rs2 load.
        # State 1: operate, start rs1 high read, maybe write rd low

        # State 2: latch accumulator, start rs2 load
        # State 3: operate, maybe write rd high
        # State 4:
        # - LW: write bottom half to rd, start top load
        # - LH/LB: write bottom half to rd, note sign
        # - Bxx: add low half
        # - SW: write high half
        # - SLT: write register bottom half
        # - Shifts: shift one more bit and hold this state until done

        # State 5:
        # - LW: write top half
        # - LH/LB: write sign or zero to rd
        # - Bxx: add high half, write PC
        # - Shifts: write result, high half

        return m
