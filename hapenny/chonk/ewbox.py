# The EW-Box, responsible for execute and writeback.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *

from hapenny import StreamSig, AlwaysReady, onehot_choice, oneof, mux, hihalf, lohalf, choosehalf
from hapenny.chonk.sbox import STATE_COUNT
from hapenny.chonk.regfile32 import RegWrite
from hapenny.bus import BusPort
from hapenny.decoder import Decoder, ImmediateDecoder, DecodeSignals

class EWBox(Component):
    """The EW-Box does execute and writeback -- basically all of the CPU logic
    except for fetch and sequencing.

    States:

    State 0: use rs1 value read by FD-Box, latch it into accumulator, start rs2
        read.
    State 1: operate on accumulator and rs2/immediate, write rd if appropriate.
        Instructions generally end here.
    State 2: depends.
    - Loads: write to rd
    - Bxx: perform branch target computation
    - SLT: rewrite rd to set flag
    - Shifts: shift one more bit and hold this state until done; write rd.

    Parameters
    ----------
    reset_vector (int): address where the CPU will begin fetching instructions
        after reset. Default 0 if not provided.
    addr_width (int): number of low-order bits that are significant in memory
        addresses. The default is 32; if this is reduced, memory and I/O
        devices will appear to repeat at higher addresses because the top bits
        won't be decoded. Note that this parameter is in terms of byte
        addresses (the numbers RV32I software deals with); the actual bus port
        has addr_width-2 address lines because it addresses words.
    prog_addr_width (int): number of low-order bits that are significant in
        instruction addresses. This determines the width of the PC register(s)
        and fetch path. If program storage is in the lower section of the
        address range, and I/O devices higher, you can set this parameter to
        smaller than addr_width to save some area. If not explicitly
        overridden, this is the same as addr_width.  addr_width.

    Attributes
    ----------
    onehot_state (input): state input from the S-Box
    inst_next (input): instruction word from FD.

    rf_read_cmd (output): read command to the register file, intended to be OR'd.
    rf_resp (input): value most recently read from the register file.

    bus (port): our connection to the memory fabric.

    pc_next (output): PC to FD for fetch of next instruction. We need to provide
        a useful next-PC value here by the end of state 0 to get a useful fetch
        done. We then need to hold that value through subsequent states. If we
        violate this, we need to generate a bubble.
    from_the_top (output): signals S-Box to rewind state when asserted.
    rf_write_cmd (port): output to register file to write registers.
    full (output): indicates whether we're doing useful work. When we have a
        real instruction from FD, and we're honoring it (i.e. it is not a
        speculative fetch that became irrelevant due to a change in control
        flow), we set this. It is clear at reset to create an initial pipeline
        bubble until the first fetch completes, and we clear it on instruction
        stream changes.
    pc (output): Current contents of PC register, mostly for debug port.
    debug_pc_write (port): while halted, can overwrite the PC with a word
        address (bottom two bits implied as zero).
    hold (output): Indicates that we're going to repeat this state, signals
        s-box to maintain it.
    """
    onehot_state: In(STATE_COUNT)
    rf_read_cmd: Out(AlwaysReady(5))
    rf_resp: In(32)
    inst_next: In(32)
    from_the_top: Out(1)
    rf_write_cmd: Out(AlwaysReady(RegWrite(5)))
    full: Out(1)
    hold: Out(1)

    debug_pc_write: In(StreamSig(32 - 2))
    debug_inst: Out(32)

    def __init__(self, *,
                 reset_vector = 0,
                 addr_width = 32,
                 prog_addr_width = 32,
                 counters = False,
                 ):
        super().__init__()
        assert reset_vector.bit_length() <= prog_addr_width, \
                f"reset vector 0x{reset_vector:x} won't fit in PC"

        # Create a bus port of sufficient width to address anything on the bus.
        # (Width is -1 because we're addressing halfwords.)
        self.bus = BusPort(addr = addr_width - 2, data = 32).create()

        # The PC width is -2 because it's addressing words.
        self.pc_next = Signal(prog_addr_width - 2)

        self.accum = Signal(32)
        self.pc = Signal(prog_addr_width - 2, reset = reset_vector >> 2)

        self.counters = counters

    def elaborate(self, platform):
        m = Module()

        cycle_counter = Signal(32)
        instret_counter = Signal(32)

        # Nested decoder bits
        m.submodules.dec = Decoder()
        m.submodules.imm = imm = ImmediateDecoder()
        # We register the output of the Decoder, above, whenever we load a new
        # instruction. This cuts the critical path from bus response to our
        # various control signals.
        dec = Signal(DecodeSignals)

        m.d.comb += [
            # Feed the inst_next output into the decoder, which will effectively
            # produce our control signals one cycle too early.
            m.submodules.dec.inst.eq(self.inst_next),
            # Note that the immediate decoder's output is derived from the dec
            # register, so it happens on the same cycle as the other decode
            # outputs.
            imm.inst.eq(dec.inst),

            self.debug_inst.eq(dec.inst),
        ]

        # The Adder
        #
        # The adder computes the sum of the accumulator register, the adder_rhs
        # mux (below), and the adder_carry_in signal.
        #
        # It's used both to compute sums for things like ADD and branches, and
        # also to perform comparisons.
        adder_carry_in = Signal(1)

        adder_immediate = Signal(32)
        adder_rhs_pos = Signal(32)
        adder_rhs = Signal(32)

        adder_carry_out = Signal(1)
        adder_result = Signal(32)
        zero_out = Signal(1)

        m.d.comb += [
            # The immediate selection for the adder has never come up on the
            # critical path, so it pays to put more levels of logic _here_ and
            # then have as thin of a mux as possible to choose between this or
            # the register file at the very end.
            adder_immediate.eq(onehot_choice(self.onehot_state, {
                1: oneof([
                    (dec.is_auipc_or_lui, imm.u),
                    (dec.is_jal, imm.j),
                    (dec.is_any_imm_i, imm.i),
                    (dec.is_store, imm.s),
                ]),
                # In state 2 we only use the adder to branch.
                2: imm.b,
            })),

            # Adder RHS operand selection. We choose the "positive" version of
            # the operand here and potentially complement it below.
            #
            # This mux is complicated because of branches. Branches are the
            # only operation that perform two independent additions: first a
            # comparison, then a branch target calculation. So, we have to be
            # careful to only select the register in state 1.
            adder_rhs_pos.eq(mux(
                dec.is_any_reg_to_adder & self.onehot_state[1],
                self.rf_resp,
                adder_immediate,
            )),
            # Generate the final adder_rhs value by conditionally complementing
            # in operate state. (We limit this to operate state to avoid
            # complementing the branch displacement used to compute
            # destinations in state 2.
            adder_rhs.eq(
                adder_rhs_pos ^
                (dec.is_adder_rhs_complemented &
                     self.onehot_state[1]).replicate(32)
            ),
            # Adder implementation:
            Cat(adder_result, adder_carry_out).eq(
                self.accum + adder_rhs + adder_carry_in,
            ),
            zero_out.eq(adder_result == 0),
        ]
        # Adder carry-in selection. Currently this is registered and so it
        # needs to be computed a cycle early.
        m.d.sync += [
            adder_carry_in.eq(onehot_choice(self.onehot_state, {
                # Initially set saved carry for instructions that subtract,
                # clear otherwise.
                0: dec.is_b
                    | dec.is_neg_reg_to_adder
                    | dec.is_neg_imm_i
                    | (dec.is_alu_rr & dec.inst[30]),
                # We need to clear saved carry for the branch target computation
                # on a taken branch -- clearing it unconditionally doesn't hurt
                # and saves some logic.
                1: 0,
            }, default = adder_carry_in)),
        ]

        # Comparator
        #
        # The comparator is used for branch operations and SLT/SLTU. It uses the
        # adder result, so to compare two numbers we subtract them through the
        # adder -- which is to say, complement the RHS operand and ensure
        # saved_carry is set.
        #
        # The combinational comparator outputs come at the end of the adder's
        # carry chain, and are often the slowest signal in the datapath. As a
        # result, nothing in the chip relies on them directly -- instead, they
        # get registered and their one-cycle-delayed counterparts get used.
        #
        # We delay the zero signal for symmetry, though it's not a critical
        # path.
        signed_less_than = Signal(1)
        unsigned_less_than = Signal(1)
        m.d.comb += [
            unsigned_less_than.eq(~adder_carry_out),
            signed_less_than.eq(mux(
                self.accum[-1] ^ ~adder_rhs[-1],
                self.accum[-1],
                ~adder_carry_out,
            )),
        ]
        # Generate delayed versions.
        signed_less_than_d = Signal(1)
        unsigned_less_than_d = Signal(1)
        zero_out_d = Signal(1)
        m.d.sync += [
            zero_out_d.eq(zero_out),
            signed_less_than_d.eq(signed_less_than),
            unsigned_less_than_d.eq(unsigned_less_than),
        ]

        # Branch comparator condition select
        #
        # Using the decoded funct3 field, this determines whether a branch would
        # be taken based on the comparator output. This uses the delayed
        # comparator outputs, and so is itself delayed one cycle -- the branch
        # decision becomes available in state 2, as a result.
        branch_taken_d = Signal(1)
        m.d.comb += branch_taken_d.eq(onehot_choice(dec.funct3_is, {
            (0b000, 0b001): zero_out_d,
            (0b100, 0b101): signed_less_than_d,
            (0b110, 0b111): unsigned_less_than_d,
        }) ^ dec.funct3[0])

        # The Shifter
        #
        # We use a dead-simple single-bit-per-cycle shifter, and as a result a
        # shift can take up to 31 additional cycles.
        #
        # During the shift, the bits being shifted are held in the accumulator.
        # The shift_amt register tracks how many bits are yet to go -- when not
        # shifting, its contents are undefined.
        shift_amt = Signal(5)

        # Shifts are the only thing we do that can cause a state hold in the
        # S-Box -- we repeat state 2 until the shift is done.
        m.d.comb += self.hold.eq(
            dec.is_shift & self.onehot_state[2] & (shift_amt != 0)
        )

        m.d.sync += [
            shift_amt.eq(onehot_choice(self.onehot_state, {
                # Load shift_amt in state 1. Note that the immediate value for
                # immediate shifts is taken from the rs2 _field_ in the
                # instruction, not the corresponding register.
                1: mux(
                    dec.is_alu_ri,
                    dec.rs2,
                    self.rf_resp[:5],
                ),
                # Count shift_amt down while in state 2.
                2: shift_amt - 1,
                # Otherwise, do not update the register.
            }, default = shift_amt)),
            # The shifter update rule is in the accumulator mux.
        ]

        # Bus interface and effective address generation.
        m.d.comb += [
            # When a transaction actually issues:
            self.bus.cmd.valid.eq(
                # All bus transactions are conditional upon us being full.
                self.full & onehot_choice(self.onehot_state, {
                    # Both loads and stores generate traffic in state 1.
                    1: dec.is_load | dec.is_store,
                })
            ),
            # The address we generate to the bus. We only generate output in
            # state 1, to avoid interfering with bus usage by the FD-Box in
            # state 0.
            self.bus.cmd.payload.addr.eq(onehot_choice(self.onehot_state, {
                1: Cat(adder_result[2:]),
            })),
            # The lane strobes determine whether we're doing a store, and can
            # affect a single byte within a word.
            self.bus.cmd.payload.lanes.eq(onehot_choice(self.onehot_state, {
                # Again, we only use the bus in state 1.
                1: mux(
                    self.full & dec.is_store,
                    onehot_choice(dec.funct3_is, {
                        0b000: Cat(
                            adder_result[:2] == 0,
                            adder_result[:2] == 1,
                            adder_result[:2] == 2,
                            adder_result[:2] == 3,
                        ),
                        0b001: Cat(
                            adder_result[1] == 0,
                            adder_result[1] == 0,
                            adder_result[1] == 1,
                            adder_result[1] == 1,
                        ),
                    }, default = 0b1111),
                    0,
                ),
            })),
            # Data we output to the bus. We don't have to be careful about this,
            # because it's only relevant during stores -- and we're the only box
            # that generates stores.
            self.bus.cmd.payload.data.eq(onehot_choice(dec.funct3_is, {
                # For byte stores, we repeat the same LSBs across all byte
                # lanes.
                0b000: self.rf_resp[:8].replicate(4),
                # For halfword stores, we repeat the halves.
                0b001: self.rf_resp[:16].replicate(2),
                # Otherwise we just send the word unmodified.
            }, default = self.rf_resp)),
        ]

        # Program Counter and instruction lifecycle management.
        #
        # The address of the _current_ instruction we're executing is always
        # held in the self.pc register (as a word address).
        #
        # The FD-Box needs us to produce the address of the _next_ instruction
        # by state 1. We do that in the common case using a dedicated
        # incrementer (pc_inc).
        #
        # This is effectively fetch speculation, and like all speculation it can
        # prove to be incorrect -- in that case, we generate a bubble and
        # produce the corrected address to let FD-Box catch back up. The
        # start_bubble signal causes this to happen when asserted on the final
        # cycle of an instruction.
        #
        # The final cycle of an instruction is indicated through the
        # end_of_instruction signal.
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
                        | (dec.is_b & branch_taken_d & self.onehot_state[2]),
                    # the computed PC,
                    adder_result[2:],
                    # Otherwise, PC+1
                    pc_inc,
                ),
                # When not full, send current PC to fetch current instruction
                # rather than next, as we are no longer speculating.
                self.pc,
            )),
            # Instruction termination
            end_of_instruction.eq(onehot_choice(self.onehot_state, {
                1: oneof([
                    # Unconditionally end the instruction at cycle 1 if we're in
                    # a bubble.
                    (~self.full, 1),
                    # instructions that always end in state 1:
                    (dec.is_jal_or_jalr, 1),
                    (dec.is_auipc_or_lui, 1),
                    # Branches do not end here.
                    # ALU operations generally end here, but shifts and SLTs
                    # continue.
                    (dec.is_alu, ~dec.funct3_is[0b011] &
                     ~dec.funct3_is[0b010] &
                     ~dec.is_shift
                     ),
                    # Stores end here.
                    (dec.is_store, 1),
                ]),
                # Everything else ends in state 2, though we may be asserting
                # hold.
                2: ~self.hold,
            })),
            # We signal state restart as a side effect of the EOI signal.
            self.from_the_top.eq(end_of_instruction),

            # Bubble control, gated on self.full:
            start_bubble.eq(self.full & oneof([
                (dec.is_jal_or_jalr, 1),
                # Taken branches create a bubble from state 2. Not-taken
                # branches, and loads and shifts, can make it to state 2
                # without bubbling.
                (dec.is_b, self.onehot_state[2] & branch_taken_d),
            ])),
        ]
        m.d.sync += [
            # Program counter updates.
            self.pc.eq(oneof([
                # In the halted sate, we allow the program counter to be
                # overwritten from the debug port.
                (self.onehot_state[STATE_COUNT - 1] & self.debug_pc_write.valid,
                 self.debug_pc_write.payload),
                # Otherwise, in the final cycle of any instruction, we latch the
                # pc_next value we were sending to FD-Box.
                (end_of_instruction & self.full, self.pc_next),
                # In all other circumstancs we leave the register unchanged.
            ], default = self.pc)),
            # Bubble logic.
            self.full.eq(oneof([
                # If halted, we are not full, to ensure a fetch and refill when
                # we resume.
                (self.onehot_state[STATE_COUNT - 1], 0),
                # Otherwise, we become empty only at the end of an instruction
                # when a bubble is required.
                (end_of_instruction, ~start_bubble),
            ], default = self.full)),
        ]

        # Maintaining the counters
        if self.counters:
            m.d.sync += [
                cycle_counter.eq(cycle_counter + 1),
                instret_counter.eq(mux(
                    self.full & end_of_instruction,
                    instret_counter + 1,
                    instret_counter,
                )),
            ]

        # Load lane mixer.
        #
        # RV32 has relatively fancy load instructions, at least compared to its
        # stores. We assume a relatively simple bus that only performs
        # word-aligned reads. This section bridges the gap between them.
        load_result = Signal(32)
        ea_lsbs = Signal(2)
        m.d.sync += ea_lsbs.eq(mux(
            self.bus.cmd.valid,
            adder_result[:2],
            ea_lsbs,
        ))
        m.d.comb += [
            # Bottom byte of load data: can be any byte of the response,
            # depending on EA LSBs.
            load_result[:8].eq(oneof([
                (ea_lsbs == 0, self.bus.resp[:8]),
                (ea_lsbs == 1, self.bus.resp[8:16]),
                (ea_lsbs == 2, self.bus.resp[16:24]),
                (ea_lsbs == 3, self.bus.resp[24:]),
            ])),
            # Next byte of load data: extend byte if required.
            load_result[8:16].eq(oneof([
                # LB (signed): sign extend
                (dec.funct3_is[0b000], load_result[7].replicate(8)),
                # LBU (unsigned): zero extend
                (dec.funct3_is[0b100], 0),
                # Otherwise, use the byte from the bus.
            ], default = mux(
                ~ea_lsbs[1],
                self.bus.resp[8:16],
                self.bus.resp[24:],
            ))),
            # Top half of data
            load_result[16:].eq(oneof([
                # LB (signed): sign extend
                (dec.funct3_is[0b000], load_result[7].replicate(16)),
                # LBU (unsigned): zero extend
                (dec.funct3_is[0b100], 0),
                # LH (signed): sign extend
                (dec.funct3_is[0b001], load_result[15].replicate(16)),
                # LHU (unsigned): zero extend
                (dec.funct3_is[0b101], 0),
                # Otherwise, use the top half of the bus response.
            ], default = self.bus.resp[16:])),
        ]

        # Register file read port.
        m.d.comb += [
            # We have control of the read port on every cycle but the last, when
            # the FD-Box kicks off the read of rs1. While we
            # only use the read result in state 0-1 and only drive it in state
            # 0, it simplifies the logic to issue unused reads in other states.
            self.rf_read_cmd.valid.eq(~end_of_instruction),
            # Because there's only one actual read port, and our nets are OR'd
            # with FD-Box's, we need to generate 0 here if we don't intend to
            # use the register file.
            self.rf_read_cmd.payload.eq(oneof([
                # rs2
                (~end_of_instruction, dec.rs2),
            ])),
        ]

        # Register file write port.
        #
        # We have exclusive control of this port in all states but the halted
        # state.
        m.d.comb += [
            # When do we write? All writes gated on self.full.
            self.rf_write_cmd.valid.eq(self.full & onehot_choice(self.onehot_state, {
                1: dec.writes_rd_normally | (self.counters & dec.is_system),
                # loads, shifts, and SLTs write in state 2, stores and branches
                # make it there but don't write.
                # do not.
                2: dec.is_load | dec.is_shift | dec.is_alu,
            })),
            # We always write the register selected by the instruction.
            self.rf_write_cmd.payload.reg.eq(dec.rd),
            # Value to write:
            self.rf_write_cmd.payload.value.eq(oneof([
                # Writing the result from the adder is often the critical path,
                # if it has to go through several layers of mux logic on the
                # way. So, we pre-decode the need to do so in
                # writes_adder_to_reg and hoist the check to the outermost layer
                # of mux here.
                (dec.writes_adder_to_reg, adder_result),

                # JAL and JALR both store the incremented program counter.
                (dec.is_jal_or_jalr, Cat(0, 0, pc_inc)),
                # Loads write from the load mixer.
                (dec.is_load, load_result),
                # Selects among the faster ALU results (i.e. not the adder)
                (dec.is_alu, onehot_choice(dec.funct3_is, {
                    # Shifts
                    (0b001, 0b101): self.accum,
                    # SLT
                    0b010: signed_less_than_d,
                    # SLTU
                    0b011: unsigned_less_than_d,
                    # XOR
                    0b100: self.accum ^ adder_rhs,
                    # OR
                    0b110: self.accum | adder_rhs,
                    # AND
                    0b111: self.accum & adder_rhs,
                })),
                (self.counters & dec.is_system, onehot_choice(self.onehot_state, {
                    1: mux(imm.i[1], instret_counter, cycle_counter),
                })),
            ])),
        ]

        # Next instruction capture.
        #
        # On end_of_instruction we register the output from the next-instruction
        # decoder.
        m.d.sync += dec.eq(mux(
            end_of_instruction,
            m.submodules.dec.out,
            dec,
        ))

        # Accumulator update. This mashes together a bunch of concerns from
        # various instructions, by necessity, since they share the accumulator.
        m.d.sync += self.accum.eq(onehot_choice(self.onehot_state, {
            # Load the accumulator with a new LHS in state 0. The LHS is
            # usually a value read from a register, but is occasionally the PC.
            0: oneof([
                # Load program counter instead.
                (dec.is_auipc_or_jal, Cat(0, 0, self.pc)),
                # Make the adder pass through the immediate without changes.
                (dec.is_lui, 0),
                # Otherwise, regfile:
            ], default = self.rf_resp),
            # Load the accumulator in state 1 with various values expected by
            # instructions that continue into state 2.
            1: mux(
                # For branches,
                dec.is_b,
                # load the PC to begin the target calculation.
                Cat(0, 0, self.pc),
                # Otherwise preserve it for shifts and EA generation.
                self.accum,
            ),
            2: oneof([
                # Shifts use the accumulator as the shift register.
                (dec.is_shift, mux(
                    shift_amt != 0,
                    mux(
                        dec.funct3[2] == 0, # left shift
                        Cat(0, self.accum[:-1]),
                        Cat(self.accum[1:], dec.inst[30] & self.accum[-1]),
                    ),
                    self.accum,
                )),
                # Otherwise, trash it.
            ]),
        }, default = self.accum))

        # Debug port support
        m.d.comb += [
            self.debug_pc_write.ready.eq(self.onehot_state[STATE_COUNT - 1]),
        ]

        return m
