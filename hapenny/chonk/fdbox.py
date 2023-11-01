# The FD-Box, responsible for fetch and decode during execution.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from hapenny import StreamSig, AlwaysReady, onehot_choice, mux, oneof
from hapenny.chonk.sbox import STATE_COUNT
from hapenny.bus import BusPort

class FDBox(Component):
    """The FD-Box fetches and decodes instructions.

    Based on a PC (provided by the EW-box) the FD-box generates bus
    transactions to collect both halfwords of an instruction, and then provides
    it on an output signal to the EW-box.

    Parameters
    ----------
    prog_addr_width (integer): number of bits in a program address, 32 by default
        but can be shrunk to save logic.

    Attributes
    ----------
    onehot_state (input): state input from the S-Box
    pc (input): program counter from EW-box.
    rf_cmd (output): read command to the register file, intended to be OR'd.
    inst_next (output): instruction word for EW to use next time we restart
        from the top.
    bus (port): our connection to the memory fabric.
    from_the_top (input): signal from EW indicating that this is the final
        cycle of the instruction. We use this to gate register reads.
    """
    onehot_state: In(STATE_COUNT)
    rf_cmd: Out(AlwaysReady(5))
    inst_next: Out(32)
    from_the_top: In(1)

    def __init__(self, *,
                 prog_addr_width = 32,
                 ):
        super().__init__()

        # Create a bus port of sufficient width to fetch instructions only.
        # (Width is -2 because we're addressing words.)
        self.bus = BusPort(addr = prog_addr_width - 2, data = 32).create()

        # The PC width is -2 because it's addressing words.
        self.pc = Signal(prog_addr_width - 2)

        self.inst = Signal(32)

    def elaborate(self, platform):
        m = Module()

        # State 0: we start the fetch.
        # State 1: we receive the instruction word and begin a register read.
        # State 2+: we don't do anything.

        m.d.comb += [
            # We issue bus transactions in state 0 only.
            self.bus.cmd.valid.eq(self.onehot_state[0]),
            # In that state we put the PC on the bus.
            self.bus.cmd.payload.addr.eq(onehot_choice(self.onehot_state, {
                0: self.pc,
            })),

            # We access the register file only in the last cycle.
            self.rf_cmd.valid.eq(self.from_the_top),
            # If the last cycle is state 1, our fetch is still completing, so
            # we need to forward the bus response to the register file. If it
            # isn't state 1, we can serve out of our inst register.
            # (It's important to send zeros in other states instead of
            # hardwiring this so that we can OR.)
            self.rf_cmd.payload.eq(oneof([
                (self.from_the_top & self.onehot_state[1], self.bus.resp[15:20]),
                (self.from_the_top & ~self.onehot_state[1], self.inst[15:20]),
            ])),

            # Forward the instruction through so it's valid in states 1+. In
            # other states, serve up the contents of our registers. EW's not
            # supposed to look at this in state 0.
            self.inst_next.eq(mux(
                self.onehot_state[1],
                self.bus.resp,
                self.inst,
            )),
        ]

        m.d.sync += [
            # Latch the bottom half of the instruction at the end of state 1.
            self.inst.eq(mux(
                self.onehot_state[1],
                self.bus.resp,
                self.inst,
            )),
        ]

        return m
