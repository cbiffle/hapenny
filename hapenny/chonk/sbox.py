# The S-Box, responsible for state sequencing of other boxes.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from hapenny import StreamSig, AlwaysReady
from hapenny.bus import BusPort

# Maximum number of (unique) states needed by any instruction, plus one
# additional for halt. (Note that repeated states when e.g. shifting do not
# count as unique states.)
STATE_COUNT = 3 + 1

class SBox(Component):
    """The S-Box sequences the other components.

    The S-Box implements a state counter that counts up through the maximum
    number of unique states required by any instruction. The count can be reset,
    signaling the end of one instruction and the beginning of the next, by
    asserting the from_the_top input.

    The state counter, and output, are both one-hot.

    Attributes
    ----------
    from_the_top (input): restarts the count for the next instruction.
    hold (input): input from EW-box to keep doing this same state. Only safe for
        use after state 3 to avoid weird side effects.
    halt_request (input): when high, redirects the next from_the_top assertion
        to go to the halted state instead.
    not_a_bubble (input): indicates that the CPU is doing useful work and not
        just fetching. Used to gate transitions to halt state to ensure forward
        progress during single-stepping.
    onehot_state (output): one bit per possible state.
    halted(output): a handy synonym for the last onehot_state bit.
    """
    from_the_top: In(1)
    hold: In(1)
    halt_request: In(1)
    not_a_bubble: In(1)

    onehot_state: Out(STATE_COUNT)
    halted: Out(1)

    def __init__(self):
        super().__init__()

        self.onehot_state.reset = 1

    def elaborate(self, platform):
        m = Module()

        # This module is doing a lot of things by hand, because as far as I can
        # tell, Amaranth doesn't really know anything about one-hot encoding.
        # Like, there's no way to indicate that the bits are exclusive. So in an
        # attempt to get this managed like a one-hot FSM rather than a
        # STATE_COUNT-wide base-2 FSM, I'm rolling circuits by hand.

        # Inexpensive way to detect that we're leaving a halt request without
        # requiring more registers:
        end_of_halt = Signal(1)
        m.d.comb += end_of_halt.eq(
            self.onehot_state[STATE_COUNT - 1] & ~self.halt_request
        )

        # Generate one-hot counter transition circuit. In each state we clear
        # one bit and set another to advance. This can be overridden if we get
        # the signal to start again from the top.
        for state_num in range(STATE_COUNT):
            with m.If(self.from_the_top | end_of_halt):
                with m.If(self.halt_request & self.not_a_bubble):
                    # Each bit must clear itself except for the highest.
                    m.d.sync += self.onehot_state[state_num].eq(
                        state_num == STATE_COUNT - 1
                    )
                with m.Else():
                    # Each bit must clear itself except for the lowest.
                    m.d.sync += self.onehot_state[state_num].eq(state_num == 0)
            with m.Elif(self.onehot_state[state_num] & ~self.hold):
                # The final state is sticky, so, don't implement wraparound
                # logic to advance out of it. We only leave that state if we
                # receive from_the_top.
                if state_num < STATE_COUNT - 1:
                    m.d.sync += [
                        self.onehot_state[state_num].eq(0),
                        self.onehot_state[state_num + 1].eq(1),
                    ]

        m.d.comb += self.halted.eq(self.onehot_state[STATE_COUNT - 1])
        return m
