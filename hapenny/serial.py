from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from hapenny import StreamSig, AlwaysReady, mux, oneof
from hapenny.bus import BusPort

class ReceiveCore(Component):
    rx: In(1)
    sample_clock: In(1)
    rdr: Out(8)
    empty: Out(1)
    read_strobe: In(1)

    def __init__(self, oversample = 16):
        super().__init__()

        self.oversample = oversample

    def elaborate(self, platform):
        m = Module()

        state = Signal(range(4))
        bits_left = Signal(range(8))
        timer = Signal(range(self.oversample))
        have_data = Signal(1)

        m.d.comb += [
            self.empty.eq(~have_data),
        ]

        m.d.sync += timer.eq(oneof([
            # Set to delay half a bit period from initial negative edge.
            (self.sample_clock & (state == 0), (self.oversample // 2) - 1),
            # Count down in all other states until we reach 0.
            (self.sample_clock & (state != 0) & (timer != 0), timer - 1),
            # Once we reach 0, reset to a full bit time.
            (self.sample_clock & (state != 0) & (timer == 0), self.oversample - 1),
        ], default = timer))

        m.d.sync += state.eq(oneof([
            # Leave state 0 if we see the falling edge.
            (self.sample_clock & (state == 0), ~self.rx),
            # If it's still low at the midpoint of the start bit, proceed.
            # Otherwise, treat it as a glitch and reset.
            (self.sample_clock & (state == 1) & (timer == 0), mux(~self.rx, 2, 0)),
            # Automatically advance when we've done all the bits in state 2.
            (self.sample_clock & (state == 2) & (timer == 0), mux(bits_left == 0, 3, 2)),
            # Automatically advance at the end of the stop bit.
            (self.sample_clock & (state == 3) & (timer == 0), 0),
        ], default = state))

        m.d.sync += bits_left.eq(oneof([
            # Configure for 7 bits after the first one.
            (self.sample_clock & (timer == 0), mux(state == 1, 7, bits_left - 1)),
        ], default = bits_left))

        m.d.sync += self.rdr.eq(oneof([
            (self.sample_clock & (state == 2) & (timer == 0), Cat(self.rdr[1:], self.rx)),
        ], default = self.rdr))

        m.d.sync += have_data.eq(oneof([
            # The way this is expressed, newly arriving data will override the
            # read strobe -- the two cases will OR if they occur
            # simultaneously, and the 0 loses.
            (self.sample_clock & (state == 3) & (timer == 0), self.rx),
            (self.read_strobe, 0),
        ], default = have_data))

        return m


class TransmitCore(Component):
    tx: Out(1)
    sample_clock: In(1)
    thr_write: In(AlwaysReady(8))
    busy: Out(1)

    def __init__(self, oversample = 16):
        super().__init__()

        self.oversample = oversample

    def elaborate(self, platform):
        m = Module()

        # We use this as a shift register containing: start bit, 8 data bits, 2
        # stop bits. Its LSB is our output state, so it's important that it
        # reset to 1; the other bits can reset to whatever value.
        thr = Signal(1 + 8, reset = 1)

        tx_bits_left = Signal(range(1 + 8 + 2))
        tx_timer = Signal(range(self.oversample))

        with m.If(self.sample_clock):
            with m.If(tx_bits_left != 0):
                with m.If(tx_timer == 0):
                    m.d.sync += [
                        thr.eq(Cat(thr[1:], 1)),
                        tx_timer.eq(self.oversample - 1),
                        tx_bits_left.eq(tx_bits_left - 1),
                    ]
                with m.Else():
                    m.d.sync += tx_timer.eq(tx_timer - 1)

        # Transmit output
        m.d.comb += self.tx.eq(thr[0])

        # Control register interface.
        m.d.comb += self.busy.eq(tx_bits_left != 0)

        with m.If(self.thr_write.valid):
            m.d.sync += [
                # Load THR with the start bit.
                thr.eq(Cat(0, self.thr_write.payload)),
                tx_bits_left.eq(1 + 8 + 2),
                tx_timer.eq(self.oversample - 1),
            ]

        return m


class OversampleClock(Component):
    out: Out(1)

    def __init__(self, baud_rate = 19200, oversample = 16, clock_freq = None):
        super().__init__()
        self.baud_rate = baud_rate
        self.oversample = oversample
        self.clock_freq = clock_freq

    def elaborate(self, platform):
        m = Module()

        # We divide the system clock to our baud rate * oversample and use that
        # clock for sampling. This is a compromise between low cost transmit
        # (where we could divide the clock all the way down to the baud rate
        # without issue) and accurate receive (where higher sampling rates are
        # better but cost more flops).
        clock_freq = self.clock_freq or platform.default_clk_frequency
        our_freq = self.baud_rate * self.oversample
        divisor = int(round(clock_freq / our_freq))
        print(f"UART configured for {self.baud_rate} from input clock {clock_freq}, divisor = {divisor}")
        actual_freq = clock_freq / self.oversample / divisor
        freq_error = abs(actual_freq - self.baud_rate) / self.baud_rate
        print(f"Actual baud rate will be: {actual_freq} (error: {freq_error * 100:.3}%)")
        assert freq_error< 0.01, "Error: cannot achieve requested UART frequency"

        sample_clock = Signal(1)
        sample_counter = Signal(range(divisor))
        # Generate a pulse on every sample period for one (fast) clock cycle.
        m.d.comb += self.out.eq(sample_counter == 0)

        m.d.sync += sample_counter.eq(mux(self.out, divisor - 1, sample_counter - 1))

        return m

class TransmitOnlyUart(Component):
    """The world's crappiest UART!

    The low byte of any write goes into the transmit holding register and will
    be sent out promptly.

    Reads return a status register where bit 0 indicates BUSY.
    """
    bus: In(BusPort(addr = 0, data = 16))
    tx: Out(1)

    def __init__(self, baud_rate = 19200, oversample = 16, clock_freq = None):
        super().__init__()

        self.baud_rate = baud_rate
        self.oversample = oversample
        self.clock_freq = clock_freq

    def elaborate(self, platform):
        m = Module()
        m.submodules.clkdiv = clkdiv = OversampleClock(
            baud_rate = self.baud_rate,
            oversample = self.oversample,
            clock_freq = self.clock_freq,
        )

        m.submodules.txr = txr = TransmitCore(oversample = self.oversample)
        m.d.comb += [
            txr.sample_clock.eq(clkdiv.out),
            self.tx.eq(txr.tx),
            self.bus.resp.eq(txr.busy),

            txr.thr_write.payload.eq(self.bus.payload.data[:8]),
        ]

        with m.If(self.bus.cmd.valid & self.bus.cmd.payload.lanes[0]):
            m.d.comb += txr.thr_write.valid.eq(1)

        return m

class ReceiveOnlyUart(Component):
    """The world's other crappiest UART!

    This can receive a single frame and hold it in registers.

    On any read, this will return the frame in the low 8 bits, plus bit 15 set
    if there's actual data. This is intended to be used with LH to easily get
    the "data full" flag into the MSB where it can be tested with bltz.

    And, read sensitive, why not.
    """
    bus: In(BusPort(addr = 0, data = 16))
    rx: In(1)

    def __init__(self, baud_rate = 19200, oversample = 16, clock_freq = None):
        super().__init__()

        self.baud_rate = baud_rate
        self.clock_freq = clock_freq

    def elaborate(self, platform):
        m = Module()

        m.submodules.clkdiv = clkdiv = OversampleClock(
            baud_rate = self.baud_rate,
            oversample = self.oversample,
            clock_freq = self.clock_freq,
        )

        m.submodules.rxr = rxr = ReceiveCore(oversample = self.oversample)
        m.d.comb += [
            rxr.rx.eq(self.rx),
            rxr.sample_clock.eq(clkdiv.out),
            rxr.read_strobe.eq(self.bus.cmd.valid & ~self.bus.cmd.payload.lanes.any()),
        ]

        m.d.sync += [
            self.bus.resp[:8].eq(rxr.rdr),
            self.bus.resp[15].eq(rxr.empty),
        ]

        return m

class BidiUart(Component):
    """A slightly less crappy UART.

    This combines the transmit and receive logic using a shared clock divider,
    to save some space if you need both directions.

    Register Layout
    ---------------
    0x0000   RDR - data in low 8 bits, empty flag in bit 15, read-sensitive
    0x0002   THR - reads as 0 if TX is idle, writes send low 8 bits
    """
    bus: In(BusPort(addr = 1, data = 16))
    tx: In(1)
    rx: In(1)

    def __init__(self, baud_rate = 19200, oversample = 16, clock_freq = None):
        super().__init__()

        self.baud_rate = baud_rate
        self.oversample = oversample
        self.clock_freq = clock_freq

    def elaborate(self, platform):
        m = Module()

        # Clock divider for sampling
        m.submodules.clkdiv = clkdiv = OversampleClock(
            baud_rate = self.baud_rate,
            oversample = self.oversample,
            clock_freq = self.clock_freq,
        )

        # Receive state machine.
        m.submodules.rxr = rxr = ReceiveCore(oversample = self.oversample)
        m.d.comb += [
            rxr.rx.eq(self.rx),
            rxr.sample_clock.eq(clkdiv.out),
        ]

        # Transmit machine.

        m.submodules.txr = txr = TransmitCore(oversample = self.oversample)
        m.d.comb += [
            txr.sample_clock.eq(clkdiv.out),
            self.tx.eq(txr.tx),
        ]

        # Bus read port. We register this so that state doesn't change by the
        # time the output is read. This is particularly a problem for the
        # read-sensitive RDR register.
        m.d.sync += [
            self.bus.resp[:8].eq(mux(
                self.bus.cmd.payload.addr[0],
                txr.busy,
                rxr.rdr,
            )),
            self.bus.resp[15].eq(
                ~self.bus.cmd.payload.addr[0] & rxr.empty
            ),
        ]

        # Read-sense logic for receive side.
        m.d.comb += rxr.read_strobe.eq(
            self.bus.cmd.valid
            & ~self.bus.cmd.payload.lanes.any()
            & ~self.bus.cmd.payload.addr[0]
        )

        # Write logic for TX side.
        m.d.comb += txr.thr_write.payload.eq(self.bus.cmd.payload.data[:8])

        m.d.comb += txr.thr_write.valid.eq(
            self.bus.cmd.valid
            & self.bus.cmd.payload.lanes[0]
            & self.bus.cmd.payload.addr[0]
        )

        return m

