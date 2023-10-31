from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from hapenny import StreamSig, AlwaysReady, mux, oneof
from hapenny.bus import BusPort

class MinimalOutputPort(Component):
    """An absolutely dead-simple output port. Pipes any data written through to
    pins. Does not support reading back the state of the pins, or any fancy
    manipulation.

    Use this when space is at a premium; otherwise, see OutputPort. Also,
    measure the actual area requirement of the port -- in many cases, OutputPort
    is the same cost for more functionality.

    Memory Map
    ----------
    +00: pins (byte write supported)

    Parameters
    ----------
    pins (integer): number of pins to implement, 0-16.

    Attributes
    ----------
    bus (port): connection to the fabric.
    pins (signal array): output to pins.
    """
    bus: In(BusPort(addr = 0, data = 16))

    def __init__(self, pins):
        super().__init__()
        self.pins = Signal(pins)

    def elaborate(self, platform):
        m = Module()

        m.d.sync += self.pins[:8].eq(mux(
            self.bus.cmd.valid & self.bus.cmd.payload.lanes[0],
            self.bus.cmd.payload.data[:8],
            self.pins[:8],
        ))
        m.d.sync += self.pins[8:].eq(mux(
            self.bus.cmd.valid & self.bus.cmd.payload.lanes[1],
            self.bus.cmd.payload.data[8:],
            self.pins[8:],
        ))

        return m

class OutputPort(Component):
    """A block of general-purpose outputs that can be changed simultaneously.

    Memory Map
    ----------
    +0  sets pins when written
    +2  ORs value with current pin state
    +4  ANDs the complement of the value written with the current pin state.
    +6  XORs value with the current pin state

    All registers support byte writes to affect only half the pins.

    Parameters
    ----------
    pins (integer): number of pins to implement (1-16)
    read_back (boolean): when True (default), the state of the pins can be read
        back. When False, reads always return zero. Turning off read-back can
        save some space.

    Attributes
    ----------
    bus (port): connection to bus fabric
    pins (signal array): the output pins
    """
    bus: In(BusPort(addr = 2, data = 16))

    def __init__(self, pins, read_back = True):
        super().__init__()
        self.pins = Signal(pins)
        self.read_back = read_back

    def elaborate(self, platform):
        m = Module()

        a = self.bus.cmd.payload.addr
        d = self.bus.cmd.payload.data

        m.d.sync += self.pins[:8].eq(mux(
            self.bus.cmd.valid & self.bus.cmd.payload.lanes[0],
            oneof([
                (a == 1, self.pins[:8] | d[:8]),
                (a == 2, self.pins[:8] & ~d[:8]),
                (a == 3, self.pins[:8] ^ d[:8]),
             ], default = d[:8]),
            self.pins,
        ))
        m.d.sync += self.pins[8:].eq(mux(
            self.bus.cmd.valid & self.bus.cmd.payload.lanes[1],
            oneof([
                (a == 1, self.pins[8:] | d[8:]),
                (a == 2, self.pins[8:] & ~d[8:]),
                (a == 3, self.pins[8:] ^ d[8:]),
            ], default = d[8:]),
            self.pins,
        ))

        if self.read_back:
            # We can service reads trivially by just permanently connecting our
            # register to the bus.
            m.d.comb += self.bus.resp.eq(self.pins)

        return m

class InputPort(Component):
    """A simple input port peripheral. Can read the state of pins.

    Memory Map
    ----------
    +00: pins (read only, writes ignored)

    Parameters
    ----------
    pins (integer): number of pins to implement, 0-16.

    Attributes
    ----------
    bus (port): connection to the fabric.
    pins (signal array): input from pins.
    """
    bus: In(BusPort(addr = 0, data = 16))

    def __init__(self, pins):
        super().__init__()
        self.pins = Signal(pins)

    def elaborate(self, platform):
        m = Module()

        # Am I the simplest peripheral? I think so!

        # Register inputs to cut the path from pins, and also to avoid leaking
        # metastability.
        pins_r = Signal(self.pins.shape().width)
        m.d.sync += pins_r.eq(self.pins)

        # Always output the state from the last cycle onto the bus. This has the
        # nice side effect of returning the state of the pins when the read was
        # _issued_ rather than when it completed.
        m.d.comb += self.bus.resp.eq(pins_r)

        return m
