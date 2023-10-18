from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from hapenny import StreamSig, AlwaysReady
from hapenny.bus import BusPort

class OutputPort(Component):
    bus: In(BusPort(addr = 0, data = 16))

    def __init__(self, pins):
        super().__init__()
        self.pins = Signal(pins)

    def elaborate(self, platform):
        m = Module()

        with m.If(self.bus.cmd.valid & self.bus.cmd.payload.lanes[0]):
            m.d.sync += self.pins[:8].eq(self.bus.cmd.payload.data[:8])
        with m.If(self.bus.cmd.valid & self.bus.cmd.payload.lanes[1]):
            m.d.sync += self.pins[8:].eq(self.bus.cmd.payload.data[8:])

        return m
