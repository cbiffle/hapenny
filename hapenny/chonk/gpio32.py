from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from hapenny.bus import BusPort

class OutputPort32(Component):
    bus: In(BusPort(addr = 0, data = 32))

    def __init__(self, pins):
        super().__init__()
        self.pins = Signal(pins)

    def elaborate(self, platform):
        m = Module()

        with m.If(self.bus.cmd.valid & self.bus.cmd.payload.lanes[0]):
            m.d.sync += self.pins[:8].eq(self.bus.cmd.payload.data[:8])
        with m.If(self.bus.cmd.valid & self.bus.cmd.payload.lanes[1]):
            m.d.sync += self.pins[8:16].eq(self.bus.cmd.payload.data[8:16])
        with m.If(self.bus.cmd.valid & self.bus.cmd.payload.lanes[2]):
            m.d.sync += self.pins[16:24].eq(self.bus.cmd.payload.data[16:24])
        with m.If(self.bus.cmd.valid & self.bus.cmd.payload.lanes[3]):
            m.d.sync += self.pins[24:].eq(self.bus.cmd.payload.data[24:])

        return m
