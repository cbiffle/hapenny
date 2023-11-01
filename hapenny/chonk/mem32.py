# Reusable memory with our bus interface.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from hapenny import StreamSig, AlwaysReady, mux
from hapenny.bus import BusPort

import hapenny.mem # for stitching together 16-bit primitives

class BasicMemory(Elaboratable):
    """A dead-simple 32-bit-wide memory with the Hapenny bus interface.

    This uses an Amaranth generic memory internally, which relies on inference
    in the synthesis tools to map to a specific type of resource such as block
    RAM. In practice it won't map to uninitialized RAM (like the iCE40UP5K's
    SPRAM) because Amaranth insists on generating it with an initializer; for
    that you'll need another module.

    Parameters
    ----------
    depth (integer): number of 32-bit words in the memory. If omitted,
        contents must be provided, and depth is inferred from len(contents).
    contents (list of integer): initialization contents of the memory. If
        omitted, depth must be provided, and the RAM is implicitly zeroed.
    read_only (boolean): if overridden to True, the memory will not respond to
        write strobes. This is useful for using an initialized block RAM as a
        program ROM.

    Attributes
    ----------
    bus: a BusPort with the minimum number of addr bits required to address
        'depth' words, and a 32-bit data path.
    """

    def __init__(self, *,
                 depth = None,
                 contents = [],
                 read_only = False):
        super().__init__()

        if depth is None:
            assert len(contents) > 0, "either depth or contents must be provided"
            depth = len(contents)

        addr_bits = (depth - 1).bit_length()

        self.bus = BusPort(addr = addr_bits, data = 32).flip().create()

        self.m = Memory(
            width = 32,
            depth = depth,
            name = "basicram",
            init = contents,
        )

        self.read_only = False

    def elaborate(self, platform):
        m = Module()

        m.submodules.m = self.m

        rp = self.m.read_port(transparent = False)


        m.d.comb += [
            rp.addr.eq(self.bus.cmd.payload.addr),
            rp.en.eq(self.bus.cmd.valid & (self.bus.cmd.payload.lanes == 0)),
            self.bus.resp.eq(rp.data),
        ]

        if not self.read_only:
            wp = self.m.write_port(granularity = 8)
            m.d.comb += [
                wp.addr.eq(self.bus.cmd.payload.addr),
                wp.data.eq(self.bus.cmd.payload.data),
            ]
            for i, lane in enumerate(self.bus.cmd.payload.lanes):
                m.d.comb += wp.en[i].eq(self.bus.cmd.valid & lane)

        return m

class SpramMemory(Component):
    """A pair of 256 kiB / 32 kiB SPRAMs on the UP5K, joined to make a 32-bit
    wide memory.

    This module exists because getting Amaranth to generate a memory that Yosys
    is willing to map to SPRAM is currently hard.

    SPRAMs are uninitialized at reset and can retain content across both design
    and device resets. As a result, this module doesn't support a read_only
    mode, because its contents would be indeterminate (yet not random enough to
    be interesting).

    Attributes
    ----------
    bus: bus interface with 14 address bits.
    """
    bus: In(BusPort(addr = 14, data = 32))

    def elaborate(self, platform):
        m = Module()

        m.submodules.lo = lo = hapenny.mem.SpramMemory()
        m.submodules.hi = hi = hapenny.mem.SpramMemory()

        m.d.comb += [
            lo.bus.cmd.valid.eq(self.bus.cmd.valid),
            lo.bus.cmd.payload.addr.eq(self.bus.cmd.payload.addr),
            lo.bus.cmd.payload.data.eq(self.bus.cmd.payload.data[:16]),
            lo.bus.cmd.payload.lanes.eq(self.bus.cmd.payload.lanes[:2]),

            hi.bus.cmd.valid.eq(self.bus.cmd.valid),
            hi.bus.cmd.payload.addr.eq(self.bus.cmd.payload.addr),
            hi.bus.cmd.payload.data.eq(self.bus.cmd.payload.data[16:]),
            hi.bus.cmd.payload.lanes.eq(self.bus.cmd.payload.lanes[2:]),

            self.bus.resp[:16].eq(lo.bus.resp),
            self.bus.resp[16:].eq(hi.bus.resp),
        ]

        return m

