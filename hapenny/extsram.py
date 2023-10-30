from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from hapenny import StreamSig, AlwaysReady
from hapenny.bus import BusPort

class ExternalSRAM(Component):
    """An interface to 16-bit-wide external asynchronous SRAM.

    Parameters
    ----------
    address_bits (integer): number of implemented address bits at the physical
        interface -- so, not including address bit 0 since the memory is 16
        bits wide.

    Attributes
    ----------
    sram_oe (out): output enable to SRAM, active high. Enables the SRAM's
        output drivers during a read cycle.
    sram_we (out): write enable to SRAM, active high.
    sram_lanes (out): byte select lines to SRAM, a 1 during a write means the
        corresponding byte is written, a 0 leaves it untouched.
    addr_to_sram (out): address, width determined by the 'address_bits'
        parameter.
    data_to_sram (out): 16-bit data path to SRAM. Unidirectional because FPGAs
        like that, becomes bidirectional at the I/O pin.
    data_from_sram (in): 16-bit data path from SRAM.

    bus (port): connection to the SoC bus.
    """
    clock_90: In(1)

    sram_oe: Out(1)
    sram_we: Out(1)
    sram_lanes: Out(2)
    data_to_sram: In(16)
    data_from_sram: Out(16)

    def __init__(self, *, address_bits):
        super().__init__()
        self.bus = BusPort(addr = address_bits, data = 16).flip().create()

        self.addr_to_sram = Signal(address_bits)

        self.address_bits = address_bits

    def elaborate(self, platform):
        m = Module()

        # Register the bus inputs when we're selected, so that we can maintain
        # stable outputs. Note that we register "lanes" separately from the
        # "write" signal because the SRAM requires lanes to be asserted to
        # read!
        r_addr = Signal(self.address_bits)
        r_data_to_sram = Signal(16)
        r_lanes = Signal(2)
        r_write = Signal()
        r_read = Signal()

        # Automatically clear any write request on the cycle after it occurs, so
        # that we don't sit there repeatedly writing from this interface while
        # the CPU is off doing other things.
        with m.If(r_write):
            m.d.sync += [
                r_write.eq(0),
                r_read.eq(0),
            ]
       
        # Copy any bus transaction into the registers. This will override the
        # clearing above.
        with m.If(self.bus.cmd.valid):
            m.d.sync += [
                r_addr.eq(self.bus.cmd.payload.addr),
                r_data_to_sram.eq(self.bus.cmd.payload.data),
                r_write.eq(self.bus.cmd.payload.lanes.any()),
                r_read.eq(~self.bus.cmd.payload.lanes.any()),
                # Our bus doesn't use lane signals on read. The external bus
                # does. Convert.
                r_lanes.eq(self.bus.cmd.payload.lanes
                    | (~self.bus.cmd.payload.lanes.any()).replicate(2)),
            ]

        # Present transactions from our registers on the bus output.
        m.d.comb += [
            self.addr_to_sram.eq(r_addr),
            self.data_to_sram.eq(r_data_to_sram),
            self.sram_lanes.eq(r_lanes),

            # Assert the (active high) output enable line whenever we're not
            # writing. Conversely, deassert it on write cycles. The phase
            # offset of our write-enable output gives the drivers time to turn
            # off.
            self.sram_oe.eq(r_read),
            # Combine our write enable (active high) with the incoming
            # phase-shifted clock to generate a write pulse in the center of
            # each write cycle.
            self.sram_we.eq(r_write & self.clock_90),
        ]
        # Responses come back combinationally, in what is likely to be the slow
        # path.
        m.d.comb += self.bus.resp.eq(self.data_from_sram)

        return m
