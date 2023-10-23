from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from hapenny import StreamSig, AlwaysReady

class BusCmd(Signature):
    def __init__(self, *, addr, data):
        if isinstance(data, int):
            lanes = (data + 7) // 8
        else:
            lanes = (data.width + 7) // 8
        super().__init__({
            'addr': Out(addr),
            'lanes': Out(lanes),
            'data': Out(data)
        })

class BusPort(Signature):
    def __init__(self, *, addr, data):
        super().__init__({
            'cmd': Out(AlwaysReady(BusCmd(addr=addr, data=data))),
            'resp': In(data),
        })

def partial_decode(m, bus, width):
    port = BusPort(addr = width, data = bus.cmd.payload.data.shape()).flip().create()
    m.d.comb += [
        bus.cmd.payload.addr.eq(port.cmd.payload.addr),
        bus.cmd.payload.data.eq(port.cmd.payload.data),
        bus.cmd.payload.lanes.eq(port.cmd.payload.lanes),
        bus.cmd.valid.eq(port.cmd.valid),

        port.resp.eq(bus.resp),
    ]
    return port

class SimpleFabric(Elaboratable):
    def __init__(self, devices):
        assert len(devices) > 0
        data_bits = max(p.cmd.payload.data.shape().width for p in devices)
        addr_bits = max(p.cmd.payload.addr.shape().width for p in devices)
        sig = BusPort(addr = addr_bits, data = data_bits).flip()
        for d in devices:
            assert sig.is_compliant(d)
        self.devices = devices
        self.extra_bits = (len(devices) - 1).bit_length()
        self.addr_bits = addr_bits
        self.data_bits = data_bits

        self.bus = BusPort(addr = addr_bits + self.extra_bits, data =
                                 data_bits).flip().create()

    def elaborate(self, platform):
        m = Module()

        # Registered; set when a previously addressed device needs to make a
        # response.
        expecting = Signal(1)
        # When expecting, this is the address of the device.
        expecting_id = Signal(self.extra_bits)
        devid = Signal(self.extra_bits)
        m.d.comb += devid.eq(self.bus.cmd.payload.addr[self.addr_bits:])

        # Stop expecting automatically after one cycle.
        with m.If(expecting):
            m.d.sync += expecting.eq(0)
        # Record that we are expecting any time we see a load transaction.
        with m.If(self.bus.cmd.valid & (self.bus.cmd.payload.lanes == 0)):
            m.d.sync += [
                expecting.eq(1),
                expecting_id.eq(devid),
            ]


        for (i, d) in enumerate(self.devices):
            # Fan out the incoming address, data, and lanes to every device.
            m.d.comb += [
                d.cmd.payload.addr.eq(self.bus.cmd.payload.addr),
                d.cmd.payload.data.eq(self.bus.cmd.payload.data),
                d.cmd.payload.lanes.eq(self.bus.cmd.payload.lanes),
            ]
            # Only propagate cmd valid to the specific addressed device.
            dv = Signal(1, name = f"valid_{i}")
            m.d.comb += [
                dv.eq(self.bus.cmd.valid & (devid == i)),
                d.cmd.valid.eq(dv),
            ]

        # Fan the response data in based on who we're listening to.
        response_data = None
        for (i, d) in enumerate(self.devices):
            data = d.resp & (expecting & (expecting_id ==
                                                  i)).replicate(self.data_bits)
            if response_data is None:
                response_data = data
            else:
                response_data = response_data | data

        m.d.comb += self.bus.resp.eq(response_data)

        return m
