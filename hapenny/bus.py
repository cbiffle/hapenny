from functools import reduce

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
    assert width >= bus.cmd.payload.addr.shape().width, \
            "can't use partial_decode to make a bus narrower"
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
        print(f"fabric configured for {addr_bits} addr bits, {data_bits} data bits")
        for i, d in enumerate(devices):
            assert sig.is_compliant(d), \
                    f"device #{i} does not have {addr_bits} addr bits: {d.cmd.payload.addr.shape()}"
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

        # Start expecting on any load transaction. Stop expecting otherwise.
        m.d.sync += expecting.eq(self.bus.cmd.valid & (self.bus.cmd.payload.lanes == 0))
        # We can update this unconditionally every cycle because (1) responses
        # always come exactly one cycle after the request and (2) this ID is
        # ignored if we're not expecting.
        m.d.sync += expecting_id.eq(devid)

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
        response_data = []
        for (i, d) in enumerate(self.devices):
            data = d.resp & (expecting & (expecting_id ==
                                                  i)).replicate(self.data_bits)
            response_data.append(data)

        m.d.comb += self.bus.resp.eq(reduce(lambda a, b: a | b, response_data))

        return m
