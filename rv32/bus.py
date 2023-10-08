from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from rv32 import StreamSig, AlwaysReady

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
            'cmd': Out(StreamSig(BusCmd(addr=addr, data=data))),
            'resp': In(StreamSig(data)),
        })

def partial_decode(m, bus, width):
    port = BusPort(addr = width, data = bus.cmd.payload.data.shape()).flip().create()
    m.d.comb += [
        bus.cmd.payload.addr.eq(port.cmd.payload.addr),
        bus.cmd.payload.data.eq(port.cmd.payload.data),
        bus.cmd.payload.lanes.eq(port.cmd.payload.lanes),
        bus.cmd.valid.eq(port.cmd.valid),
        port.cmd.ready.eq(bus.cmd.ready),

        port.resp.valid.eq(bus.resp.valid),
        port.resp.payload.eq(bus.resp.payload),
        bus.resp.ready.eq(port.resp.ready),
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

        cmd_readies = Signal(len(self.devices))
        resp_valids = Signal(len(self.devices))
        for (i, d) in enumerate(self.devices):
            m.d.comb += [
                cmd_readies[i].eq(d.cmd.ready),
                resp_valids[i].eq(d.resp.valid),
            ]

        # registered; set when a previously addressed device needs to make a
        # response.
        expecting = Signal(1)
        # When expecting, this is the address of the device.
        expecting_id = Signal(self.extra_bits)
        # Stop expecting if we see it happen.
        with m.If(self.bus.resp.ready & self.bus.resp.valid):
            m.d.sync += expecting.eq(0)
        # Record that we are expecting any time we see a load transaction.
        with m.If(self.bus.cmd.ready & self.bus.cmd.valid &
                  (self.bus.cmd.payload.lanes == 0)):
            m.d.sync += [
                expecting.eq(1),
                expecting_id.eq(self.bus.cmd.payload.addr[self.addr_bits:]),
            ]

        # Set when we're not stalling.
        ok = Signal(1)
        m.d.comb += ok.eq(~expecting | (resp_valids >> expecting_id)[0])

        devid = Signal(self.extra_bits)
        m.d.comb += devid.eq(self.bus.cmd.payload.addr[self.addr_bits:])

        for (i, d) in enumerate(self.devices):
            # Fan out the incoming address, data, and lanes to every device.
            m.d.comb += [
                d.cmd.payload.addr.eq(self.bus.cmd.payload.addr),
                d.cmd.payload.data.eq(self.bus.cmd.payload.data),
                d.cmd.payload.lanes.eq(self.bus.cmd.payload.lanes),
            ]
            # Only propagate cmd valid to the specific addressed device, and
            # don't propagate it if we're stalling.
            dv = Signal(1, name = f"valid_{i}")
            m.d.comb += [
                dv.eq(self.bus.cmd.valid & ok & (devid == i)),
                d.cmd.valid.eq(dv),
            ]
            # Only propagate response ready signal to the device we're expecting
            # to hear from.
            m.d.comb += d.resp.ready.eq(self.bus.resp.ready & expecting &
                                        (expecting_id == i))

        # Screen the command-ready signal based on the address bits and whether
        # we're stalling.
        m.d.comb += self.bus.cmd.ready.eq(ok & (cmd_readies >> devid)[0])
        # Screen the response-valid signal similarly, but don't stall it.
        m.d.comb += self.bus.resp.valid.eq((resp_valids >> devid)[0])

        # Fan the response data in based on who we're listening to.
        response_data = None
        for (i, d) in enumerate(self.devices):
            data = d.resp.payload & (expecting & (expecting_id ==
                                                  i)).replicate(self.data_bits)
            if response_data is None:
                response_data = data
            else:
                response_data = response_data | data

        m.d.comb += self.bus.resp.payload.eq(response_data)

        return m
