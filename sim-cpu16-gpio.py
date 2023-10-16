from amaranth import *
from amaranth.sim import Simulator, Delay, Settle
from amaranth.back import verilog
from amaranth.lib.wiring import *
from amaranth.lib.enum import *

from hapenny.cpu16 import Cpu
from hapenny.bus import BusPort, partial_decode, SimpleFabric
from hapenny.gpio import OutputPort
from hapenny import *

class TestPhase(Enum):
    INIT = 0
    SETUP = 1
    RUN = 2
    CHECK = 3

class TestMemory(Component):
    bus: In(BusPort(addr = 8, data = 16))

    inspect: In(BusPort(addr = 8, data = 16))

    def __init__(self, contents):
        super().__init__()

        self.m = Memory(
            width = 16,
            depth = 256,
            name = "testram",
            init = contents,
        )

    def elaborate(self, platform):
        m = Module()

        m.submodules.m = self.m

        rp = self.m.read_port(transparent = False)
        wp = self.m.write_port(granularity = 8)

        m.d.comb += [
            rp.addr.eq(self.bus.cmd.payload.addr),
            rp.en.eq(self.bus.cmd.valid & (self.bus.cmd.payload.lanes == 0b00)),

            wp.addr.eq(self.bus.cmd.payload.addr),
            wp.data.eq(self.bus.cmd.payload.data),
            wp.en[0].eq(self.bus.cmd.valid & self.bus.cmd.payload.lanes[0]),
            wp.en[1].eq(self.bus.cmd.valid & self.bus.cmd.payload.lanes[1]),

            # Nothing causes this memory to stop being available.
            self.bus.cmd.ready.eq(1),
        ]

        # Delay the read enable signal by one cycle to use as output valid.
        # TODO this isn't really correct and ignores ready.
        delayed_read = Signal(1)
        m.d.sync += delayed_read.eq(rp.en)

        m.d.comb += [
            self.bus.resp.valid.eq(delayed_read),
            self.bus.resp.payload.eq(rp.data),
        ]

        # Do it all again for the inspect port

        inspect_rp = self.m.read_port(transparent = False)
        inspect_wp = self.m.write_port(granularity = 8)

        m.d.comb += [
            inspect_rp.addr.eq(self.inspect.cmd.payload.addr[1:]),
            inspect_rp.en.eq(self.inspect.cmd.valid &
                             (self.inspect.cmd.payload.lanes == 0)),

            inspect_wp.addr.eq(self.inspect.cmd.payload.addr[1:]),
            inspect_wp.data.eq(self.inspect.cmd.payload.data),
            inspect_wp.en[0].eq(self.inspect.cmd.valid &
                                self.inspect.cmd.payload.lanes[0]),
            inspect_wp.en[1].eq(self.inspect.cmd.valid &
                                self.inspect.cmd.payload.lanes[1]),

            # Nothing causes this memory to stop being available.
            self.inspect.cmd.ready.eq(1),
        ]

        # Delay the read enable signal by one cycle to use as output valid.
        # TODO this isn't really correct and ignores ready.
        inspect_delayed_read = Signal(1)
        m.d.sync += inspect_delayed_read.eq(inspect_rp.en)

        m.d.comb += [
            self.inspect.resp.valid.eq(inspect_delayed_read),
            self.inspect.resp.payload.eq(inspect_rp.data),
        ]

        return m

def halt():
    yield uut.halt_request.eq(1)
    attempts = 0
    while (yield uut.halted) == 0:
        attempts += 1
        if attempts > 40:
            raise Exception("CPU didn't halt after 40 cycles")
        yield
        yield Settle()

def resume():
    yield uut.halt_request.eq(0)
    attempts = 0
    while (yield uut.halted) == 1:
        attempts += 1
        if attempts > 40:
            raise Exception("CPU didn't resume after 40 cycles")
        yield
        yield Settle()

def single_step():
    yield from resume()
    # skip FETCH state
    yield
    yield from halt()

def write_ureg(reg, value):
    yield uut.debug.reg_write.payload.addr.eq(reg)
    yield uut.debug.reg_write.payload.value.eq(value)
    yield uut.debug.reg_write.valid.eq(1)
    yield
    yield uut.debug.reg_write.valid.eq(0)

def write_reg(reg, value):
    yield from write_ureg(reg, value & 0xFFFF)
    yield from write_ureg(reg | 0x20, value >> 16)

def read_ureg(reg):
    yield uut.debug.reg_read.payload.eq(reg)
    yield uut.debug.reg_read.valid.eq(1)
    yield
    yield uut.debug.reg_read.valid.eq(0)
    yield Settle()
    return (yield uut.debug.reg_value.payload)

def read_reg(reg):
    bottom = yield from read_ureg(reg)
    top = yield from read_ureg(reg | 0x20)
    return bottom | (top << 16)

def write_pc(value):
    yield uut.debug.pc_write.payload.eq(value)
    yield uut.debug.pc_write.valid.eq(1)
    yield
    yield uut.debug.pc_write.valid.eq(0)

def read_pc():
    yield Settle()
    return (yield uut.debug.pc)

def write_mem(addr, value):
    yield mem.inspect.cmd.payload.addr.eq(addr)
    yield mem.inspect.cmd.payload.data.eq(value & 0xFFFF)
    yield mem.inspect.cmd.payload.lanes.eq(0b11)
    yield mem.inspect.cmd.valid.eq(1)
    yield
    yield mem.inspect.cmd.payload.addr.eq(addr + 2)
    yield mem.inspect.cmd.payload.data.eq(value >> 16)
    yield mem.inspect.cmd.payload.lanes.eq(0b11)
    yield mem.inspect.cmd.valid.eq(1)
    yield
    yield mem.inspect.cmd.payload.lanes.eq(0)
    yield mem.inspect.cmd.valid.eq(0)

def read_mem(addr):
    yield mem.inspect.cmd.payload.addr.eq(addr)
    yield mem.inspect.cmd.payload.lanes.eq(0)
    yield mem.inspect.cmd.valid.eq(1)
    yield
    yield mem.inspect.cmd.payload.addr.eq(addr + 2)
    yield mem.inspect.cmd.valid.eq(1)
    yield Settle()
    bottom = yield mem.inspect.resp.payload
    yield
    yield mem.inspect.cmd.valid.eq(0)
    yield Settle()
    top = yield mem.inspect.resp.payload

    return bottom | (top << 16)

def test_inst(name, inst, *, before = {}, after = {}):
    print(f"{name} ... ", end='')
    yield phase.eq(TestPhase.SETUP)
    for r in range(1, 32):
        if r not in before:
            yield from write_reg(r, 0xDEADBEEF)

    start_address = 0

    for key, value in before.items():
        if isinstance(key, int):
            yield from write_reg(key, value)
        elif isinstance(key, str) and key[0] == '@':
            yield from write_mem(int(key[1:], 16), value)
        elif key == 'PC' or key == 'pc':
            yield from write_pc(value)
            start_address = value
        else:
            raise Exception(f"unexpected before key: {key}")

    yield from write_mem(start_address, inst)
    yield from write_pc(start_address)

    yield phase.eq(TestPhase.RUN)
    yield from single_step()

    yield phase.eq(TestPhase.CHECK)
    try:
        for key, value in after.items():
            if isinstance(key, int):
                actual = yield from read_reg(key)
                assert actual == value, \
                        f"r{key} should be 0x{value:x} but is 0x{actual:x}"
            elif isinstance(key, str) and key[0] == '@':
                addr = int(key[1:], 16)
                actual = yield from read_mem(addr)
                assert actual == value, \
                        f"M[0x{addr:x}] should be 0x{value:x} but is 0x{actual:x}"
            elif key == 'PC' or key == 'pc':
                actual = yield from read_pc()
                assert actual == value, \
                        f"PC should be 0x{value:x} but is 0x{actual:x}"
            else:
                raise Exception(f"unexpected after key: {key}")
        for r in range(1, 32):
            if r not in after:
                if r in before:
                    expected = before[r]
                else:
                    expected = 0xDEADBEEF
                actual = yield from read_reg(r)
                assert actual == expected, \
                    f"r{r} should not have changed but is now 0x{actual:x}"



    except Exception as e:
        raise Exception(f"test case {name} failed due to above exception") from e

    print("PASS")


if __name__ == "__main__":
    m = Module()
    m.submodules.uut = uut = Cpu()
    m.submodules.mem = mem = TestMemory([
        # 00000000 <reset>:
        #    0:   20000093                li      ra,512
        0x0093,
        0x2000,
        #    4:   00100113                li      sp,1
        0x0113,
        0x0010,
        #    8:   00100193                li      gp,1
        0x0193,
        0x0010,
        # 
        # 0000000c <loop>:
        #    c:   00209023                sh      sp,0(ra)
        0x9023,
        0x0020,
        #   10:   00018213                mv      tp,gp
        0x8213,
        0x0001,
        # 
        # 00000014 <loop2>:
        #   14:   fff20213                addi    tp,tp,-1
        0x0213,
        0xfff2,
        #   18:   fe020ee3                beqz    tp,14 <loop2>
        0x0ee3,
        0xfe02,
        #   1c:   00009023                sh      zero,0(ra)
        0x9023,
        0x0000,
        #   20:   00018213                mv      tp,gp
        0x8213,
        0x0001,
        # 
        # 00000024 <loop3>:
        #   24:   fff20213                addi    tp,tp,-1
        0x0213,
        0xfff2,
        #   28:   fe0206e3                beqz    tp,14 <loop2>
        0x06e3,
        0xfe02,
        #   2c:   fe1ff06f                j       c <loop>
        0xf06f,
        0xfe1f,
    ])
    m.submodules.port = port = OutputPort(1)

    phase = Signal(TestPhase)

    m.submodules.bus = fabric = SimpleFabric([
        mem.bus,
        partial_decode(m, port.bus, 8),
    ])

    connect(m, uut.bus, partial_decode(m, fabric.bus, 31))

    ports = [
        phase,
        uut.ustate,
        uut.hi,
        uut.bus.cmd.valid,
        uut.bus.cmd.payload.addr,
        uut.bus.cmd.payload.data,
        uut.bus.cmd.payload.lanes,
        uut.bus.resp.valid,
        uut.bus.resp.payload,
        port.pins,
    ]

    sim = Simulator(m)
    sim.add_clock(1e-6)

    with sim.write_vcd(vcd_file="test.vcd", gtkw_file="test.gtkw", traces=ports):
        sim.run_until(100e-6, run_passive = True)
