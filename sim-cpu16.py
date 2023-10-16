from amaranth import *
from amaranth.sim import Simulator, Delay, Settle
from amaranth.back import verilog
from amaranth.lib.wiring import *
from amaranth.lib.enum import *

from hapenny.cpu16 import Cpu
from hapenny.bus import BusPort, partial_decode, SimpleFabric
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
    start = yield cycle_counter
    # do not generate halt during fetch state
    yield
    yield from halt()
    # Subtract 1 here to exclude the trailing fetch state where the halt was
    # acknowledged.
    return (yield cycle_counter) - 1 - start

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

    if isinstance(inst, int):
        instruction_count = 1
        yield from write_mem(start_address, inst)
    elif isinstance(inst, list):
        instruction_count = len(inst)
        for (i, word) in enumerate(inst):
            yield from write_mem(start_address + 4 * i, word)
    else:
        raise Exception(f"invalid instruction value: {inst}")

    yield from write_pc(start_address)

    yield phase.eq(TestPhase.RUN)
    cycle_count = 0
    for i in range(instruction_count):
        cycle_count += yield from single_step()

    print(f"({cycle_count} cyc) ", end='')

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
        if 'PC' not in after:
            actual = yield from read_pc()
            value = start_address + instruction_count * 4
            assert actual == value, \
                    f"PC should be 0x{value:x} but is 0x{actual:x}"




    except Exception as e:
        raise Exception(f"test case {name} failed due to above exception") from e

    print("PASS")


if __name__ == "__main__":
    m = Module()
    m.submodules.uut = uut = Cpu(
        has_interrupt = "m",
    )
    m.submodules.mem = mem = TestMemory([
        0b00000000000000000000_00000_1101111, # JAL x0, .
    ])
    m.submodules.mem2 = mem2 = TestMemory([
        0b00000000000000000000_00000_1101111, # JAL x0, .
    ])

    phase = Signal(TestPhase)
    cycle_counter = Signal(32)
    m.d.sync += cycle_counter.eq(cycle_counter + 1)

    m.submodules.bus = fabric = SimpleFabric([
        partial_decode(m, mem.bus, 31),
    ])

    connect(m, uut.bus, fabric.bus)

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
    ]

    verilog_src = verilog.convert(m, ports=ports)
    with open("sim-cpu.v", "w") as v:
        v.write(verilog_src)

    sim = Simulator(m)
    sim.add_clock(1e-6)

    def process():
        yield from halt()
        yield from test_inst(
            "LUI x1, 0xAAAAA000",
            0b10101010101010101010_00001_0110111,
            after={
                1: 0xAAAAA000,
                'PC': 4,
            },
        )

        yield from test_inst(
            "AUIPC x1, 0xAAAAA000",
            0b10101010101010101010_00001_0010111,
            before={
                'PC': 0xCAFC,
            },
            after={
                1: (0xAAAAA000 + 0xCAFC) & 0xFFFFFFFF,
                'PC': 0xCAFC + 4,
            },
        )

        yield from test_inst(
            "JAL x9, .",
            0b00000000000000000000_01001_1101111,
            after={
                9: 4,
                'PC': 0,
            },
        )

        yield from test_inst(
            "JAL x9, -4",
            0b1_1111111110_1_11111111_01001_1101111,
            before={
                'PC': 12,
            },
            after={
                9: 16,
                'PC': 8,
            },
        )

        yield from test_inst(
            "JALR x1, x9, 0x456",
            0b010001010110_01001_000_00001_1100111,
            before={
                9: 0xCAFE,
            },
            after={
                9: 0xCAFE,
                1: 4,
                'PC': 0xCAFE + 0x456,
            },
        )

        branch_cases = [
            ("EQ", 0b000, 0xCAFEBABE, 0xCAFEBABE, True),
            ("EQ", 0b000, 0xCAFEBABE, 0xBAADF00D, False),
            ("NE", 0b001, 0xCAFEBABE, 0xBAADF00D, True),
            ("NE", 0b001, 0xCAFEBABE, 0xCAFEBABE, False),
            ("LT", 0b100, 0xCAFEBABE, 0x12345678, True),
            ("LT", 0b100, 0x12345678, 0xCAFEBABE, False),
            ("GE", 0b101, 0x12345678, 0xCAFEBABE, True),
            ("GE", 0b101, 0xCAFEBABE, 0x12345678, False),
            ("LTU", 0b110, 0x12345678, 0xCAFEBABE, True),
            ("LTU", 0b110, 0xCAFEBABE, 0x12345678, False),
            ("GEU", 0b111, 0xCAFEBABE, 0x12345678, True),
            ("GEU", 0b111, 0x12345678, 0xCAFEBABE, False),
        ]

        for name, opc, x1, x2, taken in branch_cases:
            desc = f"B{name} x1, x2, 0x400"
            if not taken:
                desc += " (not taken)"

            yield from test_inst(
                desc,
                0b0_100000_00010_00001_000_0000_0_1100011 | (opc << 12),
                before={
                    'PC': 0xF000,
                    1: x1,
                    2: x2,
                },
                after={
                    'PC': 0xF000 + (0x400 if taken else 4),
                },
            )

        load_cases = [
            ("LW", 0b010, 0x12345678, 0, 0x12345678),

            ("LB", 0b000, 0x12345678, 0, 0x78),
            ("LB", 0b000, 0x12345678, 1, 0x56),
            ("LB", 0b000, 0x12345678, 2, 0x34),
            ("LB", 0b000, 0x12345678, 3, 0x12),
            ("LB", 0b000, 0x92B4D6F8, 0, 0xFFFF_FFF8),
            ("LB", 0b000, 0x92B4D6F8, 1, 0xFFFF_FFD6),
            ("LB", 0b000, 0x92B4D6F8, 2, 0xFFFF_FFB4),
            ("LB", 0b000, 0x92B4D6F8, 3, 0xFFFF_FF92),

            ("LW", 0b001, 0x12345678, 0, 0x5678),
            ("LW", 0b001, 0x12345678, 2, 0x1234),
            ("LW", 0b001, 0x92B4D6F8, 0, 0xFFFF_D6F8),
            ("LW", 0b001, 0x92B4D6F8, 2, 0xFFFF_92B4),

            ("LBU", 0b100, 0x92B4D6F8, 0, 0xF8),
            ("LBU", 0b100, 0x92B4D6F8, 1, 0xD6),
            ("LBU", 0b100, 0x92B4D6F8, 2, 0xB4),
            ("LBU", 0b100, 0x92B4D6F8, 3, 0x92),

            ("LHU", 0b101, 0x92B4D6F8, 0, 0xD6F8),
            ("LHU", 0b101, 0x92B4D6F8, 2, 0x92B4),
        ]
        for mnem, opc, memword, off, reg in load_cases:
            desc = f"{mnem} x1, 0xAC(x2)"
            if off != 0:
                desc += f" (with x2={off})"

            yield from test_inst(
                desc,
                0b000010101100_00010_000_00001_0000011 | (opc << 12),
                before={
                    1: 0xBAADF00D,
                    2: off,
                    '@AC': memword,
                },
                after={
                    1: reg,
                    2: off,
                    '@AC': memword,
                    'PC': 4,
                },
            )

        store_cases = [
            ("SW", 0b010, 0xDEADBEEF, 0x12345678, 0, 0x12345678),

            ("SB", 0b000, 0xDEADBEEF, 0x12345678, 0, 0xDEADBE78),
            ("SB", 0b000, 0xDEADBEEF, 0x12345678, 1, 0xDEAD78EF),
            ("SB", 0b000, 0xDEADBEEF, 0x12345678, 2, 0xDE78BEEF),
            ("SB", 0b000, 0xDEADBEEF, 0x12345678, 3, 0x78ADBEEF),

            ("SH", 0b001, 0xDEADBEEF, 0x12345678, 0, 0xDEAD5678),
            ("SH", 0b001, 0xDEADBEEF, 0x12345678, 2, 0x5678BEEF),
        ]
        for mnem, opc, prevmem, write, off, expected in store_cases:
            desc = f"{mnem} x1, 0xAC(x2)"
            if off != 0:
                desc += f" (with x2={off})"

            yield from test_inst(
                desc,
                0b0000101_00001_00010_000_01100_0100011 | (opc << 12),
                before={
                    1: write,
                    2: off,
                    '@AC': prevmem,
                },
                after={
                    1: write,
                    2: off,
                    '@AC': expected,
                    'PC': 4,
                },
            )

        yield from test_inst(
            "ADD x1, x2, x3",
            0b0000000_00011_00010_000_00001_0110011,
            before={
                2: 0xCAFEBABE,
                3: 0xBAADF00D,
            },
            after={
                1: (0xCAFEBABE + 0xBAADF00D) & 0xFFFFFFFF,
            },
        )
        yield from test_inst(
            "OR x1, x2, x3",
            0b0000000_00011_00010_110_00001_0110011,
            before={
                2: 0xCAFEBABE,
                3: 0xBAADF00D,
            },
            after={
                1: 0xCAFEBABE | 0xBAADF00D,
            },
        )

        yield from test_inst(
            "AND x1, x2, x3",
            0b0000000_00011_00010_111_00001_0110011,
            before={
                2: 0xCAFEBABE,
                3: 0xBAADF00D,
            },
            after={
                1: 0xCAFEBABE & 0xBAADF00D,
            },
        )

        yield from test_inst(
            "SLT x1, x2, x3 (where x2 < x3)",
            0b0000000_00011_00010_010_00001_0110011,
            before={
                2: 0xCAFEBABE,
                3: 0x12345678,
            },
            after={
                1: 1,
            },
        )
        yield from test_inst(
            "SLT x1, x2, x3 (where x2 >= x3)",
            0b0000000_00011_00010_010_00001_0110011,
            before={
                2: 0x12345678,
                3: 0xCAFEBABE,
            },
            after={
                1: 0,
            },
        )
        yield from test_inst(
            "SLTU x1, x2, x3 (where x2 < x3)",
            0b0000000_00011_00010_011_00001_0110011,
            before={
                2: 0x12345678,
                3: 0xCAFEBABE,
            },
            after={
                1: 1,
            },
        )
        yield from test_inst(
            "SLTU x1, x2, x3 (where x2 >= x3)",
            0b0000000_00011_00010_011_00001_0110011,
            before={
                2: 0xCAFEBABE,
                3: 0x12345678,
            },
            after={
                1: 0,
            },
        )
        yield from test_inst(
            "ADDI x1, x2, 0x123",
            0b000100100011_00010_000_00001_0010011,
            before={
                2: 0xCAFEBABE,
            },
            after={
                1: (0xCAFEBABE + 0x123) & 0xFFFFFFFF,
            },
        )
        yield from test_inst(
            "ADDI x1, x2, -0x123",
            0b111011011101_00010_000_00001_0010011,
            before={
                2: 0xCAFEBABE,
            },
            after={
                1: (0xCAFEBABE + -0x123) & 0xFFFFFFFF,
            },
        )
        yield from test_inst(
            "SUB x1, x2, x3",
            0b0100000_00011_00010_000_00001_0110011,
            before={
                2: 0xCAFEBABE,
                3: 0xBAADF00D,
            },
            after={
                1: (0xCAFEBABE - 0xBAADF00D) & 0xFFFFFFFF,
            },
        )
        yield from test_inst(
            "XORI x1, x2, 0x123",
            0b000100100011_00010_100_00001_0010011,
            before={
                2: 0xCAFEBABE,
            },
            after={
                1: 0xCAFEBABE ^ 0x123,
            },
        )
        yield from test_inst(
            "ORI x1, x2, 0x123",
            0b000100100011_00010_110_00001_0010011,
            before={
                2: 0xCAFEBABE,
            },
            after={
                1: 0xCAFEBABE | 0x123,
            },
        )
        yield from test_inst(
            "ANDI x1, x2, 0x123",
            0b000100100011_00010_111_00001_0010011,
            before={
                2: 0xCAFEBABE,
            },
            after={
                1: 0xCAFEBABE & 0x123,
            },
        )

        yield from test_inst(
            "SLTI x1, x2, 0x123 (where x2 < 0x123)",
            0b001100100001_00010_010_00001_0010011,
            before={
                2: 0xCAFEBABE,
            },
            after={
                1: 1,
            },
        )
        yield from test_inst(
            "SLTI x1, x2, 0x123 (where x2 >= 0x123)",
            0b001100100001_00010_010_00001_0010011,
            before={
                2: 0x12345678,
            },
            after={
                1: 0,
            },
        )
        yield from test_inst(
            "SLTIU x1, x2, 0x123 (where x2 < 0x123)",
            0b001100100001_00010_011_00001_0010011,
            before={
                2: 0x42,
            },
            after={
                1: 1,
            },
        )
        yield from test_inst(
            "SLTIU x1, x2, 0x123 (where x2 >= 0x123)",
            0b001100100001_00010_011_00001_0010011,
            before={
                2: 0xCAFEBABE,
            },
            after={
                1: 0,
            },
        )
        for amt in [0, 1, 5, 31, 32]:
            yield from test_inst(
                f"SLL x1, x2, x3 (with x3={amt})",
                0b0000000_00011_00010_001_00001_0110011,
                before={
                    2: 0xCAFEBABE,
                    3: amt,
                },
                after={
                    1: (0xCAFEBABE << (amt & 0x1F)) & 0xFFFFFFFF,
                },
            )

        for amt in [0, 1, 5, 31]:
            yield from test_inst(
                f"SLLI x1, x2, {amt}",
                0b0000000_00000_00010_001_00001_0010011 | amt << 20,
                before={
                    2: 0xCAFEBABE,
                },
                after={
                    1: (0xCAFEBABE << (amt & 0x1F)) & 0xFFFFFFFF,
                },
            )

        for amt in [0, 1, 5, 31, 32]:
            yield from test_inst(
                f"SRL x1, x2, x3 (with x3={amt})",
                0b0000000_00011_00010_101_00001_0110011,
                before={
                    2: 0xCAFEBABE,
                    3: amt,
                },
                after={
                    1: 0xCAFEBABE >> (amt & 0x1F),
                },
            )

        for x2 in [0xCAFEBABE, 0xF00D]: # one negative, one positive
            for amt in [0, 1, 5, 31, 32]:
                if x2 & 0x80000000 == 0:
                    result = x2 >> (amt & 0x1F)
                else:
                    result = ((0xFFFFFFFF_00000000 | x2) >> (amt & 0x1F)) & 0xFFFFFFFF
                yield from test_inst(
                    f"SRA x1, x2, x3 (with x2=0x{x2:x}, x3={amt})",
                    0b0100000_00011_00010_101_00001_0110011,
                    before={
                        2: x2,
                        3: amt,
                    },
                    after={
                        1: result,
                    },
                )

        yield from test_inst(
            f"CSRRWI x1, mscratch, 17",
            0b0011_0100_0000_10001_101_00001_1110011,
            after={
                1: 0,
            },
        )
        yield from test_inst(
            f"CSRRS x1, mstatus, x3(=0xFF) / read back",
            [
                0b0011_0000_0000_00011_010_00001_1110011,
                0b0011_0000_0000_00000_010_00010_1110011,
            ],
            before={
                3: 0xFF,
            },
            after={
                1: 0,
                2: 0b1000_1000,
            },
        )

        yield uut.irq.eq(1)
        yield from resume()
        yield
        yield
        yield



    sim.add_sync_process(process)

    with sim.write_vcd(vcd_file="test.vcd", gtkw_file="test.gtkw", traces=ports):
        sim.run()
