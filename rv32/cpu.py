from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *

from rv32 import StreamSig, AlwaysReady

MemCmd = Signature({
    'addr': Out(32 - 2), # word-addressed
    'lanes': Out(4),
    'data': Out(32),
})

class UState(Enum):
    FETCH = 0
    INST = 1
    BEGIN = 2

class Cpu(Component):
    mem_out: Out(StreamSig(MemCmd))
    mem_in: In(StreamSig(32))

    def __init__(self):
        super().__init__()

        self.ustate = Signal(UState)
        self.regfile = Memory(
            width = 32,
            depth = 32,
            name = "regfile",
        )
        self.pc = Signal(32)
        self.inst = Signal(32)

    def elaborate(self, platform):
        m = Module()

        m.submodules.regfile = self.regfile

        regfile_rp = self.regfile.read_port()
        regfile_wp = self.regfile.write_port()

        imm_i = Signal(32)
        imm_j = Signal(32)
        m.d.comb += [
            imm_i.eq(Cat(self.inst[20], self.inst[21:25], self.inst[25:31],
                         self.inst[31].replicate(21))),
            imm_j.eq(Cat(0, self.inst[21:25], self.inst[25:31], self.inst[20],
                         self.inst[12:20], self.inst[31].replicate(11))),
        ]

        opcode = Signal(7)
        m.d.comb += opcode.eq(self.inst[0:7])

        do_fetch = Signal(1)
        fetch_addr = Signal(32)
        m.d.comb += fetch_addr.eq(self.pc) # overridden where needed

        with m.Switch(self.ustate):
            with m.Case(UState.FETCH):
                m.d.comb += [
                    # Attempt fetch of next instruction
                    self.mem_out.payload.addr.eq(self.pc[2:]),
                    self.mem_out.valid.eq(1),
                ]

                # Only treat that as successful if the interface is ready.
                with m.If(self.mem_out.ready):
                    m.d.sync += self.ustate.eq(UState.INST)

            with m.Case(UState.INST):
                m.d.comb += [
                    # Indicate to the bus that we're ready for the instruction.
                    self.mem_in.ready.eq(1),
                    # Also, go ahead and route the rs1 subfield of any incoming
                    # instruction to the register file's read port.
                    regfile_rp.addr.eq(self.mem_in.payload[15:20]),
                    regfile_rp.en.eq(1),
                ]

                # We may not receive the instruction right away. Wait for it.
                with m.If(self.mem_in.valid):
                    m.d.sync += [
                        # Latch the instruction.
                        self.inst.eq(self.mem_in.payload),
                        # Start running it.
                        self.ustate.eq(UState.BEGIN),
                    ]

            with m.Case(UState.BEGIN):
                # We've already loaded rs1 just in case.
                with m.Switch(opcode):
                    with m.Case(0b11011_11): # JAL
                        # ha, we didn't actually need rs1. Oh well!
                        jump_adder = Signal(32)
                        m.d.comb += [
                            jump_adder.eq(self.pc + imm_j),
                            do_fetch.eq(1),
                            fetch_addr.eq(jump_adder),
                        ]

                        m.d.sync += [
                            self.pc.eq(jump_adder),
                        ]

        with m.If(do_fetch):
            m.d.comb += [
                # Attempt fetch of next instruction
                self.mem_out.payload.addr.eq(fetch_addr[2:]),
                self.mem_out.valid.eq(1),
            ]

            # Only treat that as successful if the interface is ready.
            with m.If(self.mem_out.ready):
                m.d.sync += self.ustate.eq(UState.INST)

        return m
