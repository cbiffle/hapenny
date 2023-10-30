from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from hapenny import StreamSig, AlwaysReady
from hapenny.regfile16 import RegFile16
from hapenny.bus import BusPort

# Note: the binary representation of these variants is chosen to match the
# bottom two bits of funct3 of the CSR instructions, except for NONE which uses
# a spare encoding.
class WriteMode(Enum, shape = unsigned(2)):
    NONE = 0b00
    REPL = 0b01
    OR = 0b10
    ANDN = 0b11

CsrCmd = Signature({
    # Choose how write data is handled
    'mode': Out(WriteMode),
    # The write data.
    'write': Out(32),
    # The read data.
    'read': In(32),
})

# A register that can be read and written, and also updated by the machine in
# the background. In the event of a simultaneous write by software and update
# from the machine, software wins.
#
# A mask can optionally be provided to control which bits are read/write and
# which ignore writes. This should be a 32-bit integer; 1 bits are writable, 0
# bits are fixed. This should avoid synthesizing flops for fixed bits.
class CsrReg(Component):
    cmd: In(AlwaysReady(CsrCmd))

    update: In(AlwaysReady(32))
    read: Out(32)

    def __init__(self, name, width, *, reset = 0, mask = None):
        super().__init__()

        if mask is None:
            mask = (1 << width) - 1

        assert mask.bit_length() <= width, "mask wider than register (mistake?)"

        self.name = name
        self.width = width
        self.mask = mask
        self.reset = reset

    def elaborate(self, platform):
        m = Module()

        reg = Signal(self.width, name = self.name, reset = self.reset)

        m.d.comb += self.cmd.payload.read.eq(reg)
        m.d.comb += self.read.eq(reg)

        # If there's an update from the machine, apply it.
        with m.If(self.update.valid):
            m.d.sync += reg.eq(self.update.payload & self.mask)

        # Any update from software will override that.
        with m.If(self.cmd.valid):
            with m.Switch(self.cmd.payload.mode):
                with m.Case(WriteMode.NONE):
                    pass
                with m.Case(WriteMode.OR):
                    m.d.sync += reg.eq(reg | (self.cmd.payload.write &
                                       self.mask))
                with m.Case(WriteMode.ANDN):
                    m.d.sync += reg.eq(reg & ~(self.cmd.payload.write &
                                       self.mask))
                with m.Case(WriteMode.REPL):
                    m.d.sync += reg.eq((self.cmd.payload.write & self.mask)
                                       | (reg & ~self.mask))

        return m

CsrFileCmd = Signature({
    # CSR being addressed.
    'addr': Out(12),
    # Half being addressed.
    'high': Out(1),
    # Choose how write data is handled
    'mode': Out(WriteMode),
    # The write data.
    'write': Out(16),
    # The read data.
    'read': In(16),
})

# Presents a 16-bit-wide interface to the 32-bit atomic CSRs.
#
# This expects any interaction to consist of three cycles.
#
# 1. Low half write data is presented. Mode is ignored. Read data returned
# should be ignored. No CSR activity takes place. (This is expected to take
# place during the CPU's OPERATE-LO phase.)
#
# 2. High half write data presented. Mode is applied. CSR is modified if Mode is
# not NONE. High half of CSR contents _before_ write are presented on the read
# data lines. (This is expected to happen during OPERATE-HI.)
#
# 3. Low half "written" again. Mode is ignored. The low half of the CSR contents
# before the previous cycle's write are presented on the read data lines. (This
# will require an additional CPU state.)
class CsrFile16(Component):
    cmd: In(AlwaysReady(CsrFileCmd))

    def __init__(self, regmap):
        super().__init__()

        self.regmap = regmap

    def elaborate(self, platform):
        m = Module()

        low_data = Signal(16)

        # Distribute incoming bus signals to all registers. We wire up the
        # 16-bit read and write paths to the high half of the registers, because
        # we apply operations in the high phase. The low half is supplied from
        # the low write latch.
        read_data = None
        for csraddr, port in self.regmap.items():
            m.d.comb += [
                port.payload.mode.eq(self.cmd.payload.mode),
                port.payload.write[16:].eq(self.cmd.payload.write),
                port.payload.write[:16].eq(low_data),
            ]
            # Make the valid line conditional on the address and phase.
            m.d.comb += port.valid.eq(
                (self.cmd.payload.addr == csraddr)
                & self.cmd.payload.high
                & self.cmd.valid
            )
            # Drive the read data if we're addressed.
            r = port.payload.read & \
                port.valid.replicate(port.payload.read.shape().width)
            if read_data is None:
                read_data = r
            else:
                read_data = read_data | r

        if read_data is None:
            read_data = Const(0)

        with m.If(self.cmd.valid):
            with m.If(~self.cmd.payload.high):
                # Latch data being written so we can apply it atomically at the
                # high phase.
                m.d.sync += low_data.eq(self.cmd.payload.write)
                # Expose stored data.
                m.d.comb += self.cmd.payload.read.eq(low_data)
            with m.Else(): # high phase
                # Latch data being read back
                m.d.sync += low_data.eq(read_data[:16])
                # Expose high half
                m.d.comb += self.cmd.payload.read.eq(read_data[16:])

        return m
