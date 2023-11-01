# A baseline implementation of an RV32 processor for comparison,
# sharing microarchitectural details with hapenny.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
import amaranth.lib.coding

from hapenny import StreamSig, AlwaysReady, mux, oneof, onehot_choice
from hapenny.decoder import ImmediateDecoder, Decoder, DecodeSignals
from hapenny.chonk.regfile32 import RegFile, RegWrite
from hapenny.bus import BusPort, BusCmd
from hapenny.chonk.sbox import SBox, STATE_COUNT
from hapenny.chonk.fdbox import FDBox
from hapenny.chonk.ewbox import EWBox
from hapenny.rvfi import Rvfi, Mode, Ixl

# Note: all debug port signals are directional from the perspective of the DEBUG
# PROBE, not the CPU.
DebugPort = Signature({
    # Register read port. The CPU asserts READY on this port when it is halted
    # and the register file is available for inspection. Debug probes should
    # place a register number on the payload signals and assert VALID; the
    # response will come on reg_value on the next cycle.
    'reg_read': Out(StreamSig(5)),
    # Value that was read from the reg_read port above.
    'reg_value': In(32),
    # Register write command. Works roughly like reg_read, e.g. only READY when
    # the CPU is halted.
    'reg_write': Out(StreamSig(RegWrite(5))),
    # PC output from CPU. This is always valid. If the CPU's PC is narrower
    # than 32 bits (the prog_addr_width parameter) then its value is
    # zero-extended on this port.
    'pc': In(32),
    # PC override signal. Becomes READY when the CPU is halted; assert a new
    # value with VALID here to change the next instruction that will be
    # fetched. If the PC is narrower than 32 bits (the prog_addr_width
    # parameter) then the higher bits in this path are ignored.
    'pc_write': Out(StreamSig(32)),
    # State output from CPU. This is a one-hot encoding of the CPU's internal
    # execution state, mostly intended for testbenches.
    'state': In(STATE_COUNT),
})

class Cpu(Component):
    """A basic RV32I core.

    Parameters
    ----------
    addr_width (int): number of low-order bits that are significant in memory
        addresses. The default is 32; if this is reduced, memory and I/O
        devices will appear to repeat at higher addresses because the top bits
        won't be decoded. Note that this parameter is in terms of byte
        addresses (the numbers RV32I software deals with); the actual bus port
        has addr_width-2 address lines because it addresses words.
    prog_addr_width (int): number of low-order bits that are significant in
        instruction addresses. This determines the width of the PC register(s)
        and fetch path. If program storage is in the lower section of the
        address range, and I/O devices higher, you can set this parameter to
        smaller than addr_width to save some area. If not explicitly
        overridden, this is the same as addr_width.  addr_width.

    Attributes
    ----------
    bus (both): connection to the bus, 32 bit data path and `addr_width - 2`
        address bits.
    debug (both): debug port for testing or development.
    halt_request (in): when asserted (1), requests that the CPU stop at the
        next instruction boundary. Release (0) to resume.
    halted (out): raised when the CPU has halted.
    rvfi (out): RISC-V Formal Interface trace port.
    """
    halt_request: In(1)
    halted: Out(1)

    debug: In(DebugPort)
    rvfi: Out(AlwaysReady(Rvfi()))

    def __init__(self, *,
                 reset_vector = 0,
                 addr_width = 32,
                 counters = False,
                 prog_addr_width = None):
        super().__init__()

        # Capture and derive parameter values
        self.addr_width = addr_width
        self.prog_addr_width = prog_addr_width or addr_width

        # Create our parameterized ports and modules
        self.bus = BusPort(addr = addr_width - 2, data = 32).create()

        self.s = SBox()
        self.rf = RegFile()
        self.fd = FDBox(
            prog_addr_width = self.prog_addr_width,
        )
        self.ew = EWBox(
            reset_vector = reset_vector,
            addr_width = addr_width,
            prog_addr_width = self.prog_addr_width,
            counters = counters,
        )

    def elaborate(self, platform):
        m = Module()

        # Make the elaborator aware of all our submodules, and wire them up.
        m.submodules.regfile = rf = self.rf
        m.submodules.s = s = self.s
        m.submodules.fd = fd = self.fd
        m.submodules.ew = ew = self.ew

        m.d.comb += [
            fd.onehot_state.eq(s.onehot_state),
            fd.pc.eq(ew.pc_next),
            fd.from_the_top.eq(ew.from_the_top),

            ew.onehot_state.eq(s.onehot_state),
            ew.inst_next.eq(fd.inst_next),
            ew.debug_pc_write.valid.eq(self.debug.pc_write.valid),
            # Drop the bottom two bits of any incoming PC before feeding to EW.
            ew.debug_pc_write.payload.eq(self.debug.pc_write.payload[2:]),

            s.from_the_top.eq(ew.from_the_top),
            s.halt_request.eq(self.halt_request),
            s.not_a_bubble.eq(ew.full),
            s.hold.eq(ew.hold),

            self.halted.eq(s.halted),

            self.debug.reg_value.eq(rf.read_resp),
            self.debug.state.eq(s.onehot_state),
            # Internal PCs never have bits 0/1, but the debug port deals in
            # 32-bit addresses, so add LSBs when exposing the PC:
            self.debug.pc.eq(Cat(0, 0, ew.pc)),
            self.debug.pc_write.ready.eq(ew.debug_pc_write.ready),
        ]

        # Combine the register file write ports from EW (primary) and the debug
        # interface (secondary). We use an actual mux here instead of OR-ing to
        # keep the debug port from disrupting execution.
        m.d.comb += [
            rf.write_cmd.valid.eq(
                mux(
                    s.halted,
                    self.debug.reg_write.valid,
                    ew.rf_write_cmd.valid,
                ),
            ),
            rf.write_cmd.payload.reg.eq(
                mux(
                    s.halted,
                    self.debug.reg_write.payload.reg,
                    ew.rf_write_cmd.payload.reg,
                ),
            ),
            rf.write_cmd.payload.value.eq(
                mux(
                    s.halted,
                    self.debug.reg_write.payload.value,
                    ew.rf_write_cmd.payload.value,
                ),
            ),
            self.debug.reg_write.ready.eq(s.halted),
        ]

        # Combine the register file read ports from EW, FD, and debug. We OR
        # the EW/FD ports together because those modules are well behaved, but
        # explicitly gate signals from the debug port to only work when we're
        # halted.
        m.d.comb += [
            rf.read_cmd.valid.eq(
                fd.rf_cmd.valid | ew.rf_read_cmd.valid
                | (self.debug.reg_read.valid & s.halted)
            ),
            rf.read_cmd.payload.eq(
                fd.rf_cmd.payload | ew.rf_read_cmd.payload
                | oneof([(s.halted, self.debug.reg_read.payload)])
            ),
            ew.rf_resp.eq(rf.read_resp),
            self.debug.reg_read.ready.eq(s.halted),
        ]
        # Combine the bus access ports. The debug port can't drive our bus, so
        # this is simpler.
        m.d.comb += [
            self.bus.cmd.valid.eq(
                fd.bus.cmd.valid | ew.bus.cmd.valid
            ),
            # Note that this will implicitly zero-extend the FD address if it's
            # shorter than the full bus (because prog_addr_width is dialed
            # back).
            self.bus.cmd.payload.addr.eq(
                fd.bus.cmd.payload.addr | ew.bus.cmd.payload.addr
            ),
            self.bus.cmd.payload.data.eq(
                fd.bus.cmd.payload.data | ew.bus.cmd.payload.data
            ),
            self.bus.cmd.payload.lanes.eq(
                fd.bus.cmd.payload.lanes | ew.bus.cmd.payload.lanes
            ),

            fd.bus.resp.eq(self.bus.resp),
            ew.bus.resp.eq(self.bus.resp),
        ]

        # Trace port
        m.submodules.rvfi_adapter = rvfi = RvfiPort()
        m.d.comb += [
            rvfi.state.eq(s.onehot_state),
            rvfi.full.eq(ew.full),
            rvfi.end_of_instruction.eq(ew.from_the_top),
            rvfi.pc.eq(Cat(0, 0, ew.pc)),
            rvfi.pc_next.eq(Cat(0, 0, ew.pc_next)),
            rvfi.insn.eq(ew.debug_inst),
            rvfi.rf_read_resp_snoop.eq(rf.read_resp),

            rvfi.rf_read_snoop.valid.eq(rf.read_cmd.valid),
            rvfi.rf_read_snoop.payload.eq(rf.read_cmd.payload),

            rvfi.rf_write_snoop.valid.eq(rf.write_cmd.valid),
            rvfi.rf_write_snoop.payload.reg.eq(rf.write_cmd.payload.reg),
            rvfi.rf_write_snoop.payload.value.eq(rf.write_cmd.payload.value),

            rvfi.bus_snoop.valid.eq(self.bus.cmd.valid),
            rvfi.bus_snoop.payload.addr.eq(self.bus.cmd.payload.addr),
            rvfi.bus_snoop.payload.data.eq(self.bus.cmd.payload.data),
            rvfi.bus_snoop.payload.lanes.eq(self.bus.cmd.payload.lanes),
            rvfi.bus_resp_snoop.eq(self.bus.resp),
        ]
        connect(m, rvfi.rvfi_out, flipped(self.rvfi))

        return m

class RvfiPort(Component):
    state: In(STATE_COUNT)
    full: In(1)
    end_of_instruction: In(1)
    pc: In(32)
    pc_next: In(32)
    insn: In(32)

    rf_read_snoop: In(AlwaysReady(5))
    rf_read_resp_snoop: In(32)

    rf_write_snoop: In(AlwaysReady(RegWrite(5)))

    bus_snoop: In(AlwaysReady(BusCmd(addr = 30, data = 32)))
    bus_resp_snoop: In(32)

    rvfi_out: Out(AlwaysReady(Rvfi()))

    def elaborate(self, platform):
        m = Module()

        m.d.comb += [
            self.rvfi_out.payload.ixl.eq(Ixl._32),
            self.rvfi_out.payload.mode.eq(Mode.M),
        ]

        load_expected = Signal(1)
        after_end = Signal()
        rs1_addr_d = Signal(5)

        m.d.sync += after_end.eq(self.end_of_instruction)

        with m.If(self.end_of_instruction):
            m.d.sync += [
                self.rvfi_out.valid.eq(self.full),
                self.rvfi_out.payload.order.eq(self.rvfi_out.payload.order + 1),
                self.rvfi_out.payload.pc_wdata.eq(self.pc_next),

                rs1_addr_d.eq(self.rf_read_snoop.payload),
            ]
        with m.Else():
            m.d.sync += self.rvfi_out.valid.eq(0)

        with m.If(after_end):
            m.d.sync += [
                # Clear the things that accumulate
                self.rvfi_out.payload.halt.eq(0),
                self.rvfi_out.payload.mem_wmask.eq(0),
                self.rvfi_out.payload.mem_wdata.eq(0),
                self.rvfi_out.payload.mem_rmask.eq(0),
                self.rvfi_out.payload.mem_rdata.eq(0),
                self.rvfi_out.payload.rd_addr.eq(0),
                self.rvfi_out.payload.rd_wdata.eq(0),

                self.rvfi_out.payload.rs1_addr.eq(rs1_addr_d),
            ]

        with m.If(self.full):
            with m.If(self.state[0]):
                m.d.sync += [
                    self.rvfi_out.payload.rs1_rdata.eq(self.rf_read_resp_snoop),
                    self.rvfi_out.payload.rs2_addr.eq(self.rf_read_snoop.payload),

                    self.rvfi_out.payload.pc_rdata.eq(self.pc),

                    self.rvfi_out.payload.insn.eq(self.insn),
                ]

            with m.If(self.state[1]):
                m.d.sync += [
                    self.rvfi_out.payload.rs2_rdata.eq(self.rf_read_resp_snoop),
                ]

            with m.If(self.rf_write_snoop.valid):
                m.d.sync += [
                    self.rvfi_out.payload.rd_wdata.eq(self.rf_write_snoop.payload.value),
                    self.rvfi_out.payload.rd_addr.eq(self.rf_write_snoop.payload.reg),
                ]

            with m.If(load_expected):
                m.d.sync += load_expected.eq(0)
                m.d.sync += self.rvfi_out.payload.mem_rdata.eq(
                    self.bus_resp_snoop
                )

            # Ignore bus activity in state 0 as RVFI doesn't consider fetch
            # traffic.
            with m.If(self.bus_snoop.valid & ~self.state[0]):
                m.d.sync += [
                    # Present addresses word-aligned
                    self.rvfi_out.payload.mem_addr.eq(Cat(0, 0, self.bus_snoop.payload.addr)),
                    # Set masks.
                    self.rvfi_out.payload.mem_wmask.eq(self.bus_snoop.payload.lanes),
                    self.rvfi_out.payload.mem_rmask.eq((~self.bus_snoop.payload.lanes.any()).replicate(4)),
                ]
                with m.If(self.bus_snoop.payload.lanes[0]):
                    m.d.sync += self.rvfi_out.payload.mem_wdata[:8].eq(
                        self.bus_snoop.payload.data[:8]
                    )
                with m.If(self.bus_snoop.payload.lanes[1]):
                    m.d.sync += self.rvfi_out.payload.mem_wdata[8:16].eq(
                        self.bus_snoop.payload.data[8:16]
                    )
                with m.If(self.bus_snoop.payload.lanes[2]):
                    m.d.sync += self.rvfi_out.payload.mem_wdata[16:24].eq(
                        self.bus_snoop.payload.data[16:24]
                    )
                with m.If(self.bus_snoop.payload.lanes[3]):
                    m.d.sync += self.rvfi_out.payload.mem_wdata[24:].eq(
                        self.bus_snoop.payload.data[24:]
                    )
                with m.If(self.bus_snoop.payload.lanes == 0):
                    m.d.sync += load_expected.eq(1)

        return m
