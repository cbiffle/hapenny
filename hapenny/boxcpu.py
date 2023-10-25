# An RV32I implementation using a 16-bit datapath to save space.

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
import amaranth.lib.coding

from hapenny import StreamSig, AlwaysReady, csr16, mux, oneof, onehot_choice
from hapenny.decoder import ImmediateDecoder, Decoder, DecodeSignals
from hapenny.regfile16 import RegFile16, RegWrite
from hapenny.bus import BusPort
from hapenny.csr16 import CsrFile16, CsrReg
from hapenny.sbox import SBox, STATE_COUNT
from hapenny.fdbox import FDBox
from hapenny.ewbox import EWBox

# Note: all debug port signals are directional from the perspective of the DEBUG
# PROBE, not the CPU.
DebugPort = Signature({
    # Register read port. The CPU asserts READY on this port when it is halted
    # and the register file is available for inspection. Debug probes should
    # place a register number on the payload signals and assert VALID; the
    # response will come on reg_value on the next cycle.
    'reg_read': Out(StreamSig(5 + 1)),
    # Value that was read from the reg_read port above.
    'reg_value': In(16),
    # Register write command. Works roughly like reg_read, e.g. only READY when
    # the CPU is halted.
    'reg_write': Out(StreamSig(RegWrite(5 + 1))),
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
    """An RV32I core using a 16-bit datapath, with overlapped fetch and
    execute for reasonable performance.

    Parameters
    ----------
    addr_width (int): number of low-order bits that are significant in memory
        addresses. The default is 32; if this is reduced, memory and I/O
        devices will appear to repeat at higher addresses because the top bits
        won't be decoded. Note that this parameter is in terms of byte
        addresses (the numbers RV32I software deals with); the actual bus port
        has addr_width-1 address lines because it addresses halfwords.
    prog_addr_width (int): number of low-order bits that are significant in
        instruction addresses. This determines the width of the PC register(s)
        and fetch path. If program storage is in the lower section of the
        address range, and I/O devices higher, you can set this parameter to
        smaller than addr_width to save some area. If not explicitly
        overridden, this is the same as addr_width.  addr_width.

    Attributes
    ----------
    bus (both): connection to the bus, 16 bit data path and `addr_width - 1`
        address bits.
    debug (both): debug port for testing or development.
    halt_request (in): when asserted (1), requests that the CPU stop at the
        next instruction boundary. Release (0) to resume.
    halted (out): raised when the CPU has halted.
    """
    halt_request: In(1)
    halted: Out(1)

    debug: In(DebugPort)

    def __init__(self, *,
                 reset_vector = 0,
                 addr_width = 32,
                 prog_addr_width = None):
        super().__init__()

        # Capture and derive parameter values
        self.addr_width = addr_width
        self.prog_addr_width = prog_addr_width or addr_width

        # Create our parameterized ports and modules
        self.bus = BusPort(addr = addr_width - 1, data = 16).create()

        self.s = SBox()
        self.rf = RegFile16()
        self.fd = FDBox(
            prog_addr_width = self.prog_addr_width,
        )
        self.ew = EWBox(
            reset_vector = reset_vector,
            addr_width = addr_width,
            prog_addr_width = self.prog_addr_width,
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

        return m
