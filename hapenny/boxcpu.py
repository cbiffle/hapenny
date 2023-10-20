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
from hapenny.sbox import SBox
from hapenny.fdbox import FDBox
from hapenny.ewbox import EWBox

DebugPort = Signature({
    'reg_read': Out(StreamSig(7)),
    'reg_value': In(16),
    'reg_write': Out(StreamSig(RegWrite)),
    'pc': In(32),
    'pc_write': Out(StreamSig(32)),
    'ctx': Out(1),
    'ctx_write': In(StreamSig(1)),
})

class Cpu(Component):
    halt_request: In(1)
    halted: Out(1)

    debug: In(DebugPort)

    def __init__(self, *,
                 addr_width = 32,
                 prog_addr_width = None):
        super().__init__()

        self.bus = BusPort(addr = addr_width - 1, data = 16).create()

        self.addr_width = addr_width
        if prog_addr_width is None:
            self.prog_addr_width = self.addr_width
        else:
            self.prog_addr_width = prog_addr_width

        self.s = SBox()
        self.rf = RegFile16()
        self.fd = FDBox(
            prog_addr_width = self.prog_addr_width,
        )
        self.ew = EWBox(
            prog_addr_width = self.prog_addr_width,
        )

    def elaborate(self, platform):
        m = Module()

        m.submodules.regfile = rf = self.rf
        m.submodules.s = s = self.s
        m.submodules.fd = fd = self.fd
        m.submodules.ew = ew = self.ew

        m.d.comb += [
            fd.onehot_state.eq(s.onehot_state),
            fd.pc.eq(ew.pc_next),
            fd.ctx.eq(ew.ctx_next),
            fd.from_the_top.eq(ew.from_the_top),

            ew.onehot_state.eq(s.onehot_state),
            ew.inst_next.eq(fd.inst_next),

            s.from_the_top.eq(ew.from_the_top),
            s.halt_request.eq(self.halt_request),
            s.not_a_bubble.eq(ew.full),
            s.hold.eq(ew.hold),

            self.halted.eq(s.halted),

            self.debug.reg_value.eq(rf.read_resp),
        ]

        # Manually combine the register file write port.
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

        # Manually combine the register file read ports.
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
        # Manually combine the bus access ports.
        m.d.comb += [
            self.bus.cmd.valid.eq(
                fd.bus.cmd.valid | ew.bus.cmd.valid
            ),
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

        # Extra debug wiring
        m.d.comb += [
            self.debug.pc.eq(Cat(0, 0, ew.pc)),
            self.debug.pc_write.ready.eq(ew.debug_pc_write.ready),

            ew.debug_pc_write.valid.eq(self.debug.pc_write.valid),
            ew.debug_pc_write.payload.eq(self.debug.pc_write.payload),
        ]

        return m
