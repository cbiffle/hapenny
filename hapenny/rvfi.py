from functools import reduce

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.lib.enum import *
from amaranth.lib.coding import Encoder, Decoder

from hapenny import StreamSig, AlwaysReady

class Mode(Enum, shape = unsigned(2)):
    U = 0
    S = 1
    # 2 is reserved
    M = 3

class Ixl(Enum, shape = unsigned(2)):
    _32 = 1
    _64 = 2

def Rvfi(ilen = 32, xlen = 32):
    return Signature({
        # instruction index, unique per instruction retired, no gaps
        'order': Out(64),
        'insn': Out(ilen),
        'trap': Out(1),
        'halt': Out(1),
        'intr': Out(1),
        'mode': Out(Mode),
        'ixl': Out(Ixl, reset = Ixl._32),

        'rs1_addr': Out(5),
        'rs2_addr': Out(5),
        'rs1_rdata': Out(xlen),
        'rs2_rdata': Out(xlen),

        'rd_addr': Out(5),
        'rd_wdata': Out(xlen),

        'pc_rdata': Out(xlen),
        'pc_wdata': Out(xlen),

        'mem_addr': Out(xlen),
        'mem_rmask': Out(xlen // 8),
        'mem_wmask': Out(xlen // 8),
        'mem_rdata': Out(xlen),
        'mem_wdata': Out(xlen),
    })
