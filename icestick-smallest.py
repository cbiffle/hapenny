# This is the smallest happeny SoC model for the Icestick, to show off its size
# when configured with only a basic assembly program and single peripheral.
#
# This is not a spectacularly _useful_ configuration, and is smaller than the
# "small" configurations most other small RV32I SoCs include, so the numbers
# don't necessarily compare directly.
#
# Mostly, I use this to keep an eye on the minimum size with a circuit that
# isn't so simple that it optimizes away in synthesis.

import itertools
from functools import reduce
import argparse
import struct
from pathlib import Path

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.build import ResourceError, Resource, Pins, Attrs
from amaranth_boards.test.blinky import Blinky
from amaranth_boards.icestick import ICEStickPlatform
import amaranth.lib.cdc

from hapenny import StreamSig
import hapenny.boxcpu
import hapenny.cpu16
from hapenny.bus import BusPort, SimpleFabric, partial_decode
from hapenny.gpio import OutputPort
from hapenny.mem import BasicMemory

bootloader = Path("smallest-toggle.bin").read_bytes()
boot_image = struct.unpack("<" + "H" * (len(bootloader) // 2), bootloader)

image_or = reduce(lambda a, b: a|b, boot_image)
image_and = reduce(lambda a, b: a&b, boot_image)

problems = set(b for b in range(16)
                 if image_or & (1 << b) == 0
                 or image_and & (1 << b) != 0)

if problems:
    print("WARNING: contents of boot ROM may cause logic to be optimized out.")
    print("Size estimates generated from such an image would be misleading.")
    print(f"The following bit positions are constant: {problems}")

# the blinky program does not use RAM at all, so we can fit it in a single block RAM.
RAM_WORDS = 256 * 1
RAM_ADDR_BITS = (RAM_WORDS - 1).bit_length()

# Add an extra bit to the implemented bus so we can also address I/O.
BUS_ADDR_BITS = RAM_ADDR_BITS + 1

class Test(Elaboratable):
    def __init__(self, *, new_model = False):
        super().__init__()
        self.new_model = new_model

    def elaborate(self, platform):
        m = Module()

        # Gotta do some clock gymnastics here. We want the PLL on so that we can
        # run faster than the Icestick's 12 MHz crystal can go. However, setting
        # up our own sync domain _silently disables_ the Amaranth
        # ICE40Platform's reset delay, which is necssary to work around an
        # undocumented erratum in the iCE40 BRAM that has been chasing me for at
        # least six years.
        #
        # So, we're going to reconstruct it manually.
        clk12 = platform.request("clk12", dir = "-")

        # 15us delay, 12 MHz clock: 180 cycles
        por_delay = int(15e-6 * 12e6)
        m.domains += ClockDomain("por", reset_less=True, local=True)
        por_timer = Signal(range(por_delay))
        por_ready = Signal()
        m.d.comb += ClockSignal("por").eq(clk12.io)
        with m.If(por_timer == por_delay):
            m.d.por += por_ready.eq(1)
        with m.Else():
            m.d.por += por_timer.eq(por_timer + 1)

        cd_sync = ClockDomain("sync")
        m.domains += cd_sync
        m.d.comb += ResetSignal("sync").eq(~por_ready)

        if self.new_model:
            F = 66e6 # Hz
            pll_f, pll_q = 87, 4
        else:
            F = 66e6 # Hz
            pll_f, pll_q = 87, 4

        platform.add_clock_constraint(cd_sync.clk, F)
        print(f"Configuring SoC for {F/1000000:.03} MHz")

        # PLL settings below must generate F from 12MHz; use icepll to adjust.
        m.submodules.pll = Instance(
            "SB_PLL40_CORE",
            p_FEEDBACK_PATH = "SIMPLE",
            p_DIVR = 0,
            p_DIVF = pll_f,
            p_DIVQ = pll_q,
            p_FILTER_RANGE = 1,

            i_REFERENCECLK = clk12.io,
            i_RESETB = 1,
            o_PLLOUTGLOBAL = cd_sync.clk,
        )

        # Ok, back to the design.
        if self.new_model:
            m.submodules.cpu = cpu = hapenny.boxcpu.Cpu(
                # +1 to adjust from bus halfword addressing to CPU byte
                # addressing.
                addr_width = BUS_ADDR_BITS + 1,
                # Program addresses only need to be able to address program
                # memory, so configure the PC and fetch port to be narrower.
                # (+1 because, again, our RAM is halfword addressed but this
                # parameter is in bytes.)
                prog_addr_width = RAM_ADDR_BITS + 1,
            )
        else:
            m.submodules.cpu = cpu = hapenny.cpu16.Cpu(
                # +1 to adjust from bus halfword addressing to CPU byte
                # addressing.
                addr_width = BUS_ADDR_BITS + 1,
            )
        m.submodules.mem = mem = BasicMemory(depth = RAM_WORDS,
                                             contents = boot_image)
        # Make the simplest output port possible.
        m.submodules.outport = outport = OutputPort(1, read_back = False)
        m.submodules.fabric = fabric = SimpleFabric([
            mem.bus,
            partial_decode(m, outport.bus, RAM_ADDR_BITS),
        ])

        connect(m, cpu.bus, fabric.bus)

        for i in range(1):
            led = platform.request("led", i)
            m.d.comb += led.o.eq(outport.pins[i])

        return m

parser = argparse.ArgumentParser(
    prog = "icestick-smallest",
    description = "Script for synthesizing smallest image for HX1K",
)
parser.add_argument('--new', help = 'use new CPU revision', required = False, action = 'store_true')
args = parser.parse_args()

p = ICEStickPlatform()
p.build(Test(new_model = args.new), do_program = True)
