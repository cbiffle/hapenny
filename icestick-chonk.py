# This is a demo for the "chonk" core, which is the hapenny v2
# microarchitecture modified to have a 32-bit datapath and lower cycle count.
# This provides a useful apples-to-apples comparison with the hapenny cores,
# since it's using similar microarchitectural techniques and running the same
# code.

import itertools
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
import hapenny.chonk.cpu
from hapenny.bus import BusPort, SimpleFabric, partial_decode
from hapenny.chonk.gpio32 import OutputPort32
from hapenny.chonk.mem32 import BasicMemory

bootloader = Path("smallest-toggle.bin").read_bytes()
boot_image = struct.unpack("<" + "I" * (len(bootloader) // 4), bootloader)
for i, word in enumerate(boot_image):
    print(f"{i*4:08x}  {word:08x}")

# the blinky program does not use RAM at all, so we can fit it in a single
# block RAM. We'll use half of one, wastefully, to preserve the memory map.
RAM_WORDS = 128 * 1
RAM_ADDR_BITS = (RAM_WORDS - 1).bit_length()

# Add an extra bit to the implemented bus so we can also address I/O.
BUS_ADDR_BITS = RAM_ADDR_BITS + 1
print(f"BUS_ADDR_BITS = {BUS_ADDR_BITS}")

class Test(Elaboratable):
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

        F = 55e6 # Hz
        pll_r, pll_f, pll_q, filter_range = 0, 72, 4, 1

        platform.add_clock_constraint(cd_sync.clk, F)
        print(f"Configuring SoC for {F/1000000:.03} MHz")

        # PLL settings below must generate F from 12MHz; use icepll to adjust.
        m.submodules.pll = Instance(
            "SB_PLL40_CORE",
            p_FEEDBACK_PATH = "SIMPLE",
            p_DIVR = pll_r,
            p_DIVF = pll_f,
            p_DIVQ = pll_q,
            p_FILTER_RANGE = filter_range,

            i_REFERENCECLK = clk12.io,
            i_RESETB = 1,
            o_PLLOUTGLOBAL = cd_sync.clk,
        )

        # Ok, back to the design.
        m.submodules.cpu = cpu = hapenny.chonk.cpu.Cpu(
            # +2 to adjust from bus word addressing to CPU byte
            # addressing.
            addr_width = BUS_ADDR_BITS + 2,
            # Program addresses only need to be able to address program
            # memory, so configure the PC and fetch port to be narrower.
            # (+2 because, again, our RAM is word addressed but this parameter
            # is in bytes.)
            prog_addr_width = RAM_ADDR_BITS + 2,
        )
        m.submodules.mem = mem = BasicMemory(depth = RAM_WORDS,
                                             contents = boot_image)
        # Make the simplest output port possible.
        m.submodules.outport = outport = OutputPort32(1)
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
    prog = "icestick-smallestbig",
    description = "Script for synthesizing smallest image for HX1K",
)
args = parser.parse_args()

p = ICEStickPlatform()
p.build(Test(), do_program = True)
