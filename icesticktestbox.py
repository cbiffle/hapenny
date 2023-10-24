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
from hapenny.boxcpu import Cpu
from hapenny.bus import BusPort, SimpleFabric, partial_decode
from hapenny.serial import BidiUart
from hapenny.mem import BasicMemory

bootloader = Path("tiny-bootloader.bin").read_bytes()
boot_image = struct.unpack("<" + "h" * (len(bootloader) // 2), bootloader)

# tiny-bootloader is written in a high-level language and needs to have a stack,
# so the minimum here is two 256-halfword (512-byte) RAMs.
RAM_WORDS = 256 * 2
RAM_ADDR_BITS = (RAM_WORDS - 1).bit_length()

BUS_ADDR_BITS = RAM_ADDR_BITS + 1

class Test(Elaboratable):
    def __init__(self):
        super().__init__()

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

        F = 66e6 # Hz
        platform.add_clock_constraint(cd_sync.clk, F)
        print(f"Configuring SoC for {F/1000000:.03} MHz")

        # PLL settings below must generate F from 12MHz; use icepll to adjust.
        m.submodules.pll = Instance(
            "SB_PLL40_CORE",
            p_FEEDBACK_PATH = "SIMPLE",
            p_DIVR = 0,
            p_DIVF = 87,
            p_DIVQ = 4,
            p_FILTER_RANGE = 1,

            i_REFERENCECLK = clk12.io,
            i_RESETB = 1,
            o_PLLOUTGLOBAL = cd_sync.clk,
        )

        # Ok, back to the design.
        m.submodules.cpu = cpu = Cpu(
            # +1 to adjust from bus halfword addressing to CPU byte addressing.
            addr_width = BUS_ADDR_BITS + 1,
            # Program addresses only need to be able to address program memory,
            # so configure the PC and fetch port to be narrower. (+1 because,
            # again, our RAM is halfword addressed but this parameter is in
            # bytes.)
            prog_addr_width = RAM_ADDR_BITS + 1,
        )
        m.submodules.mem = mem = BasicMemory(depth = RAM_WORDS,
                                             contents = boot_image)
        m.submodules.uart = uart = BidiUart(baud_rate = 115_200,
                                            clock_freq = F)
        m.submodules.fabric = fabric = SimpleFabric([
            mem.bus,
            partial_decode(m, uart.bus, RAM_ADDR_BITS),
        ])

        connect(m, cpu.bus, fabric.bus)

        uartpins = platform.request("uart", 0)
        rx_post_sync = Signal()
        m.submodules.rxsync = amaranth.lib.cdc.FFSynchronizer(
            i = uartpins.rx.i,
            o = rx_post_sync,
            o_domain = "sync",
            reset = 1,
            stages = 2,
        )
        m.d.comb += [
            uartpins.tx.o.eq(uart.tx),
            uart.rx.eq(rx_post_sync),
        ]

        return m

p = ICEStickPlatform()
p.build(Test(), do_program = True)
