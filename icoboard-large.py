# Icoboard example using the external SRAM.

import itertools
import argparse
import struct
from pathlib import Path

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.build import ResourceError, Resource, Pins, Attrs
from boards.icoboard import IcoboardPlatform

from hapenny import StreamSig
from hapenny.boxcpu import Cpu
from hapenny.bus import BusPort, SimpleFabric, partial_decode
from hapenny.gpio import OutputPort
from hapenny.serial import BidiUart
from hapenny.mem import BasicMemory, SpramMemory
from hapenny.extsram import ExternalSRAM

BOOT_ROM_WORDS = 256
BOOT_ROM_ADDR_BITS = (BOOT_ROM_WORDS - 1).bit_length()

bootloader = Path("icolarge-bootloader.bin").read_bytes()
boot_image = struct.unpack("<" + "h" * (len(bootloader) // 2), bootloader)

assert len(boot_image) <= BOOT_ROM_WORDS, \
        f"bootloader is {len(boot(image))} words long, too big for boot ROM"

class Test(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        # The Icoboard's input clock is 100MHz, which seems ... optimistic.
        # Let's drop it to something we can use.

        # Gotta do some clock gymnastics here. We want the PLL on so that we can
        # run faster than the Icestick's 12 MHz crystal can go. However, setting
        # up our own sync domain _silently disables_ the Amaranth
        # ICE40Platform's reset delay, which is necssary to work around an
        # undocumented erratum in the iCE40 BRAM that has been chasing me for at
        # least six years.
        #
        # So, we're going to reconstruct it manually.
        clk100 = platform.request("clk100", dir = "-")

        # 15us delay, 100 MHz clock: 1500 cycles
        por_delay = int(15e-6 * 100e6)
        m.domains += ClockDomain("por", reset_less=True, local=True)
        por_timer = Signal(range(por_delay))
        por_ready = Signal()
        m.d.comb += ClockSignal("por").eq(clk100.io)
        with m.If(por_timer == por_delay):
            m.d.por += por_ready.eq(1)
        with m.Else():
            m.d.por += por_timer.eq(por_timer + 1)

        cd_sync = ClockDomain("sync")
        m.domains += cd_sync
        m.d.comb += ResetSignal("sync").eq(~por_ready)

        #F = 25.781e6 # Hz
        #pll_r, pll_f, pll_q, filter_range = 3, 0, 5, 2
        F = 50e6 # Hz
        pll_r, pll_f, pll_q, filter_range = 1, 0, 4, 4


        platform.add_clock_constraint(cd_sync.clk, F)
        print(f"Configuring SoC for {F/1000000:.05} MHz")

        clk_90 = Signal()

        # PLL settings below must generate F from 12MHz; use icepll to adjust.
        m.submodules.pll = Instance(
            "SB_PLL40_2F_CORE",
            p_FEEDBACK_PATH = "PHASE_AND_DELAY",
            #p_DELAY_ADJUSTMENT_MODE_FEEDBACK = "FIXED",
            #p_DELAY_ADJUSTMENT_MODE_RELATIVE = "FIXED",
            p_PLLOUT_SELECT_PORTA = "SHIFTREG_0deg",
            p_PLLOUT_SELECT_PORTB = "SHIFTREG_90deg",
            p_SHIFTREG_DIV_MODE = 0,
            #p_FDA_FEEDBACK = 0b1111,
            #p_FDA_RELATIVE = 0b1111,
            p_DIVR = pll_r,
            p_DIVF = pll_f,
            p_DIVQ = pll_q,
            p_FILTER_RANGE = filter_range,

            i_REFERENCECLK = clk100.io,
            i_RESETB = 1,
            o_PLLOUTGLOBALA = cd_sync.clk,
            o_PLLOUTGLOBALB = clk_90,
        )

        # Memory map should be:
        # 0000_0000     external SRAM (1 MiB)
        # 0010_0000     boot ROM
        # 0010_1000     UART
        # 0010_2000     output port

        m.submodules.cpu = cpu = Cpu(
            reset_vector = 0x10_0000,  # boot ROM
            # We need 21-bit addressing to reach both all of the 1 MiB SRAM and
            # our boot ROM. This also gives us about a MiB of peripheral space,
            # which is great, so we set both program and load/store address
            # widths to 21.
            addr_width = 21,
            # We'll turn on the counters because we've got the space, and this
            # makes the output from Dhrystone a lot more interesting.
            counters = True,
        )

        # Create our RAMs.
        m.submodules.extsram = extsram = ExternalSRAM(address_bits = 19)
        m.submodules.bootmem = bootmem = BasicMemory(depth = BOOT_ROM_WORDS,
                                                     contents = boot_image)
        m.submodules.mem = mem = BasicMemory(depth = BOOT_ROM_WORDS)

        # Create a subfabric for the top mebibyte of the addressible space. This
        # will include both our I/O devices and our boot ROM. We'll give each
        # thing a 4096 byte (2048-halfword) region.
        m.submodules.port = port = OutputPort(3)
        m.submodules.uart = uart = BidiUart(baud_rate = 115200,
                                            clock_freq = F)
        m.submodules.iofabric = iofabric = SimpleFabric([
            partial_decode(m, bootmem.bus, 11),     # 0x____0000
            partial_decode(m, uart.bus, 11),        # 0x____1000
            partial_decode(m, port.bus, 11),        # 0x____2000
        ])

        # Create the top-level fabric to unite memory and I/O.
        m.submodules.fabric = fabric = SimpleFabric([
            extsram.bus,                            # 0x0000_0000
            partial_decode(m, iofabric.bus, 19),    # 0x0010_0000
        ])

        connect(m, cpu.bus, fabric.bus)

        # Add some things describing how I've got PMODs connected.
        # USB-serial is connected on top row of PMOD 1
        platform.add_resources([
            Resource("tx", 0, Pins("2", dir="o", conn=("pmod", 1))),
            Resource("rx", 0, Pins("3", dir="i", conn=("pmod", 1))),
        ])


        # UART wiring
        tx = platform.request("tx", 0)
        rx = platform.request("rx", 0)
        m.d.comb += [
            tx.o[0].eq(uart.tx),
            uart.rx.eq(rx.i[0]),
        ]

        # LED wiring
        for i in range(3):
            led = platform.request("led", i)
            m.d.comb += led.o.eq(port.pins[i])

        # SRAM wiring. NOTE: Amaranth models all the SRAM control signals as
        # active-high and inverts at the pin. This means all of our signals
        # here are active-high. This is potentially confusing, hence this
        # comment.
        sram = platform.request("sram", 0)
        m.d.comb += [
            # Our SRAM interface requires a 90-degree-shifted version of the
            # clock.
            extsram.clock_90.eq(clk_90),

            sram.cs.o.eq(1), # amaranth inverts this
            sram.oe.o.eq(extsram.sram_oe),
            sram.we.o.eq(extsram.sram_we),
            sram.dm.o.eq(extsram.sram_lanes),

            sram.a.o.eq(extsram.addr_to_sram),

            sram.d.o.eq(extsram.data_to_sram),
            sram.d.oe.eq(extsram.sram_we),
            extsram.data_from_sram.eq(sram.d.i),
        ]

        return m

p = IcoboardPlatform()
p.build(Test(), do_program = True)
