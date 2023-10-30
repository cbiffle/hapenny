import os
import subprocess
from pathlib import Path

from amaranth.build import *
from amaranth.vendor import *
from amaranth_boards.resources import *


__all__ = ["IcoboardPlatform"]


class IcoboardPlatform(LatticeICE40Platform):
    device      = "iCE40HX8K"
    package     = "CT256"
    default_clk = "clk100"
    resources   = [
        Resource("clk100", 0, Pins("R9", dir="i"),
                 Clock(100e6), Attrs(GLOBAL=True, IO_STANDARD="SB_LVCMOS")),

        *LEDResources(pins="C8 F7 K9", attrs=Attrs(IO_STANDARD="SB_LVCMOS")),

        *ButtonResources(pins="K11 P13", attrs=Attrs(IO_STANDARD="SB_LVCMOS")),

        SRAMResource(0,
            cs_n="M7", oe_n="L5", we_n="T7",
            a="N2 K5 J5 M5 P4 N5 P5 P7 M6 P6 T8 T1 P2 R1 N3 P1 M11 P10 P8",
            d="T2 R3 T3 R4 R5 T5 R6 T6 N4 M4 L6 M3 L4 L3 K4 K3",
            dm_n="J4 J3",
            attrs=Attrs(IO_STANDARD="SB_LVCMOS"),
        ),

        *SPIFlashResources(0,
            cs_n="R12", clk="R11", copi="P12", cipo="P11",
            attrs=Attrs(IO_STANDARD="SB_LVCMOS")
        ),
    ]
    connectors  = [
        Connector("pmod", 1, "D8 B9 B10 B11 - - B8 A9 A10 A11 - -"),
        Connector("pmod", 2, "A5 A2 C3 B4 - - B7 B6 B3 B5 - -"),
        Connector("pmod", 3, "L9 G5 L7 N6 - - N9 P9 M8 N7 - -"),
        Connector("pmod", 4, "T15 T14 T11 R10 - - R14 T13 T10 T9 - -"),
    ]

    def toolchain_program(self, products, name):
        icoprog = os.environ.get("ICOPROG", "icoprog")
        with products.extract("{}.bin".format(name)) as bitstream_filename:
            bitstream = Path(bitstream_filename).read_bytes()
            subprocess.run([icoprog, "-p"], input=bitstream, check=True)


if __name__ == "__main__":
    from amaranth_boards.test.blinky import *
    IcoboardPlatform().build(Blinky(), do_program=True)
