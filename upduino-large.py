import itertools
import argparse
import struct
from pathlib import Path

from amaranth import *
from amaranth.lib.wiring import *
from amaranth.build import ResourceError, Resource, Pins, Attrs
from amaranth_boards.upduino_v3 import UpduinoV3Platform

from hapenny import StreamSig
from hapenny.boxcpu import Cpu
from hapenny.bus import BusPort, SimpleFabric, partial_decode
from hapenny.gpio import OutputPort
from hapenny.serial import BidiUart

RAM_WORDS = 256 * 1
RAM_ADDR_BITS = (RAM_WORDS - 1).bit_length()

BUS_ADDR_BITS = RAM_ADDR_BITS + 1

print(f"boot memory will use {RAM_ADDR_BITS}-bit addressing")

bootloader = Path("upduino-bootloader.bin").read_bytes()
boot_image = struct.unpack("<" + "h" * (len(bootloader) // 2), bootloader)

class BulkMemory(Component):
    bus: In(BusPort(addr = 14, data = 16))

    def elaborate(self, platform):
        m = Module()

        m.submodules.spram = Instance(
            "SB_SPRAM256KA",
            i_CLOCK = ClockSignal("sync"),
            i_ADDRESS = self.bus.cmd.payload.addr,
            i_DATAIN = self.bus.cmd.payload.data,
            # Weirdly, write enables are at the nibble level.
            i_MASKWREN = Cat(
                self.bus.cmd.payload.lanes[0],
                self.bus.cmd.payload.lanes[0],
                self.bus.cmd.payload.lanes[1],
                self.bus.cmd.payload.lanes[1],
            ),
            i_WREN = self.bus.cmd.payload.lanes != 0,
            i_CHIPSELECT = self.bus.cmd.valid,
            i_POWEROFF = 1, # active fucking low
            i_STANDBY = 0,
            i_SLEEP = 0,
            o_DATAOUT = self.bus.resp,
        )

        return m

class TestMemory(Component):
    bus: In(BusPort(addr = RAM_ADDR_BITS, data = 16))

    def __init__(self, contents):
        super().__init__()

        self.m = Memory(
            width = 16,
            depth = RAM_WORDS,
            name = "testram",
            init = contents,
            # Force this into block RAM on Yosys
            attrs = {
                'ram_style': 'block',
            },
        )

    def elaborate(self, platform):
        m = Module()

        m.submodules.m = self.m

        rp = self.m.read_port(transparent = False)
        wp = self.m.write_port(granularity = 8)

        m.d.comb += [
            rp.addr.eq(self.bus.cmd.payload.addr),
            rp.en.eq(self.bus.cmd.valid & (self.bus.cmd.payload.lanes == 0)),

            wp.addr.eq(self.bus.cmd.payload.addr),
            wp.data.eq(self.bus.cmd.payload.data),
            wp.en[0].eq(self.bus.cmd.valid & self.bus.cmd.payload.lanes[0]),
            wp.en[1].eq(self.bus.cmd.valid & self.bus.cmd.payload.lanes[1]),
        ]

        m.d.comb += [
            self.bus.resp.eq(rp.data),
        ]

        return m


class Test(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        m.submodules.cpu = cpu = Cpu(
            reset_vector = 0x0_8000,
            # +1 to adjust from bus halfword addressing to CPU byte addressing.
            addr_width = 2 + 14 + 1,
            # Program addresses only need to be able to address RAM, not I/O,
            # so configure the PC and fetch port to be narrower. (+1 because,
            # again, our RAM is halfword addressed but this parameter is in
            # bytes.)
            prog_addr_width = 1 + 14 + 1,
        )
        m.submodules.bootmem = bootmem = TestMemory(boot_image)
        m.submodules.bulkmem0 = bulkmem0 = BulkMemory()
        m.submodules.port = port = OutputPort(1)
        m.submodules.uart = uart = BidiUart(baud_rate = 115200)
        m.submodules.fabric = fabric = SimpleFabric([
            # Put all the potentially executable RAM in the bottom portion of
            # the address space, to allow PC and fetch circuitry to be slightly
            # narrower. This helps with timing.
            bulkmem0.bus,                       # 0x0000_0000
            partial_decode(m, bootmem.bus, 14), # 0x0000_8000
            partial_decode(m, port.bus, 14),    # 0x0001_0000
            partial_decode(m, uart.bus, 14),    # 0x0001_8000
        ])

        connect(m, cpu.bus, fabric.bus)
        platform.add_resources([
            Resource("tx", 0, Pins("7", dir="o", conn=("j", 0))),
            Resource("rx", 0, Pins("8", dir="i", conn=("j", 0))),
        ])
        def get_all_resources(name):
            resources = []
            for number in itertools.count():
                try:
                    resources.append(platform.request(name, number))
                except ResourceError:
                    break
            return resources

        tx = platform.request("tx", 0)
        rx = platform.request("rx", 0)

        rgb_led = platform.request("rgb_led", 0)
        m.d.comb += [
            rgb_led.r.o.eq(cpu.halted),
            rgb_led.g.o.eq(port.pins[0]),
            tx.o[0].eq(uart.tx),
            uart.rx.eq(rx.i[0]),
        ]

        return m

p = UpduinoV3Platform()
p.hfosc_div = 1 # divide 48MHz by 2**1 = 24 MHz
p.build(Test(), do_program = True)
