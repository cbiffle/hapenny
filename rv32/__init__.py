from amaranth import *
from amaranth.lib import data, enum, wiring
from amaranth.lib.wiring import In, Out

class StreamSig(wiring.Signature):
    def __init__(self, payload_shape):
        super().__init__({
            'payload': Out(payload_shape),
            'valid': Out(1),
            'ready': In(1),
        })

class AlwaysReady(wiring.Signature):
    def __init__(self, payload_shape):
        super().__init__({
            'payload': Out(payload_shape),
            'valid': Out(1),
        })


