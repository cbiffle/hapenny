from amaranth import *
from amaranth.lib import data, enum, wiring
from amaranth.lib.enum import Enum
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

# Builds a mux but out of AND and OR, which often generates cheaper logic on
# 4LUT devices.
def mux(select, one, zero):
    if isinstance(one, Enum):
        one = one.value
    if isinstance(one, int):
        one = Const(one)
    if isinstance(zero, Enum):
        zero = zero.value
    if isinstance(zero, int):
        zero = Const(zero)
    n = max(one.shape().width, zero.shape().width)
    select = select.any() # force to 1 bit
    return (
        (select.replicate(n) & one) | (~select.replicate(n) & zero)
    )

# Builds an output net that chooses between options based on a onehot control
# signal.
#
# onehot_sig should be a signal of N bits, and options should be a dict with at
# most N entries. Each key in the dict is a bit number in onehot_sig, and the
# corresponding value will be produced as output.
#
# If the onehot invariant is violated and more than one bit in onehot_sig is 1,
# this will OR together the results. This is often cheaper than using explicit
# muxes on 4LUT devices.
def onehot_choice(onehot_sig, options):
    assert len(options) > 0
    output = None
    for (choice, result) in options.items():
        if isinstance(choice, Enum):
            choice = choice.value
        if isinstance(result, Enum):
            result = result.value
        if isinstance(result, int):
            result = Const(result)

        case = onehot_sig[choice].replicate(result.shape().width) & result
        if output is not None:
            output = output | case
        else:
            output = case

    return output

# Builds a chained mux that selects between a set of options, which must be
# mutually exclusive.
#
# 'options' is a list of pairs. The first element in each pair is evaluated as a
# boolean condition. If 1, the second element is OR'd into the result.
#
# This means if more than one condition is true simultaneously, the result will
# bitwise OR the results together.
#
# If you've got a onehot control signal instead of a bunch of separate condition
# strobes, see onehot_choice.
def oneof(options, default = None):
    assert len(options) > 0
    output = None
    any_match = None
    for (condition, result) in options:
        if isinstance(condition, int):
            condition = Const(result)
        if isinstance(result, Enum):
            result = result.value
        if isinstance(result, int):
            result = Const(result)
        
        if any_match is not None:
            any_match = any_match | condition.any()
        else:
            any_match = condition.any()

        case = condition.any().replicate(result.shape().width) & result

        if output is not None:
            output = output | case
        else:
            output = case

    if default is not None:
        no_match = ~any_match
        output = output | (no_match.replicate(default.shape().width) & default)

    return output

def hihalf(signal):
    return signal[16:]

def lohalf(signal):
    return signal[:16]

# Selects between the halfwords of (32-bit) signal: if hi is 1, chooses the
# high half, otherwise the low half.
def choosehalf(hi, signal):
    return mux(hi, hihalf(signal), lohalf(signal))


