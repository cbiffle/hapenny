from amaranth import *
from amaranth.lib import data, enum, wiring
from amaranth.lib.enum import Enum
from amaranth.lib.wiring import In, Out

from functools import reduce

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
# most N entries. Each key in the dict is a bit number in onehot_sig, or a
# tuple of bit numbers, and the corresponding value will be produced as output
# when the indicated bit(s) are set in the state.
#
# If a default is provided, it will be used if none of the explicit conditions
# in the options map fires. By default, the default is zero.
#
# This assumes all bits in the onehot_sig are mutually exclusive, and combines
# each path using a bitwise OR instead of muxes, which is often cheaper on 4LUT
# devices. However, this means if the onehot invariant is violated, you'll get
# nonsense output. If that concerns you, see oneof instead.
def onehot_choice(onehot_sig, options, default = None):
    assert len(options) > 0
    output = []
    matches = []
    for (choice, result) in options.items():
        if isinstance(choice, Enum):
            choice = choice.value
        if isinstance(choice, list) or isinstance(choice, tuple):
            pass
        else:
            # Force choice to be a sequence
            choice = [choice]
        if isinstance(result, Enum):
            result = result.value
        if isinstance(result, int):
            result = Const(result)

        condition = reduce(lambda a, b: a | b, map(lambda s: onehot_sig[s],
                                                   choice))
        matches.append(condition)

        case = condition.replicate(result.shape().width) & result

        output.append(case)

    if default is not None:
        if isinstance(default, Enum):
            default = default.value
        if isinstance(default, int):
            default = Const(default)
        no_match = ~reduce(lambda a, b: a | b, matches)
        output.append(no_match.replicate(default.shape().width) & default)

    return reduce(lambda a, b: a | b, output)

# Builds a chained mux that selects between a set of options, which must be
# mutually exclusive.
#
# 'options' is a list of pairs. The first element in each pair is evaluated as a
# boolean condition. If 1, the second element is OR'd into the result.
#
# This means if more than one condition is true simultaneously, the result will
# bitwise OR the results together. It is up to you to ensure that all
# conditions are mutually exclusive.
#
# If a default is provided, it will be used when no other conditions match.
# Otherwise, the default is zero.
#
# If you've got a onehot control signal instead of a bunch of separate condition
# strobes, see onehot_choice.
def oneof(options, default = None):
    assert len(options) > 0
    output = []
    matches = []
    for (condition, result) in options:
        if isinstance(condition, int):
            condition = Const(condition)
        if isinstance(result, Enum):
            result = result.value
        if isinstance(result, int):
            result = Const(result)
        
        matches.append(condition.any())

        case = condition.any().replicate(result.shape().width) & result

        output.append(case)

    if default is not None:
        if isinstance(default, Enum):
            default = default.value
        if isinstance(default, int):
            default = Const(default)
        no_match = ~reduce(lambda a, b: a|b, matches)
        output.append(no_match.replicate(default.shape().width) & default)

    return reduce(lambda a, b: a|b, output)

def hihalf(signal):
    return signal[16:]

def lohalf(signal):
    return signal[:16]

# Selects between the halfwords of (32-bit) signal: if hi is 1, chooses the
# high half, otherwise the low half.
def choosehalf(hi, signal):
    return mux(hi, hihalf(signal), lohalf(signal))


