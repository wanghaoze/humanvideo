"""CPU-only check that both repeated setup and operator selection are safe."""
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

from trellis2_attention import configure_attention

calls = []


def original(*args, **kwargs):
    calls.append((args, kwargs))
    return 'result'


xformers = ModuleType('xformers')
ops = ModuleType('xformers.ops')
fmha = ModuleType('xformers.ops.fmha')
fw, bw = object(), object()
fmha.cutlass = SimpleNamespace(FwOp=fw, BwOp=bw)
ops.memory_efficient_attention = original
xformers.ops = ops
with patch.dict(sys.modules, {'xformers': xformers, 'xformers.ops': ops, 'xformers.ops.fmha': fmha}):
    configure_attention()
    first = ops.memory_efficient_attention
    configure_attention()
    assert ops.memory_efficient_attention is first
    assert first('q', 'k', 'v', 'bias', scale=0.5) == 'result'
    assert calls == [(('q', 'k', 'v', 'bias'), {'scale': 0.5, 'op': (fw, bw)})]
print('PASS: forced CUTLASS operator, argument forwarding, idempotent setup')
