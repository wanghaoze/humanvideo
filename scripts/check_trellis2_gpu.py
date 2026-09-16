#!/usr/bin/env python3
"""Run real CUDA and xFormers kernels before downloading large model weights."""
import os

os.environ.setdefault('CUDA_VISIBLE_DEVICES', '0')

import torch
import xformers
import xformers.ops as xops
from trellis2_attention import configure_attention

assert torch.__version__.split('+')[0] == '2.8.0', torch.__version__
assert torch.version.cuda == '12.9', torch.version.cuda
assert xformers.__version__ == '0.0.32.post2', xformers.__version__
assert torch.cuda.is_available(), 'CUDA is not available'
print('GPU:', torch.cuda.get_device_name(0))
print('Capability:', torch.cuda.get_device_capability(0))
print('Torch:', torch.__version__, 'CUDA:', torch.version.cuda)
print('xFormers:', xformers.__version__)
configure_attention()

with torch.inference_mode():
    matrix = torch.randn(64, 64, device='cuda')
    assert torch.isfinite(matrix @ matrix).all().item()
    for dtype in (torch.float16, torch.bfloat16):
        for dim in (64, 128):
            for qlens, klens in (([32], [40]), ([13, 19], [17, 23])):
                q = torch.randn(1, sum(qlens), 4, dim, device='cuda', dtype=dtype)
                k, v = [torch.randn(1, sum(klens), 4, dim, device='cuda', dtype=dtype) for _ in range(2)]
                bias = None if len(qlens) == 1 else xops.fmha.BlockDiagonalMask.from_seqlens(qlens, klens)
                print(f'Checking CUTLASS: {dtype}, head_dim={dim}, q={qlens}, kv={klens}', flush=True)
                out = xops.memory_efficient_attention(q, k, v, attn_bias=bias)
                refs = []
                qi = ki = 0
                for qlen, klen in zip(qlens, klens):
                    # Independent small FP32 reference, without xFormers dispatch.
                    qs = q[:, qi:qi+qlen].transpose(1, 2).float()
                    ks = k[:, ki:ki+klen].transpose(1, 2).float()
                    vs = v[:, ki:ki+klen].transpose(1, 2).float()
                    weights = (qs @ ks.transpose(-2, -1) / dim**0.5).softmax(dim=-1)
                    refs.append((weights @ vs).transpose(1, 2))
                    qi += qlen
                    ki += klen
                reference = torch.cat(refs, dim=1)
                torch.testing.assert_close(out.float(), reference, atol=0.02, rtol=0.02)
    torch.cuda.synchronize()
print('PASS: CUDA matmul, FP16/BF16 CUTLASS dense/variable-length attention vs FP32 reference')
