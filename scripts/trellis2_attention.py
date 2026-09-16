"""Process-local xFormers routing for the pinned Blackwell inference stack."""
from functools import wraps


def configure_attention():
    """Avoid automatic FA3 dispatch; do not modify installed package files."""
    import xformers.ops as xops
    from xformers.ops.fmha import cutlass

    original = xops.memory_efficient_attention
    if getattr(original, '_humanvideo_cutlass', False):
        return

    @wraps(original)
    def cutlass_attention(*args, **kwargs):
        kwargs['op'] = (cutlass.FwOp, cutlass.BwOp)
        return original(*args, **kwargs)

    cutlass_attention._humanvideo_cutlass = True
    xops.memory_efficient_attention = cutlass_attention
    print('humanvideo: xFormers CUTLASS explicitly selected (FA3 dispatch bypassed)', flush=True)
