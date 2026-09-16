"""Map nvidia-smi's physical GPU index to EGL's independent device enumeration."""
import argparse
import ctypes as c
from ctypes.util import find_library
import json
import os
from pathlib import Path
import subprocess


def select(index=0):
    row = subprocess.check_output(["nvidia-smi", f"--id={index}",
        "--query-gpu=pci.bus_id,uuid", "--format=csv,noheader"], text=True).strip()
    bus, uuid = [v.strip() for v in row.split(",")]
    wanted = bus.lower()[-12:]
    lib = c.CDLL(find_library("EGL"))
    lib.eglGetProcAddress.argtypes = [c.c_char_p]
    lib.eglGetProcAddress.restype = c.c_void_p
    query = c.CFUNCTYPE(c.c_uint, c.c_int, c.POINTER(c.c_void_p), c.POINTER(c.c_int))(
        lib.eglGetProcAddress(b"eglQueryDevicesEXT"))
    string = c.CFUNCTYPE(c.c_char_p, c.c_void_p, c.c_int)(
        lib.eglGetProcAddress(b"eglQueryDeviceStringEXT"))
    count = c.c_int()
    assert query(0, None, c.byref(count)), "Cannot enumerate EGL devices"
    devices = (c.c_void_p * count.value)()
    assert query(count.value, devices, c.byref(count))
    discovered = []
    # NVIDIA EGL devices may expose CUDA identity without exposing a DRM node.
    attrib = c.CFUNCTYPE(c.c_uint, c.c_void_p, c.c_int, c.POINTER(c.c_ssize_t))(
        lib.eglGetProcAddress(b"eglQueryDeviceAttribEXT"))
    os.environ.pop("CUDA_VISIBLE_DEVICES", None)
    cuda = c.CDLL("libcuda.so.1")
    assert cuda.cuInit(0) == 0, "Cannot initialize CUDA for PCI identity lookup"
    cuda.cuDeviceGetPCIBusId.argtypes = [c.c_char_p, c.c_int, c.c_int]
    for i, device in enumerate(devices):
        extensions = (string(device, 0x3055) or b"").decode()
        if "EGL_NV_device_cuda" in extensions:
            ordinal = c.c_ssize_t()
            assert attrib(device, 0x323A, c.byref(ordinal))
            pci_buffer = c.create_string_buffer(32)
            assert cuda.cuDeviceGetPCIBusId(pci_buffer, len(pci_buffer), ordinal.value) == 0
            pci = pci_buffer.value.decode().lower()[-12:]
            discovered.append({"egl_index": i, "cuda_ordinal": ordinal.value, "pci_bus": pci})
            if pci == wanted:
                return {"physical_gpu": index, "egl_index": i, "pci_bus": bus, "uuid": uuid}
            continue
        key = 0x3377 if "EGL_EXT_device_drm_render_node" in extensions else 0x3233
        if "EGL_EXT_device_drm" not in extensions:
            continue
        node = (string(device, key) or b"").decode()
        if not node:
            continue
        pci = (Path("/sys/class/drm") / Path(node).name / "device").resolve().name.lower()
        discovered.append({"egl_index": i, "node": node, "pci_bus": pci})
    raise RuntimeError(f"No EGL device matches GPU {index} ({bus}): {discovered}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--gpu", type=int, default=0)
    p.add_argument("--fields", action="store_true")
    args = p.parse_args()
    result = select(args.gpu)
    print(f'{result["egl_index"]} {result["uuid"]}' if args.fields else json.dumps(result))
