#!/usr/bin/env python3
"""Map EGL device indices to physical GPUs (Linux, NVIDIA EGL).

pyglet's headless mode opens ``eglQueryDevicesEXT()[PYGLET_HEADLESS_DEVICE]``.
That list order is not guaranteed to match ``nvidia-smi`` or CUDA numbering,
so a run that names an EGL index does not by itself name a GPU. This script
resolves every EGL device to its DRM node, the DRM node to its PCI address
through sysfs, and the PCI address to the ``nvidia-smi`` index and UUID. It
also reports ``EGL_CUDA_DEVICE_NV`` where the driver exposes it.

The query is read-only: it enumerates devices and reads their attributes but
creates no display connection, context or surface, so it allocates nothing on
any GPU. Output is one JSON object on stdout; exit status 0 when every EGL
device was resolved to exactly one GPU, 1 otherwise.
"""

from __future__ import annotations

import ctypes
import json
import os
import subprocess
import sys

EGL_EXTENSIONS = 0x3055
EGL_DRM_DEVICE_FILE_EXT = 0x3233
EGL_CUDA_DEVICE_NV = 0x323A


def _nvidia_smi_gpus() -> list:
    out = subprocess.run(
        ["nvidia-smi", "--query-gpu=index,uuid,pci.bus_id,name",
         "--format=csv,noheader"],
        check=True, capture_output=True, text=True,
    ).stdout
    gpus = []
    for line in out.strip().splitlines():
        index, uuid, bus_id, name = [part.strip() for part in line.split(",")]
        # nvidia-smi prints an 8-digit PCI domain; sysfs uses 4 digits.
        domain, rest = bus_id.split(":", 1)
        gpus.append({
            "smi_index": int(index),
            "uuid": uuid,
            "pci": f"{int(domain, 16):04x}:{rest}".lower(),
            "name": name,
        })
    return gpus


def _pci_of_drm_node(path: str) -> str:
    card = os.path.basename(path)
    target = os.path.realpath(f"/sys/class/drm/{card}/device")
    return os.path.basename(target).lower()


def main() -> int:
    egl = ctypes.CDLL("libEGL.so.1")
    egl.eglGetProcAddress.restype = ctypes.c_void_p
    egl.eglGetProcAddress.argtypes = [ctypes.c_char_p]

    def proc(name, restype, argtypes):
        address = egl.eglGetProcAddress(name.encode())
        if not address:
            raise RuntimeError(f"{name} is not available")
        return ctypes.CFUNCTYPE(restype, *argtypes)(address)

    query_devices = proc("eglQueryDevicesEXT", ctypes.c_uint,
                         [ctypes.c_int, ctypes.POINTER(ctypes.c_void_p),
                          ctypes.POINTER(ctypes.c_int)])
    query_string = proc("eglQueryDeviceStringEXT", ctypes.c_char_p,
                        [ctypes.c_void_p, ctypes.c_int])
    query_attrib = proc("eglQueryDeviceAttribEXT", ctypes.c_uint,
                        [ctypes.c_void_p, ctypes.c_int,
                         ctypes.POINTER(ctypes.c_ssize_t)])

    count = ctypes.c_int(0)
    query_devices(0, None, ctypes.byref(count))
    devices = (ctypes.c_void_p * count.value)()
    query_devices(count.value, devices, ctypes.byref(count))

    gpus = _nvidia_smi_gpus()
    by_pci = {gpu["pci"]: gpu for gpu in gpus}
    rows = []
    resolved = 0
    for index in range(count.value):
        device = devices[index]
        extensions = (query_string(device, EGL_EXTENSIONS) or b"").decode()
        row = {"egl_index": index, "extensions": extensions.split()}
        drm = query_string(device, EGL_DRM_DEVICE_FILE_EXT)
        row["drm_device_file"] = drm.decode() if drm else None
        if "EGL_NV_device_cuda" in extensions:
            value = ctypes.c_ssize_t(-1)
            if query_attrib(device, EGL_CUDA_DEVICE_NV, ctypes.byref(value)):
                row["cuda_device"] = int(value.value)
        if row["drm_device_file"]:
            pci = _pci_of_drm_node(row["drm_device_file"])
            row["pci"] = pci
            gpu = by_pci.get(pci)
            if gpu is not None:
                row.update({"smi_index": gpu["smi_index"],
                            "uuid": gpu["uuid"], "name": gpu["name"]})
                resolved += 1
        rows.append(row)

    report = {"egl_device_count": count.value, "nvidia_smi_gpus": gpus,
              "egl_devices": rows,
              "all_resolved": resolved == count.value and count.value > 0}
    print(json.dumps(report, indent=1))
    return 0 if report["all_resolved"] else 1


if __name__ == "__main__":
    sys.exit(main())
