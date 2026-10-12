#!/usr/bin/env python3
"""Probe whether this host can give the simulator the OpenGL it needs.

The simulator computes lidar and semantic rays with a GLSL ``#version 440``
compute shader (``simulation/ray_sensors/shaders/id_compute.glsl``), so it needs
an OpenGL 4.4+ context in which that shader compiles. A working CUDA driver or a
successful ``nvidia-smi`` does not establish this.

The probe mirrors the playground's own context creation: a hidden 1x1
``arcade.Window`` (see ``simulation/gui_map/playground.py``). On a host without
a display server, run it with pyglet's EGL headless mode selected through the
environment, which changes no project or framework code:

    PYGLET_HEADLESS=True PYGLET_HEADLESS_DEVICE=0 python devtools/probe_opengl.py

It then compiles the baseline compute shader with small placeholder sizes and
prints one JSON object. Exit status: 0 when an OpenGL 4.4+ context was created
and the shader compiled, 1 otherwise. Nothing is written to disk.
"""

from __future__ import annotations

import ctypes
import json
import os
import platform
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SHADER = REPO_ROOT / "src/swarm_rescue/simulation/ray_sensors/shaders/id_compute.glsl"


def _gl_string(gl, name: int) -> str:
    raw = gl.glGetString(name)
    return ctypes.cast(raw, ctypes.c_char_p).value.decode("utf-8", "replace") if raw else ""


def main() -> int:
    report = {
        "python": platform.python_version(),
        "platform": platform.platform(),
        "display": bool(os.environ.get("DISPLAY")),
        "pyglet_headless_env": os.environ.get("PYGLET_HEADLESS"),
        "pyglet_headless_device_env": os.environ.get("PYGLET_HEADLESS_DEVICE"),
    }
    ok = False
    started = time.perf_counter()
    try:
        import arcade
        import pyglet
        from pyglet import gl

        report["arcade"] = arcade.__version__
        report["pyglet"] = pyglet.version
        report["pyglet_options"] = {k: pyglet.options[k] for k in ("headless", "headless_device")}
        window = arcade.Window(width=1, height=1, visible=False)
        try:
            ctx = window.ctx
            report["gl_version"] = list(ctx.gl_version)
            report["vendor"] = _gl_string(gl, gl.GL_VENDOR)
            report["renderer"] = _gl_string(gl, gl.GL_RENDERER)
            report["version_string"] = _gl_string(gl, gl.GL_VERSION)
            report["glsl_version"] = _gl_string(gl, gl.GL_SHADING_LANGUAGE_VERSION)
            source = SHADER.read_text(encoding="utf-8")
            for placeholder, value in (("N_SENSORS", 2), ("MAX_N_RAYS", 181),
                                       ("MAX_N_INVISIBLE", 4)):
                source = source.replace(placeholder, str(value))
            ctx.compute_shader(source=source)
            report["compute_shader_compiled"] = True
            ok = tuple(ctx.gl_version) >= (4, 4)
        finally:
            window.close()
    except Exception as exc:  # report the failure instead of a traceback
        report["error"] = f"{type(exc).__name__}: {exc}"
        report.setdefault("compute_shader_compiled", False)
    report["seconds"] = round(time.perf_counter() - started, 3)
    report["opengl_ok"] = ok
    print(json.dumps(report, sort_keys=True))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
