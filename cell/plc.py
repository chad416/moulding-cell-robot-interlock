"""ctypes wrapper around the compiled ST function block CELL_PLC (build/plc/libplc.so).

The block is used as a pure transition function: scan(state bytes, inputs) -> (state bytes, outputs). The shared
library holds one static instance; rt_load / rt_save move its whole memory in and out, so the same compiled code
serves the timed run and the state-space exploration. Results are cached per (state, inputs).
"""

from __future__ import annotations

import ctypes
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build" / "plc"


def _struct(name: str, fields: list[list[str]]) -> type[ctypes.Structure]:
    return type(name, (ctypes.Structure,), {"_fields_": [(n, getattr(ctypes, t)) for n, t in fields]})


class Plc:
    def __init__(self, build_dir: Path = BUILD):
        io = json.loads((build_dir / "io_map.json").read_text())
        self.In = _struct("rt_in_t", io["inputs"])
        self.Out = _struct("rt_out_t", io["outputs"])
        self.constants: dict[str, int] = io["constants"]
        self.lib = ctypes.CDLL(str(build_dir / "libplc.so"))
        assert self.lib.rt_sizeof_in() == ctypes.sizeof(self.In), "io_map does not match libplc.so"
        assert self.lib.rt_sizeof_out() == ctypes.sizeof(self.Out), "io_map does not match libplc.so"
        self.lib.rt_scan.argtypes = [ctypes.POINTER(self.In), ctypes.POINTER(self.Out)]
        self.input_names = [n for n, _ in io["inputs"]]
        self.output_names = [n for n, _ in io["outputs"]]
        self._buf = ctypes.create_string_buffer(self.lib.rt_sizeof_state())
        self._cache: dict = {}
        self.lib.rt_init()
        self.lib.rt_save(self._buf)
        self.initial: bytes = self._buf.raw
        self.scans = 0

    def scan(self, state: bytes, inputs: tuple) -> tuple[bytes, tuple]:
        """inputs and outputs are tuples in the order of input_names / output_names."""
        key = (state, inputs)
        hit = self._cache.get(key)
        if hit is not None:
            return hit
        self.lib.rt_load(state)
        inp, out = self.In(), self.Out()
        for n, v in zip(self.input_names, inputs):
            setattr(inp, n, int(v))
        self.lib.rt_scan(ctypes.byref(inp), ctypes.byref(out))
        self.lib.rt_save(self._buf)
        res = (self._buf.raw, tuple(int(getattr(out, n)) for n in self.output_names))
        self._cache[key] = res
        self.scans += 1
        return res

    def name_of(self, prefix: str, value: int) -> str:
        for k, v in self.constants.items():
            if k.startswith(prefix) and v == value:
                return k
        return str(value)


def available(build_dir: Path = BUILD) -> bool:
    """The library is an ELF shared object: Linux or WSL only."""
    return sys.platform.startswith("linux") and (build_dir / "libplc.so").exists() and (build_dir / "io_map.json").exists()
