"""Interpreter for the subset of FANUC TP (LS file format) used in robot/*.ls.

The program is executed as a state machine: one instruction per step. The same interpreter runs in the timed
in-the-loop simulation and in the state-space exploration, so the robot model is the program file itself.

Supported: comments, UFRAME_NUM / UTOOL_NUM / R[] / $WAITTMOUT assignments, DO[]/RO[] = ON|OFF, LBL, JMP,
IF <cond>,JMP LBL[], WAIT <cond> [TIMEOUT,LBL[]], WAIT x(sec), J / L motion to PR[n:NAME], UALM[].
Conditions: DI[], DO[], RI[] = ON|OFF joined by AND / OR, with parentheses.

Robot state (hashable): (pc, pos, target, do, ro, alarm)
  pc      index into the instruction list
  pos     name of the last position reached, target: name of the position being approached or None
  do, ro  frozensets of the output numbers that are ON
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import NamedTuple


class Rob(NamedTuple):
    pc: int
    pos: str
    target: str | None
    do: frozenset
    ro: frozenset
    alarm: bool


class Ins(NamedTuple):
    line: int
    kind: str
    a: object = None
    b: object = None
    c: object = None
    text: str = ""


_SIG = re.compile(r"(DI|DO|RI|RO)\[(\d+)(?::[^\]]*)?\]=(ON|OFF)")


def _cond(text: str):
    """Compile a TP condition to a function of (di, do, ri)."""
    names = []

    def sub(m):
        names.append((m.group(1), int(m.group(2))))
        return f"(s[{len(names) - 1}] == {m.group(3) == 'ON'})"

    expr = _SIG.sub(sub, text).replace(" AND ", " and ").replace(" OR ", " or ")
    if not re.fullmatch(r"[\s()s\[\]\d=TrueFalsndo]+", expr):
        raise ValueError(f"unsupported condition: {text}")
    code = compile(expr, "<tp>", "eval")
    return lambda rd: eval(code, {"__builtins__": {}}, {"s": [rd(k, n) for k, n in names]}), names


class Program:
    def __init__(self, path: Path):
        text = Path(path).read_text(encoding="utf-8")
        self.name = re.search(r"/PROG\s+(\w+)", text).group(1)
        self.line_count = int(re.search(r"LINE_COUNT\s*=\s*(\d+);", text).group(1))
        body = text[text.index("/MN"):text.index("/POS")]
        self.lines = [(int(n), t.strip()) for n, t in re.findall(r"^\s*(\d+):\s*(.*?)\s*;\s*$", body, flags=re.M)]
        self.ins: list[Ins] = []
        self.signals: set[tuple[str, int, str]] = set()
        self.positions: set[tuple[int, str]] = set()
        self.wait_timeout_s = 30.0
        labels: dict[int, int] = {}
        pending: list[tuple[int, str, str]] = []
        for no, t in self.lines:
            for k, n, name in re.findall(r"\b(DI|DO|RI|RO)\[(\d+):([^\]]+)\]", t):
                self.signals.add((k, int(n), name))
            if t.startswith("!") or re.match(r"(UFRAME_NUM|UTOOL_NUM|R\[\d+[^\]]*\])\s*=", t):
                continue
            if m := re.fullmatch(r"\$WAITTMOUT=(\d+)", t):
                self.wait_timeout_s = int(m.group(1)) / 100.0
                continue
            if m := re.fullmatch(r"LBL\[(\d+)(?::[^\]]*)?\]", t):
                labels[int(m.group(1))] = len(self.ins)
                continue
            pending.append((no, t, ""))
            self.ins.append(Ins(no, "?", text=t))
        for i, (no, t, _) in enumerate(pending):
            self.ins[i] = self._parse(no, t, labels)

    def _parse(self, no: int, t: str, labels: dict[int, int]) -> Ins:
        if m := re.fullmatch(r"(DO|RO)\[(\d+)(?::[^\]]*)?\]=(ON|OFF)", t):
            return Ins(no, m.group(1).lower(), int(m.group(2)), m.group(3) == "ON", text=t)
        if m := re.fullmatch(r"JMP LBL\[(\d+)\]", t):
            return Ins(no, "jmp", labels[int(m.group(1))], text=t)
        if m := re.fullmatch(r"IF (.+),JMP LBL\[(\d+)\]", t):
            return Ins(no, "if", _cond(m.group(1))[0], labels[int(m.group(2))], text=t)
        if m := re.fullmatch(r"WAIT\s+(\d*\.?\d+)\(sec\)", t):
            return Ins(no, "delay", float(m.group(1)), text=t)
        if m := re.fullmatch(r"WAIT (.+?)(?: TIMEOUT,LBL\[(\d+)\])?", t):
            return Ins(no, "wait", _cond(m.group(1))[0], labels[int(m.group(2))] if m.group(2) else None, text=t)
        if m := re.fullmatch(r"([JL]) PR\[(\d+):([^\]]+)\] (\S+) (FINE|CNT\d+)", t):
            self.positions.add((int(m.group(2)), m.group(3)))
            return Ins(no, "move", m.group(3), m.group(1), m.group(4), text=t)
        if re.fullmatch(r"UALM\[\d+\]", t):
            return Ins(no, "ualm", text=t)
        raise ValueError(f"{self.name} line {no}: unsupported instruction: {t}")

    def initial(self, pos: str = "HOME") -> Rob:
        return Rob(0, pos, None, frozenset(), frozenset(), False)

    def steps(self, st: Rob, read, timeouts: bool = True) -> list[tuple[str, Rob]]:
        """Enabled steps as (kind, next state). read(kind, n) returns the level of DI / DO / RI n.
        Kinds: instr, move_start, move_done, wait_ok, timeout, delay, alarm."""
        if st.alarm:
            return []
        if st.target is not None:
            return [("move_done", st._replace(pos=st.target, target=None, pc=st.pc + 1))]
        i = self.ins[st.pc]
        nxt = st.pc + 1
        if i.kind == "do":
            return [("instr", st._replace(pc=nxt, do=st.do | {i.a} if i.b else st.do - {i.a}))]
        if i.kind == "ro":
            return [("instr", st._replace(pc=nxt, ro=st.ro | {i.a} if i.b else st.ro - {i.a}))]
        if i.kind == "jmp":
            return [("instr", st._replace(pc=i.a))]
        if i.kind == "if":
            return [("instr", st._replace(pc=i.b if i.a(read) else nxt))]
        if i.kind == "delay":
            return [("delay", st._replace(pc=nxt))]
        if i.kind == "wait":
            if i.a(read):
                return [("wait_ok", st._replace(pc=nxt))]
            return [("timeout", st._replace(pc=i.b))] if (i.b is not None and timeouts) else []
        if i.kind == "move":
            return [("move_start", st._replace(target=i.a))]
        if i.kind == "ualm":
            return [("alarm", st._replace(alarm=True))]
        raise AssertionError(i)

    def where(self, st: Rob) -> str:
        i = self.ins[st.pc] if st.pc < len(self.ins) else None
        return f"{i.line}: {i.text}" if i else "end"
