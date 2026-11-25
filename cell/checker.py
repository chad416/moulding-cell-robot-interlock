"""Explicit-state exploration of the cell model (breadth first, complete).

explore() visits every state reachable from the initial state under the scenario's fault, evaluates the hazard
predicates P1 and P2 in every state and keeps the shortest path to the first violation of each. With
`keep_edges` it also keeps the graph for can_always_reach().
"""

from __future__ import annotations

from collections import deque

from .system import State, System


def explore(sys: System, keep_edges: bool = False, limit: int = 2_000_000) -> dict:
    s0 = sys.initial()
    parent: dict[State, tuple[State | None, str]] = {s0: (None, "init")}
    queue = deque([s0])
    first = {"P1": None, "P2": None}
    count = {"P1": 0, "P2": 0}
    edges: dict[State, list[tuple[str, State]]] = {}
    n_tr = 0
    while queue:
        s = queue.popleft()
        p1, p2 = sys.hazards(s)
        for name, hit in (("P1", p1), ("P2", p2)):
            if hit:
                count[name] += 1
                if first[name] is None:
                    first[name] = s
        tr = sys.transitions(s)
        n_tr += len(tr)
        if keep_edges:
            edges[s] = tr
        for lab, s2 in tr:
            if s2 not in parent:
                parent[s2] = (s, lab)
                queue.append(s2)
                if len(parent) > limit:
                    raise RuntimeError("state limit reached; the exploration is not complete")
    return {"states": len(parent), "transitions": n_tr, "violations": count,
            "trace": {k: (path(parent, v) if v is not None else None) for k, v in first.items()},
            "parent": parent, "edges": edges}


def path(parent: dict, s: State) -> list[tuple[str, State]]:
    out = []
    while s is not None:
        prev, lab = parent[s]
        out.append((lab, s))
        s = prev
    return out[::-1]


def can_always_reach(res: dict, label: str) -> dict:
    """From how many reachable states can a transition with this label still be reached?"""
    edges = res["edges"]
    rev: dict[State, list[State]] = {}
    good = set()
    for s, tr in edges.items():
        for lab, s2 in tr:
            rev.setdefault(s2, []).append(s)
            if lab == label:
                good.add(s)
    queue = deque(good)
    while queue:
        s = queue.popleft()
        for p in rev.get(s, ()):
            if p not in good:
                good.add(p)
                queue.append(p)
    return {"states": len(edges), "can_reach": len(good), "stuck": [s for s in edges if s not in good]}


def format_trace(sys: System, trace: list[tuple[str, State]]) -> str:
    lines = []
    for i, (lab, s) in enumerate(trace):
        p1, p2 = sys.hazards(s)
        mark = "  <-- P1: mould closing on the gripper" if p1 else ("  <-- P2" if p2 else "")
        lines.append(f"{i:3d} {lab:<28} {sys.describe(s)}{mark}")
    return "\n".join(lines)
