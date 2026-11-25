"""Move times from speed and acceleration limits with a trapezoidal profile per move.

6-axis robot: J1 turns the arm plane, J2/J3 are a planar two-link arm, J5 holds the tool pitch and J6 the tool yaw
(J4 is not used). Joint moves (J) are synchronised: every joint runs its own trapezoid and the slowest one sets the
time. Linear moves (L) follow the straight line; the path speed and acceleration are limited so that no joint
exceeds its speed or acceleration limit anywhere on the line (the velocity-squared term of the joint acceleration
is neglected). Every move starts and ends at rest (FINE); blending (CNT) is not credited.

Linear robot: one trapezoid per axis stroke.
"""

from __future__ import annotations

import math


def trapezoid(dist: float, v: float, a: float) -> float:
    dist = abs(dist)
    if dist <= 0.0:
        return 0.0
    if dist <= v * v / a:
        return 2.0 * math.sqrt(dist / a)
    return dist / v + v / a


class SixAxis:
    def __init__(self, rb: dict, accel_time_s: float | None = None):
        self.l1, self.l2 = rb["link_upper_mm"], rb["link_fore_mm"]
        self.w = [math.radians(v) for v in rb["joint_speed_deg_s"]]
        ta = rb["accel_time_s"] if accel_time_s is None else accel_time_s
        self.al = [w / ta for w in self.w]
        self.v_lin = rb["linear_speed_max_mm_s"]

    def ik(self, xyz) -> list[float]:
        x, y, z = xyz
        q1 = math.atan2(y, x)
        r = math.hypot(x, y)
        d = (r * r + z * z - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        if abs(d) > 1.0:
            raise ValueError(f"point {xyz} is out of reach")
        q3 = math.acos(d)                                  # elbow bend
        q2 = math.atan2(z, r) + math.atan2(self.l2 * math.sin(q3), self.l1 + self.l2 * math.cos(q3))
        pitch = q2 - q3
        return [q1, q2, q3, 0.0, -pitch, -q1]

    def joint_move(self, a, b) -> float:
        qa, qb = self.ik(a), self.ik(b)
        return max(trapezoid(qb[j] - qa[j], self.w[j], self.al[j]) for j in range(6))

    def linear_move(self, a, b, v_cmd: float | None = None, n: int = 60) -> float:
        dist = math.dist(a, b)
        if dist == 0.0:
            return 0.0
        pts = [[a[k] + (b[k] - a[k]) * i / n for k in range(3)] for i in range(n + 1)]
        qs = [self.ik(p) for p in pts]
        ds = dist / n
        v_max = min(self.v_lin, v_cmd or self.v_lin)
        a_max = float("inf")
        for i in range(n):
            for j in range(6):
                g = abs(qs[i + 1][j] - qs[i][j]) / ds      # rad per mm of path
                if g > 1e-12:
                    v_max = min(v_max, self.w[j] / g)
                    a_max = min(a_max, self.al[j] / g)
        return trapezoid(dist, v_max, a_max)


# The move list of robot/UNLOAD_IMM.ls: (from, to, type, programmed linear speed in mm/s or None)
PROGRAM_MOVES = [
    ("WAIT_IMM", "MOULD_IN", "L", 2000), ("MOULD_IN", "GRIP", "L", 500), ("GRIP", "MOULD_IN", "L", 500),
    ("MOULD_IN", "WAIT_IMM", "L", 2000), ("WAIT_IMM", "CUT", "J", None), ("CUT", "PLACE_APPR", "J", None),
    ("PLACE_APPR", "PLACE", "L", 1000), ("PLACE", "PLACE_APPR", "L", 1000), ("PLACE_APPR", "WAIT_IMM", "J", None),
    ("HOME", "WAIT_IMM", "J", None),
]


def six_axis_move_times(cfg: dict, layout: str, accel_time_s: float | None = None) -> dict:
    rb = SixAxis(cfg["robot_6axis"], accel_time_s)
    pos = {k: v["xyz"] for k, v in cfg["layouts"][layout]["positions"].items()}
    out = {}
    for a, b, kind, v in PROGRAM_MOVES:
        out[(a, b)] = rb.joint_move(pos[a], pos[b]) if kind == "J" else rb.linear_move(pos[a], pos[b], v)
    return out


def removal_budget(cfg: dict, layout: str, accel_time_s: float | None = None) -> dict:
    """Time from "Mould open position" to "Enable mould closure" and the downstream time outside the mould."""
    lay, c, g, dev = cfg["layouts"][layout], cfg["cycle"], cfg["gripper"], cfg["cell_devices"]
    latency = 2 * cfg["plc"]["scan_s"] + 2 * cfg["robot_6axis"]["io_scan_s"]   # signal in, signal out
    grip = max(g["vacuum_on_s"], c["ejector_fwd_s"])
    if lay["kind"] == "six_axis":
        t = six_axis_move_times(cfg, layout, accel_time_s)
        t_in = t[("WAIT_IMM", "MOULD_IN")] + t[("MOULD_IN", "GRIP")]
        t_out = t[("GRIP", "MOULD_IN")] + t[("MOULD_IN", "WAIT_IMM")]
        down = (t[("WAIT_IMM", "CUT")] + dev["cutter_close_s"] + dev["cutter_open_s"] + t[("CUT", "PLACE_APPR")]
                + t[("PLACE_APPR", "PLACE")] + g["release_s"] + t[("PLACE", "PLACE_APPR")]
                + t[("PLACE_APPR", "WAIT_IMM")])
    else:
        ax, st = cfg["robot_linear"], lay["strokes_mm"]
        tv = trapezoid(st["vertical"], ax["vertical"]["v_mm_s"], ax["vertical"]["a_mm_s2"])
        tk = trapezoid(st["kick"], ax["kick"]["v_mm_s"], ax["kick"]["a_mm_s2"])
        tt = trapezoid(st["traverse"], ax["traverse"]["v_mm_s"], ax["traverse"]["a_mm_s2"])
        td = trapezoid(st["place_down"], ax["vertical"]["v_mm_s"], ax["vertical"]["a_mm_s2"])
        t_in, t_out = tv + tk, tk + tv
        down = 2 * tt + 2 * td + g["release_s"] + dev["cutter_close_s"] + dev["cutter_open_s"]
    removal = latency + t_in + grip + t_out
    return {"name": lay["name"], "t_in_s": t_in, "t_grip_eject_s": grip, "t_out_s": t_out, "latency_s": latency,
            "removal_s": removal, "downstream_s": down, "floor_m2": lay["floor_m2"], "downstream": lay["downstream"]}
