"""Wayfinder: a robot guide that leads a blind person through a busy indoor space.

Everything runs locally: MuJoCo simulates the world, the robot sees only through
its own simulated depth camera, and all decisions (mapping, planning, safety)
are made on the laptop CPU. No cloud calls.

    venv/bin/python guide_sim.py                 # record a run (random events) -> runs/*.mp4
    venv/bin/python guide_sim.py --show          # live window, real time (for the pitch)
    venv/bin/python guide_sim.py --emg           # handle grip from the EMG bridge (OSC 9001)
    venv/bin/python guide_sim.py --seed 7        # repeat an exact run
"""
import argparse
import heapq
import math
import os
import subprocess
import threading
import time

import cv2
import mujoco
import numpy as np

# ---------------------------------------------------------------- the world
W, H = 20.0, 10.0               # concourse size (m)
RES = 0.25                      # map cell size (m)
GW, GH = int(W / RES), int(H / RES)
START, GOAL = (1.5, 5.0), (18.6, 5.0)
HATCH = (8.0, 10.0, 4.0, 6.0)   # open floor hatch: NOT on the robot's map
PILLARS = [(6, 2.4), (6, 7.6), (13, 2.4), (13, 7.6)]
FURNITURE = [((4.0, 9.2), (0.9, 0.3)), ((15.5, 0.8), (0.9, 0.3)), ((16.5, 8.4), (0.6, 0.6))]
FPS = 25
N_PEOPLE = 7
GATES = [(3, 10), (7.5, 10), (11.5, 10), (16, 10), (4.5, 0), (9.5, 0), (13.5, 0), (17.5, 0)]   # platforms
HORIZON = 2.0       # how far ahead (s) the robot predicts people
DT = 1.0 / FPS


def build_xml():
    tiles = []
    for i in range(int(W)):
        for j in range(int(H)):
            cx, cy = i + 0.5, j + 0.5
            if HATCH[0] < cx < HATCH[1] and HATCH[2] < cy < HATCH[3]:
                continue
            shade = "tileA" if (i + j) % 2 else "tileB"
            tiles.append(f'<geom type="box" size=".5 .5 .05" pos="{cx} {cy} -.05" material="{shade}"/>')
    pillars = "".join(f'<geom type="cylinder" size=".3 1.5" pos="{x} {y} 1.5" rgba=".72 .72 .76 1"/>' for x, y in PILLARS)
    furn = "".join(f'<geom type="box" size="{sx} {sy} .3" pos="{x} {y} .3" rgba=".55 .4 .28 1"/>'
                   for (x, y), (sx, sy) in FURNITURE)
    cols = [".55 .3 .75", ".85 .45 .3", ".3 .45 .85", ".75 .7 .25", ".35 .65 .4", ".8 .35 .55", ".45 .45 .5"]
    crowd = "".join(f'''<body name="p{i}" mocap="true" pos="0 0 -20">
      <geom type="capsule" fromto="0 0 .25 0 0 1.35" size=".21" rgba="{cols[i % len(cols)]} 1" contype="0" conaffinity="0" group="1"/>
      <geom type="sphere" size=".14" pos="0 0 1.62" rgba=".6 .45 .35 1" contype="0" conaffinity="0"/></body>
    <body name="g{i}" mocap="true" pos="0 0 -20">
      <geom type="cylinder" size=".3 .004" rgba=".55 .3 .95 .45" contype="0" conaffinity="0"/></body>''' for i in range(N_PEOPLE))
    gates = "".join(f'<geom type="box" size=".7 .03 .55" pos="{x} {9.99 if y > 5 else 0.01} 1.0" rgba=".25 .45 .8 1"/>' for x, y in GATES)
    return f"""
<mujoco model="wayfinder">
  <visual><global offwidth="1280" offheight="960"/><quality shadowsize="4096"/>
    <map znear="0.01" zfar="60"/><headlight ambient=".35 .35 .38" diffuse=".4 .4 .4"/></visual>
  <statistic extent="25" center="10 5 0"/>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1=".93 .94 .97" rgb2=".72 .76 .82" width="256" height="256"/>
    <material name="tileA" rgba=".86 .86 .84 1"/><material name="tileB" rgba=".8 .8 .79 1"/>
  </asset>
  <worldbody>
    <light pos="10 5 9" dir="0 0 -1" diffuse=".7 .7 .7" castshadow="true"/>
    <geom type="plane" size="40 40 .1" pos="10 5 -2.5" rgba=".18 .18 .2 1"/>
    {''.join(tiles)}
    <geom type="box" size="10 .1 .7" pos="10 -.1 .7" rgba=".93 .9 .85 1"/>
    <geom type="box" size="10 .1 .7" pos="10 10.1 .7" rgba=".93 .9 .85 1"/>
    <geom type="box" size=".1 5 .7" pos="-.1 5 .7" rgba=".93 .9 .85 1"/>
    <geom type="box" size=".1 5 .7" pos="20.1 5 .7" rgba=".93 .9 .85 1"/>
    <geom type="box" size=".04 .9 1.15" pos="19.98 5 1.15" rgba=".2 .55 .5 1"/>
    <geom type="cylinder" size=".55 .005" pos="{GOAL[0]} {GOAL[1]} .005" rgba=".2 .75 .5 .6" contype="0" conaffinity="0"/>
    {pillars}{furn}
    <body name="robot" mocap="true" pos="{START[0]} {START[1]} 0">
      <geom type="box" size=".28 .2 .12" pos="0 0 .2" rgba="1 .78 .1 1" contype="0" conaffinity="0"/>
      <geom type="cylinder" size=".08 .03" pos=".17 .21 .08" euler="90 0 0" rgba=".12 .12 .12 1" contype="0" conaffinity="0"/>
      <geom type="cylinder" size=".08 .03" pos=".17 -.21 .08" euler="90 0 0" rgba=".12 .12 .12 1" contype="0" conaffinity="0"/>
      <geom type="cylinder" size=".08 .03" pos="-.17 .21 .08" euler="90 0 0" rgba=".12 .12 .12 1" contype="0" conaffinity="0"/>
      <geom type="cylinder" size=".08 .03" pos="-.17 -.21 .08" euler="90 0 0" rgba=".12 .12 .12 1" contype="0" conaffinity="0"/>
      <geom type="box" size=".05 .08 .04" pos=".24 0 .37" rgba=".1 .1 .12 1" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="-.25 0 .3 -.62 0 .95" size=".02" rgba=".15 .15 .15 1" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="-.62 -.12 .95 -.62 .12 .95" size=".028" rgba=".15 .15 .15 1" contype="0" conaffinity="0"/>
      <camera name="front" pos=".3 0 .4" xyaxes="0 -1 0 0.5 0 0.866" fovy="70"/>
    </body>
    <body name="user" mocap="true" pos="{START[0] - 1.1} {START[1]} 0">
      <geom type="capsule" fromto="0 0 .25 0 0 1.35" size=".2" rgba=".2 .62 .66 1" contype="0" conaffinity="0"/>
      <geom type="sphere" size=".14" pos="0 0 1.62" rgba=".85 .68 .55 1" contype="0" conaffinity="0"/>
    </body>
    {crowd}{gates}
    <body name="sign" mocap="true" pos="0 0 -20">
      <geom type="box" size=".08 .32 .45" pos="0 0 .45" rgba=".9 .2 .18 1" contype="0" conaffinity="0"/>
      <geom type="box" size=".085 .2 .12" pos="0 0 .6" rgba="1 1 1 1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""


# ---------------------------------------------------------------- map helpers
def cell(x, y):
    return int(np.clip(x / RES, 0, GW - 1)), int(np.clip(y / RES, 0, GH - 1))


def centre(c):
    return (c[0] + 0.5) * RES, (c[1] + 0.5) * RES


def prior_map():
    """What the robot knows before it starts: walls, pillars, furniture. Not the hatch."""
    m = np.zeros((GW, GH), np.uint8)
    m[0, :] = m[-1, :] = m[:, 0] = m[:, -1] = 1
    xs, ys = np.meshgrid((np.arange(GW) + .5) * RES, (np.arange(GH) + .5) * RES, indexing="ij")
    for x, y in PILLARS:
        m[(xs - x) ** 2 + (ys - y) ** 2 <= .35 ** 2] = 1
    for (x, y), (sx, sy) in FURNITURE:
        m[(abs(xs - x) <= sx) & (abs(ys - y) <= sy)] = 1
    return m


def disk(r_m):
    r = max(1, int(round(r_m / RES)))
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))


NBRS = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]


def astar(blocked, cost, s, g):
    if blocked[g]:
        return None
    openq, came, gs = [(0.0, s)], {s: None}, {s: 0.0}
    while openq:
        _, c = heapq.heappop(openq)
        if c == g:
            break
        for dx, dy, w in NBRS:
            n = (c[0] + dx, c[1] + dy)
            if not (0 <= n[0] < GW and 0 <= n[1] < GH) or blocked[n]:
                continue
            ng = gs[c] + w * (1 + cost[n])
            if ng < gs.get(n, 1e18):
                gs[n], came[n] = ng, c
                heapq.heappush(openq, (ng + math.hypot(g[0] - n[0], g[1] - n[1]), n))
    if g not in came:
        return None
    path, c = [], g
    while c is not None:
        path.append(c)
        c = came[c]
    return path[::-1]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------- perception
class Camera:
    def __init__(self, model, w=160, h=120):
        self.r = mujoco.Renderer(model, h, w)
        self.r.enable_depth_rendering()
        self.cam = model.camera("front").id
        self.fovy = model.cam_fovy[self.cam]
        f = 0.5 * h / math.tan(math.radians(self.fovy) / 2)
        u = (np.arange(w) - w / 2 + .5) / f
        v = -(np.arange(h) - h / 2 + .5) / f
        self.U, self.V = np.meshgrid(u, v)

    def sense(self, data):
        """Depth image -> labelled 3D points: floor / obstacle / drop (missing floor)."""
        self.r.update_scene(data, camera="front")
        d = self.r.render().copy()
        pos, R = data.cam_xpos[self.cam].copy(), data.cam_xmat[self.cam].reshape(3, 3)
        pc = np.stack([self.U * d, self.V * d, -d], -1)
        pw = pc @ R.T + pos
        valid = d < 25
        horiz = np.hypot(pw[..., 0] - pos[0], pw[..., 1] - pos[1])
        obst = valid & (pw[..., 2] > .12) & (pw[..., 2] < 2.2) & (horiz < 7)
        drop = valid & (pw[..., 2] < -.3)
        floor = valid & (abs(pw[..., 2]) <= .12)
        # where a "drop" ray crossed floor level = where floor should have been
        s = pos[2] / np.maximum(pos[2] - pw[..., 2], 1e-6)
        dropxy = pos[:2] + s[..., None] * (pw[..., :2] - pos[:2])
        label = np.zeros(d.shape, np.uint8)
        label[floor], label[obst], label[drop] = 1, 2, 3
        return dict(label=label, floor=pw[floor][:, :2], obst=pw[obst][:, :2], drop=dropxy[drop],
                    valid_frac=float(valid.mean()))


# ---------------------------------------------------------------- the agent
class Agent:
    V_MAX, W_MAX = 0.85, 1.6

    def __init__(self):
        self.prior = prior_map()
        self.seen_obst = np.zeros((GW, GH), bool)
        self.seen_drop = np.zeros((GW, GH), bool)
        self.path, self.state, self.reason = None, "STARTING", ""
        self.last_plan = -1
        self.events = []          # (t, text) spoken to the user
        self._said = {}
        self.cost_ms = 0.0
        self.prior_wide = cv2.dilate(self.prior, np.ones((3, 3), np.uint8)) > 0
        self.tracks, self.next_id = [], 0
        self.pred = np.zeros((GW, GH), bool)
        self.ghosts = []
        self.ring_min = 9.0
        self.mode = "LEAD"              # LEAD: it guides. SHADOW: you lead, it watches out for you.
        self.mode_since = 0.0
        self.calm_since = None
        self.memory = None              # what this route looked like last time
        self.journey = 1
        self.reported_new = np.zeros((GW, GH), bool)
        self.interventions = 0

    def ring(self, model, data, rx, ry, yaw, robot_body):
        """12 short-range distance sensors around the robot (front half used), like ultrasonic bumpers."""
        out = []
        geomid = np.zeros(1, np.int32)
        for a in np.linspace(-math.pi / 2, math.pi / 2, 9):
            d = np.array([math.cos(yaw + a), math.sin(yaw + a), 0.0])
            # returns that match the known map (walls, pillars) are discarded: only unmapped bodies count
            dist = mujoco.mj_ray(model, data, np.array([rx, ry, .5]), d, np.array([0, 1, 0, 0, 0, 0], np.uint8), 1, robot_body, geomid)
            out.append(dist if dist >= 0 else 9.0)
        self.ring_min = min(out)
        return self.ring_min

    def describe(self, rx, ry, yaw):
        """A short spoken picture of what's around, from what the robot has seen."""
        left = right = 0
        for tr in self.tracks:
            if not tr.get("person"):
                continue
            rel = tr["pos"] - np.array([rx, ry])
            if np.hypot(*rel) < 4.5:
                side = math.cos(yaw) * rel[1] - math.sin(yaw) * rel[0]
                left += side > 0
                right += side <= 0
        parts = []
        if left or right:
            parts.append(" and ".join(x for x in [f"{left} on your left" if left else "", f"{right} on your right" if right else ""] if x))
            parts[-1] = "People passing, " + parts[-1] + "."
        if self.seen_drop.any():
            parts.append("The open drop is behind us." if not self.drop_ahead(rx, ry, yaw) else "The drop is ahead; we're going around it.")
        d = math.hypot(GOAL[0] - rx, GOAL[1] - ry)
        parts.append(f"About {d:.0f} metres to the lifts.")
        return " ".join(parts)

    def drop_ahead(self, rx, ry, yaw):
        ix, iy = np.nonzero(self.seen_drop)
        if not len(ix):
            return False
        rel = np.stack([(ix + .5) * RES - rx, (iy + .5) * RES - ry], 1)
        return bool(np.any(rel @ np.array([math.cos(yaw), math.sin(yaw)]) > 0))

    def load_memory(self, path):
        if os.path.exists(path):
            d = np.load(path)
            self.memory = dict(obst=d["obst"], drop=d["drop"])
            self.journey = int(d["journeys"]) + 1
            self.seen_drop |= self.memory["drop"]          # known hazards are planned around from the start

    def save_memory(self, path):
        np.savez(path, obst=self.seen_obst, drop=self.seen_drop, journeys=self.journey)

    def notice_changes(self, t, rx, ry, yaw):
        """On a route walked before: say when something static is new since last time."""
        if self.memory is None:
            return
        known = cv2.dilate((self.memory["obst"] | self.prior.astype(bool)).astype(np.uint8), disk(.6)) > 0
        people = np.zeros((GW, GH), bool)
        for tr in self.tracks:
            if tr.get("person"):
                cx, cy = cell(*tr["pos"])
                people[max(0, cx - 3):cx + 4, max(0, cy - 3):cy + 4] = True
        steady = getattr(self, "obst_age", np.zeros((GW, GH))) > 40      # stayed put for 1.6 s
        new = steady & ~known & ~people & ~self.reported_new
        if new.sum() >= 3 and getattr(self, "_new_reports", 0) < 2:
            self._new_reports = getattr(self, "_new_reports", 0) + 1
            ix, iy = np.nonzero(new)
            x, y = (ix.mean() + .5) * RES, (iy.mean() + .5) * RES
            side = "left" if math.cos(yaw) * (y - ry) - math.sin(yaw) * (x - rx) > 0 else "right"
            d = math.hypot(x - rx, y - ry)
            self.say(t, f"Something new on this route since last time, {d:.0f} metres ahead on your {side}.", 1)
            self.reported_new |= cv2.dilate(new.astype(np.uint8), disk(1.0)) > 0

    def hazard_on_heading(self, rx, ry, heading, horizon=1.6):
        """Would walking this way run into a drop, an obstacle or a predicted person soon?"""
        blocked_drop = cv2.dilate(self.seen_drop.astype(np.uint8), disk(.6)) > 0
        blocked_obst = cv2.dilate(self.seen_obst.astype(np.uint8), disk(.35)) > 0
        for dist in np.arange(.3, horizon * self.V_MAX + .01, .2):
            c = cell(rx + dist * math.cos(heading), ry + dist * math.sin(heading))
            if blocked_drop[c]:
                return "drop"
            if blocked_obst[c]:
                return "obstacle"
            if self.pred[c]:
                return "person"
        return None

    def say(self, t, text, cooldown=5.0):
        urgent = text.startswith(("Stop", "Open drop", "We're at", "Handle", "Careful", "Wait", "You know", "I'll take", "Something new", "Same route", "Back to you"))
        if not urgent and not getattr(self, "verbose", True) and text in ("People crossing ahead. Adjusting our route.",):
            return
        if not urgent and self.events and t - self.events[-1][0] < 1.6:
            return
        if t - self._said.get(text, -99) > cooldown:
            self._said[text] = t
            self.events.append((t, text))

    def track_people(self, t, pts):
        """Group obstacle points into blobs, follow each blob over time, estimate its velocity."""
        excl = np.zeros((GW, GH), bool)
        if len(pts):
            ix = np.clip((pts[:, 0] / RES).astype(int), 0, GW - 1)
            iy = np.clip((pts[:, 1] / RES).astype(int), 0, GH - 1)
            keep = ~self.prior_wide[ix, iy]
            pts, ix, iy = pts[keep], ix[keep], iy[keep]
        dets = []
        if len(pts):
            g = np.zeros((GW, GH), np.uint8)
            g[ix, iy] = 1
            n, lab = cv2.connectedComponents(cv2.dilate(g, np.ones((3, 3), np.uint8)))
            comp = lab[ix, iy]
            for k in range(1, n):
                m = comp == k
                if m.sum() >= 4:
                    dets.append((pts[m].mean(0), m))
        used = set()
        for tr in self.tracks:
            best, bd = None, .8
            for j, (c, _) in enumerate(dets):
                d = float(np.hypot(*(c - tr["pos"])))
                if j not in used and d < bd:
                    best, bd = j, d
            if best is not None:
                used.add(best)
                c, m = dets[best]
                tr["hist"] = (tr["hist"] + [(t, c)])[-15:]
                tr["pos"], tr["seen"], tr["m"] = c, t, m
                if len(tr["hist"]) >= 6:
                    t0, c0 = tr["hist"][0]
                    tr["vel"] = .6 * tr["vel"] + .4 * (c - c0) / max(t - t0, 1e-3)
                    tr["person"] = tr.get("person", False) or np.hypot(*tr["vel"]) > .35
        for j, (c, m) in enumerate(dets):
            if j not in used:
                self.tracks.append(dict(id=self.next_id, pos=c, vel=np.zeros(2), hist=[(t, c)], seen=t, m=m))
                self.next_id += 1
        self.tracks = [tr for tr in self.tracks if t - tr["seen"] < .6]
        for tr in self.moving():
            if tr["seen"] == t:
                excl[ix[tr["m"]], iy[tr["m"]]] = True     # people aren't walls: don't paint them on the map
        return excl

    def erratic(self, tr):
        """0 = walks straight, 1 = keeps changing direction (from the last 0.6 s of headings)."""
        h = tr["hist"]
        if len(h) < 8:
            return .5
        steps = [h[k + 2][1] - h[k][1] for k in range(0, len(h) - 2, 2)]
        ang = [math.atan2(d[1], d[0]) for d in steps if np.hypot(*d) > .02]
        if len(ang) < 3:
            return .5
        turns = [abs(wrap(b - a)) for a, b in zip(ang, ang[1:])]
        return float(np.clip(np.mean(turns) / .5, 0, 1))

    def personal_space(self, rx, ry, yaw):
        """Distance to the nearest tracked person in front or to the side (anyone it has seen in the last 0.6 s)."""
        best = None
        for tr in self.tracks:
            if not tr.get("person"):
                continue
            d = math.hypot(tr["pos"][0] - rx, tr["pos"][1] - ry)
            ang = abs(wrap(math.atan2(tr["pos"][1] - ry, tr["pos"][0] - rx) - yaw))
            if ang < 1.4 and (best is None or d < best):
                best = d
        return best

    def moving(self):
        return [tr for tr in self.tracks if len(tr["hist"]) >= 6 and np.hypot(*tr["vel"]) > .35]

    def predict(self, rx, ry):
        """Where will each person be when the robot could get there? Only those places are avoided."""
        self.pred[:] = False
        self.ghosts = []
        for tr in self.moving():
            conflict = False
            tr["err"] = e = self.erratic(tr)
            for tk in np.arange(0, HORIZON + .01, .25):
                q = tr["pos"] + tr["vel"] * tk
                r = .3 + (.12 + .35 * e) * tk          # the further ahead, the less sure: the circle grows
                arrive = math.hypot(q[0] - rx, q[1] - ry) / self.V_MAX
                if tk == 0 or abs(arrive - tk) < .9:
                    cx, cy = cell(*q)
                    k = int(math.ceil(r / RES))
                    self.pred[max(0, cx - k):cx + k + 1, max(0, cy - k):cy + k + 1] = True
                    conflict = conflict or tk > 0
            self.ghosts.append((tr["pos"].copy(), tr["pos"] + tr["vel"] * HORIZON, conflict, .3 + (.12 + .35 * e) * HORIZON))

    def update_map(self, p, excl=None):
        def grid(xy):
            g = np.zeros((GW, GH), bool)
            if len(xy):
                g[np.clip((xy[:, 0] / RES).astype(int), 0, GW - 1), np.clip((xy[:, 1] / RES).astype(int), 0, GH - 1)] = True
            return g
        om, fm, dm = grid(p["obst"]), grid(p["floor"]), grid(p["drop"])
        if excl is not None:
            om &= ~excl
        newm = om & ~self.prior.astype(bool) & ~self.seen_obst
        young = [tr["pos"] for tr in self.tracks if len(tr["hist"]) < 6]
        if young and newm.any():            # new blobs right where a person was just spotted: not a fixed obstacle
            cx, cy = np.nonzero(newm)
            xy = np.stack([(cx + .5) * RES, (cy + .5) * RES], 1)
            near_person = np.min(np.hypot(*(xy[:, None, :] - np.array(young)[None]).transpose(2, 0, 1)), 1) < 1.0
            newm[cx[near_person], cy[near_person]] = False
        new_obst = int(newm.sum())
        new_drop = int((dm & ~self.seen_drop).sum())
        self.seen_obst[fm & ~om] = False
        self.seen_obst |= om
        if not hasattr(self, "obst_age"):
            self.obst_age = np.zeros((GW, GH), np.int32)
        self.obst_age = np.where(self.seen_obst, self.obst_age + 1, 0)
        self.seen_drop |= dm          # never forgotten: safety
        return new_obst, new_drop

    def costmap(self, with_pred=True):
        hard = (self.prior | self.seen_obst).astype(np.uint8)
        blocked = cv2.dilate(hard, disk(.5)) > 0
        if with_pred:
            blocked |= cv2.dilate(self.pred.astype(np.uint8), disk(.45)) > 0
        blocked |= cv2.dilate(self.seen_drop.astype(np.uint8), disk(.85)) > 0
        free = (~blocked).astype(np.uint8)
        dist = cv2.distanceTransform(free, cv2.DIST_L2, 3) * RES
        cost = 4.0 * np.exp(-dist / .5)
        return blocked, cost

    def plan(self, rx, ry):
        s = cell(rx, ry)
        p = None
        for with_pred in (True, False):      # if people box us in, plan without them and go slowly
            blocked, cost = self.costmap(with_pred)
            b = blocked.copy()
            b[max(0, s[0] - 2):s[0] + 3, max(0, s[1] - 2):s[1] + 3] = False
            if self.path:
                near = np.zeros((GW, GH), np.uint8)
                for x, y in self.path:
                    near[cell(x, y)] = 1
                near = cv2.dilate(near, np.ones((3, 3), np.uint8)) > 0
                cost = cost.copy()
                cost[~near] += .6          # changing sides costs extra, so it commits like a confident walker
            p = astar(b, cost, s, cell(*GOAL))
            if p:
                break
        return [centre(c) for c in p] if p else None, blocked

    def side_of(self, path, rx, ry, yaw):
        for x, y in path:
            if math.hypot(x - rx, y - ry) > 2.2:
                cross = math.cos(yaw) * (y - ry) - math.sin(yaw) * (x - rx)
                return "left" if cross > 0 else "right"
        return "ahead"

    def step(self, t, rx, ry, yaw, p, grip, tension=0.0, user_heading=None):
        t0 = time.perf_counter()
        excl = self.track_people(t, p["obst"])
        new_obst, new_drop = self.update_map(p, excl)
        self.predict(rx, ry)
        self.notice_changes(t, rx, ry, yaw)
        period = .25 if self.ghosts else .5
        replan = self.path is None or new_obst > 2 or new_drop > 0 or t - self.last_plan > period
        if replan:
            blocked_now, _ = self.costmap()
            old = self.path
            route_cut = old is not None and any(blocked_now[cell(x, y)] for x, y in old[2:])
            path, _ = self.plan(rx, ry)
            self.last_plan = t
            self.path = path
            if path is not None:
                side = self.side_of(path, rx, ry, yaw)
                if new_drop and self.memory is not None:
                    pass
                elif new_drop and not getattr(self, "_drop_announced", False):
                    self._drop_announced = True
                    d = min(math.hypot(x - rx, y - ry) for x, y in p["drop"]) if len(p["drop"]) else 0
                    self.say(t, f"Open drop {d:.0f} metres ahead. Going around it on the {side}.")
                elif route_cut and new_obst > 2:
                    self.say(t, f"Something's blocking our route. Going around on the {side}.")
                elif route_cut and any(g[2] for g in self.ghosts):
                    self.say(t, "People crossing ahead. Adjusting our route.", 8)

        # robot-frame view of what it sees right now (for the safety layer)
        c, s_ = math.cos(-yaw), math.sin(-yaw)

        def ahead(pts, xmax, half):
            if len(pts) == 0:
                return False
            rel = pts - np.array([rx, ry])
            fx = rel[:, 0] * c - rel[:, 1] * s_
            fy = rel[:, 0] * s_ + rel[:, 1] * c
            return bool(np.any((fx > 0) & (fx < xmax) & (abs(fy) < half)))

        v = w = 0.0
        dist_goal = math.hypot(GOAL[0] - rx, GOAL[1] - ry)
        if dist_goal < .45:
            self.state, self.reason = "ARRIVED", "At the lifts"
            self.say(t, "We're at the lifts. The call button is on your right.", 99)
        elif not grip:
            self.state, self.reason = "STOPPED", "Handle released: I never move without you"
            self.say(t, "Handle released. Stopping.")
        elif p["valid_frac"] < .5:
            self.state, self.reason = "UNSURE", "Camera can't see: stopping until it can"
            self.say(t, "I can't see properly. Stopping.")
        elif self.path is None:
            self.state, self.reason = "UNSURE", "No safe route: stopping and asking for help"
            self.say(t, "I can't find a safe way through. Let's wait for help.")
        else:
            # pure pursuit along the planned path
            pts = np.array(self.path)
            i = int(np.argmin(np.hypot(pts[:, 0] - rx, pts[:, 1] - ry)))
            tgt = pts[-1]
            for q in pts[i:]:
                if math.hypot(q[0] - rx, q[1] - ry) > .9:
                    tgt = q
                    break
            err = wrap(math.atan2(tgt[1] - ry, tgt[0] - rx) - yaw)
            w = float(np.clip(2.2 * err, -self.W_MAX, self.W_MAX))
            v = 0.0 if abs(err) > .8 else self.V_MAX * math.cos(err)
            self.state, self.reason = "GUIDING", "Following the safest route"
            if dist_goal < 2.0:
                v = min(v, .45)
            # yield: will someone be where we'll be in the next 2 seconds?
            arc = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(pts[i:], axis=0).T))]) if len(pts) - i > 1 else np.zeros(1)
            for tk in (.5, 1.0, 1.5, 2.0):
                k = min(int(np.searchsorted(arc, self.V_MAX * tk)), len(arc) - 1)
                me = pts[i + k]
                close = [tr for tr in self.moving() if math.hypot(*(tr["pos"] + tr["vel"] * tk - me)) < .85 + .5 * tr.get("err", .5) * tk]
                if close:
                    if max(tr.get("err", 0) for tr in close) > .6:
                        self.say(t, "Someone ahead keeps changing direction. Slowing down.", 8)
                    if tk <= 1.0:
                        v, self.state, self.reason = 0.0, "YIELDING", "Someone will cross here: letting them pass"
                    else:
                        v = min(v, .35)
                        self.state, self.reason = "YIELDING", "Slowing: someone is about to cross"
                    self.say(t, "Letting someone pass.", 6)
                    break
            near = self.personal_space(rx, ry, yaw)
            if self.ring_min < 1.2:
                near = min(near if near is not None else 9.0, self.ring_min + .25)
            if near is not None and near < 1.3:      # personal space: slow down as people get closer
                v = min(v, max(0.0, .9 * (near - .6)))
            # independent safety layer: overrides the planner
            if ahead(p["drop"], 1.4, .7):
                v, self.state, self.reason = 0.0, "SAFETY STOP", "Missing floor ahead: never step forward"
                self.say(t, "Stop. Drop ahead.")
            elif self.ring_min < .45 or (self.personal_space(rx, ry, yaw) is not None and self.personal_space(rx, ry, yaw) < .65):
                v, self.state, self.reason = 0.0, "WAITING", "Someone is very close: giving them space"
                self.say(t, "Someone's right next to us. Waiting.")
                self._blocked_since = getattr(self, "_blocked_since", t)
            elif ahead(p["obst"], .95, .42):
                v, self.state, self.reason = 0.0, "WAITING", "Something directly ahead: waiting"
                self.say(t, "Something in the way. Waiting.")
                self._blocked_since = getattr(self, "_blocked_since", t)
            else:
                if self.state in ("GUIDING", "YIELDING") and hasattr(self, "_blocked_since"):
                    if t - self._blocked_since > .8:
                        self.say(t, "Clear. Moving on.")
                    del self._blocked_since
        # --- lead / shadow: hand control back when the route is familiar and you're calm
        if self.state in ("GUIDING", "YIELDING", "WAITING"):
            self.calm_since = (self.calm_since if self.calm_since is not None else t) if tension < .35 else None
            familiar = self.journey >= 2
            safe_to_hand_back = user_heading is None or self.hazard_on_heading(rx, ry, user_heading, 2.2) is None
            if self.mode == "LEAD" and familiar and safe_to_hand_back and self.calm_since is not None and t - self.calm_since > 2 and t - self.mode_since > 3:
                self.mode, self.mode_since = "SHADOW", t
                first = not getattr(self, "_shadowed", False)
                self._shadowed = True
                self.say(t, "You know this way. You lead, and I'll watch out for you." if first else "Back to you.", 1)
            elif self.mode == "SHADOW" and tension > .6:
                self.mode, self.mode_since = "LEAD", t
                self.say(t, "I'll take over for a bit.", 1)
        if self.mode == "SHADOW" and self.state in ("GUIDING", "YIELDING") and user_heading is not None:
            hz = self.hazard_on_heading(rx, ry, user_heading)
            if hz is None:
                # you steer: follow your direction
                err = wrap(user_heading - yaw)
                w = float(np.clip(2.2 * err, -self.W_MAX, self.W_MAX))
                v = 0.0 if abs(err) > .9 else self.V_MAX * .9 * math.cos(err)
                self.state, self.reason = "SHADOWING", "You're leading; I only step in for danger"
            else:
                # step in: take over with the planner's safe route and say why
                self.interventions += 1
                side = self.side_of(self.path, rx, ry, yaw) if self.path else "ahead"
                msg = {"drop": f"Stop. There's a drop that way. Bear {side}.",
                       "obstacle": f"Careful, something in the way. Bear {side}.",
                       "person": "Wait, someone's crossing."}[hz]
                self.say(t, msg, 3)
                self.mode, self.mode_since, self.calm_since = "LEAD", t, None
                self.state, self.reason = "STEPPING IN", f"{hz} on your line: taking over"

        # --- adapt to the person, not just the room
        self.tension = tension
        if self.state in ("GUIDING", "YIELDING"):
            if tension > .6:
                v *= .75                                  # tense: slow down
                if self.state == "GUIDING":
                    self.reason = "You seem tense: slower, describing more"
                if t - getattr(self, "_last_describe", -99) > 7:
                    self._last_describe = t
                    self.say(t, self.describe(rx, ry, yaw), 1)
            elif tension < .3:
                self._calm = getattr(self, "_calm", 0) + DT  # calm for a while: talk less
        self.verbose = tension > .45
        self.cost_ms = .9 * self.cost_ms + .1 * (time.perf_counter() - t0) * 1000
        return v, w


# ---------------------------------------------------------------- the crowd (simulated people)
def seg_clear(a, b):
    """True if a walk from a to b avoids the hatch and the pillars."""
    for k in np.linspace(0, 1, 40):
        x, y = a + (b - a) * k
        if HATCH[0] - .7 < x < HATCH[1] + .7 and HATCH[2] - .7 < y < HATCH[3] + .7:
            return False
        if any(math.hypot(x - px, y - py) < .75 for px, py in PILLARS):
            return False
    return True


class Crowd:
    """People walking between platforms. They don't know about the robot except
    to sidestep it when very close, like real commuters."""

    def __init__(self, rng, n_active=N_PEOPLE):
        self.rng = rng
        self.p = [self.new_walker(mid_route=True) for _ in range(N_PEOPLE)]
        for w in self.p[n_active:]:
            w["wait"] = 1e9                          # inactive walkers stay out of the scene

    def new_walker(self, mid_route=False):
        for _ in range(200):
            a, b = self.rng.choice(len(GATES), 2, replace=False)
            A = np.array(GATES[a], float); B = np.array(GATES[b], float)
            A[1] = .5 if A[1] < 5 else 9.5
            B[1] = .5 if B[1] < 5 else 9.5
            if A[1] == B[1] or not seg_clear(A, B):
                continue
            pos = A + (B - A) * (self.rng.uniform(.05, .85) if mid_route else 0)
            if math.hypot(pos[0] - START[0], pos[1] - START[1]) < 2.5:
                continue
            return dict(pos=pos, goal=B, speed=self.rng.uniform(.9, 1.35), wait=0.0 if mid_route else self.rng.uniform(0, 2.5),
                        erratic=self.rng.random() < .35, swerve=0.0, swerve_t=self.rng.uniform(.8, 2.5))
        return dict(pos=np.array([18., .5]), goal=np.array([18., 9.5]), speed=1.0, wait=5.0)

    def step(self, robot, user):
        for i, w in enumerate(self.p):
            if w["wait"] > 0:
                w["wait"] -= DT
                continue
            to_goal = w["goal"] - w["pos"]
            d = np.linalg.norm(to_goal)
            if d < .3:
                self.p[i] = self.new_walker()
                continue
            v = to_goal / d * w["speed"]
            if w.get("erratic"):                   # hesitates, drifts, changes its mind
                w["swerve_t"] -= DT
                if w["swerve_t"] <= 0:
                    w["swerve"] = self.rng.choice([-1, 1]) * self.rng.uniform(.5, 1.1) if w["swerve"] == 0 else 0.0
                    w["swerve_t"] = self.rng.uniform(.7, 1.8)
                c_, s_ = math.cos(w["swerve"]), math.sin(w["swerve"])
                v = np.array([c_ * v[0] - s_ * v[1], s_ * v[0] + c_ * v[1]]) * (.75 if w["swerve"] else 1)
            for other in (robot, user):           # sidestep only at the last moment
                off = w["pos"] - np.array(other)
                dist = np.linalg.norm(off)
                if dist < 1.1:
                    v += off / max(dist, .1) * (1.1 - dist) * 2.0
            w["pos"] = w["pos"] + v * DT
            w["vel"] = v

    def poses(self):
        out = []
        for w in self.p:
            if w["wait"] > 0:
                out.append(None)
            else:
                v = w.get("vel", w["goal"] - w["pos"])
                out.append((w["pos"][0], w["pos"][1], math.atan2(v[1], v[0])))
        return out


# ---------------------------------------------------------------- the user's hand on the handle
class UserTension:
    """Stand-in for the forearm signal from the handle when no EMG is connected.
    0 = relaxed grip, 1 = very tense. Rises with close passes, sudden stops and edges, settles when calm."""

    def __init__(self, rng):
        self.v, self.rng, self.prev_speed = .15, rng, 0.0

    def step(self, people_dist, drop_dist, speed, familiarity=0):
        target = .15
        k = .6 if familiarity else 1.0                 # on a route you know, things rattle you less
        if people_dist < 1.3:
            target = max(target, k * (.9 - .45 * (people_dist - .5)))
        if drop_dist < 1.4:
            target = max(target, k * .75)
        if self.prev_speed - speed > .3:          # a sudden stop startles
            target = max(target, .85)
        self.prev_speed = speed
        rate = .25 if target > self.v else .04    # tenses fast, relaxes slowly
        self.v += rate * (target - self.v) + self.rng.normal(0, .01)
        self.v = float(np.clip(self.v, 0, 1))
        return self.v


# ---------------------------------------------------------------- drawing
def draw_map(agent, rx, ry, yaw, ux, uy, size=(320, 200)):
    s = 4  # px per cell
    img = np.full((GH * s, GW * s, 3), 235, np.uint8)
    blocked, _ = agent.costmap()
    img[np.repeat(np.repeat(blocked.T, s, 0), s, 1)] = (210, 210, 220)
    img[np.repeat(np.repeat(agent.prior.T.astype(bool), s, 0), s, 1)] = (70, 70, 80)
    img[np.repeat(np.repeat(agent.seen_obst.T, s, 0), s, 1)] = (60, 60, 220)
    img[np.repeat(np.repeat(agent.seen_drop.T, s, 0), s, 1)] = (0, 140, 255)
    to_px = lambda x, y: (int(x / RES * s), int(y / RES * s))
    if agent.path:
        cv2.polylines(img, [np.array([to_px(*q) for q in agent.path], np.int32)], False, (200, 160, 0), 2, cv2.LINE_AA)
    cv2.circle(img, to_px(*GOAL), 7, (90, 190, 60), 2, cv2.LINE_AA)
    for now, later, conflict, rad in agent.ghosts:
        cv2.circle(img, to_px(*later), max(2, int(rad / RES * s)), (200, 150, 210), 1, cv2.LINE_AA)
        col = (60, 60, 230) if conflict else (190, 90, 160)
        cv2.arrowedLine(img, to_px(*now), to_px(*later), col, 2, cv2.LINE_AA, tipLength=.25)
        cv2.circle(img, to_px(*now), 5, col, -1, cv2.LINE_AA)
    cv2.circle(img, to_px(ux, uy), 4, (170, 160, 50), -1, cv2.LINE_AA)
    tri = np.array([[.45, 0], [-.3, .28], [-.3, -.28]])
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    cv2.fillPoly(img, [np.array([to_px(*(np.array([rx, ry]) + R @ q)) for q in tri], np.int32)], (20, 190, 255), cv2.LINE_AA)
    img = cv2.flip(img, 0)  # y up
    return cv2.resize(img, size, interpolation=cv2.INTER_AREA)


def overlay_cam(rgb, label):
    out = rgb.astype(np.float32)
    lab = cv2.resize(label, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
    for k, col, a in [(1, (60, 200, 90), .12), (2, (230, 50, 50), .5), (3, (255, 150, 0), .65)]:
        m = lab == k
        out[m] = out[m] * (1 - a) + np.array(col) * a
    return out.astype(np.uint8)


def text(img, s, xy, scale=.5, col=(240, 240, 240), th=1):
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), th + 2, cv2.LINE_AA)
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, col, th, cv2.LINE_AA)


STATE_COL = {"GUIDING": (90, 200, 90), "SAFETY STOP": (40, 120, 255), "WAITING": (0, 200, 255), "YIELDING": (255, 170, 120), "SHADOWING": (230, 200, 90), "STEPPING IN": (40, 120, 255),
             "UNSURE": (220, 120, 255), "STOPPED": (200, 200, 200), "ARRIVED": (90, 200, 90), "STARTING": (200, 200, 200)}


# ---------------------------------------------------------------- run
def yaw_quat(y):
    return [math.cos(y / 2), 0, 0, math.sin(y / 2)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--show", action="store_true", help="live window in real time")
    ap.add_argument("--emg", action="store_true", help="handle grip from the EMG bridge (OSC /emg/effort, port 9001)")
    ap.add_argument("--grip", type=float, default=.15, help="EMG effort that counts as holding the handle")
    ap.add_argument("--no-audio", action="store_true")
    ap.add_argument("--headless", action="store_true", help="no video, just print safety metrics (for batch evaluation)")
    ap.add_argument("--memory", default="routes/concourse-to-lifts.npz", help="route memory file ('' to disable)")
    ap.add_argument("--fresh", action="store_true", help="forget this route first (journey 1)")
    ap.add_argument("--people", type=int, default=7, help="how many people walking through")
    ap.add_argument("--horizon", type=float, default=None, help="how far ahead (s) to predict people")
    ap.add_argument("--speed", type=float, default=.85, help="the user's preferred walking speed (m/s), 0.5 to 1.2")
    ap.add_argument("--tmax", type=float, default=70)
    args = ap.parse_args()
    seed = args.seed if args.seed is not None else int(time.time()) % 10000
    rng = np.random.default_rng(seed)

    model = mujoco.MjModel.from_xml_string(build_xml())
    data = mujoco.MjData(model)
    mid = {n: model.body_mocapid[model.body(n).id] for n in ["robot", "user", "sign"] + [f"p{i}" for i in range(N_PEOPLE)] + [f"g{i}" for i in range(N_PEOPLE)]}
    cam = Camera(model)
    main_r = mujoco.Renderer(model, 720, 960)
    small_r = mujoco.Renderer(model, 240, 320)
    vcam = mujoco.MjvCamera()
    vcam.type = mujoco.mjtCamera.mjCAMERA_FREE
    vcam.distance, vcam.elevation = 7.8, -46

    effort = {"v": 1.0, "t": time.time()}
    if args.emg:
        from pythonosc.dispatcher import Dispatcher
        from pythonosc.osc_server import ThreadingOSCUDPServer
        disp = Dispatcher()
        disp.map("/emg/effort", lambda a, v: effort.update(v=float(v), t=time.time()))
        srv = ThreadingOSCUDPServer(("127.0.0.1", 9001), disp)
        threading.Thread(target=srv.serve_forever, daemon=True).start()

    agent = Agent()
    agent.V_MAX = float(np.clip(args.speed, .5, 1.2))
    if args.memory:
        os.makedirs(os.path.dirname(args.memory) or ".", exist_ok=True)
        if args.fresh and os.path.exists(args.memory):
            os.remove(args.memory)
        agent.load_memory(args.memory)
    # the person's own sense of the route: the direct way they remember, without new hazards
    user_route = astar(cv2.dilate(prior_map(), disk(.45)) > 0, np.zeros((GW, GH)), cell(*START), cell(*GOAL))
    user_route = [centre(c) for c in user_route]
    rx, ry, yaw = START[0], START[1], 0.0
    trail = [(rx - 1.1, ry), (rx, ry)]
    sign_at = rng.uniform(5.3, 6.3)          # robot x when a sign is put down on its route
    crowd = Crowd(rng, min(args.people, N_PEOPLE))
    if args.horizon:
        global HORIZON
        HORIZON = args.horizon
    user_t = UserTension(rng)
    sign_on = False
    if agent.journey >= 2:
        agent.say(0.0, f"Same route as last time, journey {agent.journey}. The open hatch is still about {max(0, HATCH[0] - START[0]):.0f} metres ahead; I'll keep you clear of it.")
    else:
        agent.say(0.0, "Starting. Heading to the lifts, about 17 metres.")

    os.makedirs("runs", exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    silent = f"runs/wayfinder-{stamp}-seed{seed}-silent.mp4"
    vw = None if args.headless else cv2.VideoWriter(silent, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (1280, 720))
    m = dict(min_person=99.0, min_drop=99.0, contacts=0, stops=0)
    def rect_dist(x, y, r):
        dx = max(r[0] - x, 0, x - r[1]); dy = max(r[2] - y, 0, y - r[3])
        return math.hypot(dx, dy)
    was_stopped = False
    arrived_t = None
    t = 0.0
    while t < args.tmax:
        tick = time.time()
        # --- unscripted events (random each run)
        if not sign_on and rx > sign_at and agent.path:
            pts = np.array(agent.path)
            k = int(np.argmin(np.hypot(pts[:, 0] - rx, pts[:, 1] - ry)))
            for q in pts[k:]:
                if math.hypot(q[0] - rx, q[1] - ry) > 2.4:
                    sx, sy = q[0], q[1] + rng.uniform(-.25, .25)
                    break
            data.mocap_pos[mid["sign"]] = [sx, sy, 0]
            data.mocap_quat[mid["sign"]] = yaw_quat(rng.uniform(0, math.pi))
            sign_on = True
        # --- robot + user pose
        data.mocap_pos[mid["robot"]] = [rx, ry, 0]
        data.mocap_quat[mid["robot"]] = yaw_quat(yaw)
        d_acc, ux, uy = 0.0, *trail[0]
        for a, b in zip(reversed(trail[:-1]), reversed(trail[1:])):
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            if d_acc + seg >= 1.1:
                f = (1.1 - d_acc) / max(seg, 1e-6)
                ux, uy = b[0] + (a[0] - b[0]) * f, b[1] + (a[1] - b[1]) * f
                break
            d_acc += seg
        uyaw = math.atan2(ry - uy, rx - ux)
        data.mocap_pos[mid["user"]] = [ux, uy, 0]
        data.mocap_quat[mid["user"]] = yaw_quat(uyaw)
        crowd.step((rx, ry), (ux, uy))
        for i, pose in enumerate(crowd.poses()):
            if pose is None:
                data.mocap_pos[mid[f"p{i}"]] = [0, 0, -20]
            else:
                data.mocap_pos[mid[f"p{i}"]] = [pose[0], pose[1], 0]
                data.mocap_quat[mid[f"p{i}"]] = yaw_quat(pose[2])
        for i in range(N_PEOPLE):
            if i < len(agent.ghosts):
                gx, gy = agent.ghosts[i][1]
                data.mocap_pos[mid[f"g{i}"]] = [gx, gy, .006]
                model.geom_size[model.body(f"g{i}").geomadr[0]][0] = min(agent.ghosts[i][3], .9)
            else:
                data.mocap_pos[mid[f"g{i}"]] = [0, 0, -20]
        mujoco.mj_forward(model, data)

        # --- sense, decide, act
        p = cam.sense(data)
        grip = (effort["v"] > args.grip and time.time() - effort["t"] < 1.0) if args.emg else True
        near_p = min([math.hypot(q[0] - ux, q[1] - uy) for q in crowd.poses() if q] or [9])
        hatch_d = math.hypot(max(HATCH[0] - ux, 0, ux - HATCH[1]), max(HATCH[2] - uy, 0, uy - HATCH[3]))
        tension = min(1.0, max(0.0, (effort["v"] - args.grip) / 1.0)) if args.emg else user_t.step(near_p, hatch_d, getattr(agent, "_last_v", 0), agent.journey >= 2)
        agent.ring(model, data, rx, ry, yaw, model.body("robot").id)
        up = np.array(user_route)
        k_ = int(np.argmin(np.hypot(up[:, 0] - rx, up[:, 1] - ry)))
        tgt_ = up[min(k_ + 4, len(up) - 1)]
        user_heading = math.atan2(tgt_[1] - ry, tgt_[0] - rx) + rng.normal(0, .05)
        v, w = agent.step(t, rx, ry, yaw, p, grip, tension, user_heading)
        agent._last_v = v
        for pose in crowd.poses():
            if pose is not None:
                d_ = math.hypot(pose[0] - rx, pose[1] - ry)
                m["min_person"] = min(m["min_person"], d_)
                if d_ < .5 and not getattr(agent, "_in_contact", False):
                    m["contacts"] += 1
                    ang = abs(wrap(math.atan2(pose[1] - ry, pose[0] - rx) - yaw))
                    m.setdefault("robot_fault", 0)
                    m["robot_fault"] += int(v > .05 and ang < 1.0)
                    if os.environ.get("DBG") and v > .05 and ang < 1.0:
                        print(f"DBG t={t:.2f} d={d_:.2f} ang={ang:.2f} v={v:.2f} state={agent.state} ring={agent.ring_min:.2f} ps={agent.personal_space(rx, ry, yaw)} ped=({pose[0]:.1f},{pose[1]:.1f}) robot=({rx:.1f},{ry:.1f})")
                agent._in_contact = d_ < .5
        m["min_drop"] = min(m["min_drop"], rect_dist(rx, ry, HATCH))
        stopped = v == 0 and agent.state not in ("ARRIVED",)
        m["stops"] += int(stopped and not was_stopped)
        was_stopped = stopped
        yaw = wrap(yaw + w * DT)
        rx += v * math.cos(yaw) * DT
        ry += v * math.sin(yaw) * DT
        if math.hypot(rx - trail[-1][0], ry - trail[-1][1]) > .05:
            trail.append((rx, ry))
            trail = trail[-200:]

        # --- frame
        if args.headless:
            if agent.state == "ARRIVED":
                break
            t += DT
            continue
        vcam.lookat[:] = [rx, ry, .5]
        vcam.azimuth = math.degrees(yaw) + 32
        main_r.update_scene(data, camera=vcam)
        frame = np.zeros((720, 1280, 3), np.uint8)
        frame[:, :960] = cv2.cvtColor(main_r.render(), cv2.COLOR_RGB2BGR)
        small_r.update_scene(data, camera="front")
        frame[:240, 960:] = cv2.cvtColor(overlay_cam(small_r.render(), p["label"]), cv2.COLOR_RGB2BGR)
        frame[240:440, 960:] = draw_map(agent, rx, ry, yaw, ux, uy)
        panel = frame[440:, 960:]
        panel[:] = (28, 28, 32)
        text(frame, "ROBOT CAMERA: what it perceives", (968, 18), .42)
        text(frame, "floor  obstacle  drop", (968, 232), .38, (230, 230, 230))
        text(frame, "MAP: known / learned / planned route", (968, 258), .42)
        sc = STATE_COL.get(agent.state, (240, 240, 240))
        text(frame, agent.state, (972, 478), .8, sc, 2)
        text(frame, agent.reason[:38], (972, 506), .42)
        mode_txt = "LEAD: it guides" if agent.mode == "LEAD" else "SHADOW: you lead"
        text(frame, f"{mode_txt}  |  journey {agent.journey}", (972, 456), .4, (230, 200, 90) if agent.mode == "SHADOW" else (200, 200, 200))
        text(frame, f"speed {v:.2f} m/s   grip {'held' if grip else 'RELEASED'}", (972, 536), .42)
        bar_w = int(150 * tension)
        cv2.rectangle(frame, (1110, 690), (1260, 704), (70, 70, 80), -1)
        cv2.rectangle(frame, (1110, 690), (1110 + bar_w, 704), (90, 200, 90) if tension < .45 else (0, 170, 255) if tension < .6 else (60, 60, 230), -1)
        text(frame, "user tension (handle)", (972, 702), .42)
        text(frame, f"decide: {agent.cost_ms:.1f} ms/cycle on CPU", (972, 562), .42)
        text(frame, f"tracking {len(agent.moving())} moving people, {HORIZON:.0f} s ahead", (972, 586), .42, (180, 180, 180))
        text(frame, f"run seed {seed}  t={t:4.1f}s", (972, 612), .42, (180, 180, 180))
        text(frame, "crowd + events random each run, all on-device", (972, 636), .42, (180, 180, 180))
        recent = [e for e in agent.events if t - e[0] < 3.5]
        if recent:
            msg = '"' + recent[-1][1] + '"'
            (tw, th), _ = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, .8, 2)
            cv2.rectangle(frame, (480 - tw // 2 - 14, 650), (480 + tw // 2 + 14, 694), (20, 20, 24), -1)
            text(frame, msg, (480 - tw // 2, 682), .8, (255, 255, 255), 2)
        text(frame, "WAYFINDER  |  robot guide, simulated", (16, 32), .7, (255, 255, 255), 2)
        vw.write(frame)
        if args.show:
            cv2.imshow("Wayfinder", frame)
            if cv2.waitKey(max(1, int((DT - (time.time() - tick)) * 1000))) & 0xFF == 27:
                break
        if agent.state == "ARRIVED":
            arrived_t = arrived_t if arrived_t is not None else t
            if t - arrived_t > 3.0:
                break
        t += DT
    if args.memory and agent.state == "ARRIVED":
        agent.save_memory(args.memory)
    if args.headless:
        import json
        print(json.dumps(dict(seed=seed, arrived=agent.state == "ARRIVED", time=round(t, 1),
                              min_person=round(m["min_person"], 2), min_drop=round(m["min_drop"], 2),
                              contacts=m["contacts"], robot_fault=m.get("robot_fault", 0), stops=m["stops"],
                              journey=agent.journey, interventions=agent.interventions)))
        return
    vw.release()
    if args.show:
        cv2.destroyAllWindows()

    out = silent.replace("-silent", "")
    if args.no_audio or not agent.events:
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", silent, "-c:v", "libx264", "-pix_fmt", "yuv420p", out])
    else:
        clips, filt, inputs = [], [], ["-i", silent]
        for k, (et, msg) in enumerate(agent.events):
            f = f"runs/_say{k}.aiff"
            subprocess.run(["say", "-v", "Samantha", "-r", "185", "-o", f, msg])
            clips.append(f)
            inputs += ["-i", f]
            ms = int(et * 1000)
            filt.append(f"[{k + 1}:a]adelay={ms}|{ms}[a{k}]")
        mix = "".join(f"[a{k}]" for k in range(len(clips)))
        filt.append(f"{mix}amix=inputs={len(clips)}:normalize=0[a]")
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", *inputs, "-filter_complex", ";".join(filt),
                        "-map", "0:v", "-map", "[a]", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-c:a", "aac",
                        "-shortest", out])
        for f in clips:
            os.remove(f)
    os.remove(silent)
    print(f"seed {seed}  duration {t:.1f}s  arrived={agent.state == 'ARRIVED'}  decide={agent.cost_ms:.1f} ms")
    for et, msg in agent.events:
        print(f"  {et:5.1f}s  {msg}")
    print(out)


if __name__ == "__main__":
    main()
