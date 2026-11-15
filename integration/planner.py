"""
Camera-only local path planner for FireBot.

Pipeline (every frame):
  1. floor_mask()   : which image pixels are floor (colour model of the floor
                      patch right in front of the robot, Lab colour space)
  2. homography     : image pixels -> floor (x, y) in cm, robot frame
                      x = right of robot, y = forward from front bumper
  3. grid           : top-down occupancy grid (default 4 cm cells, 2 m x 2 m)
                      free / obstacle / unknown (outside camera view)
  4. inflate        : grow obstacles by half robot width (configuration space)
                      + soft safety margin (costly but allowed)
  5. A*             : 8-connected, octile heuristic, robot -> fire
                      (if fire unreachable: go to the reachable cell closest to it)
  6. pure pursuit   : pick path point ~35 cm ahead -> heading angle for robot

Robot never needs to know where it is in the room: map is rebuilt every frame
relative to the robot (receding-horizon local planning).

Only depends on numpy + opencv, so it can be tested without YOLO.
"""

import heapq
import json
import math

import cv2
import numpy as np

SQRT2 = math.sqrt(2.0)


def load_calibration(path):
    with open(path) as f:
        data = json.load(f)
    return np.array(data["H"], dtype=np.float64), data


class FloorPlanner:
    def __init__(self, H_img2cm,
                 x_range=(-100, 100), y_range=(0, 200), cell_cm=4,
                 robot_half_width_cm=12, safety_cm=10, lookahead_cm=35,
                 floor_thresh=20.0, unknown_cost=3.0, near_cost=4.0,
                 goal_clear_cm=20, adaptive_floor=True):
        self.H = np.asarray(H_img2cm, dtype=np.float64)
        self.cell = cell_cm
        self.x0, self.x1 = x_range
        self.y0, self.y1 = y_range
        self.W = int(round((self.x1 - self.x0) / cell_cm))
        self.Hg = int(round((self.y1 - self.y0) / cell_cm))
        # ground cm -> grid pixel (col, row); row 0 = far edge, last row = robot
        G = np.array([[1.0 / cell_cm, 0, -self.x0 / cell_cm],
                      [0, -1.0 / cell_cm, self.y1 / cell_cm],
                      [0, 0, 1]])
        self.M = G @ self.H          # image px -> grid px
        self.r_hard = int(math.ceil(robot_half_width_cm / cell_cm))
        self.r_soft = self.r_hard + int(math.ceil(safety_cm / cell_cm))
        self.lookahead = lookahead_cm
        self.floor_thresh = floor_thresh
        self.unknown_cost = unknown_cost
        self.near_cost = near_cost
        self.goal_clear = int(math.ceil(goal_clear_cm / cell_cm))
        self.adaptive_floor = adaptive_floor
        self.floor_ref = None
        self.start = (self.Hg - 1, self.W // 2)
        self._vis_cache = {}

    # ------------------------------------------------------------ geometry
    def img_to_cm(self, u, v):
        p = self.H @ np.array([u, v, 1.0])
        return p[0] / p[2], p[1] / p[2]

    def cm_to_cell(self, x, y):
        c = int((x - self.x0) / self.cell)
        r = int((self.y1 - y) / self.cell)
        return r, c

    def cell_to_cm(self, r, c):
        return self.x0 + (c + 0.5) * self.cell, self.y1 - (r + 0.5) * self.cell

    # ------------------------------------------------------------ 1. floor
    def sample_floor(self, frame):
        """Force-learn floor colour from bottom-centre patch (press 'f')."""
        self.floor_ref = self._patch_ref(self._lab(frame))

    @staticmethod
    def _lab(frame):
        return cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0),
                            cv2.COLOR_BGR2LAB).astype(np.float32)

    @staticmethod
    def _patch_ref(lab):
        h, w = lab.shape[:2]
        ph, pw = max(4, int(h * 0.08)), max(8, int(w * 0.20))
        patch = lab[h - ph:h, w // 2 - pw // 2:w // 2 + pw // 2].reshape(-1, 3)
        return np.median(patch, axis=0), float(patch[:, 1:].std())

    def floor_mask(self, frame):
        lab = self._lab(frame)
        h, w = lab.shape[:2]
        ref, spread = self._patch_ref(lab)
        if self.floor_ref is None:
            self.floor_ref = (ref, spread)
        elif self.adaptive_floor and spread < 8.0:     # only adapt on a uniform patch
            old = self.floor_ref[0]
            self.floor_ref = (0.95 * old + 0.05 * ref, spread)
        d = lab - self.floor_ref[0]
        # lightness weighted down -> shadows still count as floor
        dist = np.sqrt(0.25 * d[..., 0] ** 2 + d[..., 1] ** 2 + d[..., 2] ** 2)
        mask = (dist < self.floor_thresh).astype(np.uint8) * 255
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        # keep only floor connected to the robot (bottom centre)
        n, lbl = cv2.connectedComponents(mask)
        seed = lbl[h - 3, w // 2]
        if seed > 0:
            mask = np.where(lbl == seed, 255, 0).astype(np.uint8)
        return mask

    # ------------------------------------------------------------ 2-4. grid
    def _visible(self, shape):
        key = shape[:2]
        if key not in self._vis_cache:
            full = np.full(key, 255, np.uint8)
            v = cv2.warpPerspective(full, self.M, (self.W, self.Hg), flags=cv2.INTER_NEAREST)
            self._vis_cache[key] = v > 127
        return self._vis_cache[key]

    def build_grid(self, mask, goal=None):
        visible = self._visible(mask.shape)
        floor = cv2.warpPerspective(mask, self.M, (self.W, self.Hg),
                                    flags=cv2.INTER_LINEAR) > 127
        obstacle = visible & ~floor
        if goal is not None:   # fire + its tray are "not floor": clear them
            yy, xx = np.ogrid[:self.Hg, :self.W]
            obstacle &= (yy - goal[0]) ** 2 + (xx - goal[1]) ** 2 > self.goal_clear ** 2
        obs_u8 = obstacle.astype(np.uint8)
        hard = cv2.dilate(obs_u8, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * self.r_hard + 1,) * 2)) > 0
        soft = cv2.dilate(obs_u8, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * self.r_soft + 1,) * 2)) > 0
        # robot can't be inside an obstacle: clear a little bubble at start
        r0, c0 = self.start
        hard[max(0, r0 - 2):, max(0, c0 - 2):c0 + 3] = False

        cost = np.ones((self.Hg, self.W), np.float32)
        cost[~visible] = self.unknown_cost
        cost[soft & ~hard] += self.near_cost
        cost[hard] = np.inf
        return {"cost": cost, "obstacle": obstacle, "hard": hard,
                "soft": soft, "visible": visible}

    # ------------------------------------------------------------ 5. A*
    def astar(self, cost, start, goal):
        Hg, W = cost.shape
        cl = cost.tolist()
        gr, gc = goal

        def h(r, c):
            dr, dc = abs(r - gr), abs(c - gc)
            return (dr + dc) + (SQRT2 - 2) * min(dr, dc)

        g = {start: 0.0}
        parent = {start: None}
        closed = set()
        openq = [(h(*start), 0.0, start)]
        best, best_h = start, h(*start)
        nbrs = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2))
        inf = float("inf")
        while openq:
            _, gcur, cur = heapq.heappop(openq)
            if cur in closed:
                continue
            closed.add(cur)
            r, c = cur
            hc = h(r, c)
            if hc < best_h:
                best, best_h = cur, hc
            if cur == goal:
                break
            for dr, dc, step in nbrs:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < Hg and 0 <= nc < W):
                    continue
                cc = cl[nr][nc]
                if cc == inf:
                    continue
                if dr and dc and (cl[r][nc] == inf or cl[nr][c] == inf):
                    continue                       # no corner cutting
                ng = gcur + step * cc
                if ng < g.get((nr, nc), inf):
                    g[(nr, nc)] = ng
                    parent[(nr, nc)] = cur
                    heapq.heappush(openq, (ng + h(nr, nc), ng, (nr, nc)))
        path, node = [], best
        while node is not None:
            path.append(node)
            node = parent[node]
        path.reverse()
        return path, best == goal

    # ------------------------------------------------------------ main entry
    def plan(self, frame, fire_px=None):
        """
        frame   : BGR camera image
        fire_px : (u, v) image point where fire touches floor (bottom-centre of
                  YOLO box) or None
        returns dict with heading_deg, speed, path, reached, debug data
        """
        mask = self.floor_mask(frame)
        out = {"mask": mask, "heading": None, "speed": 0, "path": [],
               "reached": False, "goal": None, "fire_cm": None, "look": None}

        goal = None
        if fire_px is not None:
            fx, fy = self.img_to_cm(*fire_px)
            out["fire_cm"] = (fx, fy)
            # fire outside grid -> clamp along the ray to the grid border
            s = 1.0
            if fy > self.y1 - self.cell:
                s = min(s, (self.y1 - self.cell) / max(fy, 1e-6))
            if abs(fx) > self.x1 - self.cell:
                s = min(s, (self.x1 - self.cell) / abs(fx))
            fx, fy = fx * s, max(fy * s, self.y0)
            goal = self.cm_to_cell(fx, fy)
            goal = (min(max(goal[0], 0), self.Hg - 1), min(max(goal[1], 0), self.W - 1))
            out["goal"] = goal

        grid = self.build_grid(mask, goal)
        out.update(grid)
        if goal is None:
            return out

        path, reached = self.astar(grid["cost"], self.start, goal)
        out["path"], out["reached"] = path, reached
        if len(path) < 2:
            return out

        # ---------- 6. pure pursuit: first path point >= lookahead away
        pts = [self.cell_to_cm(r, c) for r, c in path]
        look = pts[-1]
        for x, y in pts:
            if math.hypot(x, y) >= self.lookahead:
                look = (x, y)
                break
        out["look"] = look
        out["heading"] = math.degrees(math.atan2(look[0], max(look[1], 1.0)))

        # ---------- speed: slow near obstacles / goal / when goal unreachable
        dist_goal = math.hypot(*out["fire_cm"])
        near_obs = any(grid["soft"][r, c] for r, c in path[:max(2, len(path) // 3)])
        speed = 70
        if dist_goal < 100 or near_obs:
            speed = 50
        if not reached:
            speed = 40
        out["speed"] = speed
        return out

    # ------------------------------------------------------------ drawing
    def render(self, res, scale=6):
        Hg, W = self.Hg, self.W
        img = np.full((Hg, W, 3), 255, np.uint8)
        if "visible" in res:
            img[~res["visible"]] = (90, 90, 90)
            img[res["soft"] & ~res["hard"]] = (170, 210, 255)
            img[res["hard"]] = (60, 120, 255)
            img[res["obstacle"]] = (0, 0, 0)
        img = cv2.resize(img, (W * scale, Hg * scale), interpolation=cv2.INTER_NEAREST)
        # 50 cm grid lines
        step = int(50 / self.cell) * scale
        for x in range(0, W * scale, step):
            cv2.line(img, (x, 0), (x, Hg * scale), (220, 220, 220), 1)
        for y in range(Hg * scale, 0, -step):
            cv2.line(img, (0, y), (W * scale, y), (220, 220, 220), 1)

        def P(rc):
            return int((rc[1] + 0.5) * scale), int((rc[0] + 0.5) * scale)

        path = res.get("path") or []
        for a, b in zip(path, path[1:]):
            cv2.line(img, P(a), P(b), (255, 80, 0), 2)
        if res.get("goal") is not None:
            cv2.circle(img, P(res["goal"]), 6, (0, 0, 255), -1)
        if res.get("look") is not None:
            cv2.circle(img, P(self.cm_to_cell(*res["look"])), 5, (0, 200, 0), -1)
        r0, c0 = self.start
        cv2.rectangle(img, (int((c0 - self.r_hard) * scale), int(r0 * scale) - 4),
                      (int((c0 + self.r_hard + 1) * scale), Hg * scale - 1), (200, 0, 200), -1)
        txt = "no fire" if res.get("heading") is None else \
            f"head {res['heading']:+.0f} deg  spd {res['speed']}  {'OK' if res['reached'] else 'partial'}"
        cv2.putText(img, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        cv2.putText(img, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return img
