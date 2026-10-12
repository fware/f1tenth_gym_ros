#!/usr/bin/env python3
"""
Follow-the-Gap LiDAR controller with a map-aware demo mission for the
RoboRacer (F1TENTH) simulator.

Subscribes : /scan  (or /opp_scan)            sensor_msgs/LaserScan
             /ego_racecar/odom (or /opp_racecar/odom)   nav_msgs/Odometry
Publishes  : /drive (or /opp_drive)           ackermann_msgs/AckermannDriveStamped

Layers (one pass per scan, ~40 Hz):
  * Geofence   - the map is loaded and anything outside the known hallways is
                 ray-cast as a "virtual wall" and merged into the scan, so the
                 open ends of the building behave like dead-end entry ways
                 instead of exits into empty space.
  * Mission    - an ordered route of waypoints (entry ways to explore, loop
                 corners, then home).  A grid planner on the map gives a path to
                 the current waypoint; the car pure-pursues a look-ahead point.
                 At the end of an entry way the car does a K-turn, and at the
                 end of the route it parks in its start hallway.
  * Gap follow - if the path ahead is blocked (e.g. by the other car) the car
                 steers into the free gap closest to where it wants to go.
                 Boxed in for several scans -> back up toward the open side.

Run inside the sim container (after the simulator is launched):
    source /opt/ros/foxy/setup.bash
    python3 gap_follow.py            # ego car  (/scan -> /drive)
    python3 gap_follow.py --opp      # opponent (/opp_scan -> /opp_drive)
Options:
    --laps N   extra laps of the loop before heading home (default 0)
    --free     plain reactive gap follow: no map, no route (original behavior)

With num_agent: 2 the simulator only advances once BOTH cars have received a
drive command, so run one instance of each (in separate terminals).
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from PIL import Image
from rclpy.node import Node
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

# ---- Tunable parameters (starting values for the default Levine map) ----
FOV_HALF_DEG = 75.0      # consider beams within +/- this angle of straight ahead
MAX_RANGE = 8.0          # clip long returns (m); long enough to tell hallways from alcoves
SMOOTH_WINDOW = 5        # moving-average window (beams)
BUBBLE_RADIUS = 0.40     # safety bubble around the closest point (m); car is ~0.3 m wide
GAP_THRESHOLD = 1.5      # a beam counts as "free" if its range exceeds this (m)
MIN_GAP_BEAMS = 10       # ignore slivers narrower than this
DEPTH_BIAS = 0.75        # 0 = aim at gap center, 1 = aim at deepest point
MAX_STEER = 0.40         # rad; sim steering limit is ~0.42
SPEED_FAST = 3.5         # m/s on straights
SPEED_MED = 2.2
SPEED_SLOW = 1.2

# ---- Recovery (getting un-stuck) ----
STUCK_DIST = 0.45        # m; anything this close straight ahead counts as blocked
STUCK_SCANS = 5          # consecutive blocked scans before recovering
# The gym's tyre model is only stable in reverse below 0.5 m/s (faster reversing
# makes its physics diverge and crashes the simulator), so all reversing is slow.
REVERSE_SPEED = -0.45    # m/s
REVERSE_TIME = 2.5       # s

# ---- Mission / map ----
MAP_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "maps", "levine.yaml")
WHEELBASE = 0.33         # m (lf + lr in the gym vehicle model)
LIDAR_OFFSET = 0.275     # m, LiDAR ahead of base_link (matches sim.yaml)
CAR_LENGTH, CAR_WIDTH = 0.58, 0.31  # m, gym vehicle footprint (base_link at its centre)
PLAN_RES = 0.10          # m, planner grid cell size
MIN_CLEARANCE = 0.30     # m, planner keeps the car center at least this far from walls
CENTER_WEIGHT = 4.0      # extra cost near walls, keeps the path mid-hallway
LOOKAHEAD = 1.5          # m, pure-pursuit look-ahead along the planned path
TURNAROUND_DEG = 110.0   # path point further behind than this -> K-turn
TURN_SPEED = 0.8         # m/s forward during a K-turn (reverse uses REVERSE_SPEED)
APPROACH_SPEED = 1.0     # m/s cap when closing in on an entry-way end
PARK_RADIUS = 0.30       # m, close enough to home to stop

# Named places on the Levine map (map frame, metres).
PLACES = {
    "east_entry":      (14.5, 0.27),    # corridor off the east junction (open end)
    "southeast_stub":  (9.75, -3.0),    # short dead end below the east junction
    "northwest_entry": (-14.3, 13.5),   # top of the north-west stub (open end to the west)
    "southwest_entry": (-13.7, -5.5),   # corridor below the south-west corner (open end)
    "ne_corner":       (9.75, 8.6),
    "nw_corner":       (-13.7, 8.6),
    "sw_corner":       (-13.7, -0.2),
    "se_corner":       (9.75, -0.2),
    "ego_home":        (-0.5, -0.55),   # ego parks in the south lane
    "opp_home":        (2.5, 0.15),     # opponent parks in the north lane, ahead of ego
}
ENTRY_WAYS = {"east_entry", "southeast_stub", "northwest_entry", "southwest_entry"}
# Both cars lap counter-clockwise; each explores different entry ways so they
# never meet head-on inside a dead end.
ROUTES = {
    "ego": ["east_entry", "ne_corner", "nw_corner", "northwest_entry", "sw_corner"],
    "opp": ["southeast_stub", "ne_corner", "nw_corner", "southwest_entry", "sw_corner"],
}
LAP = ["se_corner", "ne_corner", "nw_corner", "sw_corner"]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class MapModel:
    """The occupancy map: virtual-wall ray casting and a grid path planner."""

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            info = yaml.safe_load(f)
        img = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), info["image"])).convert("L"))
        self.res = float(info["resolution"])
        self.ox, self.oy = float(info["origin"][0]), float(info["origin"][1])
        self.h = img.shape[0]
        # Walls and unknown (grey) space count as blocked, which closes off the
        # open ends of the map.  The grey anti-aliasing halo along real walls is
        # left out so virtual walls do not creep into the hallway.
        wall = img <= 128
        unknown = (img < 250) & ~wall & ~ndimage.binary_dilation(wall, iterations=3)
        self.blocked = wall | unknown

        # Planner grid at PLAN_RES: a cell is blocked if any pixel inside it is.
        k = int(round(PLAN_RES / self.res))
        hh, ww = (self.h // k) * k, (img.shape[1] // k) * k
        coarse = self.blocked[:hh, :ww].reshape(hh // k, k, ww // k, k).any(axis=(1, 3))
        self.k = k
        clearance = ndimage.distance_transform_edt(~coarse) * PLAN_RES
        self.free = clearance >= MIN_CLEARANCE
        cost = 1.0 + CENTER_WEIGHT * np.clip(1.0 - clearance / 1.0, 0.0, 1.0)
        # nearest free cell for any cell (used when the car is close to a wall)
        _, self.nearest = ndimage.distance_transform_edt(~self.free, return_indices=True)

        ids = -np.ones(coarse.shape, dtype=np.int64)
        fy, fx = np.nonzero(self.free)
        ids[fy, fx] = np.arange(fy.size)
        self.ids = ids
        rows, cols, wts = [], [], []
        for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
            ny, nx = fy + dy, fx + dx
            ok = (ny < coarse.shape[0]) & (nx >= 0) & (nx < coarse.shape[1])
            ok[ok] = self.free[ny[ok], nx[ok]]
            step = math.hypot(dy, dx)
            rows.append(ids[fy[ok], fx[ok]])
            cols.append(ids[ny[ok], nx[ok]])
            wts.append(step * 0.5 * (cost[fy[ok], fx[ok]] + cost[ny[ok], nx[ok]]))
        n = fy.size
        self.graph = coo_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
                                shape=(n, n)).tocsr()
        self.fields = {}

    # -- coordinates --
    def to_px(self, x, y):
        return (self.h - 1 - (np.asarray(y) - self.oy) / self.res).astype(int), \
               ((np.asarray(x) - self.ox) / self.res).astype(int)

    def to_cell(self, x, y):
        r, c = self.to_px(x, y)
        r = int(np.clip(r // self.k, 0, self.free.shape[0] - 1))
        c = int(np.clip(c // self.k, 0, self.free.shape[1] - 1))
        return int(self.nearest[0, r, c]), int(self.nearest[1, r, c])

    def to_world(self, r, c):
        return self.ox + (c + 0.5) * PLAN_RES, self.oy + (self.h / self.k - 1 - r + 0.5) * PLAN_RES

    # -- sensing --
    def raycast(self, x, y, angles, max_range):
        """Distance from (x, y) to the first blocked map pixel along each angle."""
        d = np.arange(0.0, max_range, self.res)
        px = x + np.outer(np.cos(angles), d)
        py = y + np.outer(np.sin(angles), d)
        r, c = self.to_px(px, py)
        inside = (r >= 0) & (r < self.h) & (c >= 0) & (c < self.blocked.shape[1])
        hit = np.ones_like(inside)
        hit[inside] = self.blocked[r[inside], c[inside]]
        hit[:, d < 0.25] = False  # ignore the fuzzy wall edge right next to the car
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), d.size - 1)
        return d[first]

    def footprint_hits(self, x, y, yaw, margin=0.04):
        """True if the car's rectangle at this pose overlaps a blocked map pixel."""
        gx, gy = np.meshgrid(np.arange(-CAR_LENGTH / 2 - margin, CAR_LENGTH / 2 + margin + 1e-6, self.res),
                             np.arange(-CAR_WIDTH / 2 - margin, CAR_WIDTH / 2 + margin + 1e-6, self.res))
        r, c = self.to_px(x + math.cos(yaw) * gx - math.sin(yaw) * gy,
                          y + math.sin(yaw) * gx + math.cos(yaw) * gy)
        inside = (r >= 0) & (r < self.h) & (c >= 0) & (c < self.blocked.shape[1])
        return bool((~inside).any() or self.blocked[r[inside], c[inside]].any())

    # -- planning --
    def field_to(self, name, goal):
        """Path cost from every free cell to `goal` (cached per waypoint)."""
        if name not in self.fields:
            r, c = self.to_cell(*goal)
            dist = dijkstra(self.graph, directed=False, indices=int(self.ids[r, c]))
            grid = np.full(self.free.shape, np.inf)
            grid[self.free] = dist[self.ids[self.free]]
            self.fields[name] = grid
        return self.fields[name]

    def lookahead(self, field, x, y, length):
        """Walk downhill on the cost field from (x, y) for `length` metres."""
        r, c = self.to_cell(x, y)
        travelled = 0.0
        while travelled < length:
            best, br, bc = field[r, c], r, c
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < field.shape[0] and 0 <= cc < field.shape[1] and field[rr, cc] < best:
                        best, br, bc = field[rr, cc], rr, cc
            if (br, bc) == (r, c):
                break  # reached the goal cell
            travelled += math.hypot(br - r, bc - c) * PLAN_RES
            r, c = br, bc
        return self.to_world(r, c)


class GapFollow(Node):
    def __init__(self, opp=False, free_mode=False, laps=0):
        super().__init__("gap_follow_opp" if opp else "gap_follow")
        car = "opp" if opp else "ego"
        scan_topic, drive_topic = ("/opp_scan", "/opp_drive") if opp else ("/scan", "/drive")
        odom_topic = "/opp_racecar/odom" if opp else "/ego_racecar/odom"
        self.sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.stuck_count = 0
        self.recover_until = 0.0
        self.recover_steer = 0.0

        self.free_mode = free_mode
        self.pose = None
        self.turn = None          # K-turn state: dict(side, phase, since)
        self.parked = False
        if not free_mode:
            self.map = MapModel(MAP_YAML)
            self.route = ROUTES[car] + LAP * laps + [car + "_home"]
            self.wp = 0
            self.odom_sub = self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
            self.get_logger().info("Route: " + " -> ".join(self.route))
        self.get_logger().info(
            "Follow-the-Gap controller running (%s -> %s)%s"
            % (scan_topic, drive_topic, " [free mode]" if free_mode else ""))

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_scan(self, scan: LaserScan):
        now = time.monotonic()
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        ranges[~np.isfinite(ranges)] = MAX_RANGE
        n = ranges.size
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        # "blocked straight ahead" uses the real scan only (virtual walls run
        # tangent to real ones and would falsely trip it next to a wall)
        ahead = float(np.min(ranges[np.abs(angles) < math.radians(10)]))

        if not self.free_mode:
            if self.pose is None:
                return
            if self.parked:
                self.publish(0.0, 0.0)
                return
            # geofence: merge virtual walls (map edges / open ends) into the scan
            x, y, yaw = self.pose
            lx, ly = x + LIDAR_OFFSET * math.cos(yaw), y + LIDAR_OFFSET * math.sin(yaw)
            front = np.abs(angles) <= math.radians(FOV_HALF_DEG + 10)
            ranges[front] = np.minimum(
                ranges[front], self.map.raycast(lx, ly, yaw + angles[front], MAX_RANGE))

        # 6a. finish an in-progress recovery manoeuvre first
        #     (stop early if the map says a wall is right behind us)
        if now < self.recover_until and not self.motion_blocked(self.recover_steer, -0.35):
            self.publish(self.recover_steer, REVERSE_SPEED)
            return
        self.recover_until = 0.0

        # mission: where does the planned path want us to go?
        alpha = goal_dist = None
        if not self.free_mode:
            alpha, look_dist, goal_dist, entry, home = self.mission_target()
            if self.parked:
                self.publish(0.0, 0.0)
                return
            if self.turn is not None or abs(alpha) > math.radians(TURNAROUND_DEG):
                if self.k_turn(alpha, ranges, angles, now):
                    return

        # 1. forward window, clean, smooth
        half = math.radians(FOV_HALF_DEG)
        idx = np.where(np.abs(angles) <= half)[0]
        r = np.clip(ranges[idx], 0.0, MAX_RANGE)
        a = angles[idx]
        kernel = np.ones(SMOOTH_WINDOW) / SMOOTH_WINDOW
        r = np.convolve(r, kernel, mode="same")

        # 2. safety bubble around the closest point
        closest = int(np.argmin(r))
        d = max(float(r[closest]), 0.05)
        bubble_beams = int(math.ceil(math.atan2(BUBBLE_RADIUS, d) / scan.angle_increment))
        r[max(0, closest - bubble_beams):min(r.size, closest + bubble_beams + 1)] = 0.0

        # 3. find every gap; score each by the open space it contains
        free = np.append(r > GAP_THRESHOLD, False)
        gaps, best, best_score, start = [], None, 0.0, None
        for i, f in enumerate(free):
            if f and start is None:
                start = i
            elif not f and start is not None:
                if i - start >= MIN_GAP_BEAMS:
                    gaps.append((start, i))
                    score = float(np.sum(r[start:i] ** 2))  # depth counts more than width
                    if score > best_score:
                        best, best_score = (start, i), score
                start = None

        # 6b. boxed in?  count it, and trigger recovery if it persists
        if best is None or ahead < STUCK_DIST:
            self.stuck_count += 1
            if self.stuck_count >= STUCK_SCANS:
                self.start_recovery(ranges, angles, now)
            else:
                self.publish(0.0, 0.0)
            return
        self.stuck_count = 0

        if self.free_mode:
            # 4. aim toward the deep part of the best gap
            g0, g1 = best
            seg = r[g0:g1]
            deep_idx = np.where(seg >= seg.max() - 0.1)[0]   # middle of the deepest stretch
            deepest = g0 + int(round(deep_idx.mean()))
            center = (g0 + g1) // 2
            target = int(round(center + DEPTH_BIAS * (deepest - center)))
            steer = float(np.clip(a[target], -MAX_STEER, MAX_STEER))
        else:
            # 4'. follow the planned path if that direction is clear, otherwise
            #     take the free gap closest to it (e.g. to pass the other car)
            t = int(np.argmin(np.abs(a - alpha)))
            if r[t] > min(look_dist, GAP_THRESHOLD) or abs(alpha) > half:
                steer = math.atan2(2.0 * WHEELBASE * math.sin(alpha), look_dist)
            else:
                cand = np.concatenate([np.arange(g0 + 3, g1 - 3) for g0, g1 in gaps] or [[t]])
                steer = float(a[cand[np.argmin(np.abs(a[cand] - alpha))]])
            steer = float(np.clip(steer, -MAX_STEER, MAX_STEER))

        # 5. speed schedule
        if abs(steer) < 0.10 and ahead > 2.5:
            speed = SPEED_FAST
        elif abs(steer) < 0.25 and ahead > 1.0:
            speed = SPEED_MED
        else:
            speed = SPEED_SLOW
        if not self.free_mode and (entry or home):
            # ease in to entry-way ends and the parking spot
            speed = min(speed, max(APPROACH_SPEED * goal_dist / 2.0, 0.5 if home else APPROACH_SPEED))
        self.publish(steer, speed)

    def mission_target(self):
        """Advance the route; return path heading and distances for the current waypoint."""
        x, y, yaw = self.pose
        name = self.route[self.wp]
        gx, gy = PLACES[name]
        goal_dist = math.hypot(gx - x, gy - y)
        home = self.wp == len(self.route) - 1
        entry = name in ENTRY_WAYS
        reach = PARK_RADIUS if home else (0.8 if entry else 1.5)
        if goal_dist < reach:
            if home:
                self.parked = True
                self.get_logger().info("Home: parked at (%.2f, %.2f)" % (x, y))
                return 0.0, 1.0, 0.0, entry, home
            self.wp += 1
            self.get_logger().info("Reached %s -> next: %s" % (name, self.route[self.wp]))
            return self.mission_target()
        field = self.map.field_to(name, (gx, gy))
        tx, ty = self.map.lookahead(field, x, y, LOOKAHEAD)
        if home and goal_dist < LOOKAHEAD:
            tx, ty = gx, gy  # aim at the exact parking spot at the end
        alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
        return alpha, max(math.hypot(tx - x, ty - y), 0.3), goal_dist, entry, home

    def k_turn(self, alpha, ranges, angles, now):
        """Turn around in a narrow hallway.  Returns False once facing the path again."""
        if self.turn is None:
            self.turn = {"side": 1.0 if alpha >= 0 else -1.0, "phase": "fwd", "since": now}
            self.get_logger().info("Turning around (K-turn)")
        ahead = float(np.min(ranges[np.abs(angles) < math.radians(20)]))
        if abs(alpha) < math.radians(35) and ahead > 0.8:
            self.turn = None
            return False
        side = self.turn["side"]
        if self.turn["phase"] == "fwd":
            front = float(np.min(ranges[np.abs(angles) < math.radians(60)]))
            if front < 0.35 or self.motion_blocked(side * MAX_STEER, 0.3) \
                    or now - self.turn["since"] > 4.0:
                self.turn.update(phase="rev", since=now)
            else:
                self.publish(side * MAX_STEER, TURN_SPEED)
                return True
        # reverse with the wheels the other way; watch the map behind the car
        if self.motion_blocked(-side * MAX_STEER, -0.35) or now - self.turn["since"] > 3.5:
            self.turn.update(phase="fwd", since=now)
            self.publish(side * MAX_STEER, TURN_SPEED)
        else:
            self.publish(-side * MAX_STEER, REVERSE_SPEED)
        return True

    def motion_blocked(self, steer, dist):
        """Would driving `dist` metres (negative = reverse) at this steering angle
        sweep the car into a wall?  Checked on the map because the LiDAR cannot
        see behind the car (and the gym does not stop cars backing into walls)."""
        if self.free_mode or self.pose is None:
            return False
        x, y, yaw = self.pose
        for s in (dist / 2.0, dist):
            yaw2 = yaw + s / WHEELBASE * math.tan(steer)   # kinematic bicycle model
            mid = 0.5 * (yaw + yaw2)
            if self.map.footprint_hits(x + s * math.cos(mid), y + s * math.sin(mid), yaw2):
                return True
        return False

    def start_recovery(self, ranges, angles, now):
        # Which side has more room?  (+ = left, - = right)
        left = np.mean(ranges[(angles > math.radians(30)) & (angles < math.radians(120))])
        right = np.mean(ranges[(angles < -math.radians(30)) & (angles > -math.radians(120))])
        open_side = 1.0 if left >= right else -1.0
        # Reversing with the wheels turned one way swings the nose the OTHER way,
        # so steer away from the open side to point the nose toward it.
        self.recover_steer = -open_side * MAX_STEER
        self.recover_until = now + REVERSE_TIME
        self.stuck_count = 0
        self.get_logger().info(
            "Boxed in: backing up, turning toward the %s" % ("left" if open_side > 0 else "right"))
        self.publish(self.recover_steer, REVERSE_SPEED)

    def publish(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.pub.publish(msg)


def main():
    args = sys.argv[1:]
    laps = int(args[args.index("--laps") + 1]) if "--laps" in args else 0
    rclpy.init()
    node = GapFollow(opp="--opp" in args, free_mode="--free" in args, laps=laps)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)  # leave the car stopped
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
