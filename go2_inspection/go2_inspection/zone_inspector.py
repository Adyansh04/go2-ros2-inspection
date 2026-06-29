#!/usr/bin/env python3
"""zone_inspector -- VIEWPOINT + 360-degree SPIN camera inspection of ONE zone.

Replaces the old wall-follower (which only faced walls and missed room-interior objects). For a zone we:
  BUILD    : load the zone polygon, sample a few safe observation viewpoints inside it (eroded by a
             safety margin so the robot stays clear of walls).
  VP_NAV   : Nav2 (NavigateToPose) to each viewpoint. If Nav2 rejects/aborts far from the goal, SKIP it.
  VP_SPIN  : slow in-place 360-degree yaw (publish /cmd_vel directly, NOT Nav2) while YOLOE open-vocab
             detection runs CONTINUOUSLY in a background thread. Every detection is projected to the MAP
             frame via the depth camera, de-duplicated across the whole zone (class + world position),
             and the best-confidence crop is kept.
After the last viewpoint: write detections.json (every observation), objects.json (deduped uniques with
world xyz + n_observations), an objects contact sheet, an annotated zone_map.png, and report.md/.csv.

Covers walls AND interior props (drums/pallets/crates/fire/person), not just wall-mounted instruments.
Degrades gracefully: no YOLOE weights / CLIP backend -> still navigates + spins every viewpoint and
writes an empty (available:false) result, exit 0. Run-once contract: exit 0 on DONE, 1 on abort.

  ros2 run go2_inspection zone_inspector --ros-args -p zone_id:=zone_0 -p zones_file:=...
"""
import os, json, math, glob, shutil, threading, time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from nav2_msgs.action import NavigateToPose
import tf2_ros

from go2_inspection import detect_utils
from go2_inspection import report_utils

GAUGES_ROOT = os.path.expanduser("~/gauges")     # output root (kept; readers expect it)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class ZoneInspector(Node):
    def __init__(self):
        super().__init__("zone_inspector")
        self.zone_id = self.declare_parameter("zone_id", "zone_0").value
        self.zones_file = os.path.expanduser(self.declare_parameter("zones_file", "").value)
        self.out_dir = os.path.expanduser(self.declare_parameter("out_dir", "~/gauges").value)
        self.map_yaml = self.declare_parameter("map_yaml", report_utils.DEFAULT_MAP_YAML).value
        # --- viewpoints / motion ---
        self.safety_margin = float(self.declare_parameter("safety_margin", 0.5).value)   # erode poly (m)
        self.vp_spacing = float(self.declare_parameter("vp_spacing", 3.0).value)         # grid spacing (m)
        self.max_viewpoints = int(self.declare_parameter("max_viewpoints", 4).value)
        self.grid_res = float(self.declare_parameter("grid_res", 0.1).value)             # poly raster res
        self.spin_speed = float(self.declare_parameter("spin_speed", 0.4).value)         # rad/s in-place yaw
        self.spin_overlap = math.radians(float(self.declare_parameter("spin_overlap_deg", 20.0).value))
        self.nav_timeout = float(self.declare_parameter("nav_timeout", 120.0).value)
        self.build_timeout = float(self.declare_parameter("build_timeout", 20.0).value)
        # --- detection ---
        self.det_conf = float(self.declare_parameter("det_conf", 0.25).value)   # YOLOE on non-photorealistic
        #   Gazebo Fuel models yields many low-confidence misclassifications at 0.10; 0.25 keeps the stronger,
        #   more-reliable detections. Lower it (-p det_conf:=0.10) if real objects are being missed.
        self.det_weights = self.declare_parameter("det_weights", detect_utils.DEFAULT_WEIGHTS).value
        self.det_device = self.declare_parameter("det_device", os.environ.get("YOLOE_DEVICE", "")).value
        self.center_only = bool(self.declare_parameter("center_only", False).value)      # spin sees all bearings
        # --- depth -> map projection ---
        self.max_depth = float(self.declare_parameter("max_depth", 6.0).value)
        self.min_valid_frac = float(self.declare_parameter("min_valid_frac", 0.30).value)
        self.dedup_radius = float(self.declare_parameter("dedup_radius", 0.6).value)
        self.capture_pad = float(self.declare_parameter("capture_pad", 0.10).value)
        self.optical_frame = self.declare_parameter("optical_frame", "camera_link_optical").value

        self.zone_center = [0.0, 0.0]; self.zone_polygon = []; self.zone_nav = None; self.zone_label = ""
        if self.zones_file and os.path.exists(self.zones_file):
            self._load_zone()
        else:
            self.get_logger().error(f"zones_file missing: {self.zones_file}"); raise SystemExit(1)

        # sensors / actuators
        self.K = None; self.W = None; self.H = None
        self.img = None; self.img_stamp = None; self.depth = None
        self._img_lock = threading.Lock()
        self.tf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.tf, self)
        sens = ReentrantCallbackGroup()
        self.create_subscription(CameraInfo, "/camera/camera_info", self._ci, 10, callback_group=sens)
        self.create_subscription(Image, "/camera/image_raw", self._im, qos_profile_sensor_data, callback_group=sens)
        self.create_subscription(Image, "/camera/depth/image_raw", self._dep, qos_profile_sensor_data, callback_group=sens)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # detection model + background inference thread
        self.model = None; self.model_reason = ""
        self._det_lock = threading.Lock()
        self.live_dets = []; self.live_img = None; self.live_stamp = None
        self._infer_count = 0; self._stop_infer = False; self._infer_thread = None
        self._load_detector()

        # FSM state
        self.viewpoints = []; self.vp_i = 0
        self.objects = []            # deduped uniques (map xy keyed)
        self.detections = []         # every accepted observation
        self.state = "BUILD"; self.failed = False
        self.nav_done = None; self.nav_status = None
        self.build_ticks = 0; self.nav_ticks = 0
        self.spin_yaw0 = None; self.spin_last = None; self.spin_accum = 0.0; self.spin_ticks = 0
        self._proc_mark = 0
        self._clean_outputs()
        self.create_timer(0.1, self._tick, callback_group=MutuallyExclusiveCallbackGroup())
        self._start_infer()
        self.get_logger().info(f"zone_inspector ready: zone={self.zone_id} ({self.zone_label}), "
                               f"{len(self.zone_polygon)}-pt polygon, YOLOE="
                               f"{'on' if self.model else 'OFF (' + self.model_reason + ')'}")

    # ---------- detection ----------
    def _load_detector(self):
        try:
            self.model = detect_utils.load_model(self.det_weights, self.det_device, detect_utils.PROMPTS)
            self.get_logger().info(f"YOLOE loaded: {self.det_weights} ({len(detect_utils.PROMPTS)} prompts)")
        except Exception as e:
            self.model_reason = f"{type(e).__name__}: {e}"
            self.get_logger().warn(f"YOLOE disabled: {self.model_reason}; will scan but capture nothing")

    def _start_infer(self):
        if self.model is None or self._infer_thread is not None:
            return
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()

    def _infer_loop(self):
        while not self._stop_infer and rclpy.ok():
            with self._img_lock:
                img = None if self.img is None else self.img.copy()
                stamp = self.img_stamp
            if img is None:
                time.sleep(0.05); continue
            try:
                img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                dets = detect_utils.infer(self.model, img_bgr, self.det_conf, device=self.det_device)
            except Exception as e:
                if not self._stop_infer:
                    self.get_logger().warn(f"inference error: {e}", throttle_duration_sec=5.0)
                dets = []
            with self._det_lock:
                self.live_dets = dets; self.live_img = img; self.live_stamp = stamp
                self._infer_count += 1
            time.sleep(0.02)

    # ---------- outputs ----------
    def _clean_outputs(self):
        d = os.path.join(self.out_dir, self.zone_id)
        if not os.path.isdir(d):
            return
        for p in (glob.glob(os.path.join(d, "crops", "*.png"))
                  + [os.path.join(d, n) for n in ("detections.json", "objects.json",
                                                  "objects_contact_sheet.png", "zone_map.png",
                                                  "report.md", "report.csv")]):
            try:
                os.remove(p)
            except OSError:
                pass

    # ---------- zone / sensors ----------
    def _load_zone(self):
        z = next((z for z in json.load(open(self.zones_file))["zones"] if z["id"] == self.zone_id), None)
        if z is None:
            self.get_logger().error(f"{self.zone_id} not in {self.zones_file}"); raise SystemExit(1)
        self.zone_center = z["center"]
        self.zone_polygon = z.get("polygon", [])
        self.zone_nav = z.get("nav_point")
        self.zone_label = z.get("label", "")

    def _ci(self, m):
        if self.K is None and m.k[0] > 0.0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5]); self.W, self.H = m.width, m.height

    def _im(self, m):
        try:
            buf = np.frombuffer(m.data, dtype=np.uint8)
            step = m.step if m.step else m.width * 3
            if step * m.height > buf.size:
                return
            rows = np.ascontiguousarray(buf.reshape(m.height, step)[:, :m.width * 3])
            img = rows.reshape(m.height, m.width, 3)
            enc = (m.encoding or "rgb8").lower()
            if enc == "bgr8":
                img = np.ascontiguousarray(img[:, :, ::-1])
            elif enc != "rgb8":
                self.get_logger().warn(f"unsupported camera encoding '{m.encoding}'", throttle_duration_sec=10.0)
                return
        except Exception as e:
            self.get_logger().warn(f"image decode failed: {e}", throttle_duration_sec=10.0)
            return
        with self._img_lock:
            self.img = img; self.img_stamp = m.header.stamp
            if self.W is None:
                self.W, self.H = m.width, m.height

    def _dep(self, m):
        """Decode the registered 32FC1 depth image (metres, aligned to RGB)."""
        if (m.encoding or "32FC1").upper() != "32FC1":
            self.get_logger().warn(f"unexpected depth encoding '{m.encoding}'", throttle_duration_sec=10.0)
            return
        try:
            d = np.frombuffer(m.data, dtype=np.float32)
            if d.size < m.width * m.height:
                return
            self.depth = d[:m.width * m.height].reshape(m.height, m.width)
        except Exception:
            return

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
            return np.array([t.translation.x, t.translation.y]), yaw_of(t.rotation)
        except Exception:
            return None, None

    def stop(self):
        self.cmd.publish(Twist())

    def drive(self, vx, vy, vyaw=0.0):
        t = Twist(); t.linear.x = float(vx); t.linear.y = float(vy); t.angular.z = float(vyaw)
        self.cmd.publish(t)

    def _tick(self):
        getattr(self, "_st_" + self.state, lambda: None)()

    # ---------- viewpoints ----------
    def _sample_viewpoints(self):
        """Erode the zone polygon by safety_margin; sample a grid of interior points (+ nav_point),
        return up to max_viewpoints ordered nearest-first from the current robot pose."""
        poly = self.zone_polygon
        if len(poly) < 3:
            return [tuple(self.zone_nav or self.zone_center)]
        xs = [p[0] for p in poly]; ys = [p[1] for p in poly]
        res = self.grid_res; pad = self.safety_margin + 0.2
        minx, miny = min(xs) - pad, min(ys) - pad
        Wp = int((max(xs) + pad - minx) / res) + 1; Hp = int((max(ys) + pad - miny) / res) + 1
        mask = np.zeros((Hp, Wp), np.uint8)
        pts = np.array([[int((x - minx) / res), int((y - miny) / res)] for x, y in poly], np.int32)
        cv2.fillPoly(mask, [pts], 1)
        r = max(1, int(self.safety_margin / res))
        er = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))

        def to_world(cx, cy):
            return (minx + cx * res, miny + cy * res)

        def in_eroded(x, y):
            cx = int((x - minx) / res); cy = int((y - miny) / res)
            return 0 <= cx < Wp and 0 <= cy < Hp and er[cy, cx] > 0

        cands = []
        if self.zone_nav and in_eroded(*self.zone_nav):
            cands.append(tuple(self.zone_nav))
        step = max(1, int(self.vp_spacing / res))
        for cy in range(0, Hp, step):
            for cx in range(0, Wp, step):
                if er[cy, cx] > 0:
                    cands.append(to_world(cx, cy))
        if not cands:                       # eroded mask empty (thin/small zone) -> deepest free point
            if er.sum() == 0:
                cands = [tuple(self.zone_nav or self.zone_center)]
            else:
                ys2, xs2 = np.where(er > 0)
                cands = [to_world(int(xs2.mean()), int(ys2.mean()))]
        # dedup within ~1.5 m, keep order (nav_point first)
        uniq = []
        for c in cands:
            if all(math.dist(c, u) > 1.5 for u in uniq):
                uniq.append(c)
        # order nearest-first from robot
        p, _ = self.pose()
        if p is not None:
            uniq.sort(key=lambda c: math.dist(c, (float(p[0]), float(p[1]))))
        return uniq[:self.max_viewpoints]

    # ---------- FSM ----------
    def _st_BUILD(self):
        p, _ = self.pose()
        if p is None:
            self.build_ticks += 1
            if self.build_ticks * 0.1 > self.build_timeout:
                self.get_logger().error("no map->base_link TF (localization down?); ABORT")
                self.failed = True; self._finish()
            return
        self.viewpoints = self._sample_viewpoints()
        if not self.viewpoints:
            self.get_logger().warn(f"{self.zone_id}: no viewpoints; DONE"); self._finish(); return
        self.get_logger().info(f"{self.zone_id}: {len(self.viewpoints)} viewpoints {[(round(x,1),round(y,1)) for x,y in self.viewpoints]}")
        self.vp_i = 0
        self._begin_viewpoint()

    def _begin_viewpoint(self):
        self.nav_done = None; self.nav_status = None; self.nav_ticks = 0
        self.state = "VP_NAV"

    def _st_VP_NAV(self):
        vx, vy = self.viewpoints[self.vp_i]
        if self.nav_done is None:
            if not self.nav.wait_for_server(timeout_sec=0.1):
                self.get_logger().warn("waiting for Nav2 ...", throttle_duration_sec=5.0); return
            yaw = math.atan2(self.zone_center[1] - vy, self.zone_center[0] - vx)   # face zone centre
            g = NavigateToPose.Goal()
            # stamp 0 = "use latest available TF": rtabmap's map->odom correction lags sim-now by up to a
            # processing period, so a now()-stamped goal needs a future map->odom and the controller fails
            # to transform it ("Unable to transform goal pose into costmap frame"). Latest-TF is correct here.
            g.pose.header.frame_id = "map"; g.pose.header.stamp = rclpy.time.Time().to_msg()
            g.pose.pose.position.x = float(vx); g.pose.pose.position.y = float(vy)
            g.pose.pose.orientation.z = math.sin(yaw / 2); g.pose.pose.orientation.w = math.cos(yaw / 2)
            self.get_logger().info(f"viewpoint {self.vp_i + 1}/{len(self.viewpoints)}: NAV -> ({vx:.1f},{vy:.1f})")
            self.nav_done = False
            self.nav.send_goal_async(g).add_done_callback(self._nav_acc)
            return
        self.nav_ticks += 1
        if self.nav_done == "reject":
            self.get_logger().warn(f"viewpoint {self.vp_i + 1}: nav rejected; SKIP"); self._advance(); return
        if self.nav_done is True:
            if self.nav_status != 4:
                p, _ = self.pose()
                far = p is None or np.linalg.norm(p - np.array([vx, vy])) > 0.8
                if far:
                    self.get_logger().warn(f"viewpoint {self.vp_i + 1}: nav not SUCCEEDED ({self.nav_status}); SKIP")
                    self._advance(); return
            self._begin_spin(); return
        if self.nav_ticks * 0.1 > self.nav_timeout:
            self.get_logger().warn(f"viewpoint {self.vp_i + 1}: nav timeout; SKIP"); self._advance()

    def _nav_acc(self, fut):
        h = fut.result()
        if not h.accepted:
            self.nav_done = "reject"; return

        def _res(f):
            try:
                self.nav_status = f.result().status
            except Exception:
                self.nav_status = None
            self.nav_done = True
        h.get_result_async().add_done_callback(_res)

    def _begin_spin(self):
        p, y = self.pose()
        self.stop()
        self.spin_yaw0 = y; self.spin_last = y; self.spin_accum = 0.0; self.spin_ticks = 0
        with self._det_lock:
            self._proc_mark = self._infer_count
        self.get_logger().info(f"viewpoint {self.vp_i + 1}: SPIN 360deg")
        self.state = "VP_SPIN"

    def _st_VP_SPIN(self):
        p, y = self.pose()
        self.spin_ticks += 1
        max_ticks = (2 * math.pi / max(self.spin_speed, 0.05)) / 0.1 * 2.2 + 40
        if y is None:
            if self.spin_ticks > 15:
                self.get_logger().warn(f"viewpoint {self.vp_i + 1}: lost TF mid-spin; end"); self.stop(); self._advance()
            return
        if self.spin_last is not None:
            self.spin_accum += abs(ang_diff(y, self.spin_last))
        self.spin_last = y
        self._process_inference()                          # consume detections while turning
        if self.spin_accum >= 2 * math.pi + self.spin_overlap or self.spin_ticks > max_ticks:
            self.stop()
            done = "full turn" if self.spin_accum >= 2 * math.pi else "watchdog"
            self.get_logger().info(f"viewpoint {self.vp_i + 1}: spin {done} ({math.degrees(self.spin_accum):.0f}deg); "
                                   f"{len(self.objects)} unique objects so far")
            self._advance(); return
        self.drive(0.0, 0.0, self.spin_speed)

    def _advance(self):
        self.vp_i += 1
        if self.vp_i < len(self.viewpoints):
            self._begin_viewpoint()
        else:
            self.stop(); self._finish()

    # ---------- detection -> map projection -> dedup -> crop ----------
    def _process_inference(self):
        if self.model is None:
            return
        with self._det_lock:
            if self._infer_count <= self._proc_mark:
                return
            self._proc_mark = self._infer_count
            dets = list(self.live_dets); img = None if self.live_img is None else self.live_img.copy()
            stamp = self.live_stamp
        if not dets or img is None:
            return
        depth = self.depth
        for (name, conf, bbox) in dets:
            xyz = self._project(bbox, depth, stamp)
            self._accumulate(name, conf, bbox, xyz, img)

    def _project(self, bbox, depth, stamp):
        """Depth -> map (x,y,z). Median depth over the inner bbox, deproject via K to camera_link_optical,
        transform to map at the image stamp (fall back to latest)."""
        if self.K is None or depth is None:
            return None
        Hd, Wd = depth.shape[:2]
        x0, y0, x1, y1 = bbox
        dx = int((x1 - x0) * 0.1); dy = int((y1 - y0) * 0.1)
        ix0, iy0 = max(0, x0 + dx), max(0, y0 + dy); ix1, iy1 = min(Wd, x1 - dx), min(Hd, y1 - dy)
        if ix1 <= ix0 or iy1 <= iy0:
            return None
        patch = depth[iy0:iy1, ix0:ix1]
        valid = patch[np.isfinite(patch) & (patch > 0.2) & (patch < self.max_depth)]
        if valid.size == 0 or valid.size < self.min_valid_frac * max(1, patch.size):
            return None
        Z = float(np.median(valid))
        fx, fy, cx, cy = self.K
        u = 0.5 * (x0 + x1); v = 0.5 * (y0 + y1)
        Xc = (u - cx) * Z / fx; Yc = (v - cy) * Z / fy
        Pc = np.array([Xc, Yc, Z])
        try:
            t = self.tf.lookup_transform("map", self.optical_frame,
                                         stamp if stamp is not None else rclpy.time.Time(),
                                         timeout=rclpy.duration.Duration(seconds=0.2)).transform
        except Exception:
            try:
                t = self.tf.lookup_transform("map", self.optical_frame, rclpy.time.Time()).transform
            except Exception:
                return None
        R = quat_to_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        Pm = R @ Pc + np.array([t.translation.x, t.translation.y, t.translation.z])
        return [float(Pm[0]), float(Pm[1]), float(Pm[2])]

    def _accumulate(self, name, conf, bbox, xyz, img_rgb):
        """Dedup by (class, world xy within dedup_radius); keep the best-confidence crop. Objects without a
        world position dedup weakly by class (one per class as a fallback)."""
        self.detections.append({"class": name, "conf": round(float(conf), 3),
                                "bbox": [int(v) for v in bbox], "world": xyz,
                                "viewpoint": self.vp_i})
        localized = xyz is not None
        for o in self.objects:
            if o["class"] != name:
                continue
            if localized and o["world"] and o["world"][0] is not None:
                if math.dist(xyz[:2], o["world"][:2]) < self.dedup_radius:
                    o["n_observations"] += 1
                    if conf > o["confidence"]:
                        o["confidence"] = float(conf); o["world"] = xyz
                        self._save_crop(img_rgb, bbox, o)
                    return
            elif not localized and not o.get("localized", False):
                o["n_observations"] += 1
                if conf > o["confidence"]:
                    o["confidence"] = float(conf); self._save_crop(img_rgb, bbox, o)
                return
        obj = {"id": f"{self.zone_id}_{name.replace(' ', '_')}_{len(self.objects)}", "class": name,
               "confidence": float(conf), "world": xyz, "localized": localized,
               "viewpoint": self.vp_i, "n_observations": 1, "crop": None}
        self._save_crop(img_rgb, bbox, obj)
        self.objects.append(obj)
        self.get_logger().info(f"  + {name} ({conf:.2f}) "
                               f"{'@ (%.1f,%.1f)' % (xyz[0], xyz[1]) if localized else '(unlocalized)'}")

    def _save_crop(self, img_rgb, bbox, obj):
        H, W = img_rgb.shape[:2]
        x0, y0, x1, y1 = bbox
        pw = int((x1 - x0) * self.capture_pad); ph = int((y1 - y0) * self.capture_pad)
        x0 = max(0, x0 - pw); y0 = max(0, y0 - ph); x1 = min(W, x1 + pw); y1 = min(H, y1 + ph)
        if x1 <= x0 or y1 <= y0:
            return
        crop_bgr = cv2.cvtColor(img_rgb[y0:y1, x0:x1].copy(), cv2.COLOR_RGB2BGR)
        cdir = os.path.join(self.out_dir, self.zone_id, "crops"); os.makedirs(cdir, exist_ok=True)
        fn = f"{obj['id']}.png"
        cv2.imwrite(os.path.join(cdir, fn), crop_bgr)
        obj["crop"] = f"crops/{fn}"

    def _finish(self):
        d = os.path.join(self.out_dir, self.zone_id); os.makedirs(d, exist_ok=True)
        avail = self.model is not None
        json.dump({"zone": self.zone_id, "label": self.zone_label, "available": avail,
                   "reason": "" if avail else self.model_reason, "failed": self.failed,
                   "n_detections": len(self.detections), "detections": self.detections},
                  open(os.path.join(d, "detections.json"), "w"), indent=2)
        json.dump({"zone": self.zone_id, "label": self.zone_label, "available": avail,
                   "reason": "" if avail else self.model_reason, "failed": self.failed,
                   "n_objects": len(self.objects), "objects": self.objects},
                  open(os.path.join(d, "objects.json"), "w"), indent=2)
        try:
            detect_utils.contact_sheet(d, self.objects)
            report_utils.plot_zone_map(self.zone_id, self.zone_polygon, self.viewpoints, self.objects,
                                       os.path.join(d, "zone_map.png"), self.map_yaml)
            report_utils.write_zone_report(self.zone_id, self.zone_label, self.objects, d)
        except Exception as e:
            self.get_logger().warn(f"report/plot failed: {e}")
        self._stop_infer = True
        tag = "ABORTED" if self.failed else "DONE"
        self.get_logger().info(f"{self.zone_id}: inspection {tag} -- {len(self.objects)} unique objects "
                               f"({len(self.detections)} observations) -> {d}")
        self.state = "DONE"


def main(args=None):
    import sys
    rclpy.init(args=args)
    n = ZoneInspector()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        while rclpy.ok() and n.state != "DONE":
            ex.spin_once(timeout_sec=0.1)
        n.get_logger().info("inspection complete; shutting down")
    except KeyboardInterrupt:
        pass
    failed = n.failed or n.state != "DONE"
    n._stop_infer = True
    if n._infer_thread is not None:
        n._infer_thread.join(timeout=10.0)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
