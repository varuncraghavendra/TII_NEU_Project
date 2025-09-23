#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
from shapely.geometry import Polygon
import numpy as np
import networkx as nx
import pickle
import os
from hashlib import md5
import time
import threading
import tkinter as tk
import paho.mqtt.client as mqtt
import json
from typing import Tuple, Optional, List

# ----------------------------- LIVE collector hook -----------------------------
# Keep this import exactly as in your project. If the collector is down/unreachable,
# we will auto-switch to SIM mode so the run continues with dynamic metrics.
from influx_hook import get_average_rsrp_3s

# =============================== CONFIG ========================================
BROKER = "broker.hivemq.com"
PORT   = 1883
TOPIC  = "colosseum/update"
DRONE_ID = 1

# Retries & fallback for RSRP fetch
RSRP_RETRIES = 1          # extra tries after first failure
RSRP_RETRY_WAIT = 2.0     # seconds between retries
RSRP_FAIL_VALUE = -999.0  # sentinel (never send None)

# Search weights
WEIGHT_RSRP = 0.7
WEIGHT_UE   = 0.3

# Data source behavior
AUTO_SWITCH_TO_SIM = True    # if LIVE fails, use SIM so values change
AUTO_SWITCH_BACK_TO_LIVE = True  # if LIVE recovers, flip back

# Area / polygon (clockwise)
CUSTOM_BOUNDS = {
    'top_left'    : [42.34138538637752, -71.08300209045412],
    'top_right'   : [42.340314805239075, -71.08198285102846],
    'bottom_right': [42.337903948379996, -71.08519077301027],
    'bottom_left' : [42.33866528158443, -71.08609199523927]
}

# Cache keys (original style)
CENTER_LAT = 42.33894284868896
CENTER_LNG = -71.08613491058351
RADIUS_METERS = 1000
GRID_SPACING_METERS = 5

# Your exact cache filenames
EXACT_BUILDINGS_CACHE = "buildings_cache_e7ba8105.pkl"
EXACT_GRAPH_CACHE     = "graph_cache_91fc2d6b.pkl"

# UE beacons (for SIM scoring). Edit to your real UE GPS if you prefer.
SIM_UE_BEACONS = [
    (42.338824, -71.084579),
    (42.338562, -71.084783),
    (42.338705, -71.085942)
]

# SIM tuning (distance-based improvement)
SIM_MAX_RSRP_AT_TARGET = -65.0   # dBm when on top of best spot
SIM_MIN_RSRP_FAR       = -112.0  # dBm far away
SIM_MAX_UES_AT_TARGET  = 8       # when close/optimal
SIM_MIN_UES_FAR        = 0
SIM_EFFECT_RADIUS_M    = 180.0   # meters for decay profile
# ==============================================================================


# =============================== MQTT =========================================
_mqtt = mqtt.Client(protocol=mqtt.MQTTv311)
try:
    _mqtt.connect(BROKER, PORT, 60)
    _mqtt.loop_start()
except Exception as e:
    print(f"[WARN] MQTT connect failed: {e}")

def mqtt_publish_flat(lat, lon, iteration, rsrp, distance, signal_quality, target_ue):
    """
    EXACT flat schema you gave originally; never None.
    """
    payload = {
        "drone_id": DRONE_ID,
        "lat": float(lat),
        "lon": float(lon),
        "rsrp": float(rsrp),
        "distance": float(distance),
        "signal_quality": float(signal_quality),
        "iteration": int(iteration),
        "target_ue": int(target_ue)
    }
    try:
        _mqtt.publish(TOPIC, json.dumps(payload))
    except Exception as e:
        print(f"[WARN] MQTT publish failed: {e}")
# ==============================================================================


# =============================== DASHBOARD =====================================
class DroneDashboard:
    def __init__(self, title="Drone Live Dashboard"):
        self.root = tk.Tk()
        self.root.title(title)
        self.root.geometry("700x420")
        self.labels = {}
        fields = [
            "Iteration",
            "Current Lat",
            "Current Lon",
            "Current RSRP (dBm)",
            "Signal Quality (%)",
            "Connected UEs (#)",
            "Distance to Target (m)",
            "Best-so-far RSRP (dBm)",
            "Best-so-far UEs (#)",
            "Data Source"
        ]
        for i, f in enumerate(fields):
            tk.Label(self.root, text=f + ":").grid(row=i, column=0, sticky="w", padx=10, pady=6)
            self.labels[f] = tk.Label(self.root, text="---", width=42, anchor="w")
            self.labels[f].grid(row=i, column=1, sticky="w", padx=10)

        self.root.grid_columnconfigure(0, minsize=260)
        self.root.grid_columnconfigure(1, minsize=380)
        self._lock = threading.Lock()

    def update_position(self, iteration, lat, lon):
        with self._lock:
            self.labels["Iteration"].config(text=str(iteration))
            self.labels["Current Lat"].config(text=f"{lat:.6f}")
            self.labels["Current Lon"].config(text=f"{lon:.6f}")
            self.root.update_idletasks()

    def update_measurements(self, rsrp, sigq, ues, source):
        with self._lock:
            self.labels["Current RSRP (dBm)"].config(text=f"{rsrp:.1f}")
            self.labels["Signal Quality (%)"].config(text=f"{sigq:.1f}")
            self.labels["Connected UEs (#)"].config(text=f"{int(ues)}")
            self.labels["Data Source"].config(text=source)
            self.root.update_idletasks()

    def update_distance_to_target(self, meters):
        with self._lock:
            self.labels["Distance to Target (m)"].config(text=f"{meters:.1f}")
            self.root.update_idletasks()

    def update_bests(self, best_rsrp, best_ues):
        with self._lock:
            self.labels["Best-so-far RSRP (dBm)"].config(text=f"{best_rsrp:.1f}")
            self.labels["Best-so-far UEs (#)"].config(text=f"{best_ues}")
            self.root.update_idletasks()

    def start(self):
        self.root.mainloop()
# ==============================================================================


# =============================== GEO / UTILS ===================================
def get_cache_filename(cache_type="buildings"):
    key = f"{CENTER_LAT}_{CENTER_LNG}_{RADIUS_METERS}"
    if cache_type == "graph":
        key += f"_grid{GRID_SPACING_METERS}m"
    return f"{cache_type}_cache_{md5(key.encode()).hexdigest()[:8]}.pkl"

def meters_to_degrees(meters, is_longitude=False, center_lat=None):
    if is_longitude and center_lat is not None:
        return meters / (111111.0 * np.cos(np.radians(center_lat)))
    else:
        return meters / 111111.0

def degrees_to_meters(deg, is_longitude=False, center_lat=None):
    if is_longitude and center_lat is not None:
        return deg * (111111.0 * np.cos(np.radians(center_lat)))
    else:
        return deg * 111111.0

def haversine_m(lat1, lon1, lat2, lon2) -> float:
    # close enough for short distances (uses flat-earth approx via degrees_to_meters)
    dy = degrees_to_meters(lat2 - lat1)
    dx = degrees_to_meters(lon2 - lon1, is_longitude=True, center_lat=(lat1+lat2)/2)
    return float(np.hypot(dx, dy))

def create_custom_polygon():
    # (lng, lat) order; clockwise
    return [
        (CUSTOM_BOUNDS['top_left'][1],     CUSTOM_BOUNDS['top_left'][0]),
        (CUSTOM_BOUNDS['top_right'][1],    CUSTOM_BOUNDS['top_right'][0]),
        (CUSTOM_BOUNDS['bottom_right'][1], CUSTOM_BOUNDS['bottom_right'][0]),
        (CUSTOM_BOUNDS['bottom_left'][1],  CUSTOM_BOUNDS['bottom_left'][0])
    ]

def get_polygon_area_meters(corners):
    return Polygon(corners).area * (111111**2)

def get_midpoint(p1, p2):
    return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)

def cut_polygon_in_half(corners, best_edge_idx):
    c_idx = best_edge_idx
    d_idx = (best_edge_idx + 1) % 4
    a_idx = (best_edge_idx + 2) % 4
    b_idx = (best_edge_idx + 3) % 4
    point_c = corners[c_idx]
    point_d = corners[d_idx]
    point_a = corners[a_idx]
    point_b = corners[b_idx]
    mid_ad = get_midpoint(point_a, point_d)
    mid_bc = get_midpoint(point_b, point_c)
    return [point_c, point_d, mid_bc, mid_ad]

def try_load(path):
    if os.path.isfile(path):
        with open(path, "rb") as f:
            return pickle.load(f)
    return None

def load_graph():
    """
    Try EXACT filenames first, then md5-style fallback, else raise.
    """
    buildings_data = try_load(EXACT_BUILDINGS_CACHE)
    graph_data     = try_load(EXACT_GRAPH_CACHE)
    if buildings_data is not None and graph_data is not None:
        print(f"[CACHE] Loaded buildings -> {EXACT_BUILDINGS_CACHE}")
        print(f"[CACHE] Loaded graph     -> {EXACT_GRAPH_CACHE}")
        if not all(k in graph_data for k in ('grid','graph','key_to_node')):
            raise RuntimeError(f"{EXACT_GRAPH_CACHE} missing keys: grid/graph/key_to_node")
        return buildings_data, graph_data['grid'], graph_data['graph'], graph_data['key_to_node']

    # fallback
    cache_file = get_cache_filename("graph")
    buildings_cache = get_cache_filename("buildings")
    print(f"[CACHE] Falling back to md5 caches:\n        buildings -> {buildings_cache}\n        graph     -> {cache_file}")
    if not (os.path.isfile(buildings_cache) and os.path.isfile(cache_file)):
        raise FileNotFoundError(
            "Cache files not found. Tried:\n"
            f" - {EXACT_BUILDINGS_CACHE}\n - {EXACT_GRAPH_CACHE}\n"
            f" - {buildings_cache}\n - {cache_file}"
        )
    with open(cache_file, 'rb') as f:
        cached_data = pickle.load(f)
    with open(buildings_cache, 'rb') as f:
        buildings = pickle.load(f)
    if not all(k in cached_data for k in ('grid','graph','key_to_node')):
        raise RuntimeError(f"{cache_file} missing keys: grid/graph/key_to_node")
    return buildings, cached_data['grid'], cached_data['graph'], cached_data['key_to_node']

def find_nearest_graph_node(G, target_lat, target_lng):
    nearest_node, min_d = None, float('inf')
    for node in G.nodes():
        nl, ng = G.nodes[node]['lat'], G.nodes[node]['lng']
        d = np.hypot(nl - target_lat, ng - target_lng)
        if d < min_d:
            min_d, nearest_node = d, node
    return nearest_node

def get_shortest_path_between_points(G, lat1, lng1, lat2, lng2):
    n1 = find_nearest_graph_node(G, lat1, lng1)
    n2 = find_nearest_graph_node(G, lat2, lng2)
    if n1 is None or n2 is None:
        return None, None
    try:
        nodes = nx.shortest_path(G, n1, n2, weight='weight')
        coords = [(G.nodes[n]['lat'], G.nodes[n]['lng']) for n in nodes]
        length = nx.shortest_path_length(G, n1, n2, weight='weight')
        return coords, length
    except nx.NetworkXNoPath:
        return None, None
# ==============================================================================


# =============================== DATA SOURCE ===================================
class DataSource:
    """
    Handles LIVE vs SIM data seamlessly.
    - LIVE: uses get_average_rsrp_3s()
    - SIM: uses distance to nearest UE beacon to generate plausible RSRP/UEs
    Auto-switches between them depending on availability.
    """
    def __init__(self):
        self.mode = "LIVE"  # or "SIM"
        self.last_rsrp: Optional[float] = None
        self.last_ues: int = 0
        self.sim_target: Optional[Tuple[float,float]] = None
        self.current_pos: Optional[Tuple[float,float]] = None

    def set_target(self, lat: float, lon: float):
        self.sim_target = (lat, lon)

    def set_current(self, lat: float, lon: float):
        self.current_pos = (lat, lon)

    def _try_live_once(self) -> Tuple[Optional[float], Optional[int]]:
        res = get_average_rsrp_3s()
        if not res:
            return None, None
        vals = [v for v in res.values() if v != float('-inf')]
        if not vals:
            return None, 0
        return float(sum(vals)/len(vals)), int(len(vals))

    def _live_fetch(self) -> Tuple[Optional[float], Optional[int]]:
        last_err = None
        for t in range(RSRP_RETRIES + 1):
            try:
                avg, ues = self._try_live_once()
                if avg is not None:
                    return avg, ues
            except Exception as e:
                last_err = e
                print(f"[LIVE] Error collecting data: {e}")
            if t < RSRP_RETRIES:
                time.sleep(RSRP_RETRY_WAIT)
        if last_err:
            print(f"[LIVE] Unavailable after retries: {last_err}")
        else:
            print(f"[LIVE] Unavailable: No data")
        return None, None

    def _sim_fetch(self) -> Tuple[float, int]:
        """
        RSRP improves and UE count increases as we get closer to the nearest UE beacon
        (or to the explicit target if set).
        """
        if self.current_pos is None:
            return RSRP_FAIL_VALUE, 0
        lat, lon = self.current_pos

        # choose reference: nearest beacon or explicit target
        ref_lat, ref_lon = None, None
        if self.sim_target is not None:
            ref_lat, ref_lon = self.sim_target
        else:
            # nearest UE beacon
            best_d = float('inf')
            for b_lat, b_lon in SIM_UE_BEACONS:
                d = haversine_m(lat, lon, b_lat, b_lon)
                if d < best_d:
                    best_d = d
                    ref_lat, ref_lon = b_lat, b_lon

        d = haversine_m(lat, lon, ref_lat, ref_lon)
        x = np.clip(1.0 - (d / SIM_EFFECT_RADIUS_M), 0.0, 1.0)
        rsrp = SIM_MIN_RSRP_FAR + x * (SIM_MAX_RSRP_AT_TARGET - SIM_MIN_RSRP_FAR)

        # UE count heuristic: more UEs as we get closer to *either* nearest beacon or cluster center
        # Use proximity to *all* beacons for a nicer curve.
        inv_d_sum = 0.0
        for b_lat, b_lon in SIM_UE_BEACONS:
            dist = haversine_m(lat, lon, b_lat, b_lon)
            inv_d_sum += 1.0 / max(dist, 1.0)

        inv_d_norm = inv_d_sum / len(SIM_UE_BEACONS)  # larger when closer overall
        inv_d_norm = np.clip(inv_d_norm / (1.0 / 10.0), 0.0, 1.0)  # normalize with ~10m ref
        ues = SIM_MIN_UES_FAR + inv_d_norm * (SIM_MAX_UES_AT_TARGET - SIM_MIN_UES_FAR)
        ues = int(round(ues))

        return float(rsrp), int(ues)

    def get(self) -> Tuple[float, int, str]:
        """
        Returns (rsrp, ues, source_string).
        Auto-switch logic:
          - Try LIVE if currently LIVE
          - If LIVE fails and AUTO_SWITCH_TO_SIM -> SIM
          - If SIM and AUTO_SWITCH_BACK_TO_LIVE -> occasionally ping LIVE and switch back on success
        """
        # Try LIVE
        if self.mode == "LIVE":
            avg, ues = self._live_fetch()
            if avg is not None:
                self.last_rsrp, self.last_ues = avg, ues
                return avg, ues, "LIVE"
            if AUTO_SWITCH_TO_SIM:
                print("[DATASRC] Switching to SIM (LIVE unavailable).")
                self.mode = "SIM"

        # SIM mode
        rsrp, ues = self._sim_fetch()
        self.last_rsrp, self.last_ues = rsrp, ues

        # Opportunistic probe back to LIVE
        if AUTO_SWITCH_BACK_TO_LIVE:
            avg, ues_live = self._live_fetch()
            if avg is not None:
                print("[DATASRC] LIVE recovered. Switching back to LIVE.")
                self.mode = "LIVE"
                self.last_rsrp, self.last_ues = avg, ues_live
                return avg, ues_live, "LIVE"

        return rsrp, ues, "SIM"

DATA = DataSource()
# ==============================================================================


# =============================== OPTIMIZER =====================================
class PolygonDroneOptimizer:
    def __init__(self, initial_corners, graph, dashboard=None):
        self.initial_corners = initial_corners       # [(lng,lat), ...]
        self.graph = graph
        self.dashboard = dashboard

        self.iteration = 0
        self.current_position: Optional[Tuple[float,float]] = None  # (lat, lon)
        self.current_target:   Optional[Tuple[float,float]] = None  # (lat, lon)
        self.full_path: List[Tuple[float,float,str]] = []
        self.trajectory: List[Tuple[float,float,float,int,str,int]] = []
        self.best_rsrp = -1e9
        self.best_ues = 0

    def _label_to_target_idx(self, label: str) -> int:
        if isinstance(label, str) and label.startswith("corner_"):
            return int(label.split("_")[1]) + 1
        return 1

    def _signal_quality(self, rsrp: float) -> float:
        if not np.isfinite(rsrp):
            return 0.0
        return float(np.clip((rsrp + 120.0) / 60.0 * 100.0, 0.0, 100.0))

    def _publish_and_dash(self, lat, lon, label, leg_distance_m=0.0, rsrp=None, ues=None, source="LIVE"):
        if rsrp is None:
            rsrp = DATA.last_rsrp if DATA.last_rsrp is not None else RSRP_FAIL_VALUE
        if ues is None:
            ues = DATA.last_ues if DATA.last_ues is not None else 0
        sigq = self._signal_quality(rsrp)
        tgt_idx = self._label_to_target_idx(label)

        mqtt_publish_flat(lat, lon, self.iteration, rsrp, leg_distance_m, sigq, tgt_idx)

        if self.dashboard:
            dist_to_tgt = 0.0
            if self.current_target is not None:
                dist_to_tgt = haversine_m(lat, lon, self.current_target[0], self.current_target[1])
            self.dashboard.update_position(self.iteration, lat, lon)
            self.dashboard.update_measurements(rsrp, sigq, ues, source)
            self.dashboard.update_distance_to_target(dist_to_tgt)

        improved = False
        if np.isfinite(rsrp) and rsrp > self.best_rsrp:
            self.best_rsrp = rsrp
            improved = True
        if ues > self.best_ues:
            self.best_ues = ues
            improved = True
        if improved and self.dashboard:
            self.dashboard.update_bests(self.best_rsrp, self.best_ues)

    def _measure_here(self, lat, lon, label="measurement"):
        DATA.set_current(lat, lon)
        rsrp, ues, source = DATA.get()
        self._publish_and_dash(lat, lon, label, leg_distance_m=0.0, rsrp=rsrp, ues=ues, source=source)
        self.trajectory.append((lat, lon, rsrp, ues, label, self.iteration))
        print(f"[{source}] ({lat:.6f},{lon:.6f}) -> RSRP={rsrp:.1f} dBm, UEs={ues}")
        return rsrp, ues, source

    def move_step_by_step(self, target_lat, target_lon, label="movement"):
        self.current_target = (target_lat, target_lon)
        DATA.set_target(target_lat, target_lon)

        if self.current_position is None:
            print(f"Teleporting to start: ({target_lat:.6f}, {target_lon:.6f})")
            self.current_position = (target_lat, target_lon)
            self.full_path.append((target_lat, target_lon, label))
            self._measure_here(target_lat, target_lon, label)
            time.sleep(0.15)
            return

        cur_lat, cur_lon = self.current_position
        path_coords, path_len = get_shortest_path_between_points(self.graph, cur_lat, cur_lon, target_lat, target_lon)

        if path_coords and len(path_coords) > 1:
            total_m = float(path_len * 111111.0) if path_len else 0.0
            print(f"Moving via {len(path_coords)} waypoints to {label} (~{total_m:.1f} m)")
            for i, (lat, lon) in enumerate(path_coords):
                if i == 0:
                    continue
                DATA.set_current(lat, lon)
                rsrp, ues, source = DATA.get()  # dynamic update at waypoint
                self._publish_and_dash(lat, lon, label, leg_distance_m=total_m, rsrp=rsrp, ues=ues, source=source)
                self.full_path.append((lat, lon, "travel_to_" + label if i < len(path_coords)-1 else label))
                time.sleep(0.25)
            self.current_position = (target_lat, target_lon)
            print(f"Reached target after traveling {total_m:.1f} m")
        else:
            print("No path found, moving directly")
            self.current_position = (target_lat, target_lon)
            self.full_path.append((target_lat, target_lon, label))
            self._measure_here(target_lat, target_lon, label)
            time.sleep(0.05)

    def polygon_bisection_search_real(self, min_area_threshold=100):
        print("Starting REAL 4-corner polygon bisection search...")

        # Publish corners first (and measure once so ROS2 + UI get initial values)
        for idx, (lng, lat) in enumerate(self.initial_corners):
            self.iteration = 0
            self.current_target = (lat, lng)
            DATA.set_target(lat, lng)
            self._measure_here(lat, lng, f"corner_{idx}")

        input("Press Enter to start the optimization (Ctrl+C to cancel)...")

        current = self.initial_corners.copy()
        measured_cache = {}  # {(lat,lon): (rsrp, ues)}
        labels = ["corner_0", "corner_1", "corner_2", "corner_3"]

        while True:
            self.iteration += 1
            print(f"\n{'='*60}\nITERATION {self.iteration}\n{'='*60}")
            area = get_polygon_area_meters(current)
            print(f"Current polygon area: {area:.1f} sq m")
            if area < min_area_threshold:
                print("Polygon small enough. Stopping.")
                break

            print("Current polygon corners:")
            for i, (lng, lat) in enumerate(current):
                print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")

            # measure four corners (first iter all; subsequent only two new)
            corner_vals: List[Tuple[float,int]] = []
            for i, (lng, lat) in enumerate(current):
                key = (round(lat, 8), round(lng, 8))
                need_measure = (self.iteration == 1) or (i >= 2) or (key not in measured_cache)
                if need_measure:
                    self.move_step_by_step(lat, lng, labels[i])
                    rsrp_val, ue_cnt, _ = self._measure_here(lat, lng, labels[i])
                    measured_cache[key] = (rsrp_val, ue_cnt)
                    corner_vals.append((rsrp_val, ue_cnt))
                else:
                    rsrp_cached, ues_cached = measured_cache[key]
                    print(f"  {labels[i]} cached -> RSRP {rsrp_cached:.1f} dBm, UEs {ues_cached}")
                    self._publish_and_dash(lat, lng, labels[i], leg_distance_m=0.0,
                                           rsrp=rsrp_cached, ues=ues_cached,
                                           source=("LIVE" if DATA.mode == "LIVE" else "SIM"))
                    corner_vals.append((rsrp_cached, ues_cached))

            # Edge scoring using normalized RSRP+UE
            rsrps = [v[0] for v in corner_vals]
            uelist = [v[1] for v in corner_vals]
            max_ue = max(uelist) if uelist else 1

            edge_scores = []
            for i in range(4):
                j = (i + 1) % 4
                s_i = WEIGHT_RSRP * np.clip((rsrps[i] + 120.0)/60.0, 0.0, 1.0) + WEIGHT_UE * (uelist[i]/max(max_ue,1))
                s_j = WEIGHT_RSRP * np.clip((rsrps[j] + 120.0)/60.0, 0.0, 1.0) + WEIGHT_UE * (uelist[j]/max(max_ue,1))
                score = float(s_i + s_j)
                edge_scores.append((score, i, f"edge_{i}_{j}"))
                print(f"  Edge {i}-{j}: RSRP({rsrps[i]:.1f},{rsrps[j]:.1f}) "
                      f"UEs({uelist[i]},{uelist[j]}) -> score {score:.3f}")

            best_score, best_idx, best_name = max(edge_scores, key=lambda x: x[0])
            print(f"*** BEST EDGE: {best_name} (score {best_score:.3f})")
            current = cut_polygon_in_half(current, best_idx)

        # Final center
        center_lng = sum(lng for lng, lat in current) / 4
        center_lat = sum(lat for lng, lat in current) / 4
        self.move_step_by_step(center_lat, center_lng, "final_center")
        rsrp_final, ue_final, _ = self._measure_here(center_lat, center_lng, "final_center")

        print(f"\n{'='*60}\nOPTIMIZATION COMPLETE\n{'='*60}")
        print(f"Final optimum: ({center_lat:.6f}, {center_lng:.6f})  RSRP={rsrp_final:.1f} dBm, UEs={ue_final}")
        print(f"Final polygon area: {get_polygon_area_meters(current):.1f} sq m")
        return center_lat, center_lng, rsrp_final, ue_final, current
# ==============================================================================


# =============================== PLOTTING / SUMMARY ============================
def plot_polygon_trajectory(buildings, graph, traj, path, final_point, initial_corners, final_corners):
    fig, ax = plt.subplots(1, 1, figsize=(15, 12))

    # bounds
    all_lats = [lat for lng, lat in initial_corners] + [p[0] for p in traj]
    all_lngs = [lng for lng, lat in initial_corners] + [p[1] for p in traj]
    min_lat, max_lat = (min(all_lats), max(all_lats)) if all_lats else (0,1)
    min_lng, max_lng = (min(all_lngs), max(all_lngs)) if all_lngs else (0,1)

    # buildings
    for b in buildings:
        x, y = b.exterior.xy
        bx0, by0, bx1, by1 = b.bounds
        if (bx0 <= max_lng and bx1 >= min_lng and by0 <= max_lat and by1 >= min_lat):
            ax.plot(x, y, 'b-', linewidth=0.5, alpha=0.6)
            ax.fill(x, y, 'lightblue', alpha=0.2)

    # initial polygon
    init_x = [lng for lng, lat in initial_corners] + [initial_corners[0][0]]
    init_y = [lat for lng, lat in initial_corners] + [initial_corners[0][1]]
    ax.plot(init_x, init_y, 'r--', linewidth=2, alpha=0.8, label='Initial Polygon')

    # path
    if path:
        path_lat = [p[0] for p in path]
        path_lng = [p[1] for p in path]
        ax.plot(path_lng, path_lat, 'purple', linewidth=2, alpha=0.7, label='Flight Path', zorder=3)

    # measured points
    if traj:
        lats  = [p[0] for p in traj]
        lngs  = [p[1] for p in traj]
        rsrps = [p[2] for p in traj]
        finite = [m for m in rsrps if m != RSRP_FAIL_VALUE and np.isfinite(m)]
        vmin, vmax = (min(finite), max(finite)) if finite else (0, 1)
        sc = ax.scatter(lngs, lats, c=rsrps, cmap='RdYlGn', s=120, alpha=0.9,
                        edgecolors='black', linewidth=2, vmin=vmin, vmax=vmax, zorder=5)
        cbar = plt.colorbar(sc, ax=ax)
        cbar.set_label('RSRP (dBm)', rotation=270, labelpad=20)

        for i, (lat, lng, metric, ues, reason, it) in enumerate(traj):
            ax.annotate(f'{i+1}', (lng, lat),
                        xytext=(0, 20), textcoords='offset points',
                        fontsize=10, ha='center', color='black', fontweight='bold',
                        bbox=dict(boxstyle='circle,pad=0.3', facecolor='white',
                                  edgecolor='black', alpha=0.9),
                        zorder=8)

    if final_point:
        flat, flon, fval = final_point
        ax.plot(flon, flat, 'k^', markersize=25, label=f'Final\n{fval:.1f} dBm', zorder=7)

    ax.set_aspect('equal')
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title('Real Drone 4-Corner Polygon Bisection Search')
    pad = 0.0005
    ax.set_xlim(min_lng - pad, max_lng + pad)
    ax.set_ylim(min_lat - pad, max_lat + pad)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.savefig("trajectory.png")
    try:
        import tikzplotlib
        tikzplotlib.save("trajectory.tex")
    except Exception:
        pass
    plt.show()

def print_trajectory_summary(traj, path):
    print("\n================ TRAJECTORY SUMMARY ================")
    print(f"Total measurements: {len(traj)}")
    print(f"Total path points: {len(path)}")
# ==============================================================================


# ================================== MAIN =======================================
if __name__ == "__main__":
    dashboard = DroneDashboard()

    def worker():
        try:
            initial_corners = create_custom_polygon()
            print("Initial corners (clockwise):")
            for i, (lng, lat) in enumerate(initial_corners):
                print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")

            # Important: load BOTH caches (buildings + graph)
            buildings, grid, G, key_to_node = load_graph()

            optimizer = PolygonDroneOptimizer(initial_corners, G, dashboard=dashboard)

            final_lat, final_lon, final_metric, final_ues, final_corners = optimizer.polygon_bisection_search_real()

            print_trajectory_summary(optimizer.trajectory, optimizer.full_path)
            plot_polygon_trajectory(buildings, G, optimizer.trajectory, optimizer.full_path,
                                    (final_lat, final_lon, final_metric), initial_corners, final_corners)
        except Exception as e:
            print(f"[FATAL] {e}")

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    dashboard.start()
