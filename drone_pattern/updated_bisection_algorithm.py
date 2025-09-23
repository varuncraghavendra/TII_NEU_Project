#!/usr/bin/env python3
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
import numpy as np
import networkx as nx
import pickle
import os
from hashlib import md5
import time
import tikzplotlib
import threading
import tkinter as tk
import paho.mqtt.client as mqtt
import json
from typing import Tuple, List, Optional

# ---------- Real data function (must exist in your env) ----------
from influx_hook import get_average_rsrp_3s  # your function

# ===================== CONFIG =====================
BROKER = "broker.hivemq.com"
PORT   = 1883
TOPIC  = "colosseum/update"
DRONE_ID = 1

# Robustness knobs for RSRP fetch
RSRP_RETRIES = 1          # additional tries after first failure
RSRP_RETRY_WAIT = 2.0     # seconds between retries
RSRP_FAIL_VALUE = -999.0  # sentinel so ROS2 never gets None
ENABLE_FALLBACK_SIM = True  # simulate metrics when real data is unavailable

# Scoring weights (tune to bias RSRP vs UEs)
WEIGHT_RSRP = 0.7
WEIGHT_UE   = 0.3

# Area / polygon (clockwise)
CUSTOM_BOUNDS = {
    'top_left'    : [42.34138538637752, -71.08300209045412],
    'top_right'   : [42.340314805239075, -71.08198285102846],
    'bottom_right': [42.337903948379996, -71.08519077301027],
    'bottom_left' : [42.33866528158443, -71.08609199523927]
}

# Cache keys match your original style
CENTER_LAT = 42.33894284868896
CENTER_LNG = -71.08613491058351
RADIUS_METERS = 1000
GRID_SPACING_METERS = 5

# Exact cache filenames you said you have:
EXACT_BUILDINGS_CACHE = "buildings_cache_e7ba8105.pkl"
EXACT_GRAPH_CACHE     = "graph_cache_91fc2d6b.pkl"
# ===================================================


# ===================== MQTT =======================
_mqtt = mqtt.Client(protocol=mqtt.MQTTv311)
try:
    _mqtt.connect(BROKER, PORT, 60)
    _mqtt.loop_start()
except Exception as e:
    print(f"[WARN] MQTT connect failed: {e}")

def publish_flat(lat, lon, iteration, rsrp, distance, signal_quality, target_ue):
    """
    EXACT schema from your first program; no None values:
    {
      "drone_id": DRONE_ID, "lat": <float>, "lon": <float>,
      "rsrp": <float>, "distance": <float>, "signal_quality": <float>,
      "iteration": <int>, "target_ue": <int>
    }
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
# ==================================================


# ===================== DASHBOARD ==================
class DroneDashboard:
    def __init__(self, title="Drone Live Dashboard"):
        self.root = tk.Tk()
        self.root.title(title)
        self.root.geometry("640x380")
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
            "Best-so-far UEs (#)"
        ]
        for i, f in enumerate(fields):
            tk.Label(self.root, text=f + ":").grid(row=i, column=0, sticky="w", padx=10, pady=6)
            self.labels[f] = tk.Label(self.root, text="---", width=40, anchor="w")
            self.labels[f].grid(row=i, column=1, sticky="w", padx=10)
        self.root.grid_columnconfigure(0, minsize=260)
        self.root.grid_columnconfigure(1, minsize=360)
        self._lock = threading.Lock()

    def update_position(self, iteration, lat, lon):
        with self._lock:
            self.labels["Iteration"].config(text=str(iteration))
            self.labels["Current Lat"].config(text=f"{lat:.6f}")
            self.labels["Current Lon"].config(text=f"{lon:.6f}")
            self.root.update_idletasks()

    def update_measurements(self, rsrp, sigq, ues):
        with self._lock:
            self.labels["Current RSRP (dBm)"].config(text=f"{rsrp:.1f}")
            self.labels["Signal Quality (%)"].config(text=f"{sigq:.1f}")
            self.labels["Connected UEs (#)"].config(text=f"{int(ues)}")
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
# ==================================================


# ===================== GEO UTILS ==================
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
    Try exact filenames first. If missing, fall back to md5-named cache files.
    """
    # exact files you provided
    buildings_data = try_load(EXACT_BUILDINGS_CACHE)
    graph_data     = try_load(EXACT_GRAPH_CACHE)

    if buildings_data is not None and graph_data is not None:
        print(f"[CACHE] Loaded buildings from {EXACT_BUILDINGS_CACHE}")
        print(f"[CACHE] Loaded graph from {EXACT_GRAPH_CACHE}")
        # expect graph_data to be a dict with keys 'grid','graph','key_to_node'
        if not all(k in graph_data for k in ('grid','graph','key_to_node')):
            raise RuntimeError(f"{EXACT_GRAPH_CACHE} missing expected keys: grid/graph/key_to_node")
        return buildings_data, graph_data['grid'], graph_data['graph'], graph_data['key_to_node']

    # fallback to md5 filenames (original pattern)
    cache_file = get_cache_filename("graph")
    buildings_cache = get_cache_filename("buildings")
    print(f"[CACHE] Falling back to md5 caches:")
    print(f"        graph -> {cache_file}")
    print(f"        buildings -> {buildings_cache}")

    if not (os.path.isfile(cache_file) and os.path.isfile(buildings_cache)):
        raise FileNotFoundError(
            f"Cache files not found. Tried: "
            f"{EXACT_BUILDINGS_CACHE}, {EXACT_GRAPH_CACHE}, {buildings_cache}, {cache_file}"
        )

    with open(cache_file, 'rb') as f:
        cached_data = pickle.load(f)
    with open(buildings_cache, 'rb') as f:
        buildings = pickle.load(f)

    if not all(k in cached_data for k in ('grid','graph','key_to_node')):
        raise RuntimeError(f"{cache_file} missing expected keys: grid/graph/key_to_node")

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
# ==================================================


# ============== RSRP/UE (robust + last-good + sim) =======
_LAST_KNOWN_RSRP: Optional[float] = None
_LAST_KNOWN_UES:  int = 0

def signal_quality_from_rsrp(rsrp):
    if not np.isfinite(rsrp):
        return 0.0
    return float(np.clip((rsrp + 120.0) / 60.0 * 100.0, 0.0, 100.0))

def normalize_rsrp(rsrp):
    if not np.isfinite(rsrp):
        return 0.0
    return float(np.clip((rsrp + 120.0) / 60.0, 0.0, 1.0))

def normalize_ue(ue, max_ue):
    if max_ue <= 0:
        return 0.0
    return float(np.clip(ue / max_ue, 0.0, 1.0))

def try_fetch_rsrp_once() -> Tuple[Optional[float], Optional[int], Optional[dict]]:
    """
    One shot: returns (avg_rsrp, ue_count, raw_dict) or (None,None,None) if unusable.
    """
    res = get_average_rsrp_3s()
    if not res:
        return None, None, None
    vals = [v for v in res.values() if v != float('-inf')]
    if not vals:
        return None, None, res
    avg = float(sum(vals) / len(vals))
    return avg, int(len(vals)), res

def rsrp_retry_fetch() -> Tuple[float, int, Optional[dict], bool]:
    """
    Returns (rsrp, ues, raw_dict, from_simulator_flag).
    - Tries live a few times.
    - If unavailable and ENABLE_FALLBACK_SIM: simulator value (flag=True).
    - Else: last-known-good if available.
    - Else: sentinel (-999.0, 0).
    """
    global _LAST_KNOWN_RSRP, _LAST_KNOWN_UES
    last_err = "No data"
    for t in range(RSRP_RETRIES + 1):
        try:
            avg, ues, raw = try_fetch_rsrp_once()
            if avg is not None:
                _LAST_KNOWN_RSRP = avg
                _LAST_KNOWN_UES = ues
                return avg, ues, raw, False
        except Exception as e:
            last_err = str(e)
            print(f"[RSRP] Error collecting data: {e}")
        if t < RSRP_RETRIES:
            time.sleep(RSRP_RETRY_WAIT)

    # Fallback simulator or last-known-good or sentinel
    if ENABLE_FALLBACK_SIM and _sim_context is not None:
        rsrp_sim, ues_sim = simulate_metrics(_sim_context['current_lat'],
                                             _sim_context['current_lon'],
                                             _sim_context['target_lat'],
                                             _sim_context['target_lon'])
        _LAST_KNOWN_RSRP = rsrp_sim
        _LAST_KNOWN_UES  = ues_sim
        print(f"[RSRP] Using simulator: RSRP {rsrp_sim:.1f} dBm, UEs {ues_sim}")
        return rsrp_sim, ues_sim, None, True

    if _LAST_KNOWN_RSRP is not None:
        print(f"[RSRP] Using last known: RSRP {_LAST_KNOWN_RSRP:.1f} dBm, UEs {_LAST_KNOWN_UES}")
        return _LAST_KNOWN_RSRP, _LAST_KNOWN_UES, None, False

    print(f"[RSRP] Unavailable ({last_err}); using sentinel {RSRP_FAIL_VALUE}.")
    return float(RSRP_FAIL_VALUE), 0, None, False

# --------- Simple fallback simulator (distance-based) ----------
# Makes values improve as drone approaches the target waypoint.
# Tunables:
SIM_MAX_RSRP_AT_TARGET = -65.0   # dBm at target
SIM_MIN_RSRP_FAR       = -110.0  # dBm when far
SIM_MAX_UES_AT_TARGET  = 6       # UE count when close
SIM_MIN_UES_FAR        = 0       # UE count when far
SIM_EFFECT_RADIUS_M    = 150.0   # meters for decay

_sim_context = None  # set by optimizer to provide current/target positions

def sim_distance_m(lat1, lon1, lat2, lon2) -> float:
    # rough meters using lat/lon deltas
    dy = degrees_to_meters(lat2 - lat1)
    dx = degrees_to_meters(lon2 - lon1, is_longitude=True, center_lat=(lat1+lat2)/2)
    return float(np.hypot(dx, dy))

def simulate_metrics(cur_lat, cur_lon, tgt_lat, tgt_lon) -> Tuple[float,int]:
    d = sim_distance_m(cur_lat, cur_lon, tgt_lat, tgt_lon)
    x = np.clip(1.0 - (d / SIM_EFFECT_RADIUS_M), 0.0, 1.0)
    rsrp = SIM_MIN_RSRP_FAR + x * (SIM_MAX_RSRP_AT_TARGET - SIM_MIN_RSRP_FAR)
    ues  = int(round(SIM_MIN_UES_FAR + x * (SIM_MAX_UES_AT_TARGET - SIM_MIN_UES_FAR)))
    return float(rsrp), int(ues)
# ==================================================


# ===================== OPTIMIZER ====================
class PolygonDroneOptimizer:
    def __init__(self, initial_corners, graph, dashboard=None):
        self.initial_corners = initial_corners       # [(lng,lat), ...] clockwise
        self.graph = graph
        self.dashboard = dashboard

        self.iteration = 0
        self.current_position = None                 # (lat, lon)
        self.current_target   = None                 # (lat, lon)
        self.full_path: List[Tuple[float,float,str]] = []
        self.trajectory: List[Tuple[float,float,float,int,str,int]] = []
        self.best_rsrp = -1e9
        self.best_ues = 0

    def corner_index_from_label(self, label: str) -> int:
        if isinstance(label, str) and label.startswith("corner_"):
            return int(label.split("_")[1]) + 1
        return 1  # default

    def publish_and_update_dash(self, lat, lon, label, leg_distance_m=0.0, rsrp=None, ues=None):
        global _LAST_KNOWN_RSRP, _LAST_KNOWN_UES
        if rsrp is None:
            rsrp = _LAST_KNOWN_RSRP if _LAST_KNOWN_RSRP is not None else RSRP_FAIL_VALUE
        if ues is None:
            ues = _LAST_KNOWN_UES if _LAST_KNOWN_UES is not None else 0

        sigq = signal_quality_from_rsrp(rsrp)
        target_ue_idx = self.corner_index_from_label(label)
        publish_flat(lat, lon, self.iteration, rsrp, leg_distance_m, sigq, target_ue_idx)

        if self.dashboard:
            # Distance to target for UI
            dist_to_target = 0.0
            if self.current_target is not None:
                dist_to_target = sim_distance_m(lat, lon, self.current_target[0], self.current_target[1])
            self.dashboard.update_position(self.iteration, lat, lon)
            self.dashboard.update_measurements(rsrp, sigq, ues)
            self.dashboard.update_distance_to_target(dist_to_target)

        # best-so-far
        improved = False
        if np.isfinite(rsrp) and rsrp > self.best_rsrp:
            self.best_rsrp = rsrp
            improved = True
        if ues > self.best_ues:
            self.best_ues = ues
            improved = True
        if improved and self.dashboard:
            self.dashboard.update_bests(self.best_rsrp, self.best_ues)

    def measure_here(self, lat, lon, label="measurement"):
        """
        Poll real data; on failure optionally simulate; publish + UI; record trajectory.
        """
        # register sim context for fallback
        global _sim_context
        _sim_context = {
            'current_lat': lat,
            'current_lon': lon,
            'target_lat': self.current_target[0] if self.current_target else lat,
            'target_lon': self.current_target[1] if self.current_target else lon
        }

        rsrp_val, ue_cnt, raw, from_sim = rsrp_retry_fetch()
        self.publish_and_update_dash(lat, lon, label, leg_distance_m=0.0, rsrp=rsrp_val, ues=ue_cnt)
        self.trajectory.append((lat, lon, rsrp_val, ue_cnt, label, self.iteration))
        source = "SIM" if from_sim else "LIVE"
        print(f"[{source}] Measured at ({lat:.6f},{lon:.6f}) -> RSRP={rsrp_val:.1f} dBm, UEs={ue_cnt}")
        return rsrp_val, ue_cnt

    def move_step_by_step(self, target_lat, target_lon, label="movement"):
        """
        Move along shortest path; at each waypoint:
          - set sim context (for fallback)
          - quick RSRP/UE poll for dynamic updates
          - publish + UI
        """
        # Set current target for UI/sim
        self.current_target = (target_lat, target_lon)

        if self.current_position is None:
            print(f"Teleporting to start: ({target_lat:.6f}, {target_lon:.6f})")
            self.current_position = (target_lat, target_lon)
            self.full_path.append((target_lat, target_lon, label))
            # initial publish with a quick poll
            self.measure_here(target_lat, target_lon, label)
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
                # update sim target context for waypoint
                global _sim_context
                _sim_context = {'current_lat': lat, 'current_lon': lon,
                                'target_lat': target_lat, 'target_lon': target_lon}
                # quick measurement at each waypoint to show dynamic change
                rsrp_val, ue_cnt, _, _ = rsrp_retry_fetch()
                self.publish_and_update_dash(lat, lon, label, leg_distance_m=total_m, rsrp=rsrp_val, ues=ue_cnt)
                self.full_path.append((lat, lon, "travel_to_" + label if i < len(path_coords)-1 else label))
                time.sleep(0.25)

            self.current_position = (target_lat, target_lon)
            print(f"Reached target after traveling {total_m:.1f} m")
        else:
            print("No path found, moving directly")
            self.current_position = (target_lat, target_lon)
            self.full_path.append((target_lat, target_lon, label))
            self.measure_here(target_lat, target_lon, label)
            time.sleep(0.05)

    # ----- search logic (bisection w/ RSRP+UE scoring) -----
    def polygon_bisection_search_real(self, min_area_threshold=100):
        print("Starting REAL 4-corner polygon bisection search...")

        # Publish the 4 corners once at the beginning (as normal updates) so ROS2 can parse them
        for idx, (lng, lat) in enumerate(self.initial_corners):
            self.iteration = 0
            self.current_target = (lat, lng)
            self.measure_here(lat, lng, f"corner_{idx}")

        input("Press Enter to start the optimization (Ctrl+C to cancel)...")

        current = self.initial_corners.copy()
        measured_cache = {}     # {(lat,lon) rounded: (rsrp, ues)}
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

            corner_vals = []  # list of (rsrp, ues)
            if self.iteration == 1:
                # measure all 4
                for i, (lng, lat) in enumerate(current):
                    key = (round(lat, 8), round(lng, 8))
                    if key in measured_cache:
                        rsrp_cached, ues_cached = measured_cache[key]
                        print(f"  {labels[i]} cached -> RSRP {rsrp_cached:.1f} dBm, UEs {ues_cached}")
                        corner_vals.append((rsrp_cached, ues_cached))
                        self.publish_and_update_dash(lat, lng, labels[i], leg_distance_m=0.0, rsrp=rsrp_cached, ues=ues_cached)
                    else:
                        self.move_step_by_step(lat, lng, labels[i])
                        rsrp_val, ue_cnt = self.measure_here(lat, lng, labels[i])
                        corner_vals.append((rsrp_val, ue_cnt))
                        measured_cache[key] = (rsrp_val, ue_cnt)
            else:
                # measure only new corners (2 and 3), reuse 0 and 1
                for i, (lng, lat) in enumerate(current):
                    key = (round(lat, 8), round(lng, 8))
                    if i < 2:
                        if key in measured_cache:
                            rsrp_cached, ues_cached = measured_cache[key]
                            print(f"  {labels[i]} cached (best-edge) -> RSRP {rsrp_cached:.1f}, UEs {ues_cached}")
                            corner_vals.append((rsrp_cached, ues_cached))
                            self.publish_and_update_dash(lat, lng, labels[i], leg_distance_m=0.0, rsrp=rsrp_cached, ues=ues_cached)
                        else:
                            self.move_step_by_step(lat, lng, labels[i])
                            rsrp_val, ue_cnt = self.measure_here(lat, lng, labels[i])
                            corner_vals.append((rsrp_val, ue_cnt))
                            measured_cache[key] = (rsrp_val, ue_cnt)
                    else:
                        if key in measured_cache:
                            rsrp_cached, ues_cached = measured_cache[key]
                            print(f"  {labels[i]} cached -> RSRP {rsrp_cached:.1f}, UEs {ues_cached}")
                            corner_vals.append((rsrp_cached, ues_cached))
                            self.publish_and_update_dash(lat, lng, labels[i], leg_distance_m=0.0, rsrp=rsrp_cached, ues=ues_cached)
                        else:
                            self.move_step_by_step(lat, lng, labels[i])
                            rsrp_val, ue_cnt = self.measure_here(lat, lng, labels[i])
                            corner_vals.append((rsrp_val, ue_cnt))
                            measured_cache[key] = (rsrp_val, ue_cnt)

            # ===== edge scoring with RSRP + UEs =====
            rsrps = [v[0] for v in corner_vals]
            uelist = [v[1] for v in corner_vals]
            max_ue = max(uelist) if uelist else 1

            edge_scores = []
            for i in range(4):
                j = (i + 1) % 4
                s_i = WEIGHT_RSRP * normalize_rsrp(rsrps[i]) + WEIGHT_UE * normalize_ue(uelist[i], max_ue)
                s_j = WEIGHT_RSRP * normalize_rsrp(rsrps[j]) + WEIGHT_UE * normalize_ue(uelist[j], max_ue)
                score = s_i + s_j
                edge_scores.append((score, i, f"edge_{i}_{j}"))
                print(f"  Edge {i}-{j}: RSRP({rsrps[i]:.1f},{rsrps[j]:.1f}) UEs({uelist[i]},{uelist[j]}) -> score {score:.3f}")

            best_score, best_idx, best_name = max(edge_scores, key=lambda x: x[0])
            print(f"*** BEST EDGE: {best_name} (score {best_score:.3f})")

            current = cut_polygon_in_half(current, best_idx)

        # final center
        center_lng = sum(lng for lng, lat in current) / 4
        center_lat = sum(lat for lng, lat in current) / 4
        self.move_step_by_step(center_lat, center_lng, "final_center")
        rsrp_final, ue_final = self.measure_here(center_lat, center_lng, "final_center")

        print(f"\n{'='*60}\nOPTIMIZATION COMPLETE\n{'='*60}")
        print(f"Final optimum: ({center_lat:.6f}, {center_lng:.6f})  "
              f"RSRP={rsrp_final:.1f} dBm, UEs={ue_final}")
        print(f"Final polygon area: {get_polygon_area_meters(current):.1f} sq m")
        return center_lat, center_lng, rsrp_final, ue_final, current
# ==================================================


# ================== PLOTTING / SUMMARY ==============
def plot_polygon_trajectory(buildings, graph, traj, path, final_point, initial_corners, final_corners):
    fig, ax = plt.subplots(1, 1, figsize=(15, 12))

    # bounds
    all_lats = [lat for lng, lat in initial_corners] + [p[0] for p in traj]
    all_lngs = [lng for lng, lat in initial_corners] + [p[1] for p in traj]
    min_lat, max_lat = min(all_lats), max(all_lats)
    min_lng, max_lng = min(all_lngs), max(all_lngs)

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
    tikzplotlib.save("trajectory.tex")
    plt.show()

def print_trajectory_summary(traj, path):
    print("\n================ TRAJECTORY SUMMARY ================")
    print(f"Total measurements: {len(traj)}")
    print(f"Total path points: {len(path)}")
# =====================================================


# ======================= MAIN ========================
if __name__ == "__main__":
    dashboard = DroneDashboard()

    def worker():
        try:
            initial_corners = create_custom_polygon()
            print("Initial corners (clockwise):")
            for i, (lng, lat) in enumerate(initial_corners):
                print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")

            # CRUCIAL: load BOTH caches (buildings + graph), trying exact filenames first
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
