#!/usr/bin/env python3
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
import numpy as np
import networkx as nx
import pickle
from hashlib import md5
import time
import tikzplotlib
import threading
import tkinter as tk
import paho.mqtt.client as mqtt
import json

# If you still want to use influx_hook for real RSRP:
from influx_hook import get_average_rsrp_3s

# ===================== CONFIG =====================
BROKER = "broker.hivemq.com"
PORT   = 1883
TOPIC  = "colosseum/update"
DRONE_ID = 1

# Robustness knobs for RSRP fetch
RSRP_RETRIES = 1          # additional tries after first failure
RSRP_RETRY_WAIT = 3.0     # seconds between retries
RSRP_FAIL_VALUE = -999.0  # sentinel so ROS2 float() never sees None

# Area / polygon
CUSTOM_BOUNDS = {
    'top_left'    : [42.34138538637752, -71.08300209045412],
    'top_right'   : [42.340314805239075, -71.08198285102846],
    'bottom_right': [42.337903948379996, -71.08519077301027],
    'bottom_left' : [42.33866528158443, -71.08609199523927]
}

CENTER_LAT = 42.33894284868896
CENTER_LNG = -71.08613491058351
RADIUS_METERS = 1000
GRID_SPACING_METERS = 5
# ===================================================


# ===================== MQTT =======================
_mqtt = mqtt.Client(protocol=mqtt.MQTTv311)
try:
    _mqtt.connect(BROKER, PORT, 60)
    _mqtt.loop_start()
except Exception as e:
    print(f"[WARN] MQTT connect failed: {e}")

def _publish(lat, lon, iteration, rsrp, distance, signal_quality, target_ue):
    """
    EXACT schema as your first program:
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
        self.root.geometry("520x260")
        self.labels = {}
        fields = ["Iteration", "Current Lat", "Current Lon", "Current RSRP (dBm)",
                  "Reached Target After (m)"]
        for i, f in enumerate(fields):
            tk.Label(self.root, text=f + ":").grid(row=i, column=0, sticky="w", padx=10, pady=6)
            self.labels[f] = tk.Label(self.root, text="---", width=40, anchor="w")
            self.labels[f].grid(row=i, column=1, sticky="w", padx=10)
        self.root.grid_columnconfigure(0, minsize=200)
        self.root.grid_columnconfigure(1, minsize=300)
        self._lock = threading.Lock()

    def update_position(self, iteration, lat, lon):
        with self._lock:
            self.labels["Iteration"].config(text=str(iteration))
            self.labels["Current Lat"].config(text=f"{lat:.6f}")
            self.labels["Current Lon"].config(text=f"{lon:.6f}")
            self.root.update_idletasks()

    def update_rsrp(self, rsrp):
        with self._lock:
            self.labels["Current RSRP (dBm)"].config(text=f"{rsrp:.1f}" if np.isfinite(rsrp) else f"{RSRP_FAIL_VALUE:.1f}")
            self.root.update_idletasks()

    def update_travel(self, meters):
        with self._lock:
            self.labels["Reached Target After (m)"].config(text=f"{meters:.1f}")
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
    # (lng, lat) order
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

def load_graph():
    cache_file = get_cache_filename("graph")
    buildings_cache = get_cache_filename("buildings")
    print(f"Loading graph: {cache_file}")
    with open(cache_file, 'rb') as f:
        cached_data = pickle.load(f)
    print(f"Loading buildings: {buildings_cache}")
    with open(buildings_cache, 'rb') as f:
        buildings = pickle.load(f)
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


# ============== RSRP (robust real fetch) ===========
def rsrp_retry_fetch():
    """
    Try to fetch real RSRP; on failure never return None.
    On success: (rsrp_value, ue_count)
    On failure: (RSRP_FAIL_VALUE, 0)
    """
    tries = RSRP_RETRIES + 1
    last_err = None
    for t in range(tries):
        try:
            res = get_average_rsrp_3s()
            if not res:
                last_err = "No data"
            else:
                vals = [v for v in res.values() if v != float('-inf')]
                if vals:
                    avg = sum(vals)/len(vals)
                    return float(avg), int(len(vals))
                else:
                    last_err = "All -inf"
        except Exception as e:
            last_err = str(e)
            print(f"Error collecting RSRP: {e}")
        if t < tries-1:
            time.sleep(RSRP_RETRY_WAIT)
    print(f"[WARN] RSRP unavailable ({last_err}); using {RSRP_FAIL_VALUE}.")
    return float(RSRP_FAIL_VALUE), 0

def quality_from_rsrp(rsrp):
    """
    Simple placeholder mapping to 0..100. You can replace with your actual mapping.
    If RSRP is sentinel (failure), return 0.0.
    """
    if rsrp == RSRP_FAIL_VALUE or not np.isfinite(rsrp):
        return 0.0
    # Map -120..-60 dBm to 0..100
    return float(np.clip((rsrp + 120.0) / 60.0 * 100.0, 0.0, 100.0))
# ==================================================


# ===================== OPTIMIZER ====================
class PolygonDroneOptimizer:
    def __init__(self, initial_corners, graph, dashboard=None):
        self.initial_corners = initial_corners       # [(lng,lat), ...] clockwise
        self.graph = graph
        self.dashboard = dashboard
        self.iteration = 0
        self.current_position = None                 # (lat, lon)
        self.full_path = []                          # [(lat,lon,reason), ...]
        self.trajectory = []                         # [(lat,lon,rsrp,ues,reason,iter), ...]

    def _update_dash_pos(self, lat, lon):
        if self.dashboard:
            self.dashboard.update_position(self.iteration, lat, lon)

    def _update_dash_rsrp(self, rsrp):
        if self.dashboard:
            self.dashboard.update_rsrp(rsrp)

    def _update_dash_travel(self, meters):
        if self.dashboard:
            self.dashboard.update_travel(meters)

    @staticmethod
    def _corner_index_from_reason(reason):
        # reason expected like "corner_0".."corner_3"; map to 1..4 per your schema
        if isinstance(reason, str) and reason.startswith("corner_"):
            idx = int(reason.split("_")[1])
            return idx + 1
        # default to 1 if unknown
        return 1

    def _publish_move(self, lat, lon, target_label, distance_m=0.0, rsrp_value=RSRP_FAIL_VALUE):
        """Single place to publish with correct schema & safe numeric values."""
        sigq = quality_from_rsrp(rsrp_value)
        target_ue = self._corner_index_from_reason(target_label)
        _publish(lat, lon, self.iteration, rsrp_value, float(distance_m), sigq, target_ue)

    def move_step_by_step(self, target_lat, target_lon, target_label="movement"):
        """Shortest path movement + MQTT publish + Dashboard."""
        # First placement (teleport)
        if self.current_position is None:
            print(f"Teleporting drone to start position: ({target_lat:.6f}, {target_lon:.6f})")
            self.current_position = (target_lat, target_lon)
            self.full_path.append((target_lat, target_lon, target_label))
            self._publish_move(target_lat, target_lon, target_label, distance_m=0.0, rsrp_value=RSRP_FAIL_VALUE)
            self._update_dash_pos(target_lat, target_lon)
            time.sleep(0.15)
            return

        cur_lat, cur_lon = self.current_position
        path_coords, path_len = get_shortest_path_between_points(self.graph, cur_lat, cur_lon, target_lat, target_lon)

        if path_coords and len(path_coords) > 1:
            print(f"Moving via {len(path_coords)} waypoints to {target_label}")
            # total meters (convert deg length to meters)
            total_m = float(path_len * 111111.0) if path_len else 0.0
            # step through waypoints
            for i, (lat, lon) in enumerate(path_coords):
                if i == 0:
                    continue
                # publish waypoint (rsrp unknown during transit)
                self._publish_move(lat, lon, target_label, distance_m=total_m, rsrp_value=RSRP_FAIL_VALUE)
                self.full_path.append((lat, lon, "travel_to_" + target_label if i < len(path_coords)-1 else target_label))
                self._update_dash_pos(lat, lon)
                time.sleep(0.4)
            self.current_position = (target_lat, target_lon)
            print(f"Reached target after traveling {total_m:.1f}m")
            self._update_dash_travel(total_m)
        else:
            print("No path found, moving directly")
            self.current_position = (target_lat, target_lon)
            self.full_path.append((target_lat, target_lon, target_label))
            self._publish_move(target_lat, target_lon, target_label, distance_m=0.0, rsrp_value=RSRP_FAIL_VALUE)
            self._update_dash_pos(target_lat, target_lon)
            self._update_dash_travel(0.0)
            time.sleep(0.05)

    def measure_rsrp_at(self, lat, lon, label="measurement"):
        print(f"\n--- Moving to {label} at ({lat:.6f}, {lon:.6f}) ---")
        self.move_step_by_step(lat, lon, label)
        print("Waiting 0.5s to stabilize...")
        time.sleep(0.5)
        print("Taking RSRP measurement...")
        rsrp_val, ue_cnt = rsrp_retry_fetch()
        # publish final measurement at point with RSRP
        self._publish_move(lat, lon, label, distance_m=0.0, rsrp_value=rsrp_val)
        self._update_dash_rsrp(rsrp_val)
        self.trajectory.append((lat, lon, rsrp_val, ue_cnt, label, self.iteration))
        print(f"Measurement complete: RSRP={rsrp_val:.1f} dBm, UEs={ue_cnt}")
        return rsrp_val

    def polygon_bisection_search_real(self, min_area_threshold=100):
        print("Starting REAL 4-corner polygon bisection search...")

        # Publish the 4 corners once at the beginning (as normal updates)
        # Order: corner_0..corner_3 → target_ue = 1..4
        for idx, (lng, lat) in enumerate(self.initial_corners):
            self._publish_move(lat, lng, f"corner_{idx}", distance_m=0.0, rsrp_value=RSRP_FAIL_VALUE)

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

            print("Current corners:")
            for i, (lng, lat) in enumerate(current):
                print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")

            corner_vals = []
            if self.iteration == 1:
                # measure all 4
                for i, (lng, lat) in enumerate(current):
                    key = (round(lat, 8), round(lng, 8))
                    if key in measured_cache:
                        rsrp_cached, _ = measured_cache[key]
                        print(f"  {labels[i]} cached -> {rsrp_cached:.1f}")
                        corner_vals.append(rsrp_cached)
                        # still publish current position so ROS sees lat/lon repeatedly
                        self._publish_move(lat, lng, labels[i], distance_m=0.0, rsrp_value=rsrp_cached)
                        self._update_dash_pos(lat, lng)
                        self._update_dash_rsrp(rsrp_cached)
                    else:
                        val = self.measure_rsrp_at(lat, lng, labels[i])
                        corner_vals.append(val)
                        measured_cache[key] = (val, 0)
            else:
                # measure only new corners (2 and 3), reuse 0 and 1
                for i, (lng, lat) in enumerate(current):
                    key = (round(lat, 8), round(lng, 8))
                    if i < 2:
                        if key in measured_cache:
                            rsrp_cached, _ = measured_cache[key]
                            print(f"  {labels[i]} cached -> {rsrp_cached:.1f}")
                            corner_vals.append(rsrp_cached)
                            self._publish_move(lat, lng, labels[i], distance_m=0.0, rsrp_value=rsrp_cached)
                            self._update_dash_pos(lat, lng)
                            self._update_dash_rsrp(rsrp_cached)
                        else:
                            val = self.measure_rsrp_at(lat, lng, labels[i])
                            corner_vals.append(val)
                            measured_cache[key] = (val, 0)
                    else:
                        if key in measured_cache:
                            rsrp_cached, _ = measured_cache[key]
                            print(f"  {labels[i]} cached -> {rsrp_cached:.1f}")
                            corner_vals.append(rsrp_cached)
                            self._publish_move(lat, lng, labels[i], distance_m=0.0, rsrp_value=rsrp_cached)
                            self._update_dash_pos(lat, lng)
                            self._update_dash_rsrp(rsrp_cached)
                        else:
                            val = self.measure_rsrp_at(lat, lng, labels[i])
                            corner_vals.append(val)
                            measured_cache[key] = (val, 0)

            # score adjacent edges
            edges = []
            for i in range(4):
                j = (i + 1) % 4
                edges.append((corner_vals[i] + corner_vals[j], i, f"edge_{i}_{j}"))
                print(f"  Edge {i}-{j}: {corner_vals[i]:.1f} + {corner_vals[j]:.1f} = {corner_vals[i]+corner_vals[j]:.1f}")

            best_score, best_idx, best_name = max(edges, key=lambda x: x[0])
            print(f"*** BEST EDGE: {best_name} (score {best_score:.1f})")
            current = cut_polygon_in_half(current, best_idx)

        # final center
        center_lng = sum(lng for lng, lat in current) / 4
        center_lat = sum(lat for lng, lat in current) / 4
        final_val = self.measure_rsrp_at(center_lat, center_lng, "final_center")

        print(f"\n{'='*60}\nOPTIMIZATION COMPLETE\n{'='*60}")
        print(f"Final optimum: ({center_lat:.6f}, {center_lng:.6f})  RSRP={final_val:.1f}")
        print(f"Final polygon area: {get_polygon_area_meters(current):.1f} sq m")
        return center_lat, center_lng, final_val, current
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
        finite = [m for m in rsrps if m != RSRP_FAIL_VALUE]
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

            buildings, grid, G, key_to_node = load_graph()
            optimizer = PolygonDroneOptimizer(initial_corners, G, dashboard=dashboard)

            final_lat, final_lon, final_metric, final_corners = optimizer.polygon_bisection_search_real()

            print_trajectory_summary(optimizer.trajectory, optimizer.full_path)
            plot_polygon_trajectory(buildings, G, optimizer.trajectory, optimizer.full_path,
                                    (final_lat, final_lon, final_metric), initial_corners, final_corners)
        except Exception as e:
            print(f"[FATAL] {e}")

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    dashboard.start()
