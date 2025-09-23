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
