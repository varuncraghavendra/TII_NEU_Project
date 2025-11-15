#!/usr/bin/env python3
import os
import math
import time
import pickle
import json
import numpy as np
import networkx as nx
import matplotlib
import matplotlib.pyplot as plt
import tikzplotlib
from shapely.geometry import Polygon, Point
from hashlib import md5
import paho.mqtt.client as mqtt

# ---------------------------------------------------------------------------
# CONFIG & CONSTANTS
# ---------------------------------------------------------------------------

USE_FAKE_HOOKS = os.getenv("USE_FAKE_HOOKS", "1") == "1"
ENABLE_UI = os.getenv("ENABLE_UI", "1") == "1"  # disable if GUI problematic

# Safe backend for headless environments
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

# Area parameters
CENTER_LAT = 42.33894284868896
CENTER_LNG = -71.08613491058351
RADIUS_METERS = 1000
GRID_SPACING_METERS = 5

# Custom bounds from your example (lat, lon)
CUSTOM_BOUNDS = {
    "top_left": [42.34138538637752, -71.08300209045412],
    "top_right": [42.340314805239075, -71.08198285102846],
    "bottom_right": [42.337903948379996, -71.08519077301027],
    "bottom_left": [42.33866528158443, -71.08609199523927],
}

MIN_AREA_STOP_M2 = 100.0  # stop bisection when polygon area is below this

# MQTT / drone
BROKER = os.getenv("MQTT_HOST", "test.mosquitto.org")
PORT = int(os.getenv("MQTT_PORT", "1883"))
TOPIC = os.getenv("MQTT_TOPIC", "colosseum/update")
DRONE_ID = int(os.getenv("DRONE_ID", "1"))
ENTITY_ID = 30

# Heartbeat for GPS setpoints (critical so PX4 doesn’t land)
GPS_HEARTBEAT_DT = float(os.getenv("GPS_HEARTBEAT_DT", "0.1"))  # seconds (~10 Hz)

DEFAULT_SEED = None
ENV_SEED = os.getenv("RANDOM_SEED")
if ENV_SEED is not None:
    try:
        DEFAULT_SEED = int(ENV_SEED)
    except Exception:
        DEFAULT_SEED = None

_mqtt = None  # global for cleanup
LIVE_FIG = None
LIVE_AX = None

UE_CONN_RADIUS_M = 60.0  # distance within which a UE is considered "connected"


# ---------------------------------------------------------------------------
# MQTT SETUP & SHUTDOWN
# ---------------------------------------------------------------------------

def init_mqtt():
    global _mqtt
    try:
        _mqtt = mqtt.Client(
            client_id="",
            protocol=mqtt.MQTTv311,
            transport="tcp",
            callback_api_version=getattr(mqtt, "CallbackAPIVersion", None).VERSION2
            if hasattr(mqtt, "CallbackAPIVersion")
            else None,
        )
    except TypeError:
        _mqtt = mqtt.Client(protocol=mqtt.MQTTv311)

    try:
        _mqtt.connect(BROKER, PORT, 60)
        _mqtt.loop_start()
        print(f"[MQTT] Connected to {BROKER}:{PORT}")
    except Exception as e:
        print(f"[MQTT] WARN: connect failed: {e}")


def shutdown():
    """Clean up resources (plots + MQTT) to avoid segfaults on repeated runs."""
    global _mqtt, LIVE_FIG, LIVE_AX
    try:
        plt.close("all")
    except Exception:
        pass

    LIVE_FIG, LIVE_AX = None, None

    if _mqtt is not None:
        try:
            _mqtt.loop_stop()
        except Exception:
            pass
        try:
            _mqtt.disconnect()
        except Exception:
            pass
        _mqtt = None
    print("[SHUTDOWN] Cleaned up MQTT and plots.")


# ---------------------------------------------------------------------------
# UTILS
# ---------------------------------------------------------------------------

def meters_to_degrees(meters, is_longitude=False, center_lat=None):
    if is_longitude and center_lat is not None:
        return meters / (111111.0 * math.cos(math.radians(center_lat)))
    else:
        return meters / 111111.0


def degrees_to_meters(degrees, is_longitude=False, center_lat=None):
    if is_longitude and center_lat is not None:
        return degrees * (111111.0 * math.cos(math.radians(center_lat)))
    else:
        return degrees * 111111.0


def degrees_to_meters_lat(dlat_deg: float) -> float:
    return dlat_deg * 111111.0


def degrees_to_meters_lon(dlon_deg: float, at_lat_deg: float) -> float:
    return dlon_deg * (111111.0 * math.cos(math.radians(at_lat_deg)))


def signal_quality_from_rsrp(rsrp: float) -> float:
    if not np.isfinite(rsrp):
        return 0.0
    return float(np.clip((rsrp + 120.0) / 60.0 * 100.0, 0.0, 100.0))


def publish_flat(lat, lon, iteration, rsrp, distance_m, signal_quality_pct, target_ue_idx, connected_ues):
    if _mqtt is None:
        return
    payload = {
        "drone_id": DRONE_ID,
        "lat": float(lat),
        "lon": float(lon),
        "rsrp": float(rsrp),
        "distance": float(distance_m),
        "signal_quality": float(signal_quality_pct),
        "iteration": int(iteration),
        "target_ue": int(target_ue_idx),
        "connected_ues": int(connected_ues),
    }
    try:
        _mqtt.publish(TOPIC, json.dumps(payload))
    except Exception as e:
        print(f"[MQTT] WARN publish failed: {e}")


class FakeMqtt:
    def send_position_update(self, lat, lng, entity_id):
        if _mqtt is None:
            return
        payload = {"drone_id": DRONE_ID, "lat": float(lat), "lon": float(lng)}
        try:
            _mqtt.publish(TOPIC, json.dumps(payload))
        except Exception as e:
            print(f"[MQTT] WARN position publish failed: {e}")


class FakeInflux:
    def get_average_rsrp_3s(self):
        return {}


if USE_FAKE_HOOKS:
    print("=" * 40)
    print("WARNING: Using FAKE MQTT (publishes GPS) and FAKE RSRP")
    print("=" * 40)
    send_position_update = FakeMqtt().send_position_update
    get_average_rsrp_3s = FakeInflux().get_average_rsrp_3s
else:
    print("=" * 40)
    print("INFO: Using REAL MQTT and REAL INFLUX")
    print("=" * 40)
    from influx_hook import get_average_rsrp_3s
    from mqtt_client import send_position_update


def sleep_with_gps(duration_s, lat, lng, entity_id):
    """
    Critical helper: instead of a bare time.sleep, we keep sending
    the SAME GPS coordinate at high rate for `duration_s`.
    This prevents PX4 from timing out Offboard / landing.
    """
    if duration_s <= 0:
        return
    steps = max(1, int(duration_s / GPS_HEARTBEAT_DT))
    for _ in range(steps):
        send_position_update(lat, lng, entity_id)
        time.sleep(GPS_HEARTBEAT_DT)


def get_cache_filename(cache_type="buildings"):
    key = f"{CENTER_LAT}_{CENTER_LNG}_{RADIUS_METERS}"
    if cache_type == "graph":
        key += f"_grid{GRID_SPACING_METERS}m"
    hash_key = md5(key.encode()).hexdigest()[:8]
    return f"{cache_type}_cache_{hash_key}.pkl"


def create_custom_polygon():
    """Create a polygon from the custom boundaries (returns list of (lng, lat))."""
    coords = [
        (CUSTOM_BOUNDS["top_left"][1], CUSTOM_BOUNDS["top_left"][0]),
        (CUSTOM_BOUNDS["top_right"][1], CUSTOM_BOUNDS["top_right"][0]),
        (CUSTOM_BOUNDS["bottom_right"][1], CUSTOM_BOUNDS["bottom_right"][0]),
        (CUSTOM_BOUNDS["bottom_left"][1], CUSTOM_BOUNDS["bottom_left"][0]),
    ]
    return coords


def get_polygon_area_meters(corners):
    """Calculate polygon area in square meters."""
    poly = Polygon(corners)  # corners are (lng, lat)
    area_sq_meters = poly.area * (111111.0**2)
    return area_sq_meters


def get_midpoint(p1, p2):
    """Midpoint between two (lng, lat) points."""
    return ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)


def cut_polygon_in_half(corners, best_edge_idx):
    """
    Best-edge bisection rule:
    For best edge CD, new polygon is [C, D, midpoint(BC), midpoint(AD)].
    corners: list of 4 (lng, lat) points in clockwise order.
    """
    c_idx = best_edge_idx
    d_idx = (best_edge_idx + 1) % 4
    a_idx = (best_edge_idx + 2) % 4
    b_idx = (best_edge_idx + 3) % 4

    point_c = corners[c_idx]
    point_d = corners[d_idx]
    point_a = corners[a_idx]
    point_b = corners[b_idx]

    mid_ad = get_midpoint(point_a, point_d)  # midpoint of AD
    mid_bc = get_midpoint(point_b, point_c)  # midpoint of BC

    new_corners = [point_c, point_d, mid_bc, mid_ad]
    return new_corners


def load_graph():
    """Load graph and buildings from cache, convert buildings to coord lists."""
    cache_file = get_cache_filename("graph")
    buildings_cache = get_cache_filename("buildings")

    print(f"Loading graph from cache: {cache_file}")
    with open(cache_file, "rb") as f:
        cached_data = pickle.load(f)

    print(f"Loading buildings from cache: {buildings_cache}")
    with open(buildings_cache, "rb") as f:
        buildings_raw = pickle.load(f)

    buildings = []
    for b in buildings_raw:
        try:
            x, y = b.exterior.xy
            buildings.append(list(zip(x, y)))  # list of (lng, lat)
        except Exception:
            continue

    return buildings, cached_data["grid"], cached_data["graph"], cached_data["key_to_node"]


def generate_random_seed(base_seed=None) -> int:
    rnd_bytes = int.from_bytes(os.urandom(4), "little")
    now = int(time.time() * 1000)
    pid = os.getpid()
    if base_seed is None:
        return (rnd_bytes ^ now ^ pid) & 0x7FFFFFFF
    return (base_seed ^ rnd_bytes ^ now ^ pid) & 0x7FFFFFFF


def find_nearest_graph_node(G, target_lat, target_lng):
    min_distance = float("inf")
    nearest_node = None
    for node in G.nodes():
        node_lat = G.nodes[node]["lat"]
        node_lng = G.nodes[node]["lng"]
        distance = (node_lat - target_lat) ** 2 + (node_lng - target_lng) ** 2
        if distance < min_distance:
            min_distance = distance
            nearest_node = node
    return nearest_node


def get_shortest_path_between_points(G, lat1, lng1, lat2, lng2):
    node1 = find_nearest_graph_node(G, lat1, lng1)
    node2 = find_nearest_graph_node(G, lat2, lng2)
    if node1 is None or node2 is None:
        return None, None
    try:
        path_nodes = nx.shortest_path(G, node1, node2, weight="weight")
        path_coords = [(G.nodes[node]["lat"], G.nodes[node]["lng"]) for node in path_nodes]
        path_length = nx.shortest_path_length(G, node1, node2, weight="weight")
        return path_coords, path_length
    except nx.NetworkXNoPath:
        print(f"No path found between ({lat1:.6f}, {lng1:.6f}) and ({lat2:.6f}, {lng2:.6f})")
        return None, None


# ---------------------------------------------------------------------------
# SINGLE-UE RSRP FIELD (FAKE MODE)
# ---------------------------------------------------------------------------

def make_single_ue_rsrp_field(ue_coord, seed=None):
    """
    RSRP field peaked at a single UE location.
    RSRP ranges: ~[-100 dBm far, -30 dBm near UE] with small noise.
    ue_coord: (lat, lng)
    """
    if ue_coord is None:
        def rsrp(lat, lon):
            return -95.0 + np.random.normal(0, 0.5)
        return rsrp

    if seed is not None:
        np.random.seed(seed)

    u_lat, u_lng = ue_coord

    width_m = np.random.uniform(40.0, 60.0)  # spread
    floor = np.random.uniform(-105.0, -95.0)  # far field
    gain = np.random.uniform(65.0, 80.0)      # peak lift

    print(
        f"[FAKE RSRP SINGLE UE] ue=({u_lat:.6f}, {u_lng:.6f}) "
        f"w≈{width_m:.1f}m floor {floor:.1f}dBm gain {gain:.1f}dB"
    )

    def rsrp(lat, lon):
        dN = (lat - u_lat) * 111111.0
        dE = (lon - u_lng) * (111111.0 * math.cos(math.radians((lat + u_lat) / 2.0)))
        r2 = dN * dN + dE * dE
        base = floor + gain * math.exp(-r2 / (2.0 * width_m ** 2))
        noise = np.random.normal(0, 0.5)
        return base + noise

    return rsrp


def get_real_rsrp_metric_robust(self):
    """
    Real / fake RSRP logic.
    - Fake mode: uses single-UE-centric field + counts UEs within UE_CONN_RADIUS_M.
    - Real mode: original robust logic with long waits.
    """
    if USE_FAKE_HOOKS:
        if not self.fake_rsrp_func or not self.current_position:
            return float("-inf"), 0
        lat, lng = self.current_position
        metric = self.fake_rsrp_func(lat, lng)

        # Count UEs within radius
        connected_ues = 0
        for u_lat, u_lng in self.ue_coordinates:
            dN = degrees_to_meters_lat(lat - u_lat)
            dE = degrees_to_meters_lon(lng - u_lng, (lat + u_lat) / 2.0)
            d = math.hypot(dN, dE)
            if d <= UE_CONN_RADIUS_M:
                connected_ues += 1

        # small delay but we ALSO send GPS so stream continues
        send_position_update(lat, lng, self.entity_id)
        time.sleep(0.05)
        return metric, connected_ues

    # REAL mode: original robust logic with retries
    max_retries = 1
    for attempt in range(max_retries + 1):
        try:
            time.sleep(5.0)
            result = get_average_rsrp_3s()
            if not result:
                print("No data collected")
                if attempt < max_retries:
                    print("Waiting 60s before retrying...")
                    time.sleep(60.0)
                    continue
                else:
                    return float("-inf"), 0

            valid_rsrp_values = []
            for rnti, avg_rsrp in result.items():
                if avg_rsrp != float("-inf"):
                    valid_rsrp_values.append(avg_rsrp)

            if not valid_rsrp_values:
                if attempt < max_retries:
                    print("No valid RSRP values. Waiting 60s...")
                    time.sleep(60.0)
                    continue
                else:
                    return float("-inf"), 0

            avg_rsrp = sum(valid_rsrp_values) / len(valid_rsrp_values)
            connected_ues = len(valid_rsrp_values)
            return avg_rsrp, connected_ues

        except Exception as e:
            print(f"Error getting RSRP data (attempt {attempt + 1}): {e}")
            if attempt < max_retries:
                print("Waiting 60s before retrying...")
                time.sleep(60.0)
            else:
                return float("-inf"), 0


# ---------------------------------------------------------------------------
# LIVE PLOT HELPERS
# ---------------------------------------------------------------------------

def init_live_plot(buildings, initial_corners, ue_points=None):
    global LIVE_FIG, LIVE_AX
    if not ENABLE_UI:
        return

    plt.ion()
    LIVE_FIG, LIVE_AX = plt.subplots(1, 1, figsize=(10, 8))

    # Buildings
    for coords in buildings:
        x = [p[0] for p in coords]
        y = [p[1] for p in coords]
        LIVE_AX.plot(x, y, "b-", linewidth=0.5, alpha=0.6)
        LIVE_AX.fill(x, y, alpha=0.2, color="lightblue")

    # Initial polygon
    poly_x = [lng for lng, lat in initial_corners] + [initial_corners[0][0]]
    poly_y = [lat for lng, lat in initial_corners] + [initial_corners[0][1]]
    LIVE_AX.plot(poly_x, poly_y, "r--", linewidth=2, alpha=0.8, label="Initial Polygon")

    # UE points (if any)
    if ue_points:
        ue_lats = [c[0] for c in ue_points]
        ue_lngs = [c[1] for c in ue_points]
        LIVE_AX.scatter(
            ue_lngs,
            ue_lats,
            c="red",
            s=60,
            marker="x",
            linewidth=2,
            alpha=0.9,
            label="UE(s)",
        )

    # Axes limits
    all_lngs = [lng for lng, lat in initial_corners]
    all_lats = [lat for lng, lat in initial_corners]
    min_lng, max_lng = min(all_lngs), max(all_lngs)
    min_lat, max_lat = min(all_lats), max(all_lats)
    pad = 0.0005
    LIVE_AX.set_xlim(min_lng - pad, max_lng + pad)
    LIVE_AX.set_ylim(min_lat - pad, max_lat + pad)

    LIVE_AX.set_aspect("equal")
    LIVE_AX.set_xlabel("Longitude")
    LIVE_AX.set_ylabel("Latitude")
    LIVE_AX.set_title("Live Drone Optimization with Best-Edge Bisection (Two-Stage Single UE)")
    LIVE_AX.grid(True, alpha=0.3)
    LIVE_AX.legend(loc="upper right")

    LIVE_FIG.tight_layout()
    LIVE_FIG.canvas.draw()
    LIVE_FIG.canvas.flush_events()
    plt.show(block=False)


def live_plot_add_measurement(lat, lng):
    global LIVE_FIG, LIVE_AX
    if LIVE_AX is None or not ENABLE_UI:
        return
    LIVE_AX.plot(lng, lat, "o", markersize=3, alpha=0.8, color="purple")
    LIVE_FIG.canvas.draw()
    LIVE_FIG.canvas.flush_events()


def live_plot_add_ue(lat, lng, label="UE"):
    """Add a UE marker to the existing live plot."""
    global LIVE_FIG, LIVE_AX
    if LIVE_AX is None or not ENABLE_UI:
        return
    LIVE_AX.scatter(
        [lng],
        [lat],
        c="red",
        s=80,
        marker="x",
        linewidth=2,
        alpha=0.9,
    )
    LIVE_AX.annotate(
        label,
        (lng, lat),
        xytext=(5, 5),
        textcoords="offset points",
        fontsize=9,
        color="black",
        fontweight="bold",
    )
    LIVE_FIG.canvas.draw()
    LIVE_FIG.canvas.flush_events()


# ---------------------------------------------------------------------------
# RANDOM SINGLE UE
# ---------------------------------------------------------------------------

def random_single_ue_in_polygon(polygon_corners, rng=None):
    """
    Generate a SINGLE UE uniformly inside the polygon.
    Returns (lat, lng).
    """
    if rng is None:
        rng = np.random.default_rng()

    poly = Polygon(polygon_corners)
    lons = [lng for (lng, lat) in polygon_corners]
    lats = [lat for (lng, lat) in polygon_corners]
    min_lng, max_lng = min(lons), max(lons)
    min_lat, max_lat = min(lats), max(lats)

    ue_lng, ue_lat = None, None

    for _ in range(5000):
        cand_lng = rng.uniform(min_lng, max_lng)
        cand_lat = rng.uniform(min_lat, max_lat)
        if poly.contains(Point(cand_lng, cand_lat)):
            ue_lng = cand_lng
            ue_lat = cand_lat
            break

    if ue_lng is None or ue_lat is None:
        # Fallback: center of polygon
        ue_lng = sum(lons) / 4.0
        ue_lat = sum(lats) / 4.0
        print("[UE SINGLE] Could not sample inside poly; using polygon center.")
    else:
        print("[UE SINGLE] Sampled UE inside polygon.")

    print(f"[UE SINGLE] UE position: ({ue_lat:.6f}, {ue_lng:.6f})")
    return ue_lat, ue_lng


# ---------------------------------------------------------------------------
# CORE OPTIMIZER CLASS (best-edge bisection algorithm)
# ---------------------------------------------------------------------------

class PolygonDroneOptimizer:
    def __init__(
        self,
        initial_corners,
        graph,
        entity_id=ENTITY_ID,
        fake_rsrp_func=None,
        ue_coordinates=None,
        start_position=None,
    ):
        self.initial_corners = initial_corners  # list of (lng, lat)
        self.graph = graph
        self.entity_id = entity_id
        self.fake_rsrp_func = fake_rsrp_func
        self.ue_coordinates = ue_coordinates or []  # list of (lat, lng)

        # (lat, lng, metric, connected_ues, reason, iteration)
        self.trajectory = []
        # (lat, lng, reason)
        self.full_path = []

        self.iteration = 0
        self.current_position = start_position  # (lat, lng) or None

    # Movement & measurement -------------------------------------------------

    def _nearest_ue_index(self, lat, lng) -> int:
        if not self.ue_coordinates:
            return 1
        best_idx = 1
        best_d2 = float("inf")
        for idx, (u_lat, u_lng) in enumerate(self.ue_coordinates, start=1):
            dN = degrees_to_meters_lat(lat - u_lat)
            dE = degrees_to_meters_lon(lng - u_lng, (lat + u_lat) / 2.0)
            d2 = dN * dN + dE * dE
            if d2 < best_d2:
                best_d2 = d2
                best_idx = idx
        return best_idx

    def move_drone_step_by_step(self, target_lat, target_lng, reason="movement"):
        """
        Step-by-step graph-routed movement, with MQTT updates.
        Every segment + short dwell uses sleep_with_gps to keep PX4 happy.
        """
        if self.current_position is None:
            print(f"Teleporting drone to start position: ({target_lat:.6f}, {target_lng:.6f})")
            send_position_update(target_lat, target_lng, self.entity_id)
            self.current_position = (target_lat, target_lng)
            self.full_path.append((target_lat, target_lng, reason))
            # small warm-up burst at start
            sleep_with_gps(0.3, target_lat, target_lng, self.entity_id)
            return

        current_lat, current_lng = self.current_position
        path_coords, path_length = get_shortest_path_between_points(
            self.graph, current_lat, current_lng, target_lat, target_lng
        )

        if path_coords and len(path_coords) > 1:
            print(f"Moving drone via {len(path_coords)} waypoints to {reason}")
            for i, (lat, lng) in enumerate(path_coords):
                if i == 0:
                    continue
                # Update current position
                self.current_position = (lat, lng)
                send_position_update(lat, lng, self.entity_id)
                path_reason = f"travel_to_{reason}"
                if i == len(path_coords) - 1:
                    path_reason = reason
                self.full_path.append((lat, lng, path_reason))
                # instead of one 0.5s sleep, stream GPS during this dwell
                sleep_with_gps(0.5, lat, lng, self.entity_id)
            self.current_position = (target_lat, target_lng)
            path_length_m = path_length * 111111.0 if path_length else 0
            print(f"Reached target after traveling {path_length_m:.1f} m")
        else:
            print("Warning: No path found to target, using direct movement")
            self.current_position = (target_lat, target_lng)
            send_position_update(target_lat, target_lng, self.entity_id)
            self.full_path.append((target_lat, target_lng, reason))
            sleep_with_gps(0.3, target_lat, target_lng, self.entity_id)

    def measure_rsrp_real(self, lat, lng, reason="measurement"):
        """Move to position and measure RSRP (real or fake)."""
        print(f"\n--- Moving to {reason} at ({lat:.6f}, {lng:.6f}) ---")
        self.move_drone_step_by_step(lat, lng, reason)
        print("Waiting 0.5s for drone to stabilize...")
        # keep streaming GPS during stabilize
        sleep_with_gps(0.5, lat, lng, self.entity_id)

        metric, connected_ues = get_real_rsrp_metric_robust(self)
        self.trajectory.append(
            (lat, lng, metric, connected_ues, reason, self.iteration)
        )
        print(f"Measurement complete: metric={metric:.1f} dBm, connected_UEs={connected_ues}")

        # Distance to nearest UE
        if self.ue_coordinates:
            u_idx = self._nearest_ue_index(lat, lng)
            u_lat, u_lng = self.ue_coordinates[u_idx - 1]
            dN = degrees_to_meters_lat(lat - u_lat)
            dE = degrees_to_meters_lon(lng - u_lng, (lat + u_lat) / 2.0)
            distance_m = math.hypot(dN, dE)
        else:
            distance_m = 0.0
            u_idx = 1

        sigq = signal_quality_from_rsrp(metric)
        publish_flat(lat, lng, self.iteration, metric, distance_m, sigq, u_idx, connected_ues)

        # Live plot marker
        live_plot_add_measurement(lat, lng)

        return metric

    # Initial polygon tracing -----------------------------------------------

    def trace_initial_polygon_perimeter(self):
        """
        Force the drone to trace the entire initial polygon perimeter:
        corner0 -> corner1 -> corner2 -> corner3 -> corner0.
        """
        print("\n" + "=" * 60)
        print("TRACING INITIAL POLYGON PERIMETER")
        print("=" * 60)

        corners = self.initial_corners
        # Convert (lng, lat) to (lat, lng)
        corner_latlng = [(lat, lng) for (lng, lat) in corners]
        # Close the loop
        corner_latlng.append(corner_latlng[0])

        for idx in range(len(corner_latlng) - 1):
            start = corner_latlng[idx]
            end = corner_latlng[idx + 1]
            reason = f"initial_perimeter_{idx}"
            print(
                f"Tracing edge {idx}: "
                f"({start[0]:.6f}, {start[1]:.6f}) -> ({end[0]:.6f}, {end[1]:.6f})"
            )
            self.move_drone_step_by_step(end[0], end[1], reason=reason)

        print("Initial polygon perimeter traced completely.\n")

    # Best-edge polygon bisection algorithm ---------------------------------

    def polygon_bisection_search_real(self, min_area_threshold=MIN_AREA_STOP_M2):
        """
        REAL 4-corner polygon bisection search with best-edge selection.
        Uses cached measurements at corners; only new midpoints are measured
        after the first iteration.
        """
        print("Starting 4-corner polygon bisection for single UE...")
        current_corners = self.initial_corners.copy()
        measured_positions = {}  # (lat, lng) -> (metric, connected_ues)

        corner_labels = ["corner_0", "corner_1", "corner_2", "corner_3"]

        while True:
            self.iteration += 1
            print("\n" + "=" * 60)
            print(f"ITERATION {self.iteration}")
            print("=" * 60)

            area_sq_meters = get_polygon_area_meters(current_corners)
            print(f"Current polygon area: {area_sq_meters:.1f} sq meters")
            if area_sq_meters < min_area_threshold:
                print("Polygon is small enough, stopping search.")
                break

            print("Current polygon corners:")
            for i, (lng, lat) in enumerate(current_corners):
                print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")

            corner_measurements = []

            if self.iteration == 1:
                print("First iteration: measuring all 4 corners")
                for i, (lng, lat) in enumerate(current_corners):
                    pos_key = (round(lat, 8), round(lng, 8))
                    if pos_key in measured_positions:
                        cached_metric, cached_ues = measured_positions[pos_key]
                        print(f"  {corner_labels[i]}: Using cached -> Metric: {cached_metric:.1f}")
                        corner_measurements.append(cached_metric)
                    else:
                        metric = self.measure_rsrp_real(lat, lng, corner_labels[i])
                        corner_measurements.append(metric)
                        last = self.trajectory[-1]
                        measured_positions[pos_key] = (last[2], last[3])
            else:
                print("Subsequent iteration: measuring only new corners if needed")
                for i, (lng, lat) in enumerate(current_corners):
                    pos_key = (round(lat, 8), round(lng, 8))
                    if pos_key in measured_positions:
                        cached_metric, cached_ues = measured_positions[pos_key]
                        print(
                            f"  {corner_labels[i]}: Using cached -> Metric: {cached_metric:.1f}"
                        )
                        corner_measurements.append(cached_metric)
                    else:
                        print(f"  {corner_labels[i]}: New corner, measuring...")
                        metric = self.measure_rsrp_real(lat, lng, corner_labels[i])
                        corner_measurements.append(metric)
                        last = self.trajectory[-1]
                        measured_positions[pos_key] = (last[2], last[3])

            # Edge scores (adjacent pairs)
            edge_scores = []
            for i in range(4):
                j = (i + 1) % 4
                score = corner_measurements[i] + corner_measurements[j]
                edge_scores.append((score, i, f"edge_{i}_{j}"))
                print(
                    f"  Edge {i}-{j}: score = "
                    f"{corner_measurements[i]:.1f} + {corner_measurements[j]:.1f} = {score:.1f}"
                )

            best_score, best_edge_idx, best_edge_name = max(edge_scores, key=lambda x: x[0])
            print(f"\n*** BEST EDGE: {best_edge_name} with combined score {best_score:.1f} ***")

            new_corners = cut_polygon_in_half(current_corners, best_edge_idx)
            print("New polygon corners after cut:")
            for i, (lng, lat) in enumerate(new_corners):
                print(f"  New corner {i}: ({lat:.6f}, {lng:.6f})")

            current_corners = new_corners

        # Final center measurement
        center_lng = sum(lng for lng, lat in current_corners) / 4.0
        center_lat = sum(lat for lng, lat in current_corners) / 4.0
        final_pos_key = (round(center_lat, 8), round(center_lng, 8))

        if final_pos_key in measured_positions:
            cached_metric, cached_ues = measured_positions[final_pos_key]
            print(f"Final center already measured: {cached_metric:.1f}")
            final_metric = cached_metric
        else:
            final_metric = self.measure_rsrp_real(center_lat, center_lng, "final_center")

        print("\n" + "=" * 60)
        print("POLYGON OPTIMIZATION COMPLETE (single UE)!")
        print("=" * 60)
        print(f"Final optimum: ({center_lat:.6f}, {center_lng:.6f})")
        print(f"Final metric: {final_metric:.1f}")
        print(f"Final polygon area: {get_polygon_area_meters(current_corners):.1f} sq meters")

        return center_lat, center_lng, final_metric, current_corners


# ---------------------------------------------------------------------------
# SUMMARY & FINAL PLOT
# ---------------------------------------------------------------------------

def print_trajectory_summary(trajectory, full_path, label=""):
    print("\n" + "=" * 50)
    print(f"TRAJECTORY SUMMARY {label}".strip())
    print("=" * 50)
    print(f"Total measurements: {len(trajectory)}")
    print(f"Total flight path points: {len(full_path)}")

    if len(full_path) > 1:
        total_distance = 0.0
        for i in range(1, len(full_path)):
            lat1, lng1 = full_path[i - 1][0], full_path[i - 1][1]
            lat2, lng2 = full_path[i][0], full_path[i][1]
            lat_dist = degrees_to_meters(lat2 - lat1)
            lng_dist = degrees_to_meters(
                lng2 - lng1,
                is_longitude=True,
                center_lat=(lat1 + lat2) / 2.0,
            )
            total_distance += math.hypot(lat_dist, lng_dist)
        print(f"Total flight distance: {total_distance:.1f} meters")

    by_iter = {}
    for lat, lng, metric, ues, reason, iteration in trajectory:
        by_iter.setdefault(iteration, []).append((lat, lng, metric, ues, reason))

    for iteration in sorted(by_iter.keys()):
        print(f"\nIteration {iteration}:")
        for lat, lng, metric, ues, reason in by_iter[iteration]:
            print(
                f"  {reason:25}: ({lat:.6f}, {lng:.6f}) -> "
                f"Metric: {metric:.1f}, UEs: {ues}"
            )

    if trajectory:
        best = max(trajectory, key=lambda x: x[2])
        print(
            f"\nBest measurement overall: Metric {best[2]:.1f} dBm "
            f"with {best[3]} UEs"
        )
        print(f"Location: ({best[0]:.6f}, {best[1]:.6f})")


def make_final_plot(
    buildings,
    initial_corners,
    ue1,
    ue2,
    full_trajectory,
    full_path,
    final_point1,
    final_point2,
):
    fig, ax = plt.subplots(1, 1, figsize=(15, 12))

    # Buildings
    for coords in buildings:
        x = [p[0] for p in coords]
        y = [p[1] for p in coords]
        ax.plot(x, y, "b-", linewidth=0.5, alpha=0.6)
        ax.fill(x, y, alpha=0.2, color="lightblue")

    # Initial polygon
    poly_x = [lng for lng, lat in initial_corners] + [initial_corners[0][0]]
    poly_y = [lat for lng, lat in initial_corners] + [initial_corners[0][1]]
    ax.plot(poly_x, poly_y, "r--", linewidth=2, alpha=0.8, label="Initial Polygon")

    # UEs
    if ue1 is not None:
        ax.scatter(
            [ue1[1]], [ue1[0]],
            c="red",
            s=100,
            marker="x",
            linewidth=2,
            alpha=0.9,
            label="UE 1",
        )
    if ue2 is not None:
        ax.scatter(
            [ue2[1]], [ue2[0]],
            c="orange",
            s=100,
            marker="x",
            linewidth=2,
            alpha=0.9,
            label="UE 2",
        )

    # Flight path (combined)
    if full_path:
        path_lats = [p[0] for p in full_path]
        path_lngs = [p[1] for p in full_path]
        ax.plot(path_lngs, path_lats, "purple", linewidth=2, alpha=0.7, label="Flight Path")

    # Measurements (combined)
    if full_trajectory:
        lats = [p[0] for p in full_trajectory]
        lngs = [p[1] for p in full_trajectory]
        metrics = [p[2] for p in full_trajectory]
        finite_metrics = [m for m in metrics if m != float("-inf")]
        if finite_metrics:
            vmin, vmax = min(finite_metrics), max(finite_metrics)
        else:
            vmin, vmax = 0, 1

        scatter = ax.scatter(
            lngs,
            lats,
            c=metrics,
            cmap="RdYlGn",
            s=80,
            alpha=0.9,
            edgecolors="black",
            linewidth=1,
            vmin=vmin,
            vmax=vmax,
            zorder=5,
        )
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label("RSRP Metric (dBm)", rotation=270, labelpad=20)

        for i, (lat, lng, metric, ues, reason, it) in enumerate(full_trajectory):
            ax.annotate(
                f"{i+1}",
                (lng, lat),
                xytext=(0, 15),
                textcoords="offset points",
                fontsize=8,
                ha="center",
                color="black",
                fontweight="bold",
                bbox=dict(
                    boxstyle="circle,pad=0.2",
                    facecolor="white",
                    edgecolor="black",
                    alpha=0.8,
                ),
                zorder=8,
            )

    # Final optima (triangle icons)
    if final_point1:
        lat_opt1, lng_opt1, m_opt1 = final_point1
        ax.plot(
            lng_opt1,
            lat_opt1,
            "k^",             # triangle marker
            markersize=16,
            label=f"Optimum UE1 ({m_opt1:.1f} dBm)",
            zorder=7,
        )
    if final_point2:
        lat_opt2, lng_opt2, m_opt2 = final_point2
        ax.plot(
            lng_opt2,
            lat_opt2,
            "g^",             # triangle marker
            markersize=16,
            label=f"Optimum UE2 ({m_opt2:.1f} dBm)",
            zorder=7,
        )

    ax.set_aspect("equal")
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("4-Corner Polygon Best-Edge Bisection (Two-Stage Single-UE Optimization)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    plt.tight_layout()
    try:
        plt.savefig("trajectory_two_stage_single_ue.png")
        tikzplotlib.save("trajectory_two_stage_single_ue.tex")
    except Exception as e:
        print(f"[PLOT] WARN saving failed: {e}")

    if ENABLE_UI:
        plt.show()
    else:
        plt.close(fig)


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        init_mqtt()
        print("Starting 4-corner polygon drone optimization with TWO sequential single UEs...")

        # Initial polygon from CUSTOM_BOUNDS
        initial_corners = create_custom_polygon()  # list of (lng, lat)
        initial_area = get_polygon_area_meters(initial_corners)
        print(f"Initial polygon area: {initial_area:.1f} sq meters")
        print("Initial corners (clockwise):")
        for i, (lng, lat) in enumerate(initial_corners):
            print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")

        # Load graph & buildings
        buildings, grid, G, key_to_node = load_graph()

        rng = np.random.default_rng(generate_random_seed(DEFAULT_SEED))

        # ----------------------- FIRST UE ---------------------------------
        ue1_lat, ue1_lng = random_single_ue_in_polygon(initial_corners, rng=rng)
        ue1 = (ue1_lat, ue1_lng)

        init_live_plot(buildings, initial_corners, ue_points=[ue1])

        rsrp_func1 = None
        if USE_FAKE_HOOKS:
            rsrp_func1 = make_single_ue_rsrp_field(
                ue1, seed=generate_random_seed(DEFAULT_SEED)
            )

        optimizer1 = PolygonDroneOptimizer(
            initial_corners,
            G,
            entity_id=ENTITY_ID,
            fake_rsrp_func=rsrp_func1,
            ue_coordinates=[ue1],
            start_position=None,  # first GPS when movement starts
        )

        # 1) Trace complete initial polygon perimeter first
        optimizer1.trace_initial_polygon_perimeter()

        # 2) Best-edge polygon bisection for UE1
        final1_lat, final1_lng, final1_metric, final1_corners = optimizer1.polygon_bisection_search_real(
            min_area_threshold=MIN_AREA_STOP_M2
        )
        final_point1 = (final1_lat, final1_lng, final1_metric)

        # ----------------------- SECOND UE --------------------------------
        print("\n" + "=" * 60)
        print("Placing SECOND UE and running bisection again within SAME initial polygon...")
        print("=" * 60)

        ue2_lat, ue2_lng = random_single_ue_in_polygon(initial_corners, rng=rng)
        ue2 = (ue2_lat, ue2_lng)

        # Add UE2 to live plot after UE1 optimization is done
        live_plot_add_ue(ue2_lat, ue2_lng, label="UE2")

        rsrp_func2 = None
        if USE_FAKE_HOOKS:
            rsrp_func2 = make_single_ue_rsrp_field(
                ue2, seed=generate_random_seed(DEFAULT_SEED)
            )

        # Start second optimizer from the final UE1 optimum position,
        # but bisection uses the SAME initial polygon area.
        optimizer2 = PolygonDroneOptimizer(
            initial_corners,
            G,
            entity_id=ENTITY_ID,
            fake_rsrp_func=rsrp_func2,
            ue_coordinates=[ue2],
            start_position=(final1_lat, final1_lng),
        )

        # Reuse the full flight path so far so the trajectory is continuous
        optimizer2.full_path = optimizer1.full_path.copy()
        optimizer2.current_position = (final1_lat, final1_lng)

        final2_lat, final2_lng, final2_metric, final2_corners = optimizer2.polygon_bisection_search_real(
            min_area_threshold=MIN_AREA_STOP_M2
        )
        final_point2 = (final2_lat, final2_lng, final2_metric)

        # ----------------------- SUMMARY & PLOT ----------------------------
        combined_trajectory = optimizer1.trajectory + optimizer2.trajectory
        combined_full_path = optimizer2.full_path  # already includes phase 1 + phase 2

        print_trajectory_summary(optimizer1.trajectory, optimizer1.full_path, label="(UE1)")
        print_trajectory_summary(optimizer2.trajectory, optimizer2.full_path, label="(UE2)")
        print_trajectory_summary(combined_trajectory, combined_full_path, label="(Combined)")

        make_final_plot(
            buildings,
            initial_corners,
            ue1,
            ue2,
            combined_trajectory,
            combined_full_path,
            final_point1,
            final_point2,
        )

    finally:
        shutdown()
