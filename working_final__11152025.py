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
from shapely.geometry import Point, Polygon
from hashlib import md5
import paho.mqtt.client as mqtt

# ---------------------------------------------------------------------------
# BASIC CONFIG
# ---------------------------------------------------------------------------

# Center / radius used to build the cached graph
CENTER_LAT = 42.33894284868896
CENTER_LNG = -71.08613491058351
RADIUS_METERS = 1000
GRID_SPACING_METERS = 5

# Custom 4-corner polygon (clockwise: top_left → top_right → bottom_right → bottom_left)
CUSTOM_BOUNDS = {
    "top_left":    [42.34138538637752, -71.08300209045412],
    "top_right":   [42.340314805239075, -71.08198285102846],
    "bottom_right":[42.337903948379996, -71.08519077301027],
    "bottom_left": [42.33866528158443,  -71.08609199523927],
}

# Area stop threshold for bisection (~100 m²)
MIN_AREA_STOP_M2 = 100.0

# UE connection radius for counting connected UEs
UE_CONN_RADIUS_M = 60.0

# Drone / entity id for messaging
ENTITY_ID = 30

# MQTT configuration
MQTT_HOST = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "colosseum/update")
DRONE_ID = int(os.getenv("DRONE_ID", "1"))

# Waypoint pacing
WAYPOINT_SLEEP_S = 0.35
STABILIZE_SLEEP_S = 0.35

# Random seed
DEFAULT_SEED = None
ENV_SEED = os.getenv("RANDOM_SEED")
if ENV_SEED is not None:
    try:
        DEFAULT_SEED = int(ENV_SEED)
    except Exception:
        DEFAULT_SEED = None

# For headless environments
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

LIVE_FIG = None
LIVE_AX = None
LIVE_ARROW = None
_MQTT_CLIENT = None


# ---------------------------------------------------------------------------
# UTILITIES
# ---------------------------------------------------------------------------

def meters_to_degrees(meters, is_longitude=False, center_lat=None):
    if is_longitude and center_lat is not None:
        return meters / (111111.0 * math.cos(math.radians(center_lat)))
    else:
        return meters / 111111.0


def degrees_to_meters(deg, is_longitude=False, center_lat=None):
    if is_longitude and center_lat is not None:
        return deg * (111111.0 * math.cos(math.radians(center_lat)))
    else:
        return deg * 111111.0


def geo_distance_m(lat1, lng1, lat2, lng2):
    lat_dist = degrees_to_meters(abs(lat2 - lat1))
    lng_dist = degrees_to_meters(
        abs(lng2 - lng1),
        is_longitude=True,
        center_lat=(lat1 + lat2) / 2.0,
    )
    return math.hypot(lat_dist, lng_dist)


def create_custom_polygon():
    return [
        (CUSTOM_BOUNDS["top_left"][1],  CUSTOM_BOUNDS["top_left"][0]),
        (CUSTOM_BOUNDS["top_right"][1], CUSTOM_BOUNDS["top_right"][0]),
        (CUSTOM_BOUNDS["bottom_right"][1], CUSTOM_BOUNDS["bottom_right"][0]),
        (CUSTOM_BOUNDS["bottom_left"][1], CUSTOM_BOUNDS["bottom_left"][0]),
    ]


def get_polygon_area_meters(corners):
    poly = Polygon(corners)
    return poly.area * (111111.0 ** 2)


def get_midpoint(p1, p2):
    return ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)


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

    new_corners = [point_c, point_d, mid_bc, mid_ad]
    return new_corners


def get_cache_filename(cache_type="buildings"):
    key = f"{CENTER_LAT}_{CENTER_LNG}_{RADIUS_METERS}"
    if cache_type == "graph":
        key += f"_grid{GRID_SPACING_METERS}m"
    hash_key = md5(key.encode()).hexdigest()[:8]
    return f"{cache_type}_cache_{hash_key}.pkl"


def load_graph():
    graph_cache = get_cache_filename("graph")
    bldg_cache = get_cache_filename("buildings")

    print(f"Loading graph from cache: {graph_cache}")
    with open(graph_cache, "rb") as f:
        cached = pickle.load(f)

    print(f"Loading buildings from cache: {bldg_cache}")
    with open(bldg_cache, "rb") as f:
        buildings = pickle.load(f)

    return buildings, cached["grid"], cached["graph"], cached["key_to_node"]


def generate_random_seed(base_seed=None):
    rnd_bytes = int.from_bytes(os.urandom(4), "little")
    now = int(time.time() * 1000)
    pid = os.getpid()
    if base_seed is None:
        return (rnd_bytes ^ now ^ pid) & 0x7FFFFFFF
    return (base_seed ^ rnd_bytes ^ now ^ pid) & 0x7FFFFFFF


# ---------------------------------------------------------------------------
# MQTT
# ---------------------------------------------------------------------------

def init_mqtt():
    global _MQTT_CLIENT
    try:
        _MQTT_CLIENT = mqtt.Client()
        _MQTT_CLIENT.connect(MQTT_HOST, MQTT_PORT, 60)
        _MQTT_CLIENT.loop_start()
        print(f"[MQTT] Connected to {MQTT_HOST}:{MQTT_PORT}")
    except Exception as e:
        print(f"[MQTT] WARNING: MQTT connect failed: {e}")
        _MQTT_CLIENT = None


def shutdown():
    global _MQTT_CLIENT, LIVE_FIG, LIVE_AX, LIVE_ARROW
    try:
        plt.close("all")
    except Exception:
        pass
    LIVE_FIG, LIVE_AX, LIVE_ARROW = None, None, None

    if _MQTT_CLIENT is not None:
        try:
            _MQTT_CLIENT.loop_stop()
        except Exception:
            pass
        try:
            _MQTT_CLIENT.disconnect()
        except Exception:
            pass
        _MQTT_CLIENT = None
    print("[SHUTDOWN] Cleaned up MQTT and plots.")


def send_position_update(lat, lng, entity_id):
    payload = {
        "drone_id": DRONE_ID,
        "lat": float(lat),
        "lon": float(lng),
        "entity_id": int(entity_id),
    }
    msg = json.dumps(payload)
    if _MQTT_CLIENT is not None:
        try:
            _MQTT_CLIENT.publish(MQTT_TOPIC, msg)
        except Exception as e:
            print(f"[MQTT] WARN publish failed: {e}")
    print(f"[WAYPOINT] {lat:.7f}, {lng:.7f} → {msg}")


# ---------------------------------------------------------------------------
# UE CLUSTER + FAKE RSRP
# ---------------------------------------------------------------------------

def generate_ues_in_polygon(polygon_corners, rng=None, min_ues=3, max_ues=8):
    if rng is None:
        rng = np.random.default_rng()

    poly = Polygon(polygon_corners)
    minx, miny, maxx, maxy = poly.bounds

    n_ues = int(rng.integers(min_ues, max_ues + 1))
    ue_coords = []
    tries = 0

    while len(ue_coords) < n_ues and tries < n_ues * 500:
        tries += 1
        lng = rng.uniform(minx, maxx)
        lat = rng.uniform(miny, maxy)
        if poly.contains(Point(lng, lat)):
            ue_coords.append((lat, lng))

    print(f"[UE CLUSTER] Generated {len(ue_coords)} UEs inside custom polygon")
    return ue_coords


def make_fake_rsrp_field(ue_coords, seed=None):
    if not ue_coords:
        def rsrp(lat, lng):
            return -95.0 + np.random.normal(0, 1.0)
        return rsrp

    if seed is not None:
        np.random.seed(seed)

    c_lat = sum(lat for lat, lng in ue_coords) / len(ue_coords)
    c_lng = sum(lng for lat, lng in ue_coords) / len(ue_coords)

    width_m = np.random.uniform(80.0, 160.0)
    floor = np.random.uniform(-105.0, -95.0)
    gain = np.random.uniform(65.0, 80.0)

    print(
        f"[FAKE RSRP] centroid=({c_lat:.6f}, {c_lng:.6f}), "
        f"width≈{width_m:.1f}m, floor={floor:.1f}dBm, gain={gain:.1f}dB"
    )

    def rsrp(lat, lng):
        d = geo_distance_m(lat, lng, c_lat, c_lng)
        base = floor + gain * math.exp(-(d ** 2) / (2.0 * width_m ** 2))
        noise = np.random.normal(0, 1.0)
        return base + noise

    return rsrp


# ---------------------------------------------------------------------------
# GRAPH PATH HELPERS
# ---------------------------------------------------------------------------

def find_nearest_graph_node(G, target_lat, target_lng):
    min_dist = float("inf")
    nearest = None
    for node in G.nodes():
        node_lat = G.nodes[node]["lat"]
        node_lng = G.nodes[node]["lng"]
        d = (node_lat - target_lat) ** 2 + (node_lng - target_lng) ** 2
        if d < min_dist:
            min_dist = d
            nearest = node
    return nearest


def get_shortest_path_between_points(G, lat1, lng1, lat2, lng2):
    node1 = find_nearest_graph_node(G, lat1, lng1)
    node2 = find_nearest_graph_node(G, lat2, lng2)

    if node1 is None or node2 is None:
        print("[PATH] No nearest nodes; straight-line fallback.")
        steps = 8
        path_coords = []
        for i in range(steps + 1):
            alpha = i / steps
            lat = lat1 + alpha * (lat2 - lat1)
            lng = lng1 + alpha * (lng2 - lng1)
            path_coords.append((lat, lng))
        return path_coords, geo_distance_m(lat1, lng1, lat2, lng2)

    try:
        path_nodes = nx.shortest_path(G, node1, node2, weight="weight")
        path_length = nx.shortest_path_length(G, node1, node2, weight="weight")
        path_coords = [(G.nodes[n]["lat"], G.nodes[n]["lng"]) for n in path_nodes]
        return path_coords, path_length
    except Exception as e:
        print(f"[PATH] shortest_path failed: {e}; straight-line fallback.")
        steps = 8
        path_coords = []
        for i in range(steps + 1):
            alpha = i / steps
            lat = lat1 + alpha * (lat2 - lat1)
            lng = lng1 + alpha * (lng2 - lng1)
            path_coords.append((lat, lng))
        return path_coords, geo_distance_m(lat1, lng1, lat2, lng2)


# ---------------------------------------------------------------------------
# LIVE PLOT HELPERS
# ---------------------------------------------------------------------------

def init_live_plot(buildings, initial_corners, ue_coords):
    global LIVE_FIG, LIVE_AX
    plt.ion()
    LIVE_FIG, LIVE_AX = plt.subplots(1, 1, figsize=(10, 8))

    if buildings:
        for b in buildings:
            x, y = b.exterior.xy
            LIVE_AX.plot(x, y, "b-", linewidth=0.5, alpha=0.6)
            LIVE_AX.fill(x, y, "lightblue", alpha=0.2)

    poly_x = [lng for (lng, lat) in initial_corners] + [initial_corners[0][0]]
    poly_y = [lat for (lng, lat) in initial_corners] + [initial_corners[0][1]]
    LIVE_AX.plot(poly_x, poly_y, "r--", linewidth=2, alpha=0.8, label="Initial Polygon")

    if ue_coords:
        ue_lats = [lat for (lat, lng) in ue_coords]
        ue_lngs = [lng for (lat, lng) in ue_coords]
        LIVE_AX.scatter(
            ue_lngs,
            ue_lats,
            c="red",
            s=80,
            marker="x",
            linewidth=2,
            alpha=0.9,
            label="UEs",
        )

    all_lats = [lat for (lng, lat) in initial_corners] + [lat for (lat, lng) in ue_coords]
    all_lngs = [lng for (lng, lat) in initial_corners] + [lng for (lat, lng) in ue_coords]
    pad = meters_to_degrees(30.0)
    LIVE_AX.set_xlim(min(all_lngs) - pad, max(all_lngs) + pad)
    LIVE_AX.set_ylim(min(all_lats) - pad, max(all_lats) + pad)

    LIVE_AX.set_aspect("equal")
    LIVE_AX.set_xlabel("Longitude")
    LIVE_AX.set_ylabel("Latitude")
    LIVE_AX.set_title("Live Drone Polygon Bisection (Fake RSRP)")
    LIVE_AX.grid(True, alpha=0.3)
    LIVE_AX.legend(loc="upper right")

    LIVE_FIG.tight_layout()
    LIVE_FIG.canvas.draw()
    LIVE_FIG.canvas.flush_events()
    plt.show(block=False)


def update_direction_arrow(prev_lat, prev_lng, next_lat, next_lng):
    global LIVE_FIG, LIVE_AX, LIVE_ARROW
    if LIVE_AX is None:
        return
    if LIVE_ARROW is not None:
        LIVE_ARROW.remove()
    LIVE_ARROW = LIVE_AX.annotate(
        "",
        xy=(next_lng, next_lat),
        xytext=(prev_lng, prev_lat),
        arrowprops=dict(arrowstyle="->", linewidth=2, color="orange"),
    )
    LIVE_FIG.canvas.draw()
    LIVE_FIG.canvas.flush_events()


def clear_direction_arrow():
    global LIVE_FIG, LIVE_AX, LIVE_ARROW
    if LIVE_AX is None:
        return
    if LIVE_ARROW is not None:
        LIVE_ARROW.remove()
        LIVE_ARROW = None
        LIVE_FIG.canvas.draw()
        LIVE_FIG.canvas.flush_events()


def live_plot_add_measurement(lat, lng):
    global LIVE_FIG, LIVE_AX
    if LIVE_AX is None:
        return
    LIVE_AX.plot(lng, lat, "o", markersize=4, alpha=0.9, color="purple")
    LIVE_FIG.canvas.draw()
    LIVE_FIG.canvas.flush_events()


# ---------------------------------------------------------------------------
# CORE OPTIMIZER
# ---------------------------------------------------------------------------

class PolygonDroneOptimizer:
    def __init__(self, initial_corners, graph, ue_coordinates, fake_rsrp_func=None, entity_id=ENTITY_ID):
        self.initial_corners = initial_corners
        self.graph = graph
        self.ue_coordinates = ue_coordinates or []
        self.fake_rsrp_func = fake_rsrp_func
        self.entity_id = entity_id

        self.trajectory = []   # (lat, lng, metric, connected_ues, reason, iteration)
        self.full_path = []    # (lat, lng, reason)
        self.iteration = 0
        self.current_position = None
        self.measured_positions = {}  # (lat, lng) → (metric, connected_ues)

    def move_drone_step_by_step(self, target_lat, target_lng, reason="movement"):
        if self.current_position is None:
            print(f"Teleporting drone to start: ({target_lat:.6f}, {target_lng:.6f})")
            send_position_update(target_lat, target_lng, self.entity_id)
            self.current_position = (target_lat, target_lng)
            self.full_path.append((target_lat, target_lng, reason))
            live_plot_add_measurement(target_lat, target_lng)
            time.sleep(0.2)
            return

        current_lat, current_lng = self.current_position
        path_coords, path_length = get_shortest_path_between_points(
            self.graph, current_lat, current_lng, target_lat, target_lng
        )

        if path_coords and len(path_coords) > 1:
            print(f"Moving via {len(path_coords)} waypoints → {reason}")
            prev_lat, prev_lng = current_lat, current_lng
            for i, (lat, lng) in enumerate(path_coords):
                if i == 0:
                    continue
                update_direction_arrow(prev_lat, prev_lng, lat, lng)
                send_position_update(lat, lng, self.entity_id)
                path_reason = f"travel_to_{reason}"
                if i == len(path_coords) - 1:
                    path_reason = reason
                self.full_path.append((lat, lng, path_reason))
                self.current_position = (lat, lng)
                live_plot_add_measurement(lat, lng)
                prev_lat, prev_lng = lat, lng
                time.sleep(WAYPOINT_SLEEP_S)
            clear_direction_arrow()
            self.current_position = (target_lat, target_lng)
        else:
            print("No graph path, using direct step")
            update_direction_arrow(current_lat, current_lng, target_lat, target_lng)
            send_position_update(target_lat, target_lng, self.entity_id)
            self.full_path.append((target_lat, target_lng, reason))
            self.current_position = (target_lat, target_lng)
            live_plot_add_measurement(target_lat, target_lng)
            time.sleep(WAYPOINT_SLEEP_S)
            clear_direction_arrow()

    def _fake_rsrp_and_ues(self, lat, lng):
        if self.fake_rsrp_func is None:
            metric = -95.0 + np.random.normal(0, 1.0)
        else:
            metric = float(self.fake_rsrp_func(lat, lng))

        connected_ues = 0
        for ue_lat, ue_lng in self.ue_coordinates:
            d = geo_distance_m(lat, lng, ue_lat, ue_lng)
            if d <= UE_CONN_RADIUS_M:
                connected_ues += 1
        return metric, connected_ues

    def measure_rsrp_fake(self, lat, lng, reason="measurement"):
        print(f"\n--- Moving to {reason} at ({lat:.6f}, {lng:.6f}) ---")
        self.move_drone_step_by_step(lat, lng, reason)
        print("Waiting for drone to stabilize...")
        time.sleep(STABILIZE_SLEEP_S)

        metric, connected_ues = self._fake_rsrp_and_ues(lat, lng)
        self.trajectory.append((lat, lng, metric, connected_ues, reason, self.iteration))

        pos_key = (round(lat, 8), round(lng, 8))
        self.measured_positions[pos_key] = (metric, connected_ues)

        print(f"Measurement: metric={metric:.1f} dBm, connected_UEs={connected_ues}")
        return metric

    def trace_polygon_boundary_and_measure_corners(self):
        print("\n" + "=" * 60)
        print("INITIAL POLYGON TRACE (CLOCKWISE) + CORNER RSRP")
        print("=" * 60)

        corners = self.initial_corners
        corner_latlng = [(lat, lng) for (lng, lat) in corners]
        corner_latlng.append(corner_latlng[0])

        self.iteration = 0
        for idx in range(len(corner_latlng) - 1):
            start_lat, start_lng = corner_latlng[idx]
            end_lat, end_lng = corner_latlng[idx + 1]

            corner_label = f"corner_{idx}"
            if idx == 0:
                self.iteration = 1

            self.measure_rsrp_fake(start_lat, start_lng, corner_label)
            self.move_drone_step_by_step(end_lat, end_lng, f"edge_{idx}")

        last_lat, last_lng = corner_latlng[-1]
        self.measure_rsrp_fake(last_lat, last_lng, "corner_last")
        print("Completed polygon boundary trace with corner measurements.\n")

    def polygon_bisection_search_fake(self, min_area_threshold=MIN_AREA_STOP_M2):
        print("Starting best-edge bisection with cached corner RSRP...")
        corners = self.initial_corners.copy()
        corner_labels = ["corner_0", "corner_1", "corner_2", "corner_3"]

        self.iteration = 1

        while True:
            print("\n" + "=" * 60)
            print(f"ITERATION {self.iteration}")
            print("=" * 60)

            area_sq_m = get_polygon_area_meters(corners)
            print(f"Current polygon area: {area_sq_m:.1f} m²")
            if area_sq_m < min_area_threshold:
                print("Area below threshold, stopping.")
                break

            print("Current polygon corners:")
            for i, (lng, lat) in enumerate(corners):
                print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")

            corner_measurements = []
            for i, (lng, lat) in enumerate(corners):
                pos_key = (round(lat, 8), round(lng, 8))
                if pos_key in self.measured_positions:
                    metric, _ues = self.measured_positions[pos_key]
                    print(f"  {corner_labels[i]}: cached metric={metric:.1f}")
                    corner_measurements.append(metric)
                else:
                    print(f"  {corner_labels[i]}: new corner, measuring...")
                    metric = self.measure_rsrp_fake(lat, lng, corner_labels[i])
                    corner_measurements.append(metric)

            edge_scores = []
            for i in range(4):
                j = (i + 1) % 4
                score = corner_measurements[i] + corner_measurements[j]
                edge_scores.append((score, i, f"edge_{i}_{j}"))
                print(
                    f"  Edge {i}-{j}: {corner_measurements[i]:.1f} + "
                    f"{corner_measurements[j]:.1f} = {score:.1f}"
                )

            best_score, best_edge_idx, best_edge_name = max(edge_scores, key=lambda x: x[0])
            print(f"\n*** BEST EDGE: {best_edge_name} with score {best_score:.1f} ***")

            new_corners = cut_polygon_in_half(corners, best_edge_idx)
            print("New polygon corners after cut:")
            for i, (lng, lat) in enumerate(new_corners):
                print(f"  New corner {i}: ({lat:.6f}, {lng:.6f})")

            corners = new_corners
            self.iteration += 1

        center_lng = sum(lng for (lng, lat) in corners) / 4.0
        center_lat = sum(lat for (lng, lat) in corners) / 4.0
        center_key = (round(center_lat, 8), round(center_lng, 8))

        if center_key in self.measured_positions:
            final_metric, _ues = self.measured_positions[center_key]
            print(f"Center already measured: metric={final_metric:.1f}")
        else:
            final_metric = self.measure_rsrp_fake(center_lat, center_lng, "final_center")

        print("\n" + "=" * 60)
        print("POLYGON OPTIMIZATION COMPLETE")
        print("=" * 60)
        print(f"Final optimum: ({center_lat:.6f}, {center_lng:.6f})")
        print(f"Final metric: {final_metric:.1f} dBm")
        print(f"Final polygon area: {get_polygon_area_meters(corners):.1f} m²")

        return center_lat, center_lng, final_metric, corners


# ---------------------------------------------------------------------------
# FINAL PLOT + SUMMARY
# ---------------------------------------------------------------------------

def plot_final_trajectory(buildings, trajectory, full_path, final_point, initial_corners, ue_coords):
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))

    all_lats = [lat for (lng, lat) in initial_corners]
    all_lngs = [lng for (lng, lat) in initial_corners]

    for lat, lng, *_ in trajectory:
        all_lats.append(lat)
        all_lngs.append(lng)
    for lat, lng in ue_coords:
        all_lats.append(lat)
        all_lngs.append(lng)

    min_lat, max_lat = min(all_lats), max(all_lats)
    min_lng, max_lng = min(all_lngs), max(all_lngs)

    if buildings:
        for b in buildings:
            x, y = b.exterior.xy
            bounds = b.bounds
            if (
                bounds[0] <= max_lng
                and bounds[2] >= min_lng
                and bounds[1] <= max_lat
                and bounds[3] >= min_lat
            ):
                ax.plot(x, y, "b-", linewidth=0.5, alpha=0.6)
                ax.fill(x, y, "lightblue", alpha=0.2)

    ue_lats = [lat for (lat, lng) in ue_coords]
    ue_lngs = [lng for (lat, lng) in ue_coords]
    ax.scatter(
        ue_lngs,
        ue_lats,
        c="red",
        s=120,
        marker="x",
        linewidth=2,
        alpha=0.9,
        label="UEs",
        zorder=6,
    )

    poly_x = [lng for (lng, lat) in initial_corners] + [initial_corners[0][0]]
    poly_y = [lat for (lng, lat) in initial_corners] + [initial_corners[0][1]]
    ax.plot(poly_x, poly_y, "r--", linewidth=2, alpha=0.8, label="Initial Polygon")

    if full_path:
        path_lats = [p[0] for p in full_path]
        path_lngs = [p[1] for p in full_path]
        ax.plot(
            path_lngs,
            path_lats,
            "purple",
            linewidth=2,
            alpha=0.7,
            label="Drone Trajectory",
            zorder=4,
        )

    if trajectory:
        lats = [p[0] for p in trajectory]
        lngs = [p[1] for p in trajectory]
        metrics = [p[2] for p in trajectory]

        finite = [m for m in metrics if m != float("-inf")]
        if finite:
            vmin, vmax = min(finite), max(finite)
        else:
            vmin, vmax = -110.0, -50.0

        sc = ax.scatter(
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
        cbar = plt.colorbar(sc, ax=ax)
        cbar.set_label("RSRP Metric (dBm)", rotation=270, labelpad=20)

        for i, (lat, lng, metric, ues, reason, it) in enumerate(trajectory):
            ax.annotate(
                f"{i+1}",
                (lng, lat),
                xytext=(0, 15),
                textcoords="offset points",
                fontsize=7,
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

    if final_point:
        f_lat, f_lng, f_metric = final_point
        ax.plot(
            f_lng,
            f_lat,
            "^",
            markersize=16,
            color="black",
            label=f"Optimum ({f_metric:.1f} dBm)",
            zorder=7,
        )

    ax.set_aspect("equal")
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("Final Drone Trajectory and Optimum (Fake RSRP)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    pad = meters_to_degrees(30.0)
    ax.set_xlim(min_lng - pad, max_lng + pad)
    ax.set_ylim(min_lat - pad, max_lat + pad)

    plt.tight_layout()
    plt.savefig("Final_Plot.png", dpi=200)
    tikzplotlib.save("Final_Plot.tex")
    plt.show()


def print_trajectory_summary(trajectory, full_path):
    print("\n" + "=" * 50)
    print("TRAJECTORY SUMMARY")
    print("=" * 50)
    print(f"Total measurements: {len(trajectory)}")
    print(f"Total flight path points: {len(full_path)}")

    if len(full_path) > 1:
        total_distance = 0.0
        for i in range(1, len(full_path)):
            lat1, lng1 = full_path[i - 1][0], full_path[i - 1][1]
            lat2, lng2 = full_path[i][0], full_path[i][1]
            total_distance += geo_distance_m(lat1, lng1, lat2, lng2)
        print(f"Total flight distance: {total_distance:.1f} m")

    by_it = {}
    for lat, lng, metric, ues, reason, it in trajectory:
        by_it.setdefault(it, []).append((lat, lng, metric, ues, reason))

    for it in sorted(by_it.keys()):
        print(f"\nIteration {it}:")
        for lat, lng, metric, ues, reason in by_it[it]:
            print(
                f"  {reason:18}: ({lat:.6f}, {lng:.6f}) → "
                f"Metric: {metric:.1f}, UEs: {ues}"
            )

    if trajectory:
        best = max(trajectory, key=lambda x: x[2])
        print(
            f"\nBest measurement: {best[2]:.1f} dBm with {best[3]} UEs "
            f"at ({best[0]:.6f}, {best[1]:.6f})"
        )


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        init_mqtt()
        print("Starting fake 4-corner polygon drone optimization...")

        initial_corners = create_custom_polygon()
        initial_area = get_polygon_area_meters(initial_corners)
        print(f"Initial polygon area: {initial_area:.1f} m²")
        print("Initial corners (clockwise):")
        for i, (lng, lat) in enumerate(initial_corners):
            print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")

        buildings, grid, G, key_to_node = load_graph()

        rng = np.random.default_rng(generate_random_seed(DEFAULT_SEED))
        ue_coords = generate_ues_in_polygon(initial_corners, rng=rng)

        init_live_plot(buildings, initial_corners, ue_coords)

        fake_rsrp_func = make_fake_rsrp_field(ue_coords, seed=generate_random_seed(DEFAULT_SEED))

        optimizer = PolygonDroneOptimizer(
            initial_corners,
            G,
            ue_coordinates=ue_coords,
            fake_rsrp_func=fake_rsrp_func,
            entity_id=ENTITY_ID,
        )

        optimizer.trace_polygon_boundary_and_measure_corners()
        input("Press Enter to start best-edge bisection (or Ctrl+C to cancel)...")
        final_lat, final_lng, final_metric, final_corners = optimizer.polygon_bisection_search_fake()

        print_trajectory_summary(optimizer.trajectory, optimizer.full_path)
        plot_final_trajectory(
            buildings,
            optimizer.trajectory,
            optimizer.full_path,
            (final_lat, final_lng, final_metric),
            initial_corners,
            ue_coords,
        )
    finally:
        shutdown()
