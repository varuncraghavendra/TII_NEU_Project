#!/usr/bin/env python3
import os
import math
import time
import json
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import tikzplotlib
from shapely.geometry import Point, Polygon
import paho.mqtt.client as mqtt

# ---------------------------------------------------------------------------
# BASIC CONFIG
# ---------------------------------------------------------------------------

CENTER_LAT = 42.33894284868896
CENTER_LNG = -71.08613491058351

# Target initial area and stop threshold
TARGET_AREA_M2 = 5000.0        # ~1000 m² square
MIN_AREA_STOP_M2 = 100.0       # stop bisection when area < 100 m²

UE_CONN_RADIUS_M = 60.0
ENTITY_ID = 30

DRONE_SPEED_MPS = 7.0           # used only for timing
WAYPOINT_PERIOD_S = 0.25        # GPS setpoint rate
MIN_TRAVEL_TIME_S = 3.0         # minimum move time
MAX_TRAVEL_TIME_S = 10.0        # clamp move time to <= 10s
MEASURE_HOLD_S = 3.0            # 3-second averaging per sample

# Path loss params for all UEs (cluster-average RSRP)
P0_DBM = -50.0                  # reference RSRP at 1 m
PATHLOSS_N = 3.0                # path-loss exponent
NOISE_SIGMA = 1.5               # dB noise

# How aggressively we shrink polygon towards best edge (0.5 = equal half)
SHRINK_FACTOR = 0.7             # >0.5 biases region towards best edge

MQTT_HOST = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "colosseum/update")
DRONE_ID = int(os.getenv("DRONE_ID", "1"))

DEFAULT_SEED = None
ENV_SEED = os.getenv("RANDOM_SEED")
if ENV_SEED is not None:
    try:
        DEFAULT_SEED = int(ENV_SEED)
    except Exception:
        DEFAULT_SEED = None

if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

LIVE_FIG = None
LIVE_AX = None
LIVE_ARROW = None
_MQTT_CLIENT = None


# ---------------------------------------------------------------------------
# GEO UTILS
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


def offset_from_home(north_m, east_m):
    lat = CENTER_LAT + meters_to_degrees(north_m)
    lng = CENTER_LNG + meters_to_degrees(
        east_m, is_longitude=True, center_lat=CENTER_LAT
    )
    return lat, lng


def create_square_polygon():
    """
    Build a ~1000 m² square, with home as bottom-left corner.
    """
    side_m = math.sqrt(TARGET_AREA_M2)

    bl_lat, bl_lng = offset_from_home(0.0, 0.0)           # home = first corner
    br_lat, br_lng = offset_from_home(0.0, side_m)
    tr_lat, tr_lng = offset_from_home(side_m, side_m)
    tl_lat, tl_lng = offset_from_home(side_m, 0.0)

    bottom_left = (bl_lng, bl_lat)
    bottom_right = (br_lng, br_lat)
    top_right = (tr_lng, tr_lat)
    top_left = (tl_lng, tl_lat)

    return [bottom_left, bottom_right, top_right, top_left]


def get_polygon_area_meters(corners):
    poly = Polygon(corners)
    return poly.area * (111111.0 ** 2)


def get_midpoint(p1, p2):
    return ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)


def weighted_point(p_from, p_to, w):
    """
    Return point closer to p_from if w>0.5.
    p(w) = w*p_from + (1-w)*p_to
    """
    return (
        w * p_from[0] + (1.0 - w) * p_to[0],
        w * p_from[1] + (1.0 - w) * p_to[1],
    )


def cut_polygon_in_half(corners, best_edge_idx):
    """
    corners: list of 4 (lng, lat) in CCW order.
    Returns a new convex quadrilateral whose area shrinks
    towards the chosen edge (not necessarily equal halves).

    For best edge CD, we pull the opposite side closer to C and D
    with a SHRINK_FACTOR > 0.5 to accelerate convergence.
    """
    c_idx = best_edge_idx
    d_idx = (best_edge_idx + 1) % 4
    a_idx = (best_edge_idx + 2) % 4
    b_idx = (best_edge_idx + 3) % 4

    point_c = corners[c_idx]
    point_d = corners[d_idx]
    point_a = corners[a_idx]
    point_b = corners[b_idx]

    # Move A closer to D, B closer to C, with weight SHRINK_FACTOR
    shrunk_da = weighted_point(point_d, point_a, SHRINK_FACTOR)
    shrunk_cb = weighted_point(point_c, point_b, SHRINK_FACTOR)

    # New polygon: C -> D -> shrunk_da -> shrunk_cb
    new_corners = [point_c, point_d, shrunk_da, shrunk_cb]
    return new_corners


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


# ---------------------------------------------------------------------------
# UE CLUSTER + CLUSTER-AVERAGE RSRP
# ---------------------------------------------------------------------------

def generate_ue_cluster_anywhere(initial_corners, rng=None,
                                 min_ues=5, max_ues=8):
    """
    Generate a cluster of 5–8 UEs anywhere inside the initial polygon.
    We first pick a random cluster center inside the polygon, then
    sample UEs in a small radius (~15 m) around it.
    """
    if rng is None:
        rng = np.random.default_rng()

    poly = Polygon(initial_corners)
    minx, miny, maxx, maxy = poly.bounds

    # Pick cluster center anywhere in polygon
    center_lng = None
    center_lat = None
    for _ in range(2000):
        test_lng = rng.uniform(minx, maxx)
        test_lat = rng.uniform(miny, maxy)
        if poly.contains(Point(test_lng, test_lat)):
            center_lng, center_lat = test_lng, test_lat
            break

    if center_lng is None:
        # fallback to centroid if random search fails
        centroid = poly.centroid
        center_lng, center_lat = centroid.x, centroid.y

    # Now sample UEs around that center within ~15 m radius
    cluster_radius_m = 15.0
    n_ues = int(rng.integers(min_ues, max_ues + 1))
    ue_coords = []
    tries = 0
    while len(ue_coords) < n_ues and tries < 5000:
        tries += 1
        r = cluster_radius_m * math.sqrt(rng.random())
        theta = 2.0 * math.pi * rng.random()
        dx_m = r * math.cos(theta)
        dy_m = r * math.sin(theta)

        lat = center_lat + meters_to_degrees(dy_m)
        lng = center_lng + meters_to_degrees(
            dx_m, is_longitude=True, center_lat=center_lat
        )

        if poly.contains(Point(lng, lat)):
            ue_coords.append((lat, lng))

    print(f"[UE CLUSTER] Generated {len(ue_coords)} UEs:")
    for i, (lat, lng) in enumerate(ue_coords):
        print(f"  UE {i}: ({lat:.6f}, {lng:.6f})")
    return ue_coords


def cluster_average_rsrp(lat, lng, ue_coords):
    """
    Compute cluster-average RSRP at the drone location:
    - For each UE within UE_CONN_RADIUS_M, compute RSRP_i using
      a simple log-distance path-loss model.
    - Return the average RSRP over all connected UEs.

    This matches the idea of "assign an average RSRP for all UEs in the cluster".
    """
    metrics = []
    for ue_lat, ue_lng in ue_coords:
        d = geo_distance_m(lat, lng, ue_lat, ue_lng)
        if d <= UE_CONN_RADIUS_M:
            d_eff = max(d, 1.0)
            base = P0_DBM - 10.0 * PATHLOSS_N * math.log10(d_eff)
            m_i = base + np.random.normal(0.0, NOISE_SIGMA)
            metrics.append(m_i)

    if not metrics:
        # no UEs connected
        return -120.0, 0

    avg_metric = float(np.mean(metrics))
    return avg_metric, len(metrics)


# ---------------------------------------------------------------------------
# LIVE PLOT
# ---------------------------------------------------------------------------

def init_live_plot(initial_corners, ue_coords):
    global LIVE_FIG, LIVE_AX
    plt.ion()
    LIVE_FIG, LIVE_AX = plt.subplots(1, 1, figsize=(10, 8))

    poly_x = [lng for (lng, lat) in initial_corners] + [initial_corners[0][0]]
    poly_y = [lat for (lng, lat) in initial_corners] + [initial_corners[0][1]]
    LIVE_AX.plot(poly_x, poly_y, "r--", linewidth=2, alpha=0.8, label="Initial ~1000 m² Square")

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
            label="UE Cluster",
        )

    all_lats = [lat for (lng, lat) in initial_corners] + [lat for (lat, lng) in ue_coords]
    all_lngs = [lng for (lng, lat) in initial_corners] + [lng for (lat, lng) in ue_coords]
    pad = meters_to_degrees(30.0)
    LIVE_AX.set_xlim(min(all_lngs) - pad, max(all_lngs) + pad)
    LIVE_AX.set_ylim(min(all_lats) - pad, max(all_lats) + pad)

    LIVE_AX.set_aspect("equal")
    LIVE_AX.set_xlabel("Longitude")
    LIVE_AX.set_ylabel("Latitude")
    LIVE_AX.set_title("Live Drone Polygon Bisection (Cluster-Average RSRP)")
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


def live_plot_add_point(lat, lng):
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
    def __init__(self, initial_corners, ue_coordinates, entity_id=ENTITY_ID):
        self.initial_corners = initial_corners
        self.ue_coordinates = ue_coordinates or []
        self.entity_id = entity_id

        self.trajectory = []   # (lat, lng, avg_metric, connected_ues, reason, iteration)
        self.full_path = []    # (lat, lng, reason)
        self.iteration = 0
        self.current_position = None
        self.measured_positions = {}  # (lat, lng) -> (avg_metric, connected_ues)

    def _stream_target_for_duration(self, target_lat, target_lng, duration_s, reason):
        repeats = max(int(duration_s / WAYPOINT_PERIOD_S), 1)
        print(f"  Streaming {reason} for {duration_s:.1f}s ({repeats} setpoints)")
        for _ in range(repeats):
            send_position_update(target_lat, target_lng, self.entity_id)
            self.full_path.append((target_lat, target_lng, reason))
            live_plot_add_point(target_lat, target_lng)
            time.sleep(WAYPOINT_PERIOD_S)

    def move_drone_to(self, target_lat, target_lng, reason="movement"):
        """
        Move between corners with time computed from distance and DRONE_SPEED_MPS.
        Note: MEASURE_HOLD_S (3s) is *only* used after arriving at the corner, so
        travel time is not part of the 3s averaging window.
        """
        if self.current_position is None:
            current_lat, current_lng = CENTER_LAT, CENTER_LNG
        else:
            current_lat, current_lng = self.current_position

        dist = geo_distance_m(current_lat, current_lng, target_lat, target_lng)
        travel_time_raw = dist / DRONE_SPEED_MPS if DRONE_SPEED_MPS > 0 else MIN_TRAVEL_TIME_S
        travel_time = max(MIN_TRAVEL_TIME_S, min(travel_time_raw, MAX_TRAVEL_TIME_S))

        print(f"Distance to {reason}: {dist:.1f} m → travel_time={travel_time:.1f}s")

        if self.current_position is not None and LIVE_AX is not None:
            LIVE_AX.plot(
                [current_lng, target_lng],
                [current_lat, target_lat],
                "k-",
                linewidth=1.5,
                alpha=0.5,
            )
            LIVE_FIG.canvas.draw()
            LIVE_FIG.canvas.flush_events()
            update_direction_arrow(current_lat, current_lng, target_lat, target_lng)

        self._stream_target_for_duration(target_lat, target_lng, travel_time, reason)
        clear_direction_arrow()
        self.current_position = (target_lat, target_lng)

    def measure_rsrp_fake(self, lat, lng, reason="measurement"):
        """
        Move to the point, then average cluster RSRP over MEASURE_HOLD_S = 3s.
        Travel time is excluded from the averaging window.
        """
        print(f"\n--- Moving to {reason} at ({lat:.6f}, {lng:.6f}) ---")
        self.move_drone_to(lat, lng, reason)

        print("Holding for measurement (3s average)...")
        samples = []
        start_t = time.time()
        last_ues = 0
        while time.time() - start_t < MEASURE_HOLD_S:
            send_position_update(lat, lng, self.entity_id)
            live_plot_add_point(lat, lng)
            metric, connected_ues = cluster_average_rsrp(lat, lng, self.ue_coordinates)
            samples.append(metric)
            last_ues = connected_ues
            self.full_path.append((lat, lng, reason + "_hold"))
            time.sleep(WAYPOINT_PERIOD_S)

        avg_metric = float(np.mean(samples)) if samples else -120.0
        print(f"Average cluster RSRP over 3s: {avg_metric:.1f} dBm, UEs={last_ues}")

        self.trajectory.append((lat, lng, avg_metric, last_ues, reason, self.iteration))
        pos_key = (round(lat, 8), round(lng, 8))
        self.measured_positions[pos_key] = (avg_metric, last_ues)

        return avg_metric

    def trace_polygon_boundary_and_measure_corners(self):
        print("\n" + "=" * 60)
        print("INITIAL ~1000 m² SQUARE TRACE (CLOCKWISE) + CORNER RSRP (3s avg)")
        print("=" * 60)

        corners = self.initial_corners
        corner_latlng = [(lat, lng) for (lng, lat) in corners]
        corner_latlng.append(corner_latlng[0])  # close the loop

        self.iteration = 0
        for idx in range(len(corner_latlng) - 1):
            start_lat, start_lng = corner_latlng[idx]
            end_lat, end_lng = corner_latlng[idx + 1]

            corner_label = f"corner_{idx}"
            if idx == 0:
                self.iteration = 1

            # At each corner: 3s average, then move to next edge
            self.measure_rsrp_fake(start_lat, start_lng, corner_label)
            self.move_drone_to(end_lat, end_lng, f"edge_{idx}")

        last_lat, last_lng = corner_latlng[-1]
        self.measure_rsrp_fake(last_lat, last_lng, "corner_last")
        print("Completed square boundary trace with corner measurements.\n")

    def polygon_bisection_search_fake(self, min_area_threshold=MIN_AREA_STOP_M2):
        """
        Best-edge bisection:
        - Use cached 3s-average RSRP at corners (cluster-average).
        - Score each edge as sum of its two corner metrics.
        - Choose best edge, measure its midpoint, then shrink polygon
          towards that edge (not equal halves) using SHRINK_FACTOR.
        - Repeat until polygon area < min_area_threshold.
        """
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
                print("Area below threshold, stopping bisection.")
                break

            print("Current polygon corners:")
            for i, (lng, lat) in enumerate(corners):
                print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")

            # Corner measurements (reuse 3s averages if cached)
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

            # Edge scores from adjacent corners
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

            # Measure midpoint of best edge (3s average, travel excluded)
            c_idx = best_edge_idx
            d_idx = (best_edge_idx + 1) % 4
            lng_c, lat_c = corners[c_idx]
            lng_d, lat_d = corners[d_idx]
            mid_lng, mid_lat = get_midpoint((lng_c, lat_c), (lng_d, lat_d))
            print(f"Measuring midpoint of best edge between corner {c_idx} and {d_idx}")
            self.measure_rsrp_fake(mid_lat, mid_lng, f"best_edge_{c_idx}_{d_idx}_mid")

            # Shrink polygon towards best edge (not necessarily equal halves)
            new_corners = cut_polygon_in_half(corners, best_edge_idx)
            new_area = get_polygon_area_meters(new_corners)
            print(f"New quadrilateral after cut (area ≈ {new_area:.1f} m²):")
            for i, (lng, lat) in enumerate(new_corners):
                print(f"  New corner {i}: ({lat:.6f}, {lng:.6f})")

            corners = new_corners
            self.iteration += 1

        # Final measurement at polygon center
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
        print(f"Final optimum center: ({center_lat:.6f}, {center_lng:.6f})")
        print(f"Final center metric (avg over 3s): {final_metric:.1f} dBm")
        print(f"Final polygon area: {get_polygon_area_meters(corners):.1f} m²")

        return center_lat, center_lng, final_metric, corners


# ---------------------------------------------------------------------------
# FINAL PLOT + SUMMARY
# ---------------------------------------------------------------------------

def plot_final_trajectory(trajectory, full_path, final_point, initial_corners, ue_coords):
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))

    all_lats = []
    all_lngs = []

    for lat, lng in ue_coords:
        all_lats.append(lat)
        all_lngs.append(lng)

    for lat, lng, _reason in full_path:
        all_lats.append(lat)
        all_lngs.append(lng)

    if final_point:
        all_lats.append(final_point[0])
        all_lngs.append(final_point[1])

    if not all_lats:
        all_lats = [lat for (lng, lat) in initial_corners]
        all_lngs = [lng for (lng, lat) in initial_corners]

    min_lat, max_lat = min(all_lats), max(all_lats)
    min_lng, max_lng = min(all_lngs), max(all_lngs)

    ue_lats = [lat for (lat, lng) in ue_coords]
    ue_lngs = [lng for (lat, lng) in ue_coords]
    if ue_coords:
        ax.scatter(
            ue_lngs,
            ue_lats,
            c="red",
            s=120,
            marker="x",
            linewidth=2,
            alpha=0.9,
            label="UE Cluster",
            zorder=6,
        )

    poly_x = [lng for (lng, lat) in initial_corners] + [initial_corners[0][0]]
    poly_y = [lat for (lng, lat) in initial_corners] + [initial_corners[0][1]]
    ax.plot(poly_x, poly_y, "r--", linewidth=2, alpha=0.8, label="Initial ~1000 m² Square")

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
        cbar.set_label("Average Cluster RSRP (dBm)", rotation=270, labelpad=20)

    if final_point:
        f_lat, f_lng, f_metric = final_point
        ax.plot(
            f_lng,
            f_lat,
            "^",
            markersize=16,
            color="green",
            label=f"Optimum Center ({f_metric:.1f} dBm)",
            zorder=7,
        )

    ax.set_aspect("equal")
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("Final Drone Trajectory and Optimum (Cluster-Average RSRP)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    pad = meters_to_degrees(10.0)
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
    print(f"Total commanded positions: {len(full_path)}")

    if len(full_path) > 1:
        total_distance = 0.0
        for i in range(1, len(full_path)):
            lat1, lng1 = full_path[i - 1][0], full_path[i - 1][1]
            lat2, lng2 = full_path[i][0], full_path[i][1]
            total_distance += geo_distance_m(lat1, lng1, lat2, lng2)
        print(f"Total flight distance (approx): {total_distance:.1f} m")

    if trajectory:
        best = max(trajectory, key=lambda x: x[2])
        print(
            f"\nBest measurement (any point): {best[2]:.1f} dBm with {best[3]} UEs "
            f"at ({best[0]:.6f}, {best[1]:.6f})"
        )


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        init_mqtt()
        print("Starting ~1000 m² square drone optimization with UE cluster...")

        initial_corners = create_square_polygon()
        initial_area = get_polygon_area_meters(initial_corners)
        print(f"Initial square area (approx): {initial_area:.1f} m²")
        print("Initial corners (clockwise, corner 0 is HOME):")
        for i, (lng, lat) in enumerate(initial_corners):
            print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")

        rng = np.random.default_rng(generate_random_seed(DEFAULT_SEED))
        ue_coords = generate_ue_cluster_anywhere(initial_corners, rng=rng)

        init_live_plot(initial_corners, ue_coords)

        optimizer = PolygonDroneOptimizer(
            initial_corners,
            ue_coordinates=ue_coords,
            entity_id=ENTITY_ID,
        )

        optimizer.trace_polygon_boundary_and_measure_corners()
        input("Press Enter to start best-edge bisection (or Ctrl+C to cancel)...")
        final_lat, final_lng, final_metric, final_corners = (
            optimizer.polygon_bisection_search_fake()
        )

        print_trajectory_summary(optimizer.trajectory, optimizer.full_path)
        plot_final_trajectory(
            optimizer.trajectory,
            optimizer.full_path,
            (final_lat, final_lng, final_metric),
            initial_corners,
            ue_coords,
        )
    finally:
        shutdown()
