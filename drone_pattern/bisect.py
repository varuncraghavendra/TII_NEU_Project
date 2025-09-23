#!/usr/bin/env python3
import requests
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, LineString
import numpy as np
import networkx as nx
import pickle
import os
from hashlib import md5
from tqdm import tqdm
import time
import tikzplotlib

# Import real data functions (kept as-is)
from influx_hook import get_average_rsrp_3s
from mqtt_client import send_position_update

# ---------------- MQTT Bridge (added) ---------------- #
import paho.mqtt.client as mqtt
import json as _json  # avoid shadowing 'json' elsewhere

_MQTT_BROKER = "broker.hivemq.com"
_MQTT_PORT = 1883
_MQTT_TOPIC = "colosseum/update"
_DRONE_ID = 1  # change if your ROS2 consumer expects a different ID

_mqtt_client = mqtt.Client(protocol=mqtt.MQTTv311)
try:
    _mqtt_client.connect(_MQTT_BROKER, _MQTT_PORT, 60)
    _mqtt_client.loop_start()
except Exception as _e:
    print(f"[WARN] MQTT connect failed: {_e}")

def publish_gps_for_ros2(lat, lng, iteration, target_ue=None, rsrp=None, distance=None, signal_quality=None):
    """
    Publish GPS in the same schema your working script uses, so goal_manager_server.py
    can parse it directly from MQTT.
    """
    payload = {
        "drone_id": int(_DRONE_ID),
        "lat": float(lat),
        "lon": float(lng),
        "rsrp": float(rsrp) if rsrp is not None else None,
        "distance": float(distance) if distance is not None else None,
        "signal_quality": float(signal_quality) if signal_quality is not None else None,
        "iteration": int(iteration) if iteration is not None else None,
        "target_ue": target_ue if isinstance(target_ue, (int, float)) else str(target_ue) if target_ue is not None else None
    }
    try:
        _mqtt_client.publish(_MQTT_TOPIC, _json.dumps(payload))
    except Exception as _e:
        print(f"[WARN] MQTT publish failed: {_e}")
# ---------------- End MQTT Bridge -------------------- #

# Area parameters - 1km radius with 5m spacing
CENTER_LAT = 42.33894284868896
CENTER_LNG = -71.08613491058351
RADIUS_METERS = 1000  # 1km radius
GRID_SPACING_METERS = 5  # 5m spacing between points

# Custom boundaries for the actual optimization area
CUSTOM_BOUNDS = {
    'top_left': [42.34138538637752, -71.08300209045412],
    'top_right': [42.340314805239075, -71.08198285102846],
    'bottom_right': [42.337903948379996, -71.08519077301027],
    'bottom_left': [42.33866528158443, -71.08609199523927]
}

#28 42.338721, -71.086028
#16 42.338689, -71.084268
#26 42.337809, -71.085008

def get_cache_filename(cache_type="buildings"):
    """Generate cache filename based on location and radius"""
    key = f"{CENTER_LAT}_{CENTER_LNG}_{RADIUS_METERS}"
    if cache_type == "graph":
        key += f"_grid{GRID_SPACING_METERS}m"
    hash_key = md5(key.encode()).hexdigest()[:8]
    return f"{cache_type}_cache_{hash_key}.pkl"

def meters_to_degrees(meters, is_longitude=False, center_lat=None):
    """Convert meters to degrees"""
    if is_longitude and center_lat is not None:
        return meters / (111111.0 * np.cos(np.radians(center_lat)))
    else:
        return meters / 111111.0

def degrees_to_meters(degrees, is_longitude=False, center_lat=None):
    """Convert degrees to meters"""
    if is_longitude and center_lat is not None:
        return degrees * (111111.0 * np.cos(np.radians(center_lat)))
    else:
        return degrees * 111111.0

def create_custom_polygon():
    """Create a polygon from the custom boundaries in clockwise order"""
    # Order: top_left -> top_right -> bottom_right -> bottom_left -> back to top_left
    coords = [
        (CUSTOM_BOUNDS['top_left'][1], CUSTOM_BOUNDS['top_left'][0]),      # (lng, lat)
        (CUSTOM_BOUNDS['top_right'][1], CUSTOM_BOUNDS['top_right'][0]),
        (CUSTOM_BOUNDS['bottom_right'][1], CUSTOM_BOUNDS['bottom_right'][0]),
        (CUSTOM_BOUNDS['bottom_left'][1], CUSTOM_BOUNDS['bottom_left'][0])
    ]
    return coords  # Return as list of (lng, lat) tuples

def get_polygon_area_meters(corners):
    """Calculate polygon area in square meters"""
    # Convert to Shapely polygon for area calculation
    poly = Polygon(corners)
    # Rough conversion from degrees^2 to meters^2
    area_sq_meters = poly.area * (111111**2)
    return area_sq_meters

def get_midpoint(point1, point2):
    """Get midpoint between two points"""
    return ((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2)

def cut_polygon_in_half(corners, best_edge_idx):
    """
    Cut polygon in half keeping the best edge.
    For best edge CD, new polygon is [C, D, midpoint(BC), midpoint(AD)]
    """
    # Get the best edge points (these stay in the new polygon)
    c_idx = best_edge_idx
    d_idx = (best_edge_idx + 1) % 4
    
    # Get the other two points
    a_idx = (best_edge_idx + 2) % 4
    b_idx = (best_edge_idx + 3) % 4
    
    point_c = corners[c_idx]
    point_d = corners[d_idx]
    point_a = corners[a_idx]
    point_b = corners[b_idx]
    
    # Create new corners: midpoint of AD and midpoint of BC
    mid_ad = get_midpoint(point_a, point_d)  # A' = midpoint of AD
    mid_bc = get_midpoint(point_b, point_c)  # B' = midpoint of BC
    
    # New polygon: [C, D, B', A'] to maintain proper clockwise order
    new_corners = [point_c, point_d, mid_bc, mid_ad]
    
    return new_corners

def load_graph():
    """Load graph from cache"""
    cache_file = get_cache_filename("graph")
    buildings_cache = get_cache_filename("buildings")
    
    print(f"Loading graph from cache: {cache_file}")
    with open(cache_file, 'rb') as f:
        cached_data = pickle.load(f)
    
    print(f"Loading buildings from cache: {buildings_cache}")
    with open(buildings_cache, 'rb') as f:
        buildings = pickle.load(f)
    
    return buildings, cached_data['grid'], cached_data['graph'], cached_data['key_to_node']

def get_real_rsrp_metric_robust():
    """Get real RSRP data with retry logic, focusing on maximizing average RSRP"""
    max_retries = 1  # Will try up to 2 times total
    
    for attempt in range(max_retries + 1):
        try:
            time.sleep(5)
            result = get_average_rsrp_3s()
            print(f"Attempt {attempt + 1}: Average RSRP per RNTI:")
            
            if not result:
                print("No data collected")
                if attempt < max_retries:
                    print("Waiting 60 seconds before retrying...")
                    time.sleep(60)
                    continue
                else:
                    return float('-inf'), 0  # No metric, 0 connected UEs
            
            valid_rsrp_values = []
            ue_count = len(result)
            
            for rnti, avg_rsrp in result.items():
                if avg_rsrp == float('-inf'):
                    print(f"  RNTI {rnti}: -∞ dBm (contains zero values)")
                else:
                    print(f"  RNTI {rnti}: {avg_rsrp:.1f} dBm")
                    valid_rsrp_values.append(avg_rsrp)
            
            # Check if we have any valid RSRP values
            if not valid_rsrp_values:
                if attempt < max_retries:
                    print("No valid RSRP values detected. Waiting 60 seconds to retry...")
                    time.sleep(60)
                    continue
                else:
                    print(f"After {max_retries + 1} attempts, no valid RSRP values. Using -inf.")
                    return float('-inf'), 0
            
            # Calculate average RSRP as the metric
            avg_rsrp = sum(valid_rsrp_values) / len(valid_rsrp_values)
            connected_ues = len(valid_rsrp_values)
            
            print(f"  -> Average RSRP: {avg_rsrp:.1f} dBm from {connected_ues} UEs")
            print(f"  -> Metric (avg RSRP): {avg_rsrp:.1f}")
            
            return avg_rsrp, connected_ues
            
        except Exception as e:
            print(f"Error getting RSRP data (attempt {attempt + 1}): {e}")
            if attempt < max_retries:
                print("Waiting 60 seconds before retrying...")
                time.sleep(60)
            else:
                return float('-inf'), 0

def find_nearest_graph_node(G, target_lat, target_lng):
    """Find the nearest node in the graph to target coordinates"""
    min_distance = float('inf')
    nearest_node = None
    
    for node in G.nodes():
        node_lat = G.nodes[node]['lat']
        node_lng = G.nodes[node]['lng']
        
        # Calculate distance
        distance = np.sqrt((node_lat - target_lat)**2 + (node_lng - target_lng)**2)
        
        if distance < min_distance:
            min_distance = distance
            nearest_node = node
    
    return nearest_node

def get_shortest_path_between_points(G, lat1, lng1, lat2, lng2):
    """Get shortest path between two coordinate points using the graph"""
    node1 = find_nearest_graph_node(G, lat1, lng1)
    node2 = find_nearest_graph_node(G, lat2, lng2)
    
    if node1 is None or node2 is None:
        return None, None
    
    try:
        path_nodes = nx.shortest_path(G, node1, node2, weight='weight')
        path_coords = [(G.nodes[node]['lat'], G.nodes[node]['lng']) for node in path_nodes]
        path_length = nx.shortest_path_length(G, node1, node2, weight='weight')
        return path_coords, path_length
    except nx.NetworkXNoPath:
        print(f"No path found between ({lat1:.6f}, {lng1:.6f}) and ({lat2:.6f}, {lng2:.6f})")
        return None, None

class PolygonDroneOptimizer:
    def __init__(self, initial_corners, graph, entity_id=30):
        self.initial_corners = initial_corners
        self.graph = graph
        self.entity_id = entity_id
        self.trajectory = []  # List of (lat, lng, metric, connected_ues, reason, iteration)
        self.full_path = []   # Complete path coordinates including travel between points
        self.iteration = 0
        self.current_position = None
        
    def move_drone_step_by_step(self, target_lat, target_lng, reason="movement"):
        """Move drone to target position step by step using shortest path and MQTT"""
        if self.current_position is None:
            # First movement - teleport to start position
            print(f"Teleporting drone to start position: ({target_lat:.6f}, {target_lng:.6f})")
            # Existing publish (kept unchanged)
            send_position_update(target_lat, target_lng, self.entity_id)
            # NEW: Publish in ROS2-expected MQTT JSON schema
            publish_gps_for_ros2(target_lat, target_lng, self.iteration, target_ue=reason)
            self.current_position = (target_lat, target_lng)
            self.full_path.append((target_lat, target_lng, reason))
            time.sleep(0.15)  # Wait for position update
            return
        
        # Find shortest path from current position to target
        current_lat, current_lng = self.current_position
        path_coords, path_length = get_shortest_path_between_points(
            self.graph, current_lat, current_lng, target_lat, target_lng
        )
        
        if path_coords and len(path_coords) > 1:
            print(f"Moving drone via {len(path_coords)} waypoints to {reason}")
            
            # Move step by step through the path
            for i, (lat, lng) in enumerate(path_coords):
                if i == 0:
                    continue  # Skip first point (current position)
                
                # Send position update via your existing MQTT path (kept)
                send_position_update(lat, lng, self.entity_id)
                # Mirror to ROS2 consumer with the same schema expected by goal manager
                publish_gps_for_ros2(lat, lng, self.iteration, target_ue=f"travel_to_{reason}" if i < len(path_coords)-1 else reason)
                
                # Update tracking
                path_reason = f"travel_to_{reason}"
                if i == len(path_coords) - 1:
                    path_reason = reason  # Mark the final destination
                self.full_path.append((lat, lng, path_reason))
                
                # Sleep between movements
                time.sleep(0.5)
            
            # Update current position
            self.current_position = (target_lat, target_lng)
            
            path_length_meters = path_length * 111111.0 if path_length else 0
            print(f"Reached target after traveling {path_length_meters:.1f}m")
        else:
            print(f"Warning: No path found to target, using direct movement")
            send_position_update(target_lat, target_lng, self.entity_id)
            publish_gps_for_ros2(target_lat, target_lng, self.iteration, target_ue=reason)
            self.full_path.append((target_lat, target_lng, reason))
            self.current_position = (target_lat, target_lng)
            time.sleep(0.05)
    
    def measure_rsrp_real(self, lat, lng, reason="measurement"):
        """Move to position and measure real RSRP with robust retry logic"""
        print(f"\n--- Moving to {reason} at ({lat:.6f}, {lng:.6f}) ---")
        
        # Move to the measurement position
        self.move_drone_step_by_step(lat, lng, reason)
        
        # Wait for drone to stabilize
        print("Waiting 0.5s for drone to stabilize...")
        time.sleep(0.5)
        
        # Take the measurement with retry logic
        print("Taking RSRP measurement...")
        metric, connected_ues = get_real_rsrp_metric_robust()
        
        # Optional: Emit measurement with metric mirrored into MQTT (safe: ROS2 can ignore extra fields)
        publish_gps_for_ros2(lat, lng, self.iteration, target_ue=reason, rsrp=metric)
        
        self.trajectory.append((lat, lng, metric, connected_ues, reason, self.iteration))
        print(f"Measurement complete: metric={metric:.1f}, connected_UEs={connected_ues}")
        
        return metric
    
    def polygon_bisection_search_real(self, min_area_threshold=100):  # 100 sq meters
        """
        Simplified polygon bisection search for 4-corner polygons
        """
        print("Starting REAL 4-corner polygon bisection search...")
        print("This works with arbitrary 4-sided shapes!")
        
        # Confirm before starting
        input("Press Enter to start the real polygon optimization (or Ctrl+C to cancel)...")
        
        current_corners = self.initial_corners.copy()
        measured_positions = {}  # Cache measurements
        
        corner_labels = ["corner_0", "corner_1", "corner_2", "corner_3"]
        
        while True:
            self.iteration += 1
            print(f"\n{'='*60}")
            print(f"ITERATION {self.iteration}")
            print(f"{'='*60}")
            
            # Check if polygon is small enough
            area_sq_meters = get_polygon_area_meters(current_corners)
            print(f"Current polygon area: {area_sq_meters:.1f} sq meters")
            
            if area_sq_meters < min_area_threshold:
                print("Polygon is small enough, stopping search.")
                break
            
            print(f"Current polygon corners:")
            for i, (lng, lat) in enumerate(current_corners):
                print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")
            
            # Measure RSRP at corners
            corner_measurements = []
            
            if self.iteration == 1:
                # First iteration: measure all 4 corners
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
                        
                        # Cache the measurement
                        last_measurement = self.trajectory[-1]
                        measured_positions[pos_key] = (last_measurement[2], last_measurement[3])
            else:
                # Subsequent iterations: only measure the 2 new corners (indices 2 and 3)
                # Corners 0 and 1 are from the previous best edge, already measured
                print("Subsequent iteration: measuring only 2 new corners")
                
                for i, (lng, lat) in enumerate(current_corners):
                    pos_key = (round(lat, 8), round(lng, 8))
                    
                    if i < 2:
                        # These are from the previous best edge, should be cached
                        if pos_key in measured_positions:
                            cached_metric, cached_ues = measured_positions[pos_key]
                            print(f"  {corner_labels[i]}: Using cached (from best edge) -> Metric: {cached_metric:.1f}")
                            corner_measurements.append(cached_metric)
                        else:
                            # Shouldn't happen, but measure if needed
                            print(f"  {corner_labels[i]}: Unexpected new measurement needed")
                            metric = self.measure_rsrp_real(lat, lng, corner_labels[i])
                            corner_measurements.append(metric)
                            last_measurement = self.trajectory[-1]
                            measured_positions[pos_key] = (last_measurement[2], last_measurement[3])
                    else:
                        # These are new corners (midpoints), need to measure
                        if pos_key in measured_positions:
                            cached_metric, cached_ues = measured_positions[pos_key]
                            print(f"  {corner_labels[i]}: Using cached -> Metric: {cached_metric:.1f}")
                            corner_measurements.append(cached_metric)
                        else:
                            print(f"  {corner_labels[i]}: New corner, measuring...")
                            metric = self.measure_rsrp_real(lat, lng, corner_labels[i])
                            corner_measurements.append(metric)
                            
                            # Cache the measurement
                            last_measurement = self.trajectory[-1]
                            measured_positions[pos_key] = (last_measurement[2], last_measurement[3])
            
            # Find the best adjacent pair (edge) by checking all 4 edges
            edge_scores = []
            for i in range(4):
                j = (i + 1) % 4  # Next corner (wrapping around)
                edge_score = corner_measurements[i] + corner_measurements[j]
                edge_scores.append((edge_score, i, f"edge_{i}_{j}"))
                print(f"  Edge {i}-{j}: score = {corner_measurements[i]:.1f} + {corner_measurements[j]:.1f} = {edge_score:.1f}")
            
            # Find best edge
            best_score, best_edge_idx, best_edge_name = max(edge_scores, key=lambda x: x[0])
            print(f"\n*** BEST EDGE: {best_edge_name} with combined score {best_score:.1f} ***")
            
            # Cut polygon in half, keeping the best edge
            new_corners = cut_polygon_in_half(current_corners, best_edge_idx)
            
            print(f"New polygon corners after cut:")
            for i, (lng, lat) in enumerate(new_corners):
                print(f"  New corner {i}: ({lat:.6f}, {lng:.6f})")
            
            current_corners = new_corners
        
        # Final measurement at polygon center
        center_lng = sum(lng for lng, lat in current_corners) / 4
        center_lat = sum(lat for lng, lat in current_corners) / 4
        
        final_pos_key = (round(center_lat, 8), round(center_lng, 8))
        if final_pos_key in measured_positions:
            cached_metric, cached_ues = measured_positions[final_pos_key]
            print(f"Final center already measured: {cached_metric:.1f}")
            final_metric = cached_metric
        else:
            final_metric = self.measure_rsrp_real(center_lat, center_lng, "final_center")
        
        print(f"\n{'='*60}")
        print(f"POLYGON OPTIMIZATION COMPLETE!")
        print(f"{'='*60}")
        print(f"Final optimum: ({center_lat:.6f}, {center_lng:.6f})")
        print(f"Final metric: {final_metric:.1f}")
        print(f"Final polygon area: {get_polygon_area_meters(current_corners):.1f} sq meters")
        
        return center_lat, center_lng, final_metric, current_corners

def plot_polygon_trajectory(buildings, graph, trajectory, full_path, final_point, initial_corners, final_corners):
    """Plot the drone's polygon optimization trajectory"""
    fig, ax = plt.subplots(1, 1, figsize=(15, 12))
    
    # UE coordinates (for display)
    ue_coordinates = [
        (42.338824, -71.084579),
        (42.338562, -71.084783),
        (42.338705, -71.085942)
    ]
    
    # Get bounds for plotting
    all_lats = [lat for lng, lat in initial_corners] + [point[0] for point in trajectory]
    all_lngs = [lng for lng, lat in initial_corners] + [point[1] for point in trajectory]
    
    min_lat, max_lat = min(all_lats), max(all_lats)
    min_lng, max_lng = min(all_lngs), max(all_lngs)
    
    # Plot buildings in the area
    for building in buildings:
        x, y = building.exterior.xy
        building_bounds = building.bounds  # (minx, miny, maxx, maxy)
        if (building_bounds[0] <= max_lng and building_bounds[2] >= min_lng and
            building_bounds[1] <= max_lat and building_bounds[3] >= min_lat):
            ax.plot(x, y, 'b-', linewidth=0.5, alpha=0.6)
            ax.fill(x, y, 'lightblue', alpha=0.2)
    
    # Plot UE locations as crosses
    ue_lats = [coord[0] for coord in ue_coordinates]
    ue_lngs = [coord[1] for coord in ue_coordinates]
    ax.scatter(ue_lngs, ue_lats, c='red', s=150, marker='x', 
               linewidth=3, alpha=0.9, 
               label='UE Locations', zorder=6)
    
    # Plot initial polygon boundary
    initial_poly_x = [lng for lng, lat in initial_corners] + [initial_corners[0][0]]
    initial_poly_y = [lat for lng, lat in initial_corners] + [initial_corners[0][1]]
    ax.plot(initial_poly_x, initial_poly_y, 'r--', linewidth=2, alpha=0.8, label='Initial Polygon')
    
    # Plot flight path
    if full_path:
        path_lats = [point[0] for point in full_path]
        path_lngs = [point[1] for point in full_path]
        ax.plot(path_lngs, path_lats, 'purple', linewidth=2, alpha=0.7, 
                label='Flight Path', zorder=3)
    
    # Plot measurement points
    if trajectory:
        lats = [point[0] for point in trajectory]
        lngs = [point[1] for point in trajectory]
        metrics = [point[2] for point in trajectory]
        connected_ues = [point[3] for point in trajectory]
        
        # Filter out -inf values for colormap
        finite_metrics = [m for m in metrics if m != float('-inf')]
        if finite_metrics:
            metric_min, metric_max = min(finite_metrics), max(finite_metrics)
        else:
            metric_min, metric_max = 0, 1
        
        # Plot measurement points
        scatter = ax.scatter(lngs, lats, c=metrics, cmap='RdYlGn', 
                           s=120, alpha=0.9, edgecolors='black', linewidth=2,
                           vmin=metric_min, vmax=metric_max, zorder=5)
        
        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('RSRP Metric', rotation=270, labelpad=20)
        
        # Add visit order numbers to measurement points
        for i, (lat, lng, metric, ues, reason, iteration) in enumerate(trajectory):
            ax.annotate(f'{i+1}', (lng, lat), 
                       xytext=(0, 20), textcoords='offset points',
                       fontsize=10, ha='center', color='black', fontweight='bold',
                       bbox=dict(boxstyle='circle,pad=0.3', facecolor='white', 
                                edgecolor='black', alpha=0.9),
                       zorder=8)
    
    # Highlight final optimum
    if final_point:
        final_lat, final_lng, final_metric = final_point
        ax.plot(final_lng, final_lat, 'k^', markersize=25, 
                label=f'Final Destination\nMetric: {final_metric:.1f}', zorder=7)
    
    # Set equal aspect ratio and labels
    ax.set_aspect('equal')
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title('Real Drone 4-Corner Polygon Bisection Search')
    
    # Set plot bounds with padding
    padding = 0.0005
    ax.set_xlim(min_lng - padding, max_lng + padding)
    ax.set_ylim(min_lat - padding, max_lat + padding)
    
    # Add grid and legend
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig("trajectory.png")
    tikzplotlib.save("trajectory.tex")
    plt.show()

def print_trajectory_summary(trajectory, full_path):
    """Print a summary of the trajectory"""
    print(f"\n{'='*50}")
    print(f"TRAJECTORY SUMMARY")
    print(f"{'='*50}")
    print(f"Total measurements: {len(trajectory)}")
    print(f"Total flight path points: {len(full_path)}")
    
    # Calculate total flight distance
    if len(full_path) > 1:
        total_distance = 0
        for i in range(1, len(full_path)):
            lat1, lng1 = full_path[i-1][0], full_path[i-1][1]
            lat2, lng2 = full_path[i][0], full_path[i][1]
            
            lat_dist = degrees_to_meters(abs(lat2 - lat1))
            lng_dist = degrees_to_meters(abs(lng2 - lng1), is_longitude=True, center_lat=(lat1+lat2)/2)
            segment_distance = np.sqrt(lat_dist**2 + lng_dist**2)
            total_distance += segment_distance
        
        print(f"Total flight distance: {total_distance:.1f} meters")
    
    # Group by iteration
    by_iteration = {}
    for lat, lng, metric, connected_ues, reason, iteration in trajectory:
        if iteration not in by_iteration:
            by_iteration[iteration] = []
        by_iteration[iteration].append((lat, lng, metric, connected_ues, reason))
    
    for iteration in sorted(by_iteration.keys()):
        points = by_iteration[iteration]
        print(f"\nIteration {iteration}:")
        for lat, lng, metric, ues, reason in points:
            print(f"  {reason:15}: ({lat:.6f}, {lng:.6f}) -> Metric: {metric:.1f}, UEs: {ues}")
    
    # Find best overall metric
    if trajectory:
        best_measurement = max(trajectory, key=lambda x: x[2])
        print(f"\nBest measurement: Metric {best_measurement[2]:.1f} with {best_measurement[3]} UEs")
        print(f"Location: ({best_measurement[0]:.6f}, {best_measurement[1]:.6f})")

if __name__ == "__main__":
    print("Starting REAL 4-corner polygon drone optimization...")
    
    # Create polygon from custom boundaries
    initial_corners = create_custom_polygon()
    initial_area = get_polygon_area_meters(initial_corners)
    print(f"Initial polygon area: {initial_area:.1f} sq meters")
    
    print("Initial corners (clockwise):")
    for i, (lng, lat) in enumerate(initial_corners):
        print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")
    
    # Load the cached graph and buildings  
    buildings, grid, G, key_to_node = load_graph()
    
    # Initialize polygon drone optimizer
    optimizer = PolygonDroneOptimizer(initial_corners, G, entity_id=30)
    
    # Run the polygon bisection search
    final_lat, final_lng, final_metric, final_corners = optimizer.polygon_bisection_search_real()
    
    # Print trajectory summary
    print_trajectory_summary(optimizer.trajectory, optimizer.full_path)
    
    # Plot results
    plot_polygon_trajectory(buildings, G, optimizer.trajectory, optimizer.full_path, 
                           (final_lat, final_lng, final_metric), initial_corners, final_corners)
