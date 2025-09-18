#!/usr/bin/env python3
import time
import math
import json
import threading
import tkinter as tk
import paho.mqtt.client as mqtt

# ---------------- MQTT Settings ---------------- #
BROKER = "broker.hivemq.com"
PORT = 1883
TOPIC = "colosseum/update"
DRONE_ID = 1

# ---------------- Square UE Coordinates ---------------- #
UE_CORNERS = [
    (42.3395, -71.0855),  # Corner A
    (42.3395, -71.0835),  # Corner B
    (42.3375, -71.0835),  # Corner C
    (42.3375, -71.0855),  # Corner D
]

STEPS_PER_LEG = 20  # steps from one corner to next

# ---------------- Utility Functions ---------------- #
def haversine(lat1, lon1, lat2, lon2):
    """Distance in meters between two GPS points"""
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def simulate_rsrp(distance, max_dist):
    """RSRP improves as distance decreases"""
    return -90 + 20 * (1 - distance / max_dist)

def simulate_signal_quality(distance, max_dist):
    """Signal quality improves as distance decreases"""
    return 50 + 50 * (1 - distance / max_dist)

# ---------------- MQTT Client ---------------- #
class DroneMQTTClient:
    def __init__(self):
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.connect(BROKER, PORT, 60)
        self.client.loop_start()

    def send_position_update(self, lat, lon, iteration, rsrp, distance, signal_quality, ue_index):
        payload = {
            "drone_id": DRONE_ID,
            "lat": lat,
            "lon": lon,
            "rsrp": rsrp,
            "distance": distance,
            "signal_quality": signal_quality,
            "iteration": iteration,
            "target_ue": ue_index
        }
        self.client.publish(TOPIC, json.dumps(payload))

# ---------------- Drone Square Path ---------------- #
class DroneSquarePath:
    def __init__(self, mqtt_client, dashboard=None):
        self.mqtt_client = mqtt_client
        self.dashboard = dashboard
        self.iteration = 0

    def run(self):
        while True:
            for i in range(len(UE_CORNERS)):
                start_ue = UE_CORNERS[i]
                end_ue = UE_CORNERS[(i + 1) % len(UE_CORNERS)]  # next corner, wrap around

                max_distance = haversine(start_ue[0], start_ue[1], end_ue[0], end_ue[1])

                for step in range(1, STEPS_PER_LEG + 1):
                    self.iteration += 1
                    alpha = step / STEPS_PER_LEG  # interpolation fraction

                    # Drone position interpolates between corners
                    lat = start_ue[0] + alpha * (end_ue[0] - start_ue[0])
                    lon = start_ue[1] + alpha * (end_ue[1] - start_ue[1])

                    # Distance to current target UE
                    distance = haversine(lat, lon, end_ue[0], end_ue[1])
                    rsrp = simulate_rsrp(distance, max_distance)
                    signal_quality = simulate_signal_quality(distance, max_distance)

                    print(f"[ITER {self.iteration}] → UE {i+1}->{(i+2) if (i+1)<len(UE_CORNERS) else 1}, "
                          f"lat={lat:.6f}, lon={lon:.6f}, Dist={distance:.2f} m, "
                          f"RSRP={rsrp:.1f} dBm, SQ={signal_quality:.1f}%")

                    # Update dashboard
                    if self.dashboard:
                        self.dashboard.update_status(lat, lon, rsrp, distance, signal_quality, self.iteration, end_ue)

                    # Publish MQTT
                    self.mqtt_client.send_position_update(lat, lon, self.iteration, rsrp, distance, signal_quality, i+1)

                    time.sleep(1)

# ---------------- Dashboard ---------------- #
class DroneDashboard:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Drone Square Path Dashboard")
        self.root.geometry("500x300")

        self.labels = {}
        fields = ["Iteration", "Drone Lat", "Drone Lon", "RSRP", "Distance", "Signal Quality", "Target UE"]
        for i, field in enumerate(fields):
            tk.Label(self.root, text=field + ":").grid(row=i, column=0, sticky="w", padx=10, pady=5)
            self.labels[field] = tk.Label(self.root, text="---")
            self.labels[field].grid(row=i, column=1, sticky="w", padx=10)

        self.prev_rsrp = None
        self.prev_signal_quality = None

    def update_status(self, lat, lon, rsrp, distance, signal_quality, iteration, target_ue):
        self.labels["Iteration"].config(text=str(iteration))
        self.labels["Drone Lat"].config(text=f"{lat:.6f}")
        self.labels["Drone Lon"].config(text=f"{lon:.6f}")
        self.labels["RSRP"].config(text=f"{rsrp:.1f} dBm")
        self.labels["Distance"].config(text=f"{distance:.2f} m")
        self.labels["Signal Quality"].config(text=f"{signal_quality:.1f}%")
        self.labels["Target UE"].config(text=f"{target_ue[0]:.6f}, {target_ue[1]:.6f}")

        # Highlight improvements
        def highlight(label_widget, new_val, prev_val):
            default_bg = self.root.cget("bg")
            if prev_val is None:
                label_widget.config(bg=default_bg)
            else:
                if new_val > prev_val:
                    label_widget.config(bg="#b6f5b6")  # green
                elif new_val < prev_val:
                    label_widget.config(bg="#f5b6b6")  # red
                else:
                    label_widget.config(bg=default_bg)

        highlight(self.labels["RSRP"], rsrp, self.prev_rsrp)
        highlight(self.labels["Signal Quality"], signal_quality, self.prev_signal_quality)

        self.prev_rsrp = rsrp
        self.prev_signal_quality = signal_quality

        self.root.update_idletasks()

    def start(self):
        self.root.mainloop()

# ---------------- Main ---------------- #
if __name__ == "__main__":
    mqtt_client = DroneMQTTClient()
    dashboard = DroneDashboard()
    square_path = DroneSquarePath(mqtt_client, dashboard=dashboard)

    t = threading.Thread(target=square_path.run)
    t.start()

    dashboard.start()
