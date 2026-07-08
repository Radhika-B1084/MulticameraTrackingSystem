import socket
import json
import time
import numpy as np
import cv2

# ---------------- Triangulation logic (from triangulation.py) ----------------

def angle_to_world_ray(az_rad, el_rad, rvec):
    """Converts Azimuth/Elevation angles into a 3D directional ray in the physical room."""
    x_norm = np.tan(az_rad)
    y_norm = -np.tan(el_rad) / np.cos(az_rad)
    ray_cam = np.array([x_norm, y_norm, 1.0])

    R_cv, _ = cv2.Rodrigues(rvec)
    R_cam_to_world = R_cv.T
    ray_world = R_cam_to_world @ ray_cam

    return ray_world / np.linalg.norm(ray_world)

def triangulate_closest_point(camera_positions, ray_directions):
    """Finds the 3D point that minimizes the orthogonal distance to all rays, plus RMS residual."""
    A = np.zeros((3, 3))
    b = np.zeros(3)

    for C, v in zip(camera_positions, ray_directions):
        projection = np.eye(3) - np.outer(v, v)
        A += projection
        b += projection @ C

    point = np.linalg.solve(A, b)

    residuals = []
    for C, v in zip(camera_positions, ray_directions):
        w = point - C
        perp = w - np.dot(w, v) * v
        residuals.append(np.linalg.norm(perp))
    rms = np.sqrt(np.mean(np.square(residuals)))

    return point, rms

# ---------------- Load camera params (pos is already world-frame, no conversion) ----------------

with open("camera_params.json", "r") as f:
    raw_params = json.load(f)

CAMERAS = {
    cam_id: {
        "rvec": np.array(p["rvec"]),
        "tvec": np.array(p["tvec"]),
        "pos":  -cv2.Rodrigues(np.array(p["rvec"]))[0].T @ np.array(p["tvec"]),
    }
    for cam_id, p in raw_params.items()
    if cam_id != "transform"
}

R_new  = np.array(raw_params["transform"]["R_new"])
origin = np.array(raw_params["transform"]["origin"])

print("Loaded camera params (old frame):")
for cam_id, cam in CAMERAS.items():
    print(f"  Cam {cam_id} position (old frame): {cam['pos'].round(4)}")

def triangulate_batch(led_no, obs_by_cam):
    """obs_by_cam: {cam_id: (h_deg, v_deg)}"""
    valid_rays = []
    valid_positions = []

    for cam_id, (h_deg, v_deg) in obs_by_cam.items():
        if cam_id not in CAMERAS:
            continue
        if h_deg == 0.0 and v_deg == 0.0:
            continue
        az_rad = np.radians(h_deg)
        el_rad = np.radians(v_deg)
        ray = angle_to_world_ray(az_rad, el_rad, CAMERAS[cam_id]["rvec"])
        valid_rays.append(ray)
        valid_positions.append(CAMERAS[cam_id]["pos"])

    active_cameras = len(valid_rays)

    if active_cameras < 2:
        print(f"[LED {led_no}] Only {active_cameras} valid detection(s) — skipping.")
        return None

    pos_old, rms = triangulate_closest_point(valid_positions, valid_rays)

    pos_new = R_new @ (pos_old - origin)

    if led_no == 2:
        print(f"[LED {led_no}] cams={list(obs_by_cam.keys())}  "
              f"pos_new=({pos_new[0]:.4f}, {pos_new[1]:.4f}, {pos_new[2]:.4f}) m  "
              f"rms={rms*1000:.2f}mm")
    return pos_new

# ---------------- Networking / receiver (from traingulation_reciever.py) ----------------

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, LISTEN_PORT))

sock.setblocking(False)
packet_buffer = []
packets_processed = 0

last_save_time = time.monotonic()

filename = f"camera_raw_data.json"
fieldnames = ['id', 'time', 'led_no', 'h', 'v']

with open(filename, 'w') as f:
    pass

current_batch_led = None
current_batch_obs = {}

try:
    while True:
        while True:
            try:
                data = sock.recv(1024)
                data = data.decode("utf-8").split(",")

                if len(data) == 5:
                    cam_id = data[0]
                    timestamp = float(data[1])
                    led_no = float(data[2])
                    h_angle = float(data[3])
                    v_angle = float(data[4])

                    packet_buffer.append({"id": cam_id, "time": timestamp, "led_no": led_no, "h": h_angle, "v": v_angle})
                    print(f"Packet received: ID={cam_id}, LED={led_no}, H={h_angle:.2f}, V={v_angle:.2f}")
            except BlockingIOError:
                break

        if len(packet_buffer) > 0:
            unprocessed_packets = packet_buffer[packets_processed:]
            unprocessed_packets.sort(key=lambda x: x["time"])
            packet_buffer[packets_processed:] = unprocessed_packets

        for packet in packet_buffer[packets_processed:]:
            led_no = packet["led_no"]

            if current_batch_led is None:
                current_batch_led = led_no

            if led_no != current_batch_led:
                triangulate_batch(current_batch_led, current_batch_obs)
                current_batch_obs = {}
                current_batch_led = led_no

            current_batch_obs[packet["id"]] = (packet["h"], packet["v"])
            packets_processed += 1

        current_time = time.monotonic()
        if current_time - last_save_time >= 60 and packets_processed > 0:

            data_to_save = packet_buffer[:packets_processed]

            with open(filename, "a") as f:
                for packet in data_to_save:
                    f.write(json.dumps(packet) + "\n")

            del packet_buffer[:packets_processed]
            packets_processed = 0
            last_save_time = current_time
            print(f"Saved {len(data_to_save)} packets to file")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("done")
    if len(packet_buffer) > 0:
        with open(filename, "a") as f:
            for packet in packet_buffer:
                f.write(json.dumps(packet) + "\n")