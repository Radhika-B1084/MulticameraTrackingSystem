import time
import struct
import cv2
import numpy as np
from picamera2 import Picamera2
from multiprocessing import shared_memory, resource_tracker
from collections import deque
import os
import socket
import math

#FUNCTION DEFINITIONS
def attach(name, timeout=None, poll=0.1):
    deadline = None if timeout is None else time.perf_counter() + timeout
    while True:
        try:
            shm = shared_memory.SharedMemory(name=name)
            try:
                resource_tracker.unregister(shm._name, "shared_memory")
            except Exception:
                pass
            return shm
        except FileNotFoundError:
            if deadline is not None and time.perf_counter() > deadline:
                raise
            time.sleep(poll)

def get_current_led(sensor_timestamp_ns, packet_timestamp_ns, total_leds=4):
    elapsed_time_ns = sensor_timestamp_ns - packet_timestamp_ns
    flashes_passed = elapsed_time_ns // 10_000_000
    current_led = flashes_passed % total_leds
    return current_led

def read_latest():
    while True:
        idx  = struct.unpack_from("<Q", buf, INDEX_OFF)[0]
        slot = struct.unpack_from(SLOT_FMT, buf, SLOTS_OFF[idx])
        if struct.unpack_from("<Q", buf, INDEX_OFF)[0] == idx:
            return slot

shm = attach("netbuf")
buf = shm.buf

#STUFF
INDEX_OFF = 0
SLOT_FMT  = "<QQddd"
SLOT_SIZE = struct.calcsize(SLOT_FMT)
SLOTS_OFF = (8, 8 + SLOT_SIZE, 8 + 2 * SLOT_SIZE)

TRIANGULATION_SERVER_IP = "192.168.0.104"
TRIANGULATION_SERVER_PORT = 5000

#INITALISATIONS
picam2 = Picamera2()
config  = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (1152,648)},
    lores = {"format": "RGB888", "size": (640, 360)},
    controls={"FrameDurationLimits": (10000, 10000), "AeEnable": False, "ExposureTime": 500}
)
picam2.configure(config)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
camera_m = np.load("camera_matrix.npy")
dist = np.load("dist_coeffs.npy")

# Constants
kp = 0.1
ki = 0.02 / 12
kd = 0.06
integral = 0.0
prev_error = 0.0
set_frame_duration = 10000
target_phase = 0.50
current_led = 0

# LED tracking and state machine
NODE_NAME = 3  # Change as needed for multi-camera setup
MEMORY_TIMEOUT = 0.04  # seconds - how long to keep LED positions in memory
led_memory = {
    0: {"pos": None, "time": 0.0},
    2: {"pos": None, "time": 0.0}
}
state = "SEARCHING"
capture_deadline = None

# Plotting
plot_errors = []
plot_times = []
start_time = time.perf_counter()

# Logging
phase_error_window = deque(maxlen=1000)
adjustment_window  = deque(maxlen=1000)
last_adjustment_timestamp = None

picam2.start()

# Connecting to the triangulation server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TRIANGULATION_SERVER_IP, TRIANGULATION_SERVER_PORT))
s.setblocking(False)  # Non-blocking for recv() exception handling

try:
    while True:
        now = time.perf_counter()  # Get current time for memory timeout tracking

        request = picam2.capture_request()
        frame = request.make_array("lores")
        metadata = request.get_metadata()
        request.release()

        actual_frame_duration = metadata["FrameDuration"]
        sensor_timestamp = metadata["SensorTimestamp"]

        (server_mono, client_recv, off_smooth, off_raw, delay) = read_latest()

        if client_recv == 0:
            print("waiting for first packet...")
            time.sleep(0.1)
            continue

        #PID LOOP CALCULATIONS
        server_now_est = (sensor_timestamp) - off_smooth * 1000
        timestamp_ns = int(server_mono * 1000)

        delta_ns = server_now_est - timestamp_ns
        delta_ns = delta_ns % (set_frame_duration * 1000)

        if delta_ns > (set_frame_duration * 1000 / 2):
            delta_ns -= (set_frame_duration * 1000)

        phase = delta_ns / (set_frame_duration * 1000)

        phase_error = (target_phase - phase)
        if phase_error > 0.5:
            phase_error -= 1.0
        if phase_error < -0.5:
            phase_error += 1.0

        derivative = phase_error - prev_error
        integral += phase_error
        nudge = (kp * phase_error + ki * integral) * 1000

        frame_duration = round(set_frame_duration + nudge)
        picam2.set_controls({"FrameDurationLimits": (frame_duration, frame_duration)})
        prev_error = phase_error

        current_time = time.perf_counter() - start_time
        plot_times.append(current_time)
        plot_errors.append(phase_error * set_frame_duration)

        if last_adjustment_timestamp is None:
            dt_since_last_us = 0.0
        else:
            dt_since_last_us = (sensor_timestamp - last_adjustment_timestamp) / 1000.0
        last_adjustment_timestamp = sensor_timestamp

        phase_error_window.append(phase_error)
        adjustment_window.append(nudge)

        #PRINTING THINGS
        print(f" ts = {timestamp_ns} difference = {server_now_est - timestamp_ns}  delta={delta_ns/1000:.0f}us  phase={phase:.3f}  error={phase_error:.3f}  nudge={nudge:.1f}us  frame_dur={frame_duration}us   metadata_duration={actual_frame_duration}")

        current_led = get_current_led(server_now_est, timestamp_ns)
        print(f"CurrentLED: {current_led}")

        #DETECTION
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        _, thresh  = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Track detected LED positions by size (assuming smaller contour is LED 0, larger is LED 1)
        detected_positions = []

        for cnt in contours:
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            detected_positions.append({"pos": (cx, cy), "radius": radius})
            cv2.circle(frame, (int(cx), int(cy)), int(radius) + 5, (0, 255, 0), 2)

            if current_led in [0, 2]:
                led_memory[current_led]["pos"] = (cx, cy)
                led_memory[current_led]["time"] = now

        # Evaluate if the entire wand is currently visible
        # Both LEDs must have been seen within the MEMORY_TIMEOUT window
        led_0_fresh = (now - led_memory[0]["time"]) < MEMORY_TIMEOUT
        led_2_fresh = (now - led_memory[2]["time"]) < MEMORY_TIMEOUT
        wand_visible = led_0_fresh and led_2_fresh

        # ---------------------------------------------------------
        # 4. CALIBRATION STATE MACHINE
        # ---------------------------------------------------------
        try:
            command = s.recv(1024).decode('utf-8')
        except (BlockingIOError, socket.error):
            command = ""

        if state == "SEARCHING":
            if wand_visible:
                s.sendall(f"{NODE_NAME},1".encode('utf-8'))
                state = "WAITING_CAPTURE"
                print(f"[{NODE_NAME}] Wand detected. Sent Ready (1).")

        elif state == "WAITING_CAPTURE":
            if not wand_visible:
                s.sendall(f"{NODE_NAME},0".encode('utf-8'))
                state = "SEARCHING"
                print(f"[{NODE_NAME}] Lost view of wand. Sent Lost (0). Resetting...")
            elif command == "CAPTURE":
                print(f"[{NODE_NAME}] Capture received! 3 second countdown...")
                capture_deadline = now + 3.0
                state = "COUNTDOWN"

        elif state == "COUNTDOWN":
            if now >= capture_deadline:
                if wand_visible:
                    # Extract the locked positions from our memory buffer
                    pos_0 = led_memory[0]["pos"]
                    pos_1 = led_memory[2]["pos"]

                    angles = []
                    # Process LED 0 then LED 1 to guarantee Az/El order matches the physical wand
                    for (cx, cy) in [pos_0, pos_1]:
                        cx_main = cx * (1152/640)
                        cy_main = cy * (648/360)
                        pixel_point = np.array([[[cx_main, cy_main]]], dtype=np.float32)
                        norm_pt = cv2.undistortPoints(pixel_point, camera_m, dist)

                        x_prime, y_prime = norm_pt[0][0][0], norm_pt[0][0][1]
                        diagonal = math.sqrt((x_prime**2) + 1.0)

                        az_deg = math.degrees(math.atan(x_prime))
                        el_deg = math.degrees(math.atan(y_prime / diagonal))
                        angles.extend([az_deg, el_deg])

                    payload = f"{NODE_NAME},{angles[0]:.4f},{angles[1]:.4f},{angles[2]:.4f},{angles[3]:.4f}"
                    s.sendall(payload.encode('utf-8'))
                    print(f"[{NODE_NAME}] Calibration angles sent! Waiting for wand removal.")
                    state = "WAITING_REMOVAL"
                else:
                    s.sendall(f"{NODE_NAME},0".encode('utf-8'))
                    print(f"[{NODE_NAME}] Capture failed (Wand occluded). Sent Lost (0). Resetting...")
                    state = "SEARCHING"

        elif state == "WAITING_REMOVAL":
            # Wait until BOTH memory slots expire (wand completely removed from frame)
            if not led_0_fresh and not led_2_fresh:
                print(f"[{NODE_NAME}] Wand removed. Ready for next placement.")
                state = "SEARCHING"

except KeyboardInterrupt:
    picam2.stop()
    cv2.destroyAllWindows()
    s.close()