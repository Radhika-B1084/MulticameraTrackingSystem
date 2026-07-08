import time
import struct
import cv2
import numpy as np
from picamera2 import Picamera2
from multiprocessing import shared_memory, resource_tracker
from collections import deque
import matplotlib.pyplot as plt
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

# Plotting
plot_errors = []
plot_times = []
start_time = time.perf_counter()

# Logging
phase_error_window = deque(maxlen=1000)
adjustment_window  = deque(maxlen=1000)
last_adjustment_timestamp = None

picam2.start()

try:
    while True:
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

        led_seen = False
        horizontal_angle_deg = 0
        vertical_angle_deg = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            led_seen = True
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            cv2.circle(frame, (int(cx), int(cy)), int(radius) + 5, (0, 255, 0), 2)
        print("Detected:", led_seen)
        print("")
        cv2.imshow("thresh", thresh)
        cv2.imshow("original", frame)
        cv2.waitKey(1)


        #ANGLE CALCULATIONS
        if led_seen:
            cx = cx*(1152/640)
            cy = cy*(648/360)
            pixel_point = np.array([[[cx,cy]]], dtype = np.float32)

            normalised_point = cv2.undistortPoints(pixel_point, camera_m, dist)

            x_prime = normalised_point[0][0][0]
            y_prime = normalised_point[0][0][1]

            diagonal = math.sqrt((x_prime**2)+1.0)

            horizontal_angle_rad = math.atan(x_prime)
            vertical_angle_rad = math.atan(y_prime/diagonal)

            horizontal_angle_deg = math.degrees(horizontal_angle_rad)
            vertical_angle_deg = math.degrees(vertical_angle_rad)

        #SENDING THE PACKET
        data = f"{1},{server_now_est},{current_led}, {horizontal_angle_deg},{vertical_angle_deg}".encode()
        sock.sendto(data, (TRIANGULATION_SERVER_IP, TRIANGULATION_SERVER_PORT))

except KeyboardInterrupt:
    picam2.stop()
    cv2.destroyAllWindows()
    '''
    plt.figure(figsize=(10, 5))
    plt.plot(plot_times, plot_errors, label="Phase Error (us)", color='blue', linewidth=1)

    plt.axhline(0, color='red', linestyle='--', alpha=0.7, label="Target")

    plt.title(f"Camera Synchronization (Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f})")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Phase Error (microseconds)")
    plt.grid(True)
    plt.legend()

    save_dir = os.path.expanduser("~/Desktop/test/PID_plots")
    os.makedirs(save_dir, exist_ok=True)

    filename = f"plot1_kp_{kp:.4f}ki_{ki:.4f}_kd_{kd:.4f}_synced3.png"

    file_path = os.path.join(save_dir, filename)
    plt.savefig(file_path)
    '''