import time
import socket
import json
import select  # Prevents infinite blocking loops and allows Ctrl+C

PORT = 5000
NUM_PLACEMENTS = 15
EXPECTED_NODES = 3
AUTO_CAPTURE_INTERVAL = 3.0  # seconds between auto-captures once all cameras see the wand

def print_instructions(placement_idx):
    print("\n")
    print(f"PLACEMENT {placement_idx}")
    if placement_idx == 0:
        print("Place wand at the origin [0,0,0] pointing exactly down the +X axis.")
    elif placement_idx == 5:
        print("You can now shift to holding the wand in the air instead of on the ground.")
    print("Waiting for all 3 cameras to see the wand...")
    print("\n")

# Initialize the server socket immediately
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', PORT))
server.listen(EXPECTED_NODES)

while True:
    print(f"\nWaiting for {EXPECTED_NODES} Pi Zeros to connect...")
    connections = []
    for _ in range(EXPECTED_NODES):
        conn, addr = server.accept()
        connections.append(conn)
        print(f"Connected to node at {addr}")

    # Start with an empty dictionary. It will dynamically add slots for "0", "1", "2"
    master_calibration_data = {}

    recv_buffers = {conn: "" for conn in connections}

    try:
        for p_idx in range(NUM_PLACEMENTS):
            print_instructions(p_idx)

            # ========================================================
            # 1. LIVE VISIBILITY TRACKING + AUTO-CAPTURE EVERY 3s
            #    Cameras send "cam_id,1" / "cam_id,0" whenever their
            #    visibility of the wand CHANGES (not continuously).
            #    We keep a running picture of who currently sees it,
            #    print only when that picture changes, and as soon as
            #    all 3 are visible together we start a fresh 3s
            #    countdown before auto-capturing.
            # ========================================================
            visible_nodes = set()
            captured_this_placement = False

            # Reset countdown anchor each time we re-enter "all visible" state
            all_visible_since = None

            while not captured_this_placement:
                readable, _, _ = select.select(connections, [], [], 0.5)

                for conn in readable:
                    try:
                        raw_data = conn.recv(1024)

                        # FAILSAFE: Catch the silent disconnect
                        if not raw_data:
                            print(f"\n[WARNING] A socket returned empty data. A Pi Zero disconnected!")
                            if conn in connections:
                                connections.remove(conn)
                            continue

                        # Multiple visibility messages can arrive back-to-back in one
                        # TCP read, so split on newlines defensively.
                        decoded = raw_data.decode('utf-8').strip()

                        for line in decoded.splitlines():
                            data = line.strip().split(",")

                            if len(data) >= 2 and data[1] in ("0", "1"):
                                cam_id = data[0]
                                is_visible = (data[1] == "1")

                                if cam_id not in master_calibration_data:
                                    master_calibration_data[cam_id] = {}

                                was_all_visible = (len(visible_nodes) == EXPECTED_NODES)

                                if is_visible:
                                    if cam_id not in visible_nodes:
                                        visible_nodes.add(cam_id)
                                        print(f"  -> {cam_id} now sees the wand. "
                                              f"({len(visible_nodes)}/{EXPECTED_NODES} visible)")
                                else:
                                    if cam_id in visible_nodes:
                                        visible_nodes.discard(cam_id)
                                        print(f"  -> {cam_id} LOST the wand. "
                                              f"({len(visible_nodes)}/{EXPECTED_NODES} visible)")

                                now_all_visible = (len(visible_nodes) == EXPECTED_NODES)

                                # Just became fully visible -> start the 3s countdown
                                if now_all_visible and not was_all_visible:
                                    all_visible_since = time.monotonic()
                                    print(f"All {EXPECTED_NODES} cameras see the wand. "
                                          f"Capturing in {AUTO_CAPTURE_INTERVAL:.0f}s if it stays visible...")

                                # Visibility dropped -> cancel any pending countdown
                                if not now_all_visible and was_all_visible:
                                    all_visible_since = None
                                    print("Visibility lost - countdown cancelled.")

                    except Exception as e:
                        print(f"Socket error during visibility check: {e}")

                # Check whether it's time to auto-capture
                if (all_visible_since is not None
                        and len(visible_nodes) == EXPECTED_NODES
                        and (time.monotonic() - all_visible_since) >= AUTO_CAPTURE_INTERVAL):

                    print("Capturing now...")

                    # ========================================================
                    # 2. SEND THE CAPTURE COMMAND
                    # ========================================================
                    for conn in connections:
                        conn.sendall("CAPTURE".encode('utf-8'))

                    # ========================================================
                    # 3. COLLECT FINAL ANGLES FROM 3 UNIQUE CAMERAS (Non-Blocking)
                    # ========================================================
                    captured_nodes = set()
                    while len(captured_nodes) < EXPECTED_NODES:
                        cap_readable, _, _ = select.select(connections, [], [], 2.0)

                        for conn in cap_readable:
                            try:
                                chunk = conn.recv(1024).decode('utf-8')
                                if not chunk:
                                    continue
                                recv_buffers[conn] += chunk

                                # Process all complete lines in the buffer
                                while '\n' in recv_buffers[conn]:
                                    line, recv_buffers[conn] = recv_buffers[conn].split('\n', 1)
                                    data = line.strip().split(",")

                                    if len(data) == 5:
                                        cam_id = data[0]
                                        angles = [float(data[1]), float(data[2]), float(data[3]), float(data[4])]

                                        if cam_id not in master_calibration_data:
                                            master_calibration_data[cam_id] = {}

                                        master_calibration_data[cam_id][f"Placement_{p_idx}"] = angles

                                        if cam_id not in captured_nodes:
                                            print(f"Received data from {cam_id}")
                                            captured_nodes.add(cam_id)

                            except Exception as e:
                                    print(f"Socket error during capture check: {e}")

                    print(f"Placement {p_idx} logged successfully. Please move the wand.")
                    captured_this_placement = True

            time.sleep(2)

        # ========================================================
        # 4. SAVE FILE & COMPLETE
        # ========================================================
        print("\nCalibration Complete! Saving to file...")
        filename = f"calibration_raw_angles_{int(time.time())}.json"

        with open(filename, "w") as f:
            json.dump(master_calibration_data, f, indent=4)
        print(f"Saved as {filename}!")

    except KeyboardInterrupt:
        # Gracefully handle Ctrl+C without throwing a giant stack trace
        print("\n\n[INFO] Calibration manually stopped (Ctrl+C).")

    except Exception as e:
        print(f"\n[ERROR] Calibration interrupted: {e}")

    finally:
        print("Cleaning up connections for this session...\n")
        for conn in connections:
            conn.close()