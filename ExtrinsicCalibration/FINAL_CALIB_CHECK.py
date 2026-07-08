import json
import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

# ==========================================================================
# 1. LOAD REAL DATA
# ==========================================================================
JSON_FILENAME = "calibration_raw_angles_1782910765.json"

with open(JSON_FILENAME, "r") as f:
    raw_data = json.load(f)

num_placements = 15
num_points = num_placements * 2

mock_obs0 = np.full((num_points, 2), np.nan)
mock_obs1 = np.full((num_points, 2), np.nan)
mock_obs2 = np.full((num_points, 2), np.nan)

# Map JSON keys to our pipeline (1 -> Cam0, 2 -> Cam1, 3 -> Cam2)
cam0_key, cam1_key, cam2_key = "1", "2", "3"

for p_idx in range(num_placements):
    gid = f"Placement_{p_idx}"
    base = p_idx * 2
    
    # Extract Camera 0 (JSON Key "1")
    if gid in raw_data.get(cam0_key, {}):
        vals = raw_data[cam0_key][gid]
        mock_obs0[base]   = np.radians([vals[0], vals[1]])  # LED A (Az, El)
        mock_obs0[base+1] = np.radians([vals[2], vals[3]])  # LED B (Az, El)
        
    # Extract Camera 1 (JSON Key "2")
    if gid in raw_data.get(cam1_key, {}):
        vals = raw_data[cam1_key][gid]
        mock_obs1[base]   = np.radians([vals[0], vals[1]])
        mock_obs1[base+1] = np.radians([vals[2], vals[3]])
        
    # Extract Camera 2 (JSON Key "3")
    if gid in raw_data.get(cam2_key, {}):
        vals = raw_data[cam2_key][gid]
        mock_obs2[base]   = np.radians([vals[0], vals[1]])
        mock_obs2[base+1] = np.radians([vals[2], vals[3]])

print(f"Successfully loaded {num_placements} placements from {JSON_FILENAME}.")

# ==========================================================================
# 2. PIPELINE UTILITIES
# ==========================================================================


def repack_parameters(cam1_rt, cam2_rt, points_3d):
    return np.hstack((cam1_rt, cam2_rt, points_3d.ravel()))

def unpack_parameters(x, num_placements):
    cam1_rt = x[0:6]
    cam2_rt = x[6:12]
    points_3d = x[12:].reshape((num_placements * 2, 3))
    return cam1_rt, cam2_rt, points_3d

def project_to_angles(P_world, r_vec, t_vec):
    R, _ = cv2.Rodrigues(r_vec)
    P_cam = R @ P_world + t_vec
    
    X, Y, Z = P_cam[0], P_cam[1], P_cam[2]
    pred_H = np.arctan2(X, Z)
    hypotenuse = np.sqrt(X**2 + Z**2)
    pred_V = np.arctan2(-Y, hypotenuse)
    return np.array([pred_H, pred_V])

def calibration_residuals(x, num_placements, obs0, obs1, obs2):
    cam1_rt, cam2_rt, points_3d = unpack_parameters(x, num_placements)
    
    # Anchor Camera 0 to the origin
    r0 = np.zeros(3) 
    t0 = np.zeros(3)
    
    r1, t1 = cam1_rt[:3], cam1_rt[3:]
    r2, t2 = cam2_rt[:3], cam2_rt[3:]
    
    residuals = []
    
    for i in range(num_placements * 2):
        P = points_3d[i]
        if not np.isnan(obs0[i][0]):
            pred0 = project_to_angles(P, r0, t0)
            residuals.extend(pred0 - obs0[i])
        if not np.isnan(obs1[i][0]):
            pred1 = project_to_angles(P, r1, t1)
            residuals.extend(pred1 - obs1[i])
        if not np.isnan(obs2[i][0]):
            pred2 = project_to_angles(P, r2, t2)
            residuals.extend(pred2 - obs2[i])
            
    scale_weight = 1.0 
    for p in range(num_placements):
        p_A = points_3d[2 * p]
        p_B = points_3d[2 * p + 1]
        calculated_dist = np.linalg.norm(p_A - p_B)
        residuals.append((calculated_dist - 0.1) * scale_weight)
        
    return np.array(residuals, dtype=np.float64)

def angles_to_normalized_coords(obs):
    """Converts Azimuth and Elevation into dimensionless Pinhole Camera coordinates."""
    coords = np.zeros((len(obs), 2))
    for i in range(len(obs)):
        if np.isnan(obs[i][0]):
            coords[i] = [np.nan, np.nan]
        else:
            az = obs[i][0]
            el = obs[i][1]
            x_norm = np.tan(az)
            y_norm = -np.tan(el) / np.cos(az) 
            coords[i] = [x_norm, y_norm]
    return coords

# ==========================================================================
# 3. INITIALIZATION
# ==========================================================================
print("\nBootstrapping initial camera coordinates via Epipolar Geometry...")

def bootstrap_camera_pose(obs0, obs_target):
    pts0_norm = angles_to_normalized_coords(obs0)
    ptsT_norm = angles_to_normalized_coords(obs_target)
    
    valid = ~np.isnan(pts0_norm[:, 0]) & ~np.isnan(ptsT_norm[:, 0])

    # VIP LIST LOGIC: Hide the first 10 points (planar)
    vip_mask = np.copy(valid)
    vip_mask[:12] = False 
    
    p0 = pts0_norm[vip_mask]
    pT = ptsT_norm[vip_mask]
    
    if len(p0) < 5:
        raise ValueError("Not enough overlapping points to bootstrap camera pose.")
        
    E, mask = cv2.findEssentialMat(p0, pT, focal=1.0, pp=(0., 0.), method=cv2.RANSAC, prob=0.999, threshold=0.001)
    _, R_raw, t_raw, mask = cv2.recoverPose(E, p0, pT, focal=1.0, pp=(0., 0.))
    
    P0 = np.hstack((np.eye(3), np.zeros((3, 1))))
    P1 = np.hstack((R_raw, t_raw))
    pts4D = cv2.triangulatePoints(P0, P1, p0.T, pT.T)
    pts3D_cam0 = (pts4D[:3, :] / pts4D[3, :]).T
    
    valid_indices = np.where(vip_mask)[0]
    idx_map = {orig_idx: array_idx for array_idx, orig_idx in enumerate(valid_indices)}
    
    scales = []
    for p in range(len(obs0) // 2):
        idxA = 2 * p
        idxB = 2 * p + 1
        
        if idxA in idx_map and idxB in idx_map: 
            ptA = pts3D_cam0[idx_map[idxA]]
            ptB = pts3D_cam0[idx_map[idxB]]
            dist = np.linalg.norm(ptA - ptB)
            
            if dist > 0.001:
                scales.append(0.1 / dist) 
                
    if scales:
        median_scale = np.median(scales)
        t_scaled = t_raw * median_scale
    else:
        t_scaled = t_raw 
        
    rvec_world_to_cam, _ = cv2.Rodrigues(R_raw)

    return np.hstack((rvec_world_to_cam.ravel(), t_scaled.ravel())), R_raw, t_scaled

def triangulate_all_points(R_local, t_local, obs0, obs_target):
    p0_norm = angles_to_normalized_coords(obs0)
    pT_norm = angles_to_normalized_coords(obs_target)
    
    proj0 = np.hstack((np.eye(3), np.zeros((3, 1))))
    proj1 = np.hstack((R_local, t_local))
    
    points_3d_cam0 = np.zeros((len(obs0), 3))
    for i in range(len(obs0)):
        if not np.isnan(p0_norm[i, 0]) and not np.isnan(pT_norm[i, 0]):
            pt4d = cv2.triangulatePoints(proj0, proj1, p0_norm[i].reshape(2,1), pT_norm[i].reshape(2,1))
            points_3d_cam0[i] = (pt4d[:3] / pt4d[3]).flatten()
            
    return points_3d_cam0

# Build the Initial Guess
init_cam1_rt, R1_raw, t1_scaled = bootstrap_camera_pose(mock_obs0, mock_obs1)
init_cam2_rt, R2_raw, t2_scaled = bootstrap_camera_pose(mock_obs0, mock_obs2)

def get_physical_position(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    return -R.T @ tvec

pos1_init = get_physical_position(init_cam1_rt[:3], init_cam1_rt[3:])
pos2_init = get_physical_position(init_cam2_rt[:3], init_cam2_rt[3:])
print("Init pos1:", pos1_init.round(4))
print("Init pos2:", pos2_init.round(4))


pts_from_1 = triangulate_all_points(R1_raw, t1_scaled, mock_obs0, mock_obs1)
pts_from_2 = triangulate_all_points(R2_raw, t2_scaled, mock_obs0, mock_obs2)
init_points_3d = (pts_from_1 + pts_from_2) / 2.0

initial_guess_vector = repack_parameters(init_cam1_rt, init_cam2_rt, init_points_3d)

# ==========================================================================
# 4. OPTIMIZATION
# ==========================================================================
print("\nStarting optimizer with Perturbation Loop...")

best_res = None
best_cost = float('inf')
current_guess = initial_guess_vector
max_attempts = 5
target_cost = 1e-9  # Realistic threshold for physical data

for attempt in range(max_attempts):
    res = least_squares(
        calibration_residuals, 
        current_guess, 
        args=(num_placements, mock_obs0, mock_obs1, mock_obs2),
        method='lm',      
        verbose=0,
        max_nfev=50000
    )
    
    print(f"Attempt {attempt + 1} finished with cost: {res.cost:.6e}")
    
    if res.cost < best_cost:
        best_cost = res.cost
        best_res = res
        
    if best_cost < target_cost:
        print("Excellent global minimum reached! Breaking out of loop.")
        break
        
    noise_level = 1e-1 # Slightly larger jiggle for real-world noise
    random_jiggle = np.random.normal(0, noise_level, size=res.x.shape)
    current_guess = res.x + random_jiggle

print(f"\nOptimization Complete! Final Best Cost: {best_cost:.6e}")


n_scale_residuals = num_placements
angular_residuals = best_res.fun[:-n_scale_residuals]   # radians
scale_residuals   = best_res.fun[-n_scale_residuals:]   # meters (wand-length constraint)

rms_angular_deg = np.degrees(np.sqrt(np.mean(angular_residuals**2)))
rms_scale_mm    = np.sqrt(np.mean(scale_residuals**2)) * 1000

print(f"\nRMS reprojection error: {rms_angular_deg:.4f}° (angular)")
print(f"RMS wand-length error:  {rms_scale_mm:.4f} mm")

# ==========================================================================
# 5. RESULTS EXTRACTION
# ==========================================================================
final_cam1, final_cam2, final_points = unpack_parameters(best_res.x, num_placements)



def print_human_angles(rvec, cam_name):
    R_cv, _ = cv2.Rodrigues(rvec)
    R_local = R_cv 
    yaw, pitch, roll = Rotation.from_matrix(R_local).as_euler('YXZ', degrees=True)
    print(f"{cam_name} Euler Angles: [Yaw: {yaw:+.2f}°, Pitch: {pitch:+.2f}°, Roll: {roll:+.2f}°]")

pos1 = get_physical_position(final_cam1[:3], final_cam1[3:])
pos2 = get_physical_position(final_cam2[:3], final_cam2[3:])

print("\n=== RAW CAMERA 1 POSE (Relative to Cam 0) ===")
print_human_angles(final_cam1[:3], "Camera 1")
print(f"Solved Position:    {pos1.round(4)} meters")

print("\n=== RAW CAMERA 2 POSE (Relative to Cam 0) ===")
print_human_angles(final_cam2[:3], "Camera 2")
print(f"Solved Position:    {pos2.round(4)} meters")

# ==========================================================================
# 6. PLANE FIT IN NEW REFERENCE FRAME
# ==========================================================================
r0 = np.zeros(3)
t0 = np.zeros(3)
r1, t1 = final_cam1[:3], final_cam1[3:]
r2, t2 = final_cam2[:3], final_cam2[3:]



def angle_to_world_ray(az_rad, el_rad, rvec, tvec):
    """Converts Az/El into a 3D unit ray in camera-0's world frame."""
    x_norm = np.tan(az_rad)
    y_norm = -np.tan(el_rad) / np.cos(az_rad)
    ray_cam = np.array([x_norm, y_norm, 1.0])
    R_cv, _ = cv2.Rodrigues(rvec)
    ray_world = R_cv.T @ ray_cam
    return ray_world / np.linalg.norm(ray_world)

def triangulate_closest_point(camera_positions, ray_directions):
    """Finds the 3D point minimising orthogonal distance to all rays."""
    A = np.zeros((3, 3))
    b = np.zeros(3)
    for C, v in zip(camera_positions, ray_directions):
        P = np.eye(3) - np.outer(v, v)
        A += P
        b += P @ C
    return np.linalg.solve(A, b)

# Pre-compute physical camera positions from solved poses
cam0_pos = get_physical_position(r0, t0)  # will be [0,0,0]
cam1_pos = get_physical_position(r1, t1)
cam2_pos = get_physical_position(r2, t2)

# Re-triangulate every LED point in placements 0-4 using 3-camera rays
NUM_PLANE_PLACEMENTS = 5
plane_points_3cam = np.full((NUM_PLANE_PLACEMENTS * 2, 3), np.nan)

for p_idx in range(NUM_PLANE_PLACEMENTS):
    for led_offset, obs_row_label in enumerate(["A", "B"]):
        i = p_idx * 2 + led_offset  # index into mock_obs arrays

        cam_positions = []
        ray_directions = []

        for (obs, rvec, tvec, cam_pos) in [
            (mock_obs0, r0, t0, cam0_pos),
            (mock_obs1, r1, t1, cam1_pos),
            (mock_obs2, r2, t2, cam2_pos),
        ]:
            if not np.isnan(obs[i][0]):
                ray = angle_to_world_ray(obs[i][0], obs[i][1], rvec, tvec)
                cam_positions.append(cam_pos)
                ray_directions.append(ray)

        if len(ray_directions) >= 2:
            plane_points_3cam[i] = triangulate_closest_point(cam_positions, ray_directions)
'''
print("\nRE-TRIANGULATED PLANE POINTS ")
for p_idx in range(NUM_PLANE_PLACEMENTS):
    i = p_idx * 2
    print(f"Placement_{p_idx}  LED_A: {plane_points_3cam[i].round(4)}   "
          f"LED_B: {plane_points_3cam[i+1].round(4)}")
'''

#Fit separate planes through LED A and LED B points ---
def fit_plane_svd(points):
    """SVD plane fit. Returns (centroid, normal)."""
    centroid = points.mean(axis=0)
    _, _, Vt = np.linalg.svd(points - centroid)
    normal = Vt[-1]               # last row = smallest singular value = normal
    normal /= np.linalg.norm(normal)
    return centroid, normal

pts_A = plane_points_3cam[0:NUM_PLANE_PLACEMENTS * 2:2]   # even: LED A
pts_B = plane_points_3cam[1:NUM_PLANE_PLACEMENTS * 2:2]   # odd:  LED B

# Drop any rows that failed triangulation 
pts_A = pts_A[~np.isnan(pts_A[:, 0])]
pts_B = pts_B[~np.isnan(pts_B[:, 0])]

centroid_A, normal_A = fit_plane_svd(pts_A)
centroid_B, normal_B = fit_plane_svd(pts_B)

print(f"\nPLANE FIT ")
print(f"LED A plane  normal: {normal_A.round(4)}")
print(f"LED B plane  normal: {normal_B.round(4)}")

# --- Step 6c: Build new reference frame ---
# Origin  : Placement_0 LED A
# X-axis  : Placement_0 LED A -> Placement_0 LED B
# Z-axis  : average of the two plane normals (consistent sign)
# Y-axis  : completes right-handed frame (Z cross X)

origin = plane_points_3cam[0]   # Placement_0 LED A
p0_B   = plane_points_3cam[1]   # Placement_0 LED B

x_axis = p0_B - origin
x_axis /= np.linalg.norm(x_axis)

# Average normal - ensure both normals point the same hemisphere before averaging
if np.dot(normal_A, normal_B) < 0:
    normal_B = -normal_B
z_axis = normal_A + normal_B
z_axis /= np.linalg.norm(z_axis)

# Gram-Schmidt: make x_axis orthogonal to z_axis
x_axis = x_axis - np.dot(x_axis, z_axis) * z_axis
x_axis /= np.linalg.norm(x_axis)

y_axis = np.cross(z_axis, x_axis)

# R_new rows are the new basis vectors; R_new @ (p - origin) gives p in new frame
R_new = np.vstack([x_axis, y_axis, z_axis])

def to_new_frame(points):
    """Transform Nx3 array from camera-0 frame into the new reference frame."""
    return (R_new @ (points - origin).T).T

# --- Step 6d: Transform everything into new frame ---

# All 30 solved points
final_points_new = to_new_frame(final_points)

# Camera positions
cam0_pos_new = to_new_frame(cam0_pos.reshape(1, 3)).flatten()
cam1_pos_new = to_new_frame(cam1_pos.reshape(1, 3)).flatten()
cam2_pos_new = to_new_frame(cam2_pos.reshape(1, 3)).flatten()

# Camera orientations: R_cam maps old-world->camera; new-world->camera is R_cam @ R_new.T
def get_euler_in_new_frame(rvec, cam_name):
    R_cam, _ = cv2.Rodrigues(rvec)
    R_cam_new = R_cam @ R_new.T
    yaw, pitch, roll = Rotation.from_matrix(R_cam_new).as_euler('YXZ', degrees=True)
    print(f"{cam_name} Euler Angles (new frame): "
          f"[Yaw: {yaw:+.2f}°, Pitch: {pitch:+.2f}°, Roll: {roll:+.2f}°]")
'''
print("\n=== SANITY CHECK: PLANE POINTS IN NEW FRAME (Z should be ~0) ===")
plane_pts_new = to_new_frame(plane_points_3cam[~np.isnan(plane_points_3cam[:, 0])])
for idx, p in enumerate(plane_pts_new):
    print(f"  point {idx}: {p.round(4)}")
'''
def extract_true_angles(rvec_solved):
    R_cv, _ = cv2.Rodrigues(rvec_solved)
    R_cam_to_world = R_cv.T
    forward_ray = R_cam_to_world[:, 2]
    dx, dy, dz = forward_ray
    dh = np.hypot(dx, dy)
    pitch = np.degrees(np.arctan2(dz, dh))
    yaw = np.degrees(np.arctan2(dy, dx))
    return yaw, pitch

def get_rvec_in_new_frame(rvec_old):
    R_old, _ = cv2.Rodrigues(rvec_old)
    R_new_frame = R_old @ R_new.T
    rvec_new, _ = cv2.Rodrigues(R_new_frame)
    return rvec_new.ravel()

rvec0_new = get_rvec_in_new_frame(r0)
rvec1_new = get_rvec_in_new_frame(r1)
rvec2_new = get_rvec_in_new_frame(r2)

print("\n CAMERA POSES IN NEW FRAME")
for label, pos, rvec_new in [
    ("Camera 0", cam0_pos_new, rvec0_new),
    ("Camera 1", cam1_pos_new, rvec1_new),
    ("Camera 2", cam2_pos_new, rvec2_new),
]:
    yaw, pitch = extract_true_angles(rvec_new)
    print(f"{label} Position: {pos.round(4)}")
    print(f"{label} Optical axis — Yaw: {yaw:.1f}°  Pitch: {pitch:.1f}°  (should be pitch ~ -70 to -90° for downward-facing)")

    
'''
print("\n=== ALL SOLVED POINTS IN NEW FRAME ===")
for p_idx in range(num_placements):
    i = p_idx * 2
    print(f"Placement_{p_idx}  "
          f"LED_A: {final_points_new[i].round(4)}   "
          f"LED_B: {final_points_new[i+1].round(4)}")

def get_rvec_in_new_frame(rvec_old):
    R_old, _ = cv2.Rodrigues(rvec_old)
    R_new_frame = R_old @ R_new.T
    rvec_new, _ = cv2.Rodrigues(R_new_frame)
    return rvec_new.ravel()
'''


rvec0_new = get_rvec_in_new_frame(r0)
rvec1_new = get_rvec_in_new_frame(r1)
rvec2_new = get_rvec_in_new_frame(r2)

camera_params = {
    "1": {"rvec": rvec0_new.tolist(), "pos": cam0_pos_new.tolist()},
    "2": {"rvec": rvec1_new.tolist(), "pos": cam1_pos_new.tolist()},
    "3": {"rvec": rvec2_new.tolist(), "pos": cam2_pos_new.tolist()},
}

with open("camera_params.json", "w") as f:
    json.dump(camera_params, f, indent=4)
print("Camera params saved to camera_params.json")

# ==========================================================================
# 7. 3D VISUALISER
# ==========================================================================
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection='3d')

# --- Helper: draw a camera pyramid ---
def draw_camera(ax, pos, rvec_old, label, color):
    R_old, _ = cv2.Rodrigues(rvec_old)
    look_cv  = R_old.T @ np.array([0, 0, 1.0])
    look_new = R_new @ look_cv
    look_new /= np.linalg.norm(look_new)

    up_cv    = R_old.T @ np.array([0, -1.0, 0])
    up_new   = R_new @ up_cv
    up_new  /= np.linalg.norm(up_new)

    right_new = np.cross(look_new, up_new)

    scale = 0.05
    tip   = pos
    base_centre = pos + look_new * scale
    corners = [
        base_centre + ( right_new + up_new) * scale * 0.6,
        base_centre + (-right_new + up_new) * scale * 0.6,
        base_centre + (-right_new - up_new) * scale * 0.6,
        base_centre + ( right_new - up_new) * scale * 0.6,
    ]

    faces = [[tip, corners[i], corners[(i+1) % 4]] for i in range(4)]
    poly  = Poly3DCollection(faces, alpha=0.6, facecolor=color, edgecolor='black', linewidth=0.5)
    ax.add_collection3d(poly)

    base_poly = Poly3DCollection([corners], alpha=0.3, facecolor=color, edgecolor='black', linewidth=0.5)
    ax.add_collection3d(base_poly)

    ax.quiver(*pos, *look_new, length=0.12, color=color,
              linewidth=2, arrow_length_ratio=0.3)
    ax.text(pos[0], pos[1], pos[2] + 0.05, label,
            color=color, fontsize=9, fontweight='bold',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none', pad=1))

# --- Draw cameras ---
cam_configs = [
    (cam0_pos_new, r0, "Cam 0", "#0077cc"),
    (cam1_pos_new, r1, "Cam 1", "#cc4400"),
    (cam2_pos_new, r2, "Cam 2", "#007700"),
]
for pos, rvec, label, color in cam_configs:
    draw_camera(ax, pos, rvec, label, color)

# --- Draw wand placements ---
for p_idx in range(num_placements):
    i    = p_idx * 2
    pt_A = final_points_new[i]
    pt_B = final_points_new[i + 1]

    is_plane = p_idx < 5
    color_A  = '#e6a800' if is_plane else '#444444'
    color_B  = '#ffcc00' if is_plane else '#888888'
    lcolor   = '#e6a800' if is_plane else '#aaaaaa'
    size_A   = 60        if is_plane else 30

    ax.scatter(*pt_A, color=color_A, s=size_A,  zorder=5, edgecolors='black', linewidths=0.5)
    ax.scatter(*pt_B, color=color_B, s=size_A*0.6, zorder=5, edgecolors='black', linewidths=0.5)
    ax.plot([pt_A[0], pt_B[0]], [pt_A[1], pt_B[1]], [pt_A[2], pt_B[2]],
            color=lcolor, linewidth=2 if is_plane else 1, alpha=0.8)

    mid = (pt_A + pt_B) / 2
    ax.text(mid[0], mid[1], mid[2] + 0.015, f"P{p_idx}",
            fontsize=7, color='black' if is_plane else '#555555', fontweight='bold')

# --- Floor grid at Z=0 ---
all_x = final_points_new[:, 0]
all_y = final_points_new[:, 1]
pad   = 0.2
gx = np.linspace(all_x.min() - pad, all_x.max() + pad, 10)
gy = np.linspace(all_y.min() - pad, all_y.max() + pad, 10)
for x in gx:
    ax.plot([x, x], [gy[0], gy[-1]], [0, 0], color='#cccccc', linewidth=0.4, alpha=0.5)
for y in gy:
    ax.plot([gx[0], gx[-1]], [y, y], [0, 0], color='#cccccc', linewidth=0.4, alpha=0.5)

# --- Origin marker ---
ax.scatter(0, 0, 0, color='red', s=120, zorder=10, marker='*', edgecolors='black', linewidths=0.5)
ax.text(0.01, 0.01, 0.02, "Origin (P0 LED-A)", color='red', fontsize=8, fontweight='bold')

# --- Axis arrows ---
arrow_len = 0.15
ax.quiver(0, 0, 0, arrow_len, 0, 0, color='red',   linewidth=2, arrow_length_ratio=0.2)
ax.quiver(0, 0, 0, 0, arrow_len, 0, color='green', linewidth=2, arrow_length_ratio=0.2)
ax.quiver(0, 0, 0, 0, 0, arrow_len, color='blue',  linewidth=2, arrow_length_ratio=0.2)
ax.text(arrow_len + 0.02, 0, 0, "X", color='red',   fontsize=10, fontweight='bold')
ax.text(0, arrow_len + 0.02, 0, "Y", color='green', fontsize=10, fontweight='bold')
ax.text(0, 0, arrow_len + 0.02, "Z", color='blue',  fontsize=10, fontweight='bold')

# --- Styling ---
ax.set_facecolor('white')
fig.patch.set_facecolor('white')
ax.set_xlabel("X (m)", fontsize=10, labelpad=8)
ax.set_ylabel("Y (m)", fontsize=10, labelpad=8)
ax.set_zlabel("Z (m)", fontsize=10, labelpad=8)
ax.xaxis.pane.fill = False
ax.yaxis.pane.fill = False
ax.zaxis.pane.fill = False
ax.xaxis.pane.set_edgecolor('#dddddd')
ax.yaxis.pane.set_edgecolor('#dddddd')
ax.zaxis.pane.set_edgecolor('#dddddd')
ax.grid(True, alpha=0.3)

# --- Legend ---
legend_elements = [
    plt.Line2D([0],[0], marker='o', color='w', markerfacecolor='#e6a800',
               markersize=10, markeredgecolor='black', label='Plane placements (P0–P4) LED A'),
    plt.Line2D([0],[0], marker='o', color='w', markerfacecolor='#ffcc00',
               markersize=8,  markeredgecolor='black', label='Plane placements (P0–P4) LED B'),
    plt.Line2D([0],[0], marker='o', color='w', markerfacecolor='#444444',
               markersize=8,  markeredgecolor='black', label='Air placements LED A'),
    plt.Line2D([0],[0], marker='o', color='w', markerfacecolor='#888888',
               markersize=6,  markeredgecolor='black', label='Air placements LED B'),
    plt.Line2D([0],[0], color='#0077cc', linewidth=2, label='Cam 0'),
    plt.Line2D([0],[0], color='#cc4400', linewidth=2, label='Cam 1'),
    plt.Line2D([0],[0], color='#007700', linewidth=2, label='Cam 2'),
    plt.Line2D([0],[0], marker='*', color='w', markerfacecolor='red',
               markersize=12, markeredgecolor='black', label='Origin (P0 LED-A)'),
]
ax.legend(handles=legend_elements, loc='upper left', fontsize=8,
          facecolor='white', edgecolor='#cccccc')

ax.set_title("Calibration Result — New Reference Frame", fontsize=13, pad=15)
ax.view_init(elev=25, azim=-60)

plt.tight_layout()
plt.show()

