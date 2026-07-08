# Multi-Camera LED Tracking System for Free-Flying Drone Pose Estimation

A three-camera, phase-locked optical tracking system built for pose estimation of a free-flying quadcopter, developed during a summer research placement in Prof. Pietro Cicuta's lab, University of Cambridge (with Sam Zhuang).

**Live tracking accuracy: 8.51mm RMS**, from three unsynchronized Raspberry Pi cameras and a self-calibrated extrinsic geometry (no external motion-capture reference used).

---

## Summary

| Stage | Result |
|---|---|
| Angular reprojection error (bundle adjustment) | 0.02° RMS |
| Wand-length error (calibration self-consistency) | 0.29mm RMS |
| Camera position accuracy (vs. tape measure) | ~1cm |
| Live tracking accuracy | 8.51mm RMS |
| Position repeatability | 1mm |

The system solves three problems in sequence:

1. **Time sync** — get three independently-clocked cameras to agree on which of four strobing LEDs is currently active.
2. **Extrinsic calibration** — recover the 3D position and orientation of all three cameras with no external tracking system to check against.
3. **Triangulation** — combine three noisy angle observations into a live 3D point.

---

## 1. Frame Synchronization

### The problem

Four LEDs strobe sequentially, each active for 10ms. Three Raspberry Pi cameras capture at 100fps, completely independently — no shared clock, no hardware trigger. If two cameras disagree about which LED is currently lit, they report angle observations for two different physical points, and triangulation silently produces meaningless output — the rays never had a reason to intersect.

### Architecture

```
LED Controller ──UDP broadcast──▶ Clock Offset Estimation (per Pi) ──▶ Shared Memory ──▶ Camera PLL
```

**LED Controller** (`sync/transmit2.py`)
Strobes 4 LEDs round-robin at a 10ms target period. Naive `sleep(10ms)` drifts due to OS scheduling jitter and variable UDP send delay. Fixed with a single accumulating correction, clamped to ±15μs per cycle:

```python
offsets[i] += clamp(kp * (duration - period), ±15)
```

Note: this is **integral-only** control — the correction accumulates every cycle and is never reset, and there's no separate fresh proportional term added on top. `kp` here functions as an integral gain, not a proportional one, despite the naming.

**Clock Offset Estimation**
Each Pi has an unsynchronized clock. Offset is computed from both monotonic and NTP timestamps broadcast by the server each cycle:

```
O = (C_mono − S_mono) − (C_NTP − S_NTP)
```

Combining both timestamp types cancels one-way network delay, which isn't possible from a single timestamp pair. The result is smoothed with an EMA (α=0.05) to filter transient packet jitter, then published lock-free to a shared memory buffer (`netbuf`).

The **monotonic** clock is used specifically because it increases smoothly with no jumps — essential for a tight feedback loop. NTP time can be adjusted/jump, which would destabilize a microsecond-scale control loop.

**Camera PLL** (`sync/camera_pll.py`)
Each frame, the camera estimates server time from the shared-memory offset, computes its phase within the current 10ms LED cycle, and corrects via:

```python
nudge = kp * phase_error + ki * integral
```

This is **proportional + integral** control — a fresh `kp * phase_error` term is summed with the accumulated integral term every cycle. (A `kd` derivative term is defined in the code but never used in the output — worth noting as unused/dead code rather than describing this as full PID.)

**Why 100fps, not higher:** `FrameDurationLimits` sets frame duration exactly. At 100fps, frame duration (10ms) matches the LED period exactly, giving the PID full bidirectional correction authority. At 120fps, frame duration (8.3ms) is shorter than the LED period — the loop can only ever shorten further, losing half its authority, and phase lock becomes unstable.

### Why IR LEDs instead of color-coded LEDs

An alternative to strobing would be giving each LED a distinct color so cameras could identify them directly, with no timing sync needed. Rejected because:
- Color detection is slower for the CV pipeline than a single grayscale threshold
- Prone to false positives from ambient light, reflections, and white-balance shifts

Instead, LEDs run in near-infrared, at the edge of the camera sensor's spectral range — outside most ambient lighting, which substantially cuts interference. The tradeoff: all LEDs now look identical to the camera. The entire sync system exists to recover the "which LED is this" information that color would have given for free, without the false-positive cost.

### LED Detection

Must complete well within the 10ms window (target: <0.5ms). Benchmarked 4 approaches:

| Method | Avg time | Notes |
|---|---|---|
| NumPy average (full) | 1.33ms | `np.nonzero()` dominates — too slow |
| NumPy (direct thresh) | 0.90ms | Faster but still 5x too slow |
| Image moments | 0.35ms | More accurate centroid, still too slow |
| **Contour detection** | **0.17ms** | Threshold → findContours → minEnclosingCircle — **winner** |

### FOV constraint

High frame rates force ISP center-cropping, shrinking effective FOV. Solved by running two simultaneous streams: a high-res main stream (unused) and a low-res `lores` stream for detection, which downsamples the full sensor instead of center-cropping. Settled on 1152×648 @ 100fps as the best achievable tradeoff — crop start coordinates are fixed at kernel build time and can't be adjusted at runtime.

---

## 2. Extrinsic Calibration

### The wand

A rigid rod with two LEDs (LED A, LED B) exactly 100mm apart. 15 placements:
- Placements 0–4: laid flat on the floor, defining the Z=0 reference plane
- Placements 5–14: held freely in the air at random positions/angles
- Placement 0: anchors the origin, pointing along +X

**Determinability check:** each placement gives 12 known values (2 angles × 2 LEDs × 3 cameras) against 5 unknowns per placement. Solving `18 + 5N < 12N` gives N ≥ 3 placements required. 15 were used — 5x the theoretical minimum.

### Data collection pipeline

Two scripts over TCP:
- **`calibration_master.py`** (laptop) — waits for all 3 cameras to report continuous wand visibility, sends `CAPTURE`, logs responses, auto-advances.
- **`camera_client_calibration.py`** (per Pi) — state machine: `SEARCHING → WAIT_CAPTURE → COUNTDOWN (3s) → WAIT_REMOVAL`.

### Initial guess — epipolar bootstrap

Bundle adjustment is a local optimizer and needs a good starting guess. The essential matrix `E` encodes the epipolar constraint (`p1ᵀ E p0 = 0`) between camera pairs, solved via RANSAC on the angle observations. Decomposing `E` gives 4 candidate (R, t) pairs; a cheirality check (points must be in front of both cameras) selects the valid one.

**Coplanarity trap:** solving for `E` requires points spanning true 3D volume. Placements 0–4 (floor) are coplanar and make `E` degenerate. Fix: mask placements 0–4 out of this step, use only the 10 air placements.

**Scale recovery:** `E`-based (R, t) is scale-invariant. The wand is triangulated under the initial estimate, its apparent length measured, and `t` rescaled until that length equals 0.1m exactly — giving bundle adjustment a near-metric starting pose.

### Bundle adjustment

Joint optimization of all camera poses and 3D point positions via `scipy.least_squares` (Levenberg-Marquardt). Residual vector:

1. **Angular residuals** — `predicted_angle − observed_angle`, for every point/camera pair
2. **Scale residuals** — `|LED_A − LED_B| − 0.1m`, for every placement

**Perturbation loop:** as a local optimizer, bad initial guesses can converge to the wrong minimum. The optimization runs up to 5 times; if a run converges above cost ~0.5, a random perturbation is applied and it re-runs. Attempt 2 typically finds the true global minimum.

### Results

- 0.02° RMS angular reprojection error
- 0.29mm RMS wand-length error
- ~1cm camera position accuracy, verified against a physical tape measure along the Z-axis

---

## 3. Triangulation

### Method

Each detected LED gives a ray in 3D space from the observing camera's position, in the direction implied by its angle observation. With 3 noisy cameras, the point minimizing summed squared perpendicular distance to all rays is found by solving a direct 3×3 linear system:

```
A·P = b
```

RMS ray-agreement (perpendicular distance from the solved point to each ray) is computed as a live diagnostic — see `triangulation/trinagulation_final_boss.py`.

### Live example

```
Packet received: ID=2, LED=2.0, H=-6.63, V=-2.70
Packet received: ID=1, LED=2.0, H=-7.89, V=-2.35
Packet received: ID=3, LED=2.0, H=15.72, V=-0.64
[LED 2.0] cams=['3', '1', '2']  pos_new=(0.1057, 0.0444, -0.0440) m  rms=8.51mm
```

---

## Known Limitations / Next Steps

- **Wand rigidity**: some residual calibration error likely comes from slight flex in the calibration wand. A stiffer wand should directly tighten the wand-length residual and downstream accuracy.
- **Distortion coefficients**: `k1`/`k2` were observed with opposite signs (non-monotonic) at one point — worth re-running checkerboard calibration with `CALIB_FIX_K3` to confirm this is resolved.
- **Single-point-pair tracking**: live tracking currently follows a single 2-LED wand. Actual drone pose estimation will use multiple LEDs constrained to one rigid body, which should meaningfully improve robustness over the two-point case here via redundancy.
- **Bad-minimum convergence**: the perturbation loop's failure mode when bundle adjustment lands in a wrong local minimum is consistent with a gimbal-lock-type degeneracy in the rotation parameterization — worth formal verification against the specific rotation representation used.

---

## Repository Structure

```
camera-tracking-system/
├── README.md
├── sync/
│   ├── transmit2.py              # LED controller, integral-only timing correction
│   └── camera_pll.py             # per-camera phase-lock loop (P+I)
├── calibration/
│   ├── camera_client_calibration.py
│   ├── calibration_master.py
│   ├── bundle_adjustment.py
│   └── camera_params.json        # example calibrated extrinsics
├── triangulation/
│   └── trinagulation_final_boss.py
├── docs/
│   ├── architecture-diagrams/
│   └── results/
└── media/
    ├── setup_photo.jpg
    └── calibration_result.png
```

## Setup / Running

_(fill in: hardware list, dependencies, run order for master/client scripts, network config)_