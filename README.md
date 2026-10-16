# F1TENTH Autonomous Racing — ROS 1 / Python

Autonomous driving algorithms for a 1/10-scale F1TENTH race car, written in Python with `rospy` (ROS 1). I built the project solo, as a robotics course project at École Polytechnique (spring 2026), modelled on the UPenn F1TENTH course.

**Goal:** minimise lap time on a real track without compromising the safety of the vehicle.

**Result:** the car won the end-of-course class race. I ran it in simulation first, then on the physical car.

https://github.com/user-attachments/assets/2cbf3941-7fe6-45b1-b526-1cbdf6e66596


*Pure pursuit raceline follower in simulation at top speed.*

---

## Overview

The project went through three stages:

1. **Basic autonomy:** small reactive algorithms on LiDAR data.
2. **Racing other cars:** a reactive racing node robust enough for wheel-to-wheel racing.
3. **Optimal racing:** an offline raceline optimiser with a velocity profile, followed on the real car.

| Component | Type | File |
|---|---|---|
| Wall following (PD controller on the side wall distance) | Online, ROS node | [`scripts/Wall_follow_opti.py`](scripts/Wall_follow_opti.py) |
| Emergency braking | Online, ROS node | [`scripts/safety.py`](scripts/safety.py) |
| Reactive racing node (follow-the-gap + disparity extender) | Online, ROS node | [`scripts/Follow_the_gap_opti_v7.py`](scripts/Follow_the_gap_opti_v7.py) |
| Raceline optimiser + velocity profile | Offline, from the occupancy map | [`scripts/Raceline_optimizer_v2.py`](scripts/Raceline_optimizer_v2.py) |
| Lap timer | Online, ROS node | [`utils/Runtime.py`](utils/Runtime.py) |
| Waypoint logger | Online, ROS node | [`utils/waypoint_logger.py`](utils/waypoint_logger.py) |

The raceline follower (pure pursuit) used in the `PP-*` videos is not included in this repository.

> **What I wrote vs. what the course provided.** I wrote the nodes and tools listed above. The simulator (`f1tenth_simulator`, with its maps and launch files) was provided by the course.

---

## 1. Reactive racing node

Subscribes to `/scan` (`sensor_msgs/LaserScan`) and publishes `AckermannDrive` commands on `/nav`.

The pipeline runs on every scan:

1. **Smoothing.** A rolling mean over the last 3 LiDAR scans reduces noise.
2. **Dynamic safety bubble.** Around the closest obstacle, ranges are set to zero over an angular width computed from the car's width and the obstacle's distance. A close obstacle therefore blocks a wide sector and a far one a narrow sector.
3. **Adaptive gap threshold.** A beam counts as free if its range is above the **75th percentile** of the current scan. The threshold adapts to the track instead of being fixed.
4. **Target point.** The node takes the **range²-weighted centroid** of the largest free gap, not its farthest point. With $p_i$ the beam endpoints in the gap and $r_i$ their ranges:

$$p^\star = \frac{\sum_i r_i^2\, p_i}{\sum_i r_i^2}$$

   Aiming at the farthest point makes the car cut corners. The weighted centroid keeps it pulled toward open space while staying centred in the gap.
5. **Steering.** A PD controller on the target angle, with exponential smoothing of the command.
6. **Speed law.** Speed decreases with steering angle and increases with free distance ahead, clipped to **1.2–6.5 m/s**.

A **disparity extender** is also implemented. It finds large jumps between neighbouring LiDAR ranges and extends the closer obstacle by half the car's width, so the car doesn't clip corners.

---

## 2. Raceline optimiser (offline)

Takes the occupancy-grid map of the track and outputs a raceline with a reference speed at every point.

### Centreline extraction

1. Threshold, then morphological opening and closing to clean the map.
2. Skeletonisation, then pruning of spurious branches.
3. Tracing the loop with 8-connectivity, enforcing direction continuity.
4. A distance transform to keep **0.2 m of clearance** from the walls.

### Curvature minimisation

The path is first smoothed with Laplacian smoothing, then **80 control points** are moved to minimise the total Menger curvature. For three consecutive points $a, b, c$:

$$\kappa(a,b,c) = \frac{4\,\mathcal{A}(a,b,c)}{\lVert a-b\rVert\,\lVert b-c\rVert\,\lVert c-a\rVert}$$

where $\mathcal{A}$ is the area of the triangle they form. The optimisation uses a decaying step size, resamples the points periodically, and respects the clearance constraint. A periodic B-spline then gives **1000 waypoints**.

### Velocity profile

1. **Lateral grip limit** at each point, with $a_{lat} = 8\ \text{m/s}^2$:

$$v_{max}(s) = \sqrt{\frac{a_{lat}}{\kappa(s)}}$$

2. **Forward pass** (acceleration limit, $a_{acc} = 3\ \text{m/s}^2$):

$$v_{i+1} \le \sqrt{v_i^2 + 2\,a_{acc}\,\Delta s}$$

3. **Backward pass** (braking limit, $a_{brk} = 6\ \text{m/s}^2$):

$$v_i \le \sqrt{v_{i+1}^2 + 2\,a_{brk}\,\Delta s}$$

Both passes follow from $v^2 = v_0^2 + 2a\,\Delta s$ at constant acceleration. The curvature is smoothed with a sliding maximum (window of 15 points) so the car slows down before a corner, and the grip limit is clipped to 1–10 m/s. The reference speed at each point is the minimum of the three limits.

**Output:** a CSV `<map name>_raceline.csv` with columns `x_m, y_m, v_ref_m_s`, read by the raceline follower.

---

## 3. Sim-to-real

The algorithms that worked in simulation needed two fixes on the real car:

| Problem on the real car | Cause | Fix |
|---|---|---|
| **Phantom gaps.** The car steered toward walls it read as open space | Poorly reflective walls gave **missing LiDAR returns** | Added **reflective material** on the walls so the LiDAR gets valid returns |
| **Overshooting corners** at the simulated speeds | **Tyre slip** on the real surface differs from the simulator | Lowered the reference velocity profile by **15%** |

**Next step:** handle missing returns in software instead of changing the track, for example by filtering the scans with a **Kalman filter** that predicts the missing ranges, as in published work on LiDAR dropouts.

---

## Results

- **Won the class race** (end of course, spring 2026).
- Reactive node speed range: **1.2–6.5 m/s**. Raceline reference speed: up to **10 m/s**.
- Lap times can be measured with [`utils/Runtime.py`](utils/Runtime.py), which prints the time of each lap from `/gt_pose`.

---

## Running

Assumes a catkin workspace with the course simulator (`f1tenth_simulator`) in `~/catkin_ws/src`.

```bash
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# Offline: compute the raceline from a map (edit yaml_path at the top of the script)
python3 scripts/Raceline_optimizer_v2.py     # writes <map name>_raceline.csv

# Simulation
roslaunch f1tenth_simulator simulator.launch
python3 scripts/Follow_the_gap_opti_v7.py    # reactive racing node
python3 scripts/safety.py                    # optional emergency braking
python3 utils/Runtime.py                     # optional lap timer
```

**Dependencies:** ROS 1, Python 3, `rospy`, `numpy`, `scipy`, `opencv-python`, `scikit-image`, `matplotlib`, `pyyaml`.

---

## Stack

Python · ROS 1 (`rospy`) · LiDAR (`LaserScan`) · Ackermann steering · occupancy grids · trajectory optimisation
