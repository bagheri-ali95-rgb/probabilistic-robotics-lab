# PRO Lab – Probabilistic Robotics with ROS 2 & TurtleBot4

> **Course:** Probabilistic Robotics Lab  
> **Task (Group 2B1):** *Choose different Motion Models* – compare the Simple (Euler) and Differential-Drive (Exact-Arc) motion models and their impact on KF, EKF, and PF estimation.

---

## Table of Contents
1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Package Structure](#package-structure)
4. [Dependencies](#dependencies)
5. [Build & Setup](#build--setup)
6. [Running the Filters](#running-the-filters)
7. [Experiments](#experiments)
8. [Evaluation & Plots](#evaluation--plots)
9. [Results Summary](#results-summary)

---

## Overview

Three state-estimation filters are implemented as ROS 2 nodes and applied to
a TurtleBot4 simulation:

| Filter | Node | Output topic |
|--------|------|-------------|
| Kalman Filter (KF) | `kf_node` | `/kf/pose` |
| Extended Kalman Filter (EKF) | `ekf_node` | `/ekf/pose` |
| Particle Filter (PF) | `pf_node` | `/pf/pose`, `/pf/particles` |

Each filter supports two motion models selected via a ROS 2 parameter:

| Model | Parameter value | Description |
|-------|----------------|-------------|
| Simple (Euler) | `simple` | Linear velocity propagation |
| Differential Drive | `differential_drive` | Exact arc-integration |

---

## System Architecture

```
Gazebo Simulation
      │
      ├── /odom          (ground truth)
      └── /cmd_vel_unstamped  (control input)
             │
    ┌────────┴────────────────┐
    │                         │
    ▼                         ▼
kf_node / ekf_node         pf_node
(C++ ROS 2 node)           (C++ ROS 2 node)
    │                         │
    ▼                         ▼
/kf/pose  /ekf/pose       /pf/pose  /pf/particles
    │                         │
    └──────────┬──────────────┘
               ▼
        evaluator_node   ──►  ~/eval_*.csv
               │
               ▼
        landmark_node    ──►  /landmarks/markers  (RViz2)
               │
               ▼
            RViz2
```

---

## Package Structure

```
probabilistic-robotics-lab/
├── prob_rob_lab_pkg/           # C++ ROS 2 package
│   ├── include/prob_rob_lab_pkg/
│   │   ├── motion_models.hpp   # SimpleMotionModel + DifferentialDriveModel
│   │   └── filter_math.hpp     # Matrix math, Joseph update
│   ├── src/
│   │   ├── kf_node.cpp         # Kalman Filter
│   │   ├── ekf_node.cpp        # Extended Kalman Filter
│   │   ├── pf_node.cpp         # Particle Filter
│   │   └── cmd_vel_pub.cpp     # Test velocity publisher
│   ├── launch/
│   │   ├── start.launch.py                   # Standard launch (all 3 filters)
│   │   ├── experiment_motion_models.launch.py # Simple vs. Diff-Drive comparison
│   │   ├── experiment_q_variation.launch.py   # Process noise variation
│   │   └── experiment_r_variation.launch.py   # Measurement noise variation
│   └── config/rviz_config.rviz
├── scripts/
│   └── plot_results.py         # Offline RMSE + trajectory plots
└── README.md
```

Python support package (`turtlebot4_estimation`):
```
src/turtlebot4_estimation/turtlebot4_estimation/
├── motion_models.py        # Python motion models
├── motion_model_node.py    # Python ROS 2 node
├── landmark_node.py        # Landmark detection + RViz2 markers
└── evaluator_node.py       # Online RMSE + CSV logging
```

---

## Dependencies

- ROS 2 Humble (or Jazzy)
- TurtleBot4 simulation packages
- `rclcpp`, `rclpy`, `geometry_msgs`, `nav_msgs`, `tf2`, `tf2_geometry_msgs`
- `visualization_msgs`, `std_msgs`
- Python: `numpy`, `matplotlib`, `pandas`

Install Python dependencies:
```bash
pip3 install numpy matplotlib pandas
```

---

## Build & Setup

```bash
cd ~/prob_ros_ws
colcon build --symlink-install
source install/local_setup.bash
```

---

## Running the Filters

### Standard launch (both motion models selectable)
```bash
# Differential Drive (default)
ros2 launch prob_rob_lab_pkg start.launch.py

# Simple model
ros2 launch prob_rob_lab_pkg start.launch.py motion_model:=simple
```

### With evaluator and landmark detection
```bash
# Terminal 1 – filters
ros2 launch prob_rob_lab_pkg start.launch.py

# Terminal 2 – evaluator (writes RMSE to console + CSV)
ros2 run turtlebot4_estimation evaluator_node

# Terminal 3 – landmark detection
ros2 run turtlebot4_estimation landmark_node
```

### Visualise in RViz2
```bash
rviz2 -d prob_rob_lab_pkg/config/rviz_config.rviz
```

Add these displays manually if needed:
- `PoseWithCovarianceStamped` → `/kf/pose`, `/ekf/pose`, `/pf/pose`
- `PoseArray` → `/pf/particles`
- `MarkerArray` → `/landmarks/markers`
- `Odometry` → `/odom`

---

## Experiments

### 1. Motion Model Comparison (Special Task)
```bash
ros2 launch prob_rob_lab_pkg experiment_motion_models.launch.py
```
Runs all 3 filters simultaneously with **both** motion models.  
Output topics: `/kf_simple/pose`, `/kf_diff/pose`, etc.  
Two CSV files are written: `~/eval_simple_*.csv` and `~/eval_diff_*.csv`.

### 2. Process Noise (Q) Variation
```bash
ros2 launch prob_rob_lab_pkg experiment_q_variation.launch.py
```
Three EKF instances with `q_scale` ∈ {0.1, 1.0, 10.0}.

### 3. Measurement Noise (R) Variation
```bash
ros2 launch prob_rob_lab_pkg experiment_r_variation.launch.py
```
Three EKF instances with R scaled by ×0.1, ×1.0, ×10.0.

### 4. Landmark Detection
The landmark node defines three landmarks at `(2,0)`, `(2,2)`, `(0,2)` metres.
When the robot is within 3 m of a landmark, a noisy range/bearing measurement
is published on `/landmarks/detected` and orange cylinder markers appear in RViz2.

---

## Evaluation & Plots

After recording a run, generate plots with:

```bash
# Single run
python3 scripts/plot_results.py ~/eval_run_*.csv --out results/

# Motion model comparison
python3 scripts/plot_results.py \
    ~/eval_simple_*.csv ~/eval_diff_*.csv \
    --labels "Simple" "Diff-Drive" \
    --out results/motion_model/
```

Generated files:
| File | Content |
|------|---------|
| `trajectory_comparison.png` | 2-D paths: GT vs. KF / EKF / PF |
| `rmse_bar.png` | RMSE bar chart per filter |
| `error_over_time.png` | Absolute position error over time |

---

## Results Summary

*(Fill in after running experiments)*

| Experiment | KF RMSE-x | EKF RMSE-x | PF RMSE-x |
|---|---|---|---|
| Simple model | – | – | – |
| Differential Drive | – | – | – |
| Q low (0.1) | – | – | – |
| Q mid (1.0) | – | – | – |
| Q high (10.0) | – | – | – |

**Key findings:**
- Simple model assumes the robot moves in a straight line between steps → accumulates heading error in curves.
- Differential-Drive model integrates the arc exactly → better accuracy for circular/curved trajectories.
- PF is the most robust but requires more computation (~500 particles).
