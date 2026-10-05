# GPS-Denied Autonomous Drone Exploration

An end-to-end software simulation and interactive exploration framework for autonomous UAV navigation, mapping, and frontier-based exploration in GPS-denied environments.

---

## 📌 Project Overview

In GPS-denied environments (e.g., indoor complexes, urban canyons, collapsed structures, underground tunnels), drones cannot rely on global satellite positioning. This project implements a fully autonomous exploration stack where a drone self-localizes, builds an occupancy grid map in real time, detects exploration frontiers, plans collision-free paths around obstacles, and navigates autonomously to explore unknown territory.

### Key Capabilities
- **GPS-Denied Autonomy**: Real-time localization and mapping using simulated depth sensors and IMU / VIO odometry.
- **Dynamic Occupancy Grid**: Real-time 2D grid generation categorized into:
  - 🟩 **Explored Area** (Free space)
  - 🟥 **Obstacles** (Walls, buildings, obstacles with safety inflation margins)
  - 🟨 **Frontiers** (Boundaries between explored and unexplored zones)
  - ⬛ **Unexplored Territory**
- **Frontier-Based Exploration**: Autonomous frontier detection, clustering, information-gain scoring, and goal selection.
- **Obstacle Avoidance & Path Planning**: A* pathfinding with clearance checks ensuring safe obstacle clearance.
- **Interactive 3D Simulation & Presentation Suite**: Real-time browser-based 3D digital twin and mission control visualizer.

---

## 🏗️ Architecture & Technology Stack

```text
[Simulated Sensors (Depth / LiDAR / VIO)]
                 │
                 ▼
     [2D Occupancy Grid Mapping]
                 │
                 ▼
      [Frontier Detection & Clustering]
                 │
                 ▼
  [A* Path Planning & Obstacle Avoidance]
                 │
                 ▼
 [Offboard Velocity / Waypoint Controller]
                 │
                 ▼
    [PX4 SITL / Gazebo Flight Dynamics]
```

### Core Technologies
| Component | Technology |
|---|---|
| **Robotics Middleware** | ROS 2 Humble |
| **Physics & Sensor Simulation** | Gazebo Harmonic / PX4 SITL |
| **DDS Bridge** | uXRCE-DDS |
| **Core Autonomy Algorithms** | C++ / Python |
| **Path Planning & Mapping** | A* Search, Occupancy Grid Mapping, Frontier Exploration |
| **Interactive Suite** | HTML5, CSS3, JavaScript (Three.js 3D Engine & Canvas 2D) |

---

## 🚀 Getting Started

### 1. Launch Interactive Simulation & Presentation Dashboard
The repository includes a standalone interactive simulation and mission control dashboard with real-time 3D flight rendering, 2D occupancy mapping, telemetry graphs, and algorithm breakdowns:

```bash
chmod +x run_presentation.sh
./run_presentation.sh
```
This starts a local HTTP server and opens the dashboard in your default browser at `http://localhost:8000`.

### 2. ROS 2 Workspace Build & Run (Ubuntu 22.04 + ROS 2 Humble)

To build the native ROS 2 packages:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --symlink-install

# Source workspace overlay
source install/setup.bash

# Launch simulation and autonomy nodes
ros2 launch simulation simulation.launch.py
```

---

## 📁 Repository Structure

```text
GPS_DeniedV2/
├── presentation_app/          # Interactive 3D/2D mission presentation dashboard
│   ├── index.html             # Main dashboard UI
│   ├── style.css              # Custom styling & themes
│   ├── app.js                 # Dashboard controller & UI event logic
│   └── sim_engine.js          # 3D Three.js physics & 2D grid autonomy engine
├── src/                       # ROS 2 source packages
│   ├── mapping/               # Real-time occupancy grid mapping nodes
│   ├── exploration/           # Frontier detection & scoring algorithms
│   ├── control/               # PX4 offboard trajectory & waypoint controllers
│   └── localization/          # VSLAM/VIO & ground truth pose estimation
├── simulation/                # Gazebo worlds, models, and launch scripts
│   ├── launch/                # ROS 2 launch configurations
│   ├── models/                # Drone and environment SDF models
│   └── worlds/                # Simulation world environments
├── mdFiles/                   # Detailed technical reports and milestone documentation
├── run_presentation.sh        # Quick-start script for presentation dashboard
├── .gitignore                 # Git ignore rules
└── README.md                  # Project overview and documentation
```

---

## 📖 Documentation
Detailed milestone reports, mathematical formulations, and algorithmic deep dives are available in the [`mdFiles/`](file:///Users/jiviteshbansal/Downloads/GPS_DeniedV2/mdFiles) directory:
- [`mdFiles/architecture.md`](file:///Users/jiviteshbansal/Downloads/GPS_DeniedV2/mdFiles/architecture.md) — System architecture and node communication
- [`mdFiles/algorithms.md`](file:///Users/jiviteshbansal/Downloads/GPS_DeniedV2/mdFiles/algorithms.md) — Mathematical details for mapping, frontier detection, and A*
- [`mdFiles/mapping.md`](file:///Users/jiviteshbansal/Downloads/GPS_DeniedV2/mdFiles/mapping.md) — Occupancy grid generation and raycasting
- [`mdFiles/exploration.md`](file:///Users/jiviteshbansal/Downloads/GPS_DeniedV2/mdFiles/exploration.md) — Frontier exploration strategies
