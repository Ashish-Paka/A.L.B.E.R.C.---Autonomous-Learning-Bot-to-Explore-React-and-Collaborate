<div align="center">

# A.L.B.E.R.C.

### Autonomous Learning Bot to Explore, React and Collaborate

![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-green.svg)
![JetPack 6.x](https://img.shields.io/badge/JetPack-6.x-76b900.svg)
![CUDA 12.x](https://img.shields.io/badge/CUDA-12.x-76b900.svg)
![Docker](https://img.shields.io/badge/Docker-ARM64%20%7C%20x86-2496ED.svg)
![Platform](https://img.shields.io/badge/Platform-Jetson%20Orin%20Nano-orange.svg)

**A three-tier autonomous mobile robot for indoor 3D mapping, dynamic obstacle evasion, and natural-language human-robot collaboration**

**Author:** Ashish Paka

**Affiliation:** M.S. Robotics and Autonomous Systems, Arizona State University

**Research Lab:** LOGOS Robotics Lab, Arizona State University

</div>

---

## Table of Contents

- [Overview](#overview)
  - [Novel Research Contribution](#novel-research-contribution)
  - [Capabilities](#capabilities)
  - [Platform at a Glance](#platform-at-a-glance)
- [Quick Start](#quick-start)
  - [Prerequisites](#prerequisites)
  - [Docker - Jetson (ARM64)](#docker--jetson-arm64)
  - [Docker - x86 Development](#docker--x86-development)
  - [Native Install](#native-install)
  - [First Run Verification](#first-run-verification)
- [Architecture](#architecture)
  - [System Data Flow](#system-data-flow)
  - [Three-Tier Hardware](#three-tier-hardware)
  - [Compute Architecture](#compute-architecture)
  - [Communication Topology](#communication-topology)
  - [ROS2 Node Graph](#ros2-node-graph)
  - [Motor Control and Odometry Pipeline](#motor-control-and-odometry-pipeline)
  - [TF2 Frame Tree and URDF](#tf2-frame-tree-and-urdf)
  - [Boot Sequence](#boot-sequence)
- [Sensor Fusion and Perception](#sensor-fusion-and-perception)
  - [Extended Kalman Filter](#extended-kalman-filter)
  - [SDK-Native Segmentation](#sdk-native-segmentation)
  - [Three Sensor Fusion Views](#three-sensor-fusion-views)
  - [Perception Fusion](#perception-fusion)
  - [SLAM](#slam)
  - [Custom Model Deployment](#custom-model-deployment)
  - [Multi-Object Tracking](#multi-object-tracking)
- [Path Planning and Navigation](#path-planning-and-navigation)
  - [Three Planning Modes](#three-planning-modes)
  - [Frontier-Based Exploration](#frontier-based-exploration)
  - [Costmap Configuration](#costmap-configuration)
  - [Local Trajectory Optimization (MPPI)](#local-trajectory-optimization-mppi)
  - [Motion Envelope](#motion-envelope)
  - [Reactive Evasion and Collaborative Movement](#reactive-evasion-and-collaborative-movement)
  - [Reinforcement Learning Integration Path](#reinforcement-learning-integration-path)
- [LLM Intelligence](#llm-intelligence)
  - [Provider Abstraction Layer](#provider-abstraction-layer)
  - [VLM-Augmented Perception](#vlm-augmented-perception)
  - [Voice Pipeline](#voice-pipeline)
  - [Task Decomposition](#task-decomposition)
  - [LLM Action Plan Schema](#llm-action-plan-schema)
  - [Semantic Map](#semantic-map)
- [User Interfaces](#user-interfaces)
  - [Phone Docked Display](#phone-docked-display)
  - [Browser Dashboard](#browser-dashboard)
  - [Mobile Browser UI](#mobile-browser-ui)
  - [VR and WebXR](#vr-and-webxr)
  - [Export & File Management](#export--file-management)
    - [Export Profiles](#export-profiles)
    - [Supported Export Formats](#supported-export-formats)
    - [ZIP Package Structure](#zip-package-structure)
    - [Three Sensor Fusion Views as Export Data Layers](#three-sensor-fusion-views-as-export-data-layers)
    - [Mesh Reconstruction Pipeline](#mesh-reconstruction-pipeline)
    - [Isaac Sim USD Export](#isaac-sim-usd-export)
    - [Gazebo SDF World Export](#gazebo-sdf-world-export)
    - [Temporal and 4D Data Export](#temporal-and-4d-data-export)
    - [Dashboard Export UI](#dashboard-export-ui)
    - [export_manager_node ROS2 Service Interface](#export_manager_node-ros2-service-interface)
- [Hardware](#hardware)
  - [Component Summary](#component-summary)
  - [Power Architecture](#power-architecture)
  - [Mechanical Design](#mechanical-design)
  - [Grounding](#grounding)
  - [Safety Switches](#safety-switches)
  - [Phone Integration](#phone-integration)
- [Docker](#docker)
  - [Container Architecture](#container-architecture)
  - [Jetson Deployment](#jetson-deployment)
  - [x86 Development](#x86-development)
  - [Environment Variables](#environment-variables)
- [Distributed Computing](#distributed-computing)
  - [Jetson-First Design](#jetson-first-design)
  - [Multi-Machine Launch](#multi-machine-launch)
  - [Testing Playground](#testing-playground)
  - [Bot Projects](#bot-projects)
- [Integration Notes](#integration-notes)
  - [Known Risks and Mitigations](#known-risks-and-mitigations)
  - [Sensor Calibration](#sensor-calibration)
  - [Thermal Management](#thermal-management)
  - [Graceful Shutdown](#graceful-shutdown)
  - [Data Logging and Bag Recording](#data-logging-and-bag-recording)
  - [Clock Synchronization](#clock-synchronization)
- [Repository Structure](#repository-structure)
- [Contributing](#contributing)
- [License](#license)
- [Appendix A: Component Datasheet Specifications](#appendix-a-component-datasheet-specifications)
- [Appendix B: Power Budget](#appendix-b-power-budget)

---

## Overview

ALBERC is a three-tier differential-drive mobile robot platform for autonomous indoor navigation, real-time 3D mapping, dynamic obstacle evasion, and natural-language human-robot collaboration. The system fuses a hemispherical 4D LiDAR (Unitree L1, 360x90 deg FOV, 21,600 pts/sec), a stereo depth camera with onboard visual-inertial odometry (Stereolabs ZED Mini, +/-1mm pose accuracy at 100 Hz), a 9-axis IMU (ICM-20948 with onboard DMP), four ultrasonic proximity sensors, and quadrature wheel encoders into a unified state estimate via an Extended Kalman Filter. The platform runs ROS2 Humble on a Jetson Orin Nano Super Developer Kit (67 TOPS, Ampere GPU, 1024 CUDA cores), with an Arduino Mega 2560 as the real-time I/O co-processor and a docked smartphone providing GPS, microphone, speaker, hotspot, and an animated face display. JetPack, the ZED SDK, and the Unitree L1 SDK are configured and operational.

Perception operates through a two-tier segmentation architecture: SDK-native segmentation from both the L1 (voxel/point cloud) and ZED (object detection with depth + sRGB) runs as an always-on baseline, with custom TensorRT models layered on top for application-specific classification. Three distinct sensor fusion views - Photorealistic, Voxel/Point Cloud, and Compound (all sensors fused with VLM Gemini context labels) - are available simultaneously via the browser dashboard and VR/WebXR.

A provider-agnostic LLM interface (supporting OpenAI GPT, Anthropic Claude, and Google Gemini via a unified API abstraction) serves as the cognitive layer - interpreting voice commands through the docked phone's microphone, decomposing high-level instructions into Nav2 action sequences, and responding via text-to-speech. A browser-based dashboard provides full telemetry, teleoperation, LLM conversational input, switchable sensor views, map management, planner selection, and 3D visualization. All spatial data is exportable as PLY/OBJ/glTF/USD/PCD/FBX via environment-specific ZIP bundles (Isaac Sim, Gazebo, Visualization, Blender, Web Viewer) and viewable in VR on Meta Quest via WebXR. Any ROS2 node can be offloaded from the Jetson to a laptop at launch time via ROS2 distributed computing.

The defining behavioral property is **mission-context-dependent dynamic reactivity**: the robot continuously tracks all moving entities in 3D via fused LiDAR-vision Kalman filtering at 10 Hz and executes evasive maneuvers at 20 Hz for any entity not explicitly designated as an interaction target by the active mission. This is not a safety fallback - it is the primary control authority.

### Novel Research Contribution

ALBERC's research contribution lies at the intersection of six capabilities rarely integrated on a single sub-$1000 indoor platform:

- **Mission-context-dependent reactive evasion** - 20 Hz time-to-collision assessment with graduated response (monitor / costmap injection / escape vector / emergency stop), context-gated against LLM-designated interaction targets. The robot evades everything it is not actively collaborating with.
- **Two-tier SDK-native + custom segmentation** - leveraging L1 and ZED SDK built-in segmentation as an always-on perception baseline, with TensorRT custom models layered for application-specific needs, avoiding the brittleness of single-model-only perception.
- **Three simultaneous sensor fusion views** - photorealistic, voxel/point cloud, and VLM-annotated compound views, providing complementary spatial understanding for navigation, mapping, and human comprehension.
- **Provider-agnostic LLM cognition with VLM grounding** - task decomposition, scene understanding, and natural-language interaction backed by any cloud or local LLM, with Gemini VLM providing spatial context labels fused into the compound view.
- **Distributed ROS2 compute** - any node in the system can run on the Jetson or an offload laptop, selected at launch time, enabling heavy workloads (SLAM, TensorRT inference, VLM queries) to be distributed without architectural changes.
- **End-to-end open-source stack** - from hardware BOM to Docker deployment, designed for reproducibility in academic robotics labs.

### Capabilities

- **SLAM:** Dual 2D/3D simultaneous localization and mapping (SLAM Toolbox + RTAB-Map) with persistent cross-session map storage
- **Navigation:** Three runtime-selectable global planners (A*, RRT*, Voronoi GVD) + local trajectory optimization (MPPI, 20 Hz) with dynamic replanning
- **Sensor Fusion:** EKF-based multi-source odometry fusion (wheel encoders, ICM-20948 IMU, ZED Mini VIO) with three independent IMU streams
- **SDK-Native Segmentation:** L1 voxel/point cloud segmentation + ZED object segmentation with depth and sRGB, always-on baseline beneath custom TensorRT models
- **Object Detection:** TensorRT-accelerated deep neural network inference (YOLO/MobileNet-SSD) on ZED Mini RGB at 15+ FPS for semantic classification
- **Object Tracking:** Multi-object 3D Kalman filtering fusing LiDAR scan differencing with vision-based detections, maintaining persistent tracks with ID, class, velocity, and confidence
- **Reactive Evasion:** 20 Hz time-to-collision assessment with four-tier graduated response, context-gated by mission interaction targets
- **Collaborative Interaction:** Voice-driven task execution via provider-agnostic LLM, person following, pet monitoring, semantic scene Q&A
- **Three Sensor Views:** Photorealistic (ZED sRGB + segmentation overlays), Voxel/Point Cloud (L1 SDK-native), Compound (all sensors fused + VLM Gemini context labels)
- **Mapping Output:** 2D occupancy grids, colored 3D point clouds, semantic map overlays - stored, streamed, and exported as PLY/OBJ/glTF/USD/PCD/FBX via environment-specific ZIP bundles (Isaac Sim, Gazebo, Visualization, Blender, Web Viewer) with scene descriptions, all sensor data, semantic metadata, trajectory, and optional ROS bag clips
- **Interfaces:** Browser dashboard (telemetry + control + LLM chat + planner selection), phone face display (animated avatar + status + chat), VR/WebXR on Meta Quest, mobile touch UI
- **Distributed Compute:** Any ROS2 node launchable on Jetson or laptop via ROS2 multi-machine configuration
- **Bot Projects:** Deploy scripted missions, ROS2 packages, Jupyter notebooks, and custom code as reusable project bundles

### Platform at a Glance

| Parameter | Value |
|---|---|
| Dimensions | ~25 x 20 x 25 cm (L x W x H) |
| Weight | ~3.5 kg (estimated with battery) |
| Drive | 2WD differential + ball caster |
| Primary Compute | Jetson Orin Nano Super (67 TOPS, 8GB LPDDR5) |
| Real-Time I/O | Arduino Mega 2560 (ATmega2560, 16 MHz) |
| Offload Compute | Laptop (any ROS2 node, launch-time selectable) |
| Phone | Docked Android/iOS, PWA face + GPS + mic + speaker + hotspot |
| Battery | 4S LiPo 4000mAh 60C, 14.8V nominal (59.2 Wh) |
| Charging | Removable (external balance charger) + onboard XT60 charge port |
| Typical Runtime | ~2.8 hours (21W typical draw) |
| Storage | 1TB NVMe SSD (PCIe Gen3 x4) - OS, SLAM maps, bag recordings, models |
| Networking | Jetson Dev Kit WiFi/BT module (M.2 Key-E), phone hotspot fallback |
| Software | ROS2 Humble, Nav2, SLAM Toolbox, RTAB-Map, TensorRT |
| LLM | Provider-agnostic (OpenAI / Claude / Gemini / local) |
| License | MIT |

---

## Quick Start

### Prerequisites

- NVIDIA Jetson Orin Nano Super Developer Kit with JetPack 6.x installed on NVMe SSD
- Docker installed on the Jetson (`nvidia-docker2` runtime configured)
- 4S LiPo battery charged (>14.0V)
- ZED Mini and Unitree L1 4D LiDAR connected via USB
- Arduino Mega flashed with ALBERC firmware and connected via USB

### Docker - Jetson (ARM64)

```bash
# Clone the repository
git clone https://github.com/<org>/alberc.git
cd alberc

# Build the ARM64 container (includes JetPack, ZED SDK, L1 SDK, ROS2 Humble)
docker compose -f docker/docker-compose.jetson.yml build

# Launch all nodes
docker compose -f docker/docker-compose.jetson.yml up
```

The container mounts `/dev` for USB device access and uses the NVIDIA runtime for GPU acceleration. All sensor SDKs, ROS2 nodes, and the web dashboard start automatically.

### Docker - x86 Development

```bash
# Build the x86 development container (simulation mode, no physical sensors)
docker compose -f docker/docker-compose.dev.yml build

# Launch with simulated sensors (Gazebo)
docker compose -f docker/docker-compose.dev.yml up
```

The x86 container replaces physical sensor drivers with Gazebo simulation plugins. The full ROS2 node graph, dashboard, and LLM stack run identically.

### Native Install

```bash
# Install ROS2 Humble (Ubuntu 22.04)
# See: https://docs.ros.org/en/humble/Installation.html

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source
source install/setup.bash
```

### First Run Verification

```bash
# Terminal 1: Launch the full system
ros2 launch alberc_bringup alberc.launch.py

# Terminal 2: Verify all nodes are running
ros2 node list | wc -l  # expect 20+

# Terminal 3: Check sensor topics
ros2 topic hz /odom           # expect ~50 Hz
ros2 topic hz /cloud           # expect ~11 Hz
ros2 topic hz /zed/rgb/image   # expect ~30 Hz

# Open browser dashboard
# Navigate to http://<jetson-ip>:80
```

---

## Architecture

### System Data Flow

```
                            PHYSICAL WORLD
                                 |
      +----------+-----------+---+----+----------+----------+----------+
      v          v           v        v          v          v          v
  +-------+ +--------+ +--------++-------+ +--------+ +-------+ +-------+
  |ZED    | |Unitree | |HC-SR04 ||TS-25  | |ICM-   | |Phone  | |Laptop |
  |Mini   | |L1 4D   | |  x4    ||GA370  | |20948  | |(dock) | |(opt.) |
  |USB3->J| |USB+12V | |ARD GPIO||ARD ISR| |ARD I2C| |WiFi->J| |ROS2   |
  +---+---+ +---+----+ +---+---++---+---+ +--+---+ +---+---+ +---+---+
      |         |          |        |        |         |         |
  +===+=========+===== ====+=== ====+========+=== =====+=========+======+
  ||  DRIVER LAYER                                                     ||
  ||  Jetson: zed_node (RGB, depth, VIO, IMU, SDK segmentation)       ||
  ||          unitree_lidar_node (3D cloud, IMU, SDK voxel segment.)  ||
  ||  Arduino->rosserial: odom_raw, imu, ultrasonic/*                 ||
  ||  Phone: GPS, mic audio, hotspot status                           ||
  +====================================+================================+
                                       |
  +====================================+================================+
  ||  PERCEPTION LAYER                                                 ||
  ||  EKF: encoders + ICM-20948 + ZED VIO -> /odom                    ||
  ||  SDK-Native Segmentation:                                         ||
  ||    L1 SDK -> voxel/point cloud segmentation (always-on)           ||
  ||    ZED SDK -> object segmentation with depth + sRGB (always-on)   ||
  ||  Custom TensorRT Models: YOLO/MobileNet layered on SDK baseline   ||
  ||  SLAM Toolbox: L1->height filter->2D scan -> /map                ||
  ||  RTAB-Map: L1 + ZED depth/RGB -> 3D colored map                  ||
  ||  Object tracker: L1 3D diff + detections -> Kalman tracks         ||
  ||  Three Views: Photorealistic | Voxel/PointCloud | Compound+VLM   ||
  ||  -> COSTMAP (static + 3D projection + inflation + dynamic)        ||
  +====================================+================================+
                                       |
  +====================================+================================+
  ||  PLANNING + EVASION LAYER                                         ||
  ||  Global Planner (A* | RRT* | Voronoi GVD) -> MPPI (20Hz) ->      ||
  ||    ARBITRATOR -> cmd_vel                                          ||
  ||                   ^                                               ||
  ||  REACTIVE EVASION (20Hz) -+                                       ||
  ||  +-- TTC per track vs /task/interaction_targets                   ||
  ||  +-- Non-targets: monitor->inject->escape->stop                   ||
  ||  +-- Targets: follow / approach / maintain-distance               ||
  ||  +-- Ultrasonic <0.15m: unconditional stop                        ||
  +====================================+================================+
                                       |
  +====================================+================================+
  ||  INTELLIGENCE LAYER                                               ||
  ||  Voice: Phone mic -> Whisper STT -> LLM -> Piper TTS             ||
  ||  LLM (OpenAI / Claude / Gemini / Local):                         ||
  ||    task decomposition -> Nav2 actions + interaction_targets        ||
  ||    scene QA from ZED RGB + 3D map context                         ||
  ||    semantic map annotation                                        ||
  ||  VLM (Gemini): compound view spatial context labels               ||
  +====================================+================================+
                                       |
  +====================================+================================+
  ||  INTERFACE LAYER                                                  ||
  ||  Phone face (PWA: animated avatar + status + chat transcription)  ||
  ||  Browser dashboard (rosbridge:9090 + Nginx:80, mobile-responsive) ||
  ||  VR/WebXR stream (Draco + WebSocket, Meta Quest target)           ||
  ||  Map manager (save/load/export PLY/OBJ/glTF/USD/PCD/FBX)          ||
  ||  Export manager (environment-specific ZIP bundles for sim/viz)    ||
  +====================================+================================+
                                       |
                    Arduino -> L298N x2 -> Motors -> Wheels
```

### Three-Tier Hardware

| Tier | Name | Location | Components |
|---|---|---|---|
| L1 | Drivetrain | Bottom | TS-25GA370 motors x2, ball caster, L298N x2, 4S LiPo 4000mAh, buck converters, safety switches |
| L2 | Electronics | Middle | Jetson Orin Nano Super, Arduino Mega, ICM-20948 (centered), HC-SR04 x4 (flush-mounted), ventilation slots |
| L3 | Perception | Top | Unitree L1 4D LiDAR (15 deg forward incline, full hemispherical clearance), ZED Mini (forward-facing, rigid mount), phone dock |

### Compute Architecture

| Node | Processor | Role |
|---|---|---|
| Jetson Orin Nano Super | 67 TOPS Ampere GPU, 6-core Cortex-A78AE | Primary compute: perception, SLAM, planning, LLM, web server |
| Arduino Mega 2560 | ATmega2560 16 MHz | Real-time I/O: motor PWM, encoder ISR, IMU I2C, ultrasonic GPIO |
| Docked Phone | ARM SoC (Android/iOS) | GPS, microphone, speaker, WiFi hotspot, Bluetooth, PWA face display |
| Laptop (optional) | Any x86/ARM with ROS2 | Offload compute: any ROS2 node relocatable at launch time |

### Communication Topology

| Link | Method | Status |
|---|---|---|
| Jetson <-> Arduino | USB serial 115200 baud (rosserial) | Configured |
| Jetson <-> L1 4D LiDAR | USB data + 12V barrel (manufacturer splitter) | SDK configured |
| Jetson <-> ZED Mini | USB 3.0 | SDK configured |
| Jetson <-> Phone | WiFi, WebSocket port 9090 | -- |
| Jetson <-> Browser | WiFi/Ethernet, rosbridge 9090, Nginx 80 | -- |
| Jetson <-> VR | WiFi, WebSocket (WebXR) or TCP (Unity bridge) | -- |
| Jetson <-> Laptop | WiFi/Ethernet, ROS2 DDS multicast | -- |
| Phone -> GPS | Phone OS location services -> WebSocket -> ROS2 | -- |
| Phone -> Audio | Mic -> WebSocket 16 kHz -> Jetson; Jetson -> WebSocket -> speaker | -- |

### ROS2 Node Graph

| Node | Package | Rate | Function |
|---|---|---|---|
| zed_node | zed_ros2_wrapper | 30 Hz | Stereo RGB + depth + VIO + 800 Hz IMU + SDK object segmentation |
| unitree_lidar_node | unitree_lidar_ros2 | 11 Hz | 3D cloud 21,600 pts/sec + 250 Hz IMU + SDK voxel segmentation |
| arduino_bridge | rosserial | 50 Hz | USB serial -> ROS2 topics (configured) |
| robot_localization | robot_localization | 50 Hz | EKF: encoders + ICM-20948 + ZED VIO -> /odom |
| slam_toolbox_node | slam_toolbox | 5 Hz | 2D graph SLAM -> /map (OccupancyGrid) |
| rtabmap_node | rtabmap_ros | 1 Hz | 3D visual-LiDAR SLAM -> colored point cloud |
| detection_node | alberc_perception | 15+ Hz | TensorRT YOLO/MobileNet on ZED RGB -> detections |
| object_tracker | alberc_perception | 10 Hz | Multi-object 3D Kalman fusion (L1 + detections) |
| view_compositor | alberc_perception | 10 Hz | Composites three sensor fusion views from SDK + custom pipelines |
| nav2_planner | nav2 | 1 Hz | Global path (A* / RRT* / Voronoi GVD, runtime-selectable) |
| nav2_controller | nav2 | 20 Hz | MPPI local trajectory -> /cmd_vel |
| nav2_bt_navigator | nav2 | Event | Behavior tree orchestration |
| costmap_2d | nav2 | 5 Hz | Global + local costmaps (L1 3D projection + inflation) |
| motor_mpc_node | alberc_motor | 50 Hz | Optional MPC wheel controller (Jetson-side, replaces Arduino PID) |
| reactive_evasion | alberc_nav | 20 Hz | TTC assessment + evasion override (primary authority) |
| llm_planner_node | alberc_ai | Event | Provider-agnostic LLM: task decomposition + scene QA |
| vlm_context_node | alberc_ai | 1 Hz | Gemini VLM: compound view spatial context labels |
| stt_node | alberc_voice | Event | Whisper STT (TensorRT on Jetson GPU) |
| tts_node | alberc_voice | Event | Piper TTS -> phone speaker |
| audio_bridge_node | alberc_phone | 16 kHz | Phone mic/speaker bidirectional WebSocket |
| phone_face_node | alberc_phone | 10 Hz | PWA animated face state + status + chat transcription |
| gps_bridge_node | alberc_phone | 1 Hz | Phone GPS -> /gps/fix (sensor_msgs/NavSatFix) |
| web_dashboard_node | alberc_ui | 30 Hz | React + rosbridge: telemetry, control, views, LLM chat, planner |
| mobile_ui_node | alberc_ui | 30 Hz | Responsive touch-optimized mobile dashboard variant |
| vr_stream_node | alberc_vr | 30 Hz | WebXR + Draco-compressed cloud streaming (Meta Quest target) |
| map_manager_node | alberc_maps | Event | Save/load/export/floor management |
| export_manager_node | alberc_export | Event | Format conversion, mesh reconstruction, ZIP packaging, export profile management |
| project_manager_node | alberc_projects | Event | Bot projects: deploy missions, packages, notebooks, custom code |

**Arduino Topics Published** (via rosserial, configured):

| Topic | Type | Rate | Source |
|---|---|---|---|
| /odom_raw | nav_msgs/Odometry | 50 Hz | TS-25GA370 encoder ISR -> differential kinematics |
| /imu/arduino | sensor_msgs/Imu | 100 Hz | ICM-20948 accel + gyro (I2C D20/D21) |
| /ultrasonic/{front,rear,left,right} | sensor_msgs/Range | 10 Hz | HC-SR04 staggered (D22-D29) |
| /motor/health | alberc_msgs/MotorHealth | 10 Hz | Per-motor RPM, PWM, current est., stall/fault flags |
| /motor/state | alberc_msgs/MotorState | 50 Hz | Per-motor commanded vs actual velocity, PID error |

**Arduino Subscribes:** /cmd_vel (PID mode) or /motor/cmd_pwm (MPC mode) -> L298N PWM + direction (D4-D9).

**Arduino Mega Pin Allocation:**

| Function | Pin(s) | Notes |
|---|---|---|
| IMU SDA | D20 | ICM-20948 I2C data (3.3V) |
| IMU SCL | D21 | ICM-20948 I2C clock (3.3V) |
| Front Ultrasonic TRIG / ECHO | D22 / D23 | HC-SR04 |
| Rear Ultrasonic TRIG / ECHO | D24 / D25 | HC-SR04 |
| Left Ultrasonic TRIG / ECHO | D26 / D27 | HC-SR04 |
| Right Ultrasonic TRIG / ECHO | D28 / D29 | HC-SR04 |
| Left Motor IN1 / IN2 / PWM | D4 / D5 / D6 | L298N #1 |
| Right Motor IN1 / IN2 / PWM | D7 / D8 / D9 | L298N #2 |
| Left Encoder A / B | D2 / D3 | ISR-capable pins |
| Right Encoder A / B | D18 / D19 | ISR-capable pins |
| Battery Voltage | A0 | Voltage divider (4S -> 0-5V range) |

### Motor Control and Odometry Pipeline

This section describes the full path from `/cmd_vel` to wheel motion and back to `/odom_raw` - the lowest layer of the navigation stack.

**Differential Drive Kinematics:**

Forward (cmd_vel to wheel velocities):
```
v_left  = v - omega * (track_width / 2)
v_right = v + omega * (track_width / 2)
```

where `v` is linear velocity (m/s), `omega` is angular velocity (rad/s), and `track_width` is the distance between wheel contact points (~0.18m).

Inverse (encoder ticks to odometry):
```
v_left  = (delta_ticks_left  / ticks_per_meter) / dt
v_right = (delta_ticks_right / ticks_per_meter) / dt
v       = (v_right + v_left) / 2
omega   = (v_right - v_left) / track_width
x      += v * cos(theta) * dt
y      += v * sin(theta) * dt
theta  += omega * dt
```

where `ticks_per_meter = 540 PPR / (pi * wheel_diameter)`. Published as `/odom_raw` (nav_msgs/Odometry) at 50 Hz. `wheel_diameter` and `track_width` are configurable parameters editable from the browser dashboard (Configuration panel) and stored in YAML - they must be recalibrated whenever wheels are changed.

**Low-Level Controller (Two Modes, Runtime-Selectable via Dashboard or ROS2 Parameter):**

*PID (Default - runs on Arduino at 50 Hz):*
Per-wheel PID velocity controller. Each wheel maintains an independent control loop:
- **Input:** target wheel velocity from differential drive kinematics applied to incoming `/cmd_vel`
- **Feedback:** encoder-derived wheel velocity (ISR-counted ticks / dt)
- **Output:** PWM duty cycle (0-255) + direction pins to L298N
- **Tuning:** Kp, Ki, Kd configurable per-wheel via ROS2 parameters (stored in YAML, pushed to Arduino over serial). Anti-windup on integrator (clamped to +/-255). Derivative term low-pass filtered (alpha = 0.1) to suppress encoder quantization noise.
- **Tradeoff:** Simple, robust, runs entirely on the Arduino with no Jetson dependency. Adequate for steady-state velocity tracking. Less precise during rapid dynamic transitions (fast start/stop, sharp turns) where the first-order PID model cannot anticipate motor response lag.

*MPC (Experimental - runs on Jetson at 50 Hz, `motor_mpc_node`):*
A lightweight Model Predictive Controller running as a ROS2 node on the Jetson, publishing target PWM values directly to the Arduino via `/motor/cmd_pwm` (bypassing the Arduino's PID loop).
- **State:** [v_left, v_right, theta_dot] from encoder feedback + IMU angular velocity
- **Model:** First-order DC motor model per wheel with identified time constant (tau) and steady-state gain (K), obtained via step-response system identification during calibration. The model captures L298N voltage drop (~2.4V) and per-motor asymmetry.
- **Horizon:** 10 steps at 20ms (200ms lookahead)
- **Constraints:** PWM limits (0-255), acceleration limits from motion envelope, voltage sag compensation (PWM adjusted based on battery voltage reported by Arduino)
- **Cost:** Velocity tracking error + control effort smoothness + left-right symmetry penalty (reduces odometric drift from motor mismatch)
- **Solver:** Quadratic program (QP), solvable in <1ms on Jetson ARM cores via OSQP
- **Tradeoff:** Better trajectory tracking during dynamic maneuvers - predicts and pre-compensates for motor lag, L298N voltage drop, and wheel asymmetry. Requires Jetson to be healthy (falls back to Arduino PID automatically if MPC node dies or latency exceeds 25ms).

**Arduino Firmware Architecture:**

The Arduino main loop runs at 200 Hz (5ms period) with time-sliced tasks:

| Interval | Task |
|---|---|
| Every 5ms | Read encoder ISR buffers, compute instantaneous wheel velocities |
| Every 10ms | Read IMU over I2C (ICM-20948 accel + gyro) |
| Every 20ms | Run PID controller (if PID mode), apply PWM + direction to L298N |
| Every 20ms | Accept /motor/cmd_pwm from Jetson (if MPC mode), apply directly to L298N |
| Every 20ms | Publish /odom_raw, /motor/state |
| Every 10ms | Publish /imu/arduino |
| Every 100ms | Read one ultrasonic sensor (round-robin, one per cycle to avoid cross-talk) |
| Every 100ms | Run motor health diagnostics, publish /motor/health |

Serial protocol: rosserial binary-packed messages over USB at 115200 baud. Each message includes header, message type ID, payload length, payload, and checksum.

**Motor Health Monitoring:**

The Arduino continuously monitors motor health and publishes `/motor/health` (alberc_msgs/MotorHealth) at 10 Hz:

| Metric | Method | Normal Range | Alarm Condition |
|---|---|---|---|
| Per-motor RPM | Encoder tick delta / dt | 0-130 RPM | RPM = 0 when PWM > 0 for > 500ms → **stall** |
| PWM duty cycle | Current command value | 0-255 | Sustained > 230 (~90%) without reaching target velocity → **saturation** |
| Commanded vs actual velocity | PID setpoint minus encoder velocity | < 10% error | Persistent error > 20% of setpoint for > 1s → **tracking fault** |
| Motor current (estimated) | Calibrated lookup: PWM duty x battery voltage / L298N resistance model | < 0.5A typical | > 0.8A sustained → **overcurrent warning** |
| L298N thermal proxy | Cumulative (duty_cycle^2 x duration) with exponential decay | Below threshold | Accumulated thermal load exceeds safe operating limit → **thermal warning** |
| Encoder health | Tick rate change between consecutive readings | Nonzero when moving | Zero ticks for > 200ms while PWM > 0 → **encoder fault** (wire break, sensor failure) |
| Battery voltage | Analog read from voltage divider on battery (Arduino A0) | 13.2-16.8V | < 13.2V → **low battery**, < 12.8V → **critical, shutdown motors** |

All alarm conditions are published on `/motor/health` with per-motor fault flags and are displayed on the browser dashboard and phone status bar.

**Motor Safety:**

| Protection | Trigger | Action | Recovery |
|---|---|---|---|
| Watchdog | No valid /cmd_vel or /motor/cmd_pwm for 500ms | Both motors zero PWM | Automatic on next valid command |
| Stall protection | Stall detected (PWM > 50, RPM = 0) for > 1s | Faulted motor disabled for 3s cooldown | Automatic after cooldown; event logged to /motor/health |
| Current limiting | Estimated current > 0.8A per motor | PWM capped to keep current below limit | Automatic; cap released when load decreases |
| Overcurrent emergency | Both motors > 0.6A simultaneously for > 2s | Both motors disabled | Requires explicit re-enable via /cmd_vel or dashboard button |
| Low battery cutoff | Battery voltage < 13.2V (3.3V/cell) | Motors disabled, warning published | Re-enable only after voltage rises above 13.5V (hysteresis) |
| Critical battery | Battery voltage < 12.8V | All motors disabled, Jetson notified for graceful shutdown | Manual recharge required |
| MPC fallback | MPC node latency > 25ms or node crash | Automatic switch to Arduino PID mode | MPC can be re-enabled via parameter or dashboard |
| SW2 hardware kill | Physical toggle switch | 12V rail disconnected from L298N boards | Manual switch re-engagement |

### TF2 Frame Tree and URDF

The robot's kinematic model is defined in a URDF/xacro file (`alberc.urdf.xacro`) that parameterizes all physical dimensions - wheel diameter, track width, sensor mount positions - as xacro arguments. These values are loaded from YAML configuration at launch time and are editable from the dashboard Configuration panel, enabling recalibration without code changes when hardware is modified.

```
map --(slam_toolbox)--> odom --(EKF)--> base_link
                                            |
                         +------------------+------------------+-----------+
                         v                  v                  v           v
                   lidar_link          camera_link         imu_link   phone_link
                   (L3, 15 deg pitch)  (L3, ZED Mini)      (L2)      (L3, dock)
                                                                         |
                                            +----------------------------+
                                            v
                                      left_wheel_link / right_wheel_link / caster_link
                                      (L1, continuous joints)
```

| Frame | Parent | Transform | Source |
|---|---|---|---|
| map -> odom | map | SLAM correction | slam_toolbox (2D) |
| odom -> base_link | odom | Fused odometry | robot_localization EKF |
| base_link -> lidar_link | base_link | Static (x, y, z, roll, pitch=15 deg, yaw) | URDF |
| base_link -> camera_link | base_link | Static (forward-facing, L3 height) | URDF |
| base_link -> imu_link | base_link | Static (centered, L2 height) | URDF |
| base_link -> phone_link | base_link | Static (L3, dock position) | URDF |
| base_link -> left_wheel_link | base_link | Continuous rotation joint | URDF + encoder state |
| base_link -> right_wheel_link | base_link | Continuous rotation joint | URDF + encoder state |

The URDF includes collision geometry (simplified box/cylinder approximations of each tier) for Gazebo/Isaac Sim simulation and visualization in RViz2.

### Boot Sequence

1. Power on. Buck converters stabilize. Jetson boots from NVMe SSD.
2. Jetson USB powers Arduino. Rosserial starts. Motors, IMU, ultrasonics, encoders initialize.
3. ROS2 launch. L1 and ZED drivers start (SDKs pre-configured). Point cloud, stereo streams, and SDK segmentation begin.
4. SLAM Toolbox loads last saved 2D map. RTAB-Map initializes 3D reconstruction.
5. Nav2 initializes with configured planner (A* default). Costmaps build from L1 3D projection + live sensors.
6. LLM loads (local model to GPU, or cloud endpoint configured). STT/TTS models load. Audio bridge connects to phone.
7. Phone PWA launches: animated face, status bar, chat transcription active.
8. Dashboard available on port 80. ALBERC announces "Ready" via phone speaker.
9. Reactive evasion active. Object tracker running. System operational.

---

## Sensor Fusion and Perception

### Extended Kalman Filter

The `robot_localization` package implements an Extended Kalman Filter that fuses three asynchronous, heterogeneous odometry sources into a single canonical pose estimate. Wheel encoder odometry (50 Hz, subject to slip and backlash) provides high-rate dead-reckoning. The ICM-20948 gyroscope (100 Hz) provides angular velocity free of wheel contact assumptions. The ZED Mini visual-inertial odometry (30 Hz, +/-1mm per the ZED SDK's tightly-coupled visual-inertial fusion of 800 Hz IMU + stereo features) provides drift-bounded position. The EKF prediction step propagates the state using the motion model; the update step fuses each sensor observation weighted by its noise covariance. The result is an odometry estimate that degrades gracefully - if visual features are lost (featureless corridor), encoder + gyro maintain the estimate; if wheels slip (smooth floor), VIO corrects; if all visual tracking fails momentarily, the IMU bridges the gap.

**EKF Sensor Configuration:**

| Source | Axes Fused | Rate | Covariance Notes |
|---|---|---|---|
| Wheel encoders (/odom_raw) | x, y, yaw | 50 Hz | High covariance on slip-prone surfaces. Primary dead-reckoning source. |
| ICM-20948 (/imu/arduino) | yaw_rate (gyro), roll, pitch (accel) | 100 Hz | Gyro bias estimated online. Magnetometer excluded. |
| ZED Mini VIO (/zed/odom) | x, y, z, roll, pitch, yaw | 30 Hz | Lowest covariance when feature-rich. Covariance inflated automatically by ZED SDK when tracking confidence drops. |

The EKF `robot_localization` configuration specifies per-sensor which state variables to fuse (the `odomN_config` and `imuN_config` matrices), preventing double-counting. Wheel encoders contribute planar motion (x, y, yaw) only. The ICM-20948 contributes angular rates and tilt. The ZED VIO contributes all six degrees - but its covariance is dynamically scaled by the ZED SDK's internal tracking confidence metric, so in featureless environments its contribution is automatically downweighted.

Three independent IMU streams exist in the system: the ICM-20948 (Arduino I2C, enters EKF directly), the ZED Mini onboard IMU (800 Hz, consumed internally by ZED SDK for VIO - its output enters the EKF as the refined VIO pose, not raw IMU), and the Unitree L1 onboard IMU (250 Hz, consumed internally by the LiDAR SDK for point cloud motion compensation - never enters the EKF directly). This architecture avoids double-counting inertial measurements while leveraging each IMU where it is most effective.

### SDK-Native Segmentation

ALBERC employs a two-tier segmentation architecture that provides robust, always-on environmental understanding without relying solely on custom-trained models.

**Tier 1 - SDK-Native (Always-On Baseline):**

- **L1 SDK Voxel/Point Cloud Segmentation:** The Unitree L1 SDK provides native voxel-based segmentation of the 360 deg x 90 deg hemispherical point cloud. The SDK classifies points into ground plane, static structure, and dynamic clusters using geometric heuristics optimized for the L1's scan pattern. This segmentation runs at the SDK's native 11 Hz rate with zero additional GPU cost, producing labeled point clouds that feed directly into the costmap and the Voxel/Point Cloud sensor view.

- **ZED SDK Object Segmentation:** The ZED SDK provides neural-network-based object detection and instance segmentation at up to 30 Hz on the Jetson GPU. Each detected object includes a 2D mask, a 3D bounding box derived from calibrated stereo depth, the sRGB texture of the segmented region, and a semantic class label (person, vehicle, animal, furniture, etc.). This SDK-native segmentation provides depth-registered, color-accurate object boundaries without requiring custom model training, and feeds both the Photorealistic sensor view and the object tracker.

**Tier 2 - Custom TensorRT Models (Layered On Top):**

Custom models (YOLOv8-nano, MobileNet-SSD v2, or domain-specific networks) are deployed as TensorRT engine files compiled for the Jetson's SM 8.7 architecture. These models ingest ZED Mini RGB frames and produce application-specific detections (e.g., household items, specific pet breeds, or custom object classes) that augment the SDK-native baseline. The two-tier architecture ensures that perception never falls to zero - even if a custom model fails, OOMs, or is not yet trained, the SDK-native segmentation continues to provide obstacle awareness and basic object classification.

### Three Sensor Fusion Views

The `view_compositor` node generates three simultaneous sensor fusion views, each optimized for a different purpose. All three are available concurrently via the browser dashboard, exportable as individual point cloud data layers (see [Export & File Management](#export--file-management) for per-view export details), and streamable to VR/WebXR.

**View 1 - Photorealistic:**
ZED Mini sRGB stereo imagery composited with ZED SDK segmentation overlays. Object masks are rendered as semi-transparent colored regions over the live camera feed, with depth-derived 3D bounding boxes and class labels. This view provides the most human-interpretable representation of the robot's immediate environment and is the default dashboard camera view.

**View 2 - Voxel / Point Cloud:**
L1 SDK-native voxel segmentation rendered as a colored 3D point cloud. Ground plane, static structure, and dynamic clusters are color-coded by segmentation class. The full 360 deg x 90 deg hemisphere is visualized in a Three.js renderer with orbit controls. This view provides the most complete geometric spatial understanding and is used for navigation debugging, map inspection, and VR export.

**View 3 - Compound (All Sensors + VLM):**
All sensor streams (L1 3D cloud, ZED depth + RGB, ultrasonic ranges, SDK segmentation outputs, and custom TensorRT detections) are fused into a unified representation. The `vlm_context_node` periodically sends a composite frame (ZED RGB + 3D map crop + tracked object list) to Gemini VLM, which returns spatial context labels ("kitchen counter with coffee maker", "hallway leading to bedroom", "person sitting on couch"). These labels are anchored to map coordinates and rendered as annotated overlays in the compound view. This view is used for LLM scene understanding, semantic mapping, and human-readable spatial reports.

### Perception Fusion

The Unitree L1 4D LiDAR provides geometric 3D structure (360 deg x 90 deg hemispherical point cloud, 21,600 pts/sec, +/-2cm accuracy). The ZED Mini provides dense stereo depth (0.1-15m, <1% error at 2m) and RGB texture. These are fused in RTAB-Map's multi-session visual-lidar SLAM for colored 3D reconstruction, and in the costmap layer where the L1's height-filtered 3D projection is combined with ZED depth and ultrasonic proximity data to produce a comprehensive obstacle field.

### SLAM

ALBERC implements dual SLAM:

**2D Graph-Based SLAM (SLAM Toolbox):** Operates on virtual 2D LaserScans produced by height-filtering the L1's 3D point cloud to a traversal-relevant band (0.10-1.50m). Incoming scans are matched against the local submap via correlative scan matching (CSM) - a brute-force search over a discretized (x, y, theta) space that maximizes scan-to-map correlation - followed by Ceres Solver nonlinear refinement of the matched pose. Each accepted scan becomes a node in a pose graph G = (V, E) where V are robot poses and E are relative-pose constraints from scan matching. Loop closures add long-range constraints when revisiting mapped areas (detected by searching for nodes within a spatial radius and re-running scan matching). The full graph is optimized via sparse Cholesky decomposition of the information matrix, globally correcting all poses. The output is a 0.05m/cell occupancy grid - the canonical navigation map.

**3D Visual-LiDAR SLAM (RTAB-Map):** Real-Time Appearance-Based Mapping fuses L1 3D point clouds with ZED Mini stereo depth and RGB. RTAB-Map maintains a memory management system: recent nodes are kept in working memory (WM), older nodes are transferred to long-term memory (LTM) based on a weight function inversely proportional to observation frequency. Loop closure detection uses bag-of-words on SURF/ORB visual features extracted from ZED RGB frames, making it robust in geometrically symmetric environments where LiDAR-only matching is ambiguous. The output is a dense, colored 3D point cloud registered to a globally consistent pose graph - used for LLM scene understanding, VR visualization, and file export.

**Global GPS-Referenced Mapping:**

ALBERC treats mapping as a continuous, location-independent process. The phone's GPS provides a coarse global anchor: when a SLAM session begins, the initial robot pose is tagged with the phone's GPS coordinates (latitude, longitude, altitude). All subsequent map data inherits this geo-reference. When the robot is powered on at a previously mapped location, it attempts to localize against the existing map database (RTAB-Map loop closure on visual features). If localization succeeds, the session continues the existing map. If no match is found, a new map region is initialized with a fresh GPS anchor.

This enables:
- **Cross-session continuity:** Return to a mapped apartment weeks later and resume navigation without remapping
- **Multi-location maps:** Maps from different buildings are stored in the same database, indexed by GPS coordinates. The dashboard's map manager displays a world view with all mapped regions on a map tile layer.
- **GPS-aided relocalization:** When visual relocalization is ambiguous (similar-looking rooms in different buildings), the GPS position disambiguates by constraining the search to nearby map regions

GPS accuracy (3-10m outdoor, 10-30m indoor via WiFi) is insufficient for navigation but adequate for map region selection and coarse global positioning.

**Multi-Floor Detection and Mapping:**

ALBERC automatically detects floor transitions and manages multi-floor maps:

- **Elevation sensing:** The phone's barometric altimeter (available on most modern smartphones, ~0.1m resolution) tracks relative altitude changes. A sustained altitude change > 2.0m triggers a floor transition event.
- **SLAM Z-axis tracking:** RTAB-Map's 3D pose graph tracks the robot's Z-coordinate. Large Z displacements corroborate barometric readings.
- **GPS altitude:** Phone GPS altitude provides a coarse absolute reference, used to assign floor numbers relative to a building's ground level (calibrated on first visit).
- **Transition detection:** The `map_manager_node` monitors all three elevation signals. When a floor transition is confirmed (barometer + SLAM Z agree, sustained for > 5s), the current SLAM session is tagged with the new floor index, and a new 2D occupancy grid layer is initialized for the new floor. The 3D map (RTAB-Map) remains continuous across floors.

The dashboard displays floor maps as a stacked layer selector. The robot automatically loads the correct floor's 2D navigation map based on current estimated elevation.

**Map Storage and Export:**

| Operation | Mechanism | Trigger |
|---|---|---|
| Save 2D | SLAM Toolbox serialize_map (per-floor) | Every 5 min + shutdown + floor transition |
| Load 2D | deserialize_map on boot | Automatic (GPS-matched region + floor) |
| Save 3D | RTAB-Map database | Continuous incremental |
| GPS anchor | map_manager GPS tagging | On session start + relocalization |
| Floor index | Barometer + SLAM Z + GPS altitude | Automatic on floor transition |
| Export PLY/OBJ/glTF | map_manager + Open3D | Dashboard button / voice / ROS2 service |
| Export USD/USDA | export_manager + usd-core | Dashboard Export Wizard / ROS2 service |
| Export PCD | export_manager + Open3D | Dashboard Export Wizard / ROS2 service |
| Export SDF world | export_manager + template XML | Dashboard Export Wizard / ROS2 service |
| Export ZIP bundle | export_manager (full pipeline) | Dashboard Export Wizard / voice / ROS2 service |
| Export trajectory | export_manager (SLAM keyframe poses) | Included in ZIP bundle (TUM format) |
| Stream 3D to VR | Draco-compressed WebSocket | Continuous 1-5 Hz |
| Stream 2D to browser | PNG grid + metadata + floor selector | Continuous 1 Hz |
| Semantic labels | LLM/VLM annotation -> YAML overlay | On VLM classification |

All map data is stored on the 1TB NVMe SSD. Storage is managed with configurable retention policies (e.g., keep last 30 days of bag recordings, unlimited map storage).

### Custom Model Deployment

Object detection runs on the Jetson's Ampere GPU via TensorRT-optimized inference. The pipeline ingests ZED Mini RGB frames at 1080p/30fps and runs a single-shot detector (MobileNet-SSD v2 or YOLOv8-nano, selected for the latency-accuracy tradeoff on the Jetson's 1024 CUDA cores + 32 Tensor Cores) to produce 2D bounding boxes with class labels (person, cat, dog, common household objects) and confidence scores at 15-30 FPS. Each detection is projected into 3D using the ZED SDK's calibrated depth map, yielding a 3D centroid (x, y, z) in the camera frame, which is then transformed to the robot's base_link frame via the TF2 tree. This 3D semantic detection is the input to the object tracker's vision branch.

The detection model is deployed as a TensorRT engine file (.engine) compiled for the Jetson's specific GPU architecture (SM 8.7), eliminating Python overhead and achieving deterministic inference latency. Model selection is a design variable: MobileNet-SSD v2 provides lower latency (~15ms) with adequate accuracy for indoor household objects; YOLOv8-nano provides higher mAP at ~30ms. Both fit within the GPU memory budget alongside ZED SDK processing and SDK-native segmentation.

### Multi-Object Tracking

The `object_tracker` node implements a multi-hypothesis tracking pipeline:

**Detection sources:** (1) LiDAR scan differencing - consecutive L1 3D clouds are subtracted to isolate non-static points, which are clustered via Euclidean distance (DBSCAN-like, min 5 points, max 0.3m inter-point) into 3D object candidates; (2) Vision-based detections - TensorRT NN detections projected to 3D via ZED depth; (3) SDK-native segmentation - ZED SDK object instances and L1 SDK dynamic clusters provide additional detection candidates with pre-computed class labels.

**Data association:** LiDAR clusters, vision detections, and SDK segmentation outputs are associated per-frame using the Hungarian algorithm on a 3D Euclidean cost matrix with a 0.5m gating threshold. Unmatched LiDAR clusters become class-unknown tracks; unmatched vision detections initialize new tracks with semantic class.

**State estimation:** Each confirmed track maintains a per-object Kalman filter with state [x, y, vx, vy] and constant-velocity motion model. The filter predicts between observations and updates on association, producing smoothed position and velocity estimates. Track lifecycle: tentative (0 confidence, < 3 associations), confirmed (>= 3 consecutive), lost (>= 5 consecutive misses), deleted (>= 10 misses).

**Output:** A TrackedObjectArray message at 10 Hz: per-track ID, 3D position, velocity vector, semantic class, bounding radius, confidence, and evasion status (evading / interaction target / monitoring).

---

## Path Planning and Navigation

### Three Planning Modes

ALBERC provides three global path planning algorithms, selectable at runtime via the browser dashboard, ROS2 parameter, or LLM command. All three operate on the same inflated 2D costmap and produce paths consumed by the MPPI local controller.

**A* (State Lattice - Default):**
SmacPlanner2D implements a State Lattice A* search on the 2D costmap. The planner discretizes the configuration space into a lattice of (x, y, theta) states connected by kinematically feasible motion primitives for ALBERC's differential-drive kinematics. The cost function combines path length, proximity to obstacles (via the inflation layer's exponential decay function), and heading changes. The result is a globally optimal kinematically feasible path from the current pose to the goal. Best for structured indoor environments with well-defined corridors.

**RRT* (Sampling-Based):**
A Rapidly-exploring Random Tree Star planner implemented as a Nav2 planner plugin. RRT* incrementally builds a tree of kinematically feasible paths by random sampling in the costmap space, with a rewiring step that guarantees asymptotic optimality. The tree is biased toward the goal with configurable bias probability (default 0.05). RRT* excels in cluttered environments with many obstacles where A*'s lattice resolution would require prohibitively fine discretization, and in open spaces where the sampling approach quickly finds near-optimal paths. Configurable parameters: max iterations (5000), step size (0.2m), goal tolerance (0.15m), rewire radius (1.0m).

**Voronoi GVD (Exploration-Optimized):**
A Generalized Voronoi Diagram planner that computes the medial axis of the free space in the occupancy grid - the set of points equidistant from the two nearest obstacles. Navigation along the GVD maximizes clearance from all obstacles, producing paths that naturally follow hallway centers and room midlines. The GVD is computed via distance transform on the costmap and updated incrementally as the map changes. This planner is selected automatically during frontier-based exploration missions (where maximizing clearance and coverage is more important than path optimality) and is available for manual selection when navigating unfamiliar spaces.

### Frontier-Based Exploration

Two exploration implementations are available, selectable from the dashboard:

**explore_lite (Default):** The standard ROS2 `explore_lite` package. Identifies frontiers (boundaries between known-free and unknown cells) in the occupancy grid, ranks them by size and distance, and sends Nav2 goals to the nearest large frontier. Exploration terminates when coverage exceeds a configurable threshold (default 95%) or no reachable frontiers remain. Reliable and well-tested.

**Custom Frontier Node (Experimental):** `alberc_exploration` implements a frontier scorer that ranks candidates by a weighted combination of: frontier size (favors large unexplored regions), travel distance (favors nearby frontiers), information gain estimate (favors frontiers adjacent to complex geometry), and exploration history (penalizes recently visited areas). This node integrates with the Voronoi GVD planner - exploration paths follow hallway centerlines for maximum clearance in unmapped space. During exploration, the VLM compound view is active: each frontier visit triggers a VLM scene classification that populates the semantic map.

Both implementations publish exploration goals to Nav2's goal interface and can be paused, resumed, or cancelled from the dashboard or via voice command.

### Costmap Configuration

**Global Costmap** (full map extent, 0.05m resolution):

| Layer | Source | Purpose |
|---|---|---|
| Static | SLAM Toolbox occupancy grid | Known walls and obstacles |
| Voxel | L1 3D cloud, height-filtered (0.10-1.50m) | 3D obstacles projected to 2D |
| Obstacle | ZED depth + ultrasonic ranges | Dense near-field obstacle detection |
| Inflation | Exponential decay from lethal cells | Inscribed radius 0.20m, inflation radius 0.55m, cost scaling 5.0 |
| Keepout | Manual zones defined in dashboard | No-go areas (stairs, restricted rooms) |

**Local Costmap** (rolling 5m x 5m, 0.05m resolution, updated 5 Hz):

| Layer | Source | Purpose |
|---|---|---|
| Voxel | L1 3D cloud (full 360 x 90 deg) | Real-time 3D obstacle detection |
| Obstacle | ZED depth + ultrasonic ranges | Near-field obstacles |
| Inflation | Same parameters as global | Safety margin around obstacles |
| Dynamic | reactive_evasion injected lethal cells | Predicted collision points from tracked moving objects |

The L1's pointcloud_to_laserscan conversion (height filter 0.10-1.50m) is performed by the `pointcloud_to_laserscan` ROS2 package configured in the launch file, producing a virtual `sensor_msgs/LaserScan` at 11 Hz consumed by SLAM Toolbox and the global costmap's voxel layer.

### Local Trajectory Optimization (MPPI)

The MPPI (Model Predictive Path Integral) controller implements stochastic optimal control. At each 20 Hz cycle, MPPI samples N = 1000-2000 candidate control sequences (v, omega) over a T = 2.0s horizon at dt = 0.05s, rolls each forward through the robot's kinematic model, evaluates a cost functional J = sum(path_following + obstacle_proximity + smoothness + goal_alignment), and computes the optimal control as the cost-weighted average over all samples: u* = sum(exp(-J_i/lambda) * u_i) / sum(exp(-J_i/lambda)). The temperature parameter lambda controls exploration-exploitation tradeoff. This produces smooth, dynamically feasible trajectories that naturally navigate around obstacles injected by the reactive evasion system.

Local costmap: rolling 5m x 5m, updated 5 Hz from L1 3D projection + ZED depth + ultrasonics. The L1's 90 deg vertical FOV captures obstacles at all heights (table edges, shelves, hanging objects) that a single-plane 2D LiDAR misses.

### Motion Envelope

Max 0.5 m/s linear (auto-reduced: 0.2 m/s near people, 0.15 m/s near pets), 1.0 rad/s angular. Acceleration: 0.5 m/s^2 linear, 1.5 rad/s^2 angular. Goal tolerance: xy 0.10m, yaw 0.15 rad.

### Reactive Evasion and Collaborative Movement

**Principle:** Evade every moving entity unless the active mission designates it as an interaction target. The LLM planner publishes /task/interaction_targets (list of tracker IDs exempt from evasion).

**Tracking:** `object_tracker` fuses L1 3D scan differencing + TensorRT vision detections + SDK-native segmentation via Hungarian association -> per-object Kalman filter (position, velocity, class, ID, confidence) at 10 Hz. The L1's 360 deg x 90 deg hemisphere detects objects from any approach angle in 3D.

**Threat Response (20 Hz, `reactive_evasion` node):**

| TTC | Velocity | Action |
|---|---|---|
| > 3.0s | Any | Monitor - log, no intervention |
| 1.5-3.0s | > 0.1 m/s | Costmap injection - lethal cells at predicted collision point (radius + 0.3m). MPPI replans naturally. Decays 2s. |
| 0.5-1.5s | > 0.2 m/s | Escape vector - perpendicular to threat, biased toward free space. Direct cmd_vel override at 0.3 m/s. |
| < 0.5s | Any | Emergency stop - zero velocity. Hold until threats > 0.8m, stable 1s. |

Ultrasonic < 0.15m: unconditional stop independent of all other processing.

**Interaction Target Behavior:** When a tracked entity IS an interaction target:

| Mission Command | Behavior |
|---|---|
| "Follow me" | Track person at 1.5m, match speed, behind-and-to-side |
| "Come here" | Navigate to person's tracked position |
| "Find the cat" | Patrol until detected -> approach 1.0m -> announce |
| "Go to X, wait for someone" | Navigate -> hold -> report on person detection |

**Social Navigation:** Configurable person identification modes (selectable from dashboard):

| Mode | Method | Comfort Distance | Use Case |
|---|---|---|---|
| All Unknown (default) | No identification. All persons get equal margin. | 1.2m | Privacy-first. Simplest. |
| Persistent Track | Person tracked for > N minutes or introduced via voice gets "known" status | Known: 0.8m, Unknown: 1.2m | No biometrics. Session-persistent. |
| Voice Enrollment | User says "ALBERC, this is Sarah" while person is tracked | Enrolled: 0.8m, Unknown: 1.2m | Natural introduction. No face data stored. |
| Face Recognition | On-device ArcFace/InsightFace via TensorRT. Registered household members identified by face embedding. | Registered: 0.8m, Unknown: 1.2m | Most automated. Privacy-sensitive - all face data on-device only, never transmitted. |

All modes: pass right, 0.2 m/s within 2m of any person, no behind approach.

**Recovery Behaviors (Escalation Chain):**

When navigation fails (goal unreachable, robot stuck, path blocked), ALBERC follows a three-tier escalation:

| Tier | Trigger | Action | Timeout |
|---|---|---|---|
| 1. Standard Recovery | Nav2 controller reports failure | Spin in place (360 deg scan), backup 0.3m, clear costmap, wait 5s, replan | 3 attempts, 30s total |
| 2. LLM Analysis | Standard recovery exhausted | Send ZED RGB + costmap crop + failure description to LLM. LLM suggests: alternative goal, different approach angle, "obstacle is temporary - wait", or "ask user for help" | 15s LLM response |
| 3. User Alert | LLM cannot resolve | Announce via TTS: "I'm stuck near [semantic location]. [LLM analysis]." Dashboard shows camera view, map, and suggested actions. Phone face shows concerned expression. Wait for user input or timeout. | Wait 60s, then enter safe idle |

### Reinforcement Learning Integration Path

The architecture is designed for future RL-based policy learning in two domains: (1) **evasion policy refinement** - the current rule-based TTC evasion system can be replaced by a learned policy trained in simulation (Gazebo/Isaac Sim) via PPO or SAC, with the state space being the tracked object array + robot odometry and the action space being (v, omega) commands, rewarded for mission progress and penalized for collisions and personal space violations; (2) **adaptive social navigation** - learning context-dependent comfort distances and passing behaviors from human demonstration data, deployable as a Nav2 controller plugin. The ROS2 architecture cleanly separates the control interface (/cmd_vel) from the policy source, allowing hot-swapping between rule-based and learned controllers without architectural changes.

The [Export & File Management](#export--file-management) system enables a complete sim-to-real-to-sim loop for RL training: real-world environments mapped by ALBERC are exported as USD scenes (Isaac Sim profile) or SDF worlds (Gazebo profile) loadable directly in the target simulator. RL policies can then be trained in the actual mapped geometry — not synthetic approximations — via Isaac Gym's massively parallel environments or Gazebo's physics simulation, then deployed back to the physical robot. This closes the loop between real-world data collection and simulation-based policy optimization.

---

## LLM Intelligence

### Provider Abstraction Layer

The `llm_planner_node` communicates with LLMs through a unified provider-agnostic interface. A configuration parameter selects the active backend:

| Provider | Model Examples | Access | Latency |
|---|---|---|---|
| **Local (Jetson)** | LLaVA-1.6 7B, Qwen2-VL 7B (INT4/INT8 TensorRT) | On-device GPU | 1-3s |
| **OpenAI** | GPT-4o, GPT-4o-mini | Phone network -> API | 2-5s |
| **Anthropic** | Claude 4 Sonnet, Claude 4 Haiku | Phone network -> API | 2-5s |
| **Google** | Gemini 2.0 Flash, Gemini 2.5 Pro | Phone network -> API | 2-5s |

The abstraction layer normalizes the request/response format: input is always (text_prompt, optional_image_base64), output is always structured JSON (action plan) or natural language (conversation). Provider failover is automatic - if the primary provider times out (5s threshold), the next in the configured priority list is tried. Local inference is the default for low-latency tasks; cloud providers handle complex multi-step reasoning or large-context scene analysis.

For vision-language queries ("what's on the kitchen counter?"), the ZED Mini RGB frame is JPEG-compressed and included in the API call. For text-only commands ("patrol the house"), no image is sent.

### VLM-Augmented Perception

The `vlm_context_node` provides spatial context labeling for the Compound sensor view via Google Gemini's vision-language model. At configurable intervals (default 1 Hz), the node packages:

1. The current ZED Mini RGB frame (JPEG, 720p)
2. A cropped region of the RTAB-Map 3D point cloud centered on the robot's current position (5m radius)
3. The current TrackedObjectArray (object IDs, classes, positions)
4. The 2D occupancy grid crop with semantic labels already assigned

This composite context is sent to Gemini with a structured prompt requesting spatial scene descriptions. The VLM returns JSON-structured labels: room identification, object relationships ("coffee maker on kitchen counter, left of sink"), activity recognition ("person sitting on couch, watching TV"), and navigational context ("narrow hallway, door open on left"). These labels are:
- Anchored to map coordinates in the semantic YAML overlay
- Rendered as text annotations in the Compound view
- Available to the LLM planner for natural-language scene understanding
- Cached with TTL (default 30s) to avoid redundant queries for static scenes

### Voice Pipeline

```
Phone mic -> wake word (Porcupine, on-phone) -> WebSocket audio stream
-> audio_bridge_node -> stt_node (Whisper small/base, TensorRT, Jetson GPU)
-> text -> llm_planner_node (local or cloud LLM)
-> action plan -> Nav2 actions + /task/interaction_targets
-> response text -> tts_node (Piper, Jetson CPU)
-> WebSocket -> phone speaker
```

**Noise handling:** Motor and fan noise is suppressed via RNNoise or Speex on the audio stream before Whisper inference. Wake word false-positive rate is tuned to < 1/hour to prevent phantom commands.

### Task Decomposition

*"Map the apartment"* -> frontier-based exploration (Voronoi GVD planner auto-selected): identify unknown-free boundaries in occupancy grid -> plan path to nearest large frontier -> scan -> repeat until coverage > 95% -> serialize map.

*"What's in the living room?"* -> navigate to living room (semantic map lookup) -> capture ZED RGB -> send to LLM with prompt "Describe everything you see in this room" -> respond via TTS.

*"Follow me and watch for the dog"* -> detect requesting user -> add to interaction_targets -> follow at 1.5m -> simultaneously monitor for dog class detections -> alert via TTS if dog enters tracked objects.

*"Patrol every 30 minutes"* -> create scheduled task: cycle semantic waypoints, capture RGB at each, query LLM for anomalies, report via TTS or dashboard notification.

### LLM Action Plan Schema

The LLM outputs structured JSON action plans validated against a schema before execution. No unvalidated LLM output reaches the navigation stack.

**Schema:**
```json
{
  "plan_id": "string (UUID)",
  "intent": "string (natural language summary)",
  "steps": [
    {
      "action": "navigate | follow | detect | capture | speak | wait | query_llm | set_param",
      "target": "string (semantic location, track ID, or parameter name)",
      "parameters": {
        "goal_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
        "timeout_s": 60,
        "speed_limit": 0.3,
        "condition": "string (optional, e.g. 'until person detected')"
      },
      "on_failure": "retry | skip | abort | escalate"
    }
  ],
  "interaction_targets": ["track_id_1", "track_id_2"],
  "planner_override": "astar | rrtstar | voronoi | null"
}
```

**Examples:**

*"Go to the kitchen"*
```json
{
  "plan_id": "a1b2c3",
  "intent": "Navigate to kitchen",
  "steps": [
    {"action": "navigate", "target": "kitchen", "parameters": {"timeout_s": 120}, "on_failure": "escalate"}
  ],
  "interaction_targets": [],
  "planner_override": null
}
```

*"Follow me and watch for the dog"*
```json
{
  "plan_id": "d4e5f6",
  "intent": "Follow requesting user while monitoring for dog",
  "steps": [
    {"action": "detect", "target": "person_nearest", "parameters": {}, "on_failure": "abort"},
    {"action": "follow", "target": "$detected_track_id", "parameters": {"distance": 1.5, "condition": "until cancelled"}, "on_failure": "retry"},
    {"action": "detect", "target": "class:dog", "parameters": {"continuous": true, "condition": "parallel"}, "on_failure": "skip"},
    {"action": "speak", "target": "Dog detected nearby", "parameters": {"condition": "on dog detection"}, "on_failure": "skip"}
  ],
  "interaction_targets": ["$detected_track_id"],
  "planner_override": null
}
```

*"Map this floor"*
```json
{
  "plan_id": "g7h8i9",
  "intent": "Explore and map current floor",
  "steps": [
    {"action": "set_param", "target": "planner", "parameters": {"value": "voronoi"}, "on_failure": "skip"},
    {"action": "navigate", "target": "explore_frontiers", "parameters": {"coverage_threshold": 0.95, "timeout_s": 1800}, "on_failure": "escalate"},
    {"action": "speak", "target": "Mapping complete. Coverage is $coverage_percent percent.", "parameters": {}, "on_failure": "skip"}
  ],
  "interaction_targets": [],
  "planner_override": "voronoi"
}
```

The `llm_planner_node` validates each step's `action` field against known action types, resolves semantic targets (e.g., "kitchen" -> map coordinates), and rejects plans with unknown actions or unreachable targets before execution begins.

**Voice Latency Budget:**

The voice pipeline follows an adaptive strategy, preferring cloud (online) LLM providers for higher quality responses:

| Stage | Local LLM | Cloud LLM | Notes |
|---|---|---|---|
| Wake word detection | ~50ms (on-phone) | ~50ms (on-phone) | Porcupine, runs on phone CPU |
| Audio stream to Jetson | ~100ms | ~100ms | WebSocket, phone to Jetson WiFi |
| Whisper STT | ~500ms (small) / ~1s (base) | ~500ms / ~1s | TensorRT on Jetson GPU |
| LLM inference | 1-3s (7B INT4) | 2-5s (cloud API) | Cloud preferred when available |
| TTS synthesis | ~200ms | ~200ms | Piper on Jetson CPU |
| Audio stream to phone | ~100ms | ~100ms | WebSocket return path |
| **Total** | **~2-4.5s** | **~3-6.5s** | Adaptive: try cloud first (5s timeout), fall back to local |

The adaptive strategy: the `llm_planner_node` sends requests to the configured cloud provider first. If the cloud response arrives within 5s, it is used. If the timeout is reached, the request is simultaneously sent to the local model. For simple commands ("go to kitchen"), the local model is fast enough (~1s) that the fallback adds minimal delay. For complex queries ("describe what's different about this room since yesterday"), the cloud provider's superior reasoning justifies the extra 2-3s.

### Semantic Map

The LLM and VLM annotate occupancy grid regions with labels (kitchen, bedroom, hallway, charging station) during exploration. VLM context labels provide richer spatial descriptions that are distilled into navigational waypoint names. Stored as YAML overlay indexed by grid coordinates. Enables natural-language goal resolution - "go to the kitchen" resolves to map coordinates without coordinate knowledge.

**YAML Schema:**
```yaml
# semantic_map.yaml
map_id: "apt_123_floor_1"
gps_anchor: {lat: 33.4242, lon: -111.9281, alt: 362.0}
regions:
  - label: "kitchen"
    aliases: ["the kitchen", "kitchen area"]
    centroid: {x: 3.2, y: 1.8}       # meters in map frame
    boundary:                          # convex polygon vertices
      - {x: 1.5, y: 0.5}
      - {x: 5.0, y: 0.5}
      - {x: 5.0, y: 3.0}
      - {x: 1.5, y: 3.0}
    floor: 1
    vlm_description: "Kitchen with granite countertop, coffee maker on left, sink center, refrigerator right"
    last_updated: "2026-04-13T14:30:00Z"
    confidence: 0.92
  - label: "hallway"
    aliases: ["the hall", "corridor"]
    centroid: {x: 6.0, y: 1.5}
    boundary: [{x: 5.0, y: 0.5}, {x: 7.5, y: 0.5}, {x: 7.5, y: 2.5}, {x: 5.0, y: 2.5}]
    floor: 1
    vlm_description: "Narrow hallway with wooden floor, door to bedroom on left"
    last_updated: "2026-04-13T14:31:00Z"
    confidence: 0.88
waypoints:
  - name: "charging_station"
    pose: {x: 0.5, y: 0.3, theta: 0.0}
    floor: 1
    type: "dock"
```

Regions are created and refined by the VLM during exploration and can be manually edited or corrected from the dashboard map manager.

---

## User Interfaces

### Phone Docked Display

A smartphone is physically docked on the robot's L3 tier and runs a Progressive Web App (PWA) that serves as ALBERC's face and primary audio interface.

**Animated Face:** A vector-animated avatar displays the robot's current emotional/operational state - idle (gentle breathing animation), listening (ear-perk animation), processing (thinking dots), speaking (lip-sync to TTS), navigating (directional gaze), error (concerned expression). The face animations are rendered client-side in the PWA using CSS/SVG keyframes, driven by state messages from the `phone_face_node` over WebSocket.

**Status Bar:** A persistent status bar below the face displays: battery voltage and percentage, current mission name, active planner (A*/RRT*/Voronoi), navigation state (idle/planning/moving/evading), WiFi signal strength, and LLM provider status. Updated at 1 Hz.

**Chat Transcription:** A scrollable overlay shows the real-time voice conversation: user speech (Whisper STT output), LLM responses, and system announcements. This provides visual confirmation of voice commands and is essential for noisy environments where voice feedback may be missed.

**Hardware Utilization:** The docked phone provides GPS (outdoor localization or coarse indoor position via WiFi), microphone (voice commands), speaker (TTS output and alerts), WiFi hotspot (optional network source for the Jetson when no infrastructure WiFi is available), and Bluetooth (future peripheral connectivity).

### Browser Dashboard

React + rosbridge_suite (WebSocket port 9090) + ros2djs, served by Nginx on Jetson port 80.

| Feature | Description |
|---|---|
| Live 2D Map | Occupancy grid + pose + path + tracked obstacles + semantic labels. Click to navigate. |
| Live 3D View | Three.js rendering of RTAB-Map colored cloud. Rotate, zoom, measure. |
| Switchable Sensor Views | Toggle between Photorealistic, Voxel/Point Cloud, and Compound+VLM views |
| Camera Feeds | ZED Mini stereo RGB + depth + SDK segmentation overlays, MJPEG 720p/15fps |
| LiDAR View | L1 3D cloud rendered live with SDK voxel segmentation coloring |
| Sensor Panel | IMU orientation, ultrasonic ranges, system temps |
| Motor Panel | Per-motor RPM, PWM duty, commanded vs actual velocity, current estimate, stall/fault indicators, PID/MPC mode, battery voltage |
| Teleoperation | Virtual joystick + WASD. Speed limiter. Evasion stays active. |
| LLM Chat | Text input to LLM planner. Conversational interface with context history. Scene QA. |
| Task Manager | Queue/monitor/cancel tasks. LLM reasoning log. Mission progress. |
| Planner Selection | Runtime switch between A*, RRT*, and Voronoi GVD. Visual path comparison. |
| Map Manager | Save/load/export maps. Floor switching. Trigger exploration. Quick-export single format (PLY/OBJ/glTF/USD/PCD/FBX). |
| Export Wizard | Multi-step export dialog: select data scope, choose target environment profile (Isaac Sim/Gazebo/Visualization/Blender/Web Viewer), configure data inclusions (all sensor data, perception outputs, navigation state, semantic metadata), preview file tree, download ZIP bundle. See [Export & File Management](#export--file-management). |
| Export History | Past exports with re-download, re-export, delete, rename/tag. Storage usage display. Configurable retention. |
| Recording | Bag recording controls: start/stop, save circular buffer, mission auto-record toggle, storage usage display. Bag clips selectable for inclusion in export bundles. |
| Person ID Mode | Switch between All Unknown, Persistent Track, Voice Enrollment, Face Recognition |
| Tracked Objects | All tracks: ID, class, position, velocity, evasion/interaction status |
| Configuration | Nav2, SLAM, evasion, LLM provider/model, VLM settings - editable without SSH |
| Diagnostics | Node status, topic Hz, TF tree, CPU/GPU/RAM/temp |

### Mobile Browser UI

The dashboard includes a responsive mobile-optimized variant for phone and tablet browsers (separate from the docked phone's PWA face). The mobile UI provides:

- Touch-optimized teleoperation joystick with haptic feedback (on supported devices)
- Swipeable sensor view cards (Photorealistic / Voxel / Compound)
- Condensed task manager with quick-action buttons (stop, home, explore)
- LLM chat with voice-to-text input via device microphone
- Responsive layout adapting to portrait and landscape orientations
- Minimal-bandwidth mode: reduces stream quality for cellular connections

### VR and WebXR

**Streams:** 3D colored cloud (Draco-compressed, 1-5 Hz), 2D occupancy grid (PNG, 1 Hz), robot 6-DOF pose (20 Hz), tracked objects (10 Hz), ZED stereo (MJPEG/H.264).

**WebXR (Meta Quest Target):** Immersive VR via browser on Meta Quest standalone headset. Three.js renders the colored 3D map + robot avatar + tracked obstacles + semantic labels. Point-to-navigate sends goals back to Nav2 via WebSocket. View 3D map exports at any milestone - exploration completion, room scan, or manual save - as navigable VR scenes.

**ROS2-Unity Bridge:** ros-tcp-connector for high-fidelity VR, multi-user sessions, and 3D annotation.

**Export:** PLY, OBJ, glTF, USD, PCD, FBX on demand via the dashboard Export Wizard, voice command, or ROS2 service. Environment-specific ZIP bundles (Isaac Sim, Gazebo, Visualization, Blender, Web Viewer) package all necessary scene descriptions, geometry, textures, sensor data, semantic metadata, and optional ROS bag clips into a single downloadable archive. See [Export & File Management](#export--file-management) for the complete export architecture.

### Export & File Management

The Export & File Management system packages all sensor data, perception outputs, navigation state, and intelligence layer artifacts into self-contained ZIP archives optimized for specific target environments. The system bridges the gap between ALBERC's ROS2 runtime data and external simulation, visualization, and analysis tools — enabling direct import into NVIDIA Isaac Sim, Gazebo Fortress, CloudCompare, Blender, and browser-based viewers without manual file assembly.

**Architecture:** The `export_manager_node` (package: `alberc_export`) orchestrates the export pipeline. It queries `map_manager_node` for spatial data (point clouds, occupancy grids, semantic maps), `rosbag2` for bag clip extraction, and RTAB-Map for the SLAM pose graph and trajectory. Format conversion is performed via Open3D (PLY, PCD, OBJ, glTF), Pixar's `usd-core` Python package (USD/USDA), Assimp (FBX), and template-based XML generation (SDF). Mesh reconstruction from dense point clouds uses Open3D's Poisson surface reconstruction. The dashboard communicates with the export manager via rosbridge WebSocket, and export progress is published on `/export/progress` for real-time UI feedback.

#### Export Profiles

Five pre-configured profiles produce environment-specific bundles. Each profile determines which formats, scene descriptions, and data categories are included by default:

| Profile | Target Environment | Primary Formats | Scene Description | Key Contents |
|---|---|---|---|---|
| **Isaac Sim** | NVIDIA Isaac Sim / Omniverse | USD/USDA, PLY, PNG, MDL | USD scene graph with Xform hierarchy, UsdPreviewSurface materials, Isaac Sim semantic schema tags | Poisson-reconstructed meshes with UV-mapped ZED sRGB textures, per-prim semantic tags matching VLM classifications, optional ALBERC robot USD (converted from URDF via `urdf_to_usd`) |
| **Gazebo** | Gazebo Fortress / Classic | SDF world, OBJ/DAE, PNG | SDF `<world>` with per-region `<model>` entries, ALBERC robot `<include>`, physics and lighting plugins | Per-semantic-region meshes as independent Gazebo models, simulated sensor plugins, `model.config` per model |
| **Visualization** | CloudCompare, MeshLab, PCL | PLY, PCD, OBJ | N/A (raw geometry) | Full and per-layer point clouds with scalar fields, semantic labels YAML, coordinate frame metadata, sensor calibration data |
| **Blender** | Blender, Unity, Unreal Engine | glTF 2.0, FBX, PNG | glTF scene graph with named nodes per semantic region | Textured meshes with embedded VLM labels as glTF extras, per-region material assignments |
| **Web Viewer** | Any modern browser (no install) | Draco-compressed glTF | Self-contained HTML (`viewer.html`) | Three.js-based interactive viewer with orbit controls, semantic region toggles, measurement tools, and embedded metadata |

All profiles include the complete sensor telemetry, perception outputs, and navigation state by default. The "Custom" profile option in the Export Wizard allows per-category toggling of every data inclusion.

#### Supported Export Formats

| Format | Library | Purpose | Profile(s) |
|---|---|---|---|
| PLY | Open3D | Point clouds with vertex colors and per-vertex scalar fields (segmentation class, confidence, VLM region ID) | All |
| PCD | Open3D | Point Cloud Library native binary format for PCL tools and CloudCompare | Visualization |
| OBJ + MTL | Open3D | Textured triangle meshes with material library | All except Web |
| glTF 2.0 | Open3D / trimesh | Scene graph with PBR materials, web-compatible, extensible via extras | Blender, Web, Visualization |
| USD / USDA | `usd-core` (Pixar OpenUSD, pip) | Isaac Sim / Omniverse native scene description with semantic schema, material bindings, and Xform hierarchy | Isaac Sim |
| FBX | Assimp | Autodesk interchange for Blender, Unity, Unreal Engine | Blender |
| SDF | Template-based XML generation | Gazebo Fortress world description with model, physics, and sensor plugin definitions | Gazebo |
| Draco glTF | `draco` + `gltfpack` | Geometry-compressed glTF for bandwidth-efficient web delivery | Web Viewer |
| MCAP | rosbag2 MCAP storage plugin | ROS2 bag clips bundled in export packages for full-fidelity sensor replay | All (optional) |
| KML | simplekml | GPS trajectory for Google Earth visualization | All (if GPS data available) |

**New dependency:** `usd-core` (Pixar's official lightweight OpenUSD Python package, ~50 MB, pip-installable) is the only significant addition. It has no Isaac Sim dependency — the exported USD files are standard OpenUSD loadable in any USD-compatible tool.

#### ZIP Package Structure

Every export produces a single `.zip` file with a deterministic internal layout. **All collected sensor data and processed outputs from the mapping session are included** — raw sensor streams from every hardware source, fused perception outputs, navigation decisions, intelligence layer artifacts, and system telemetry:

```
alberc_export_{profile}_{map_id}_{timestamp}.zip
+-- manifest.json                       # Package metadata, provenance, complete file inventory
+-- README.md                           # Human-readable description of all contents and import instructions
|
+-- scene/                              # Scene description files (profile-dependent)
|   +-- scene.usda                      # Isaac Sim: OpenUSD scene graph
|   +-- world.sdf                       # Gazebo: SDF world file
|   +-- scene.gltf                      # Blender/Web: glTF 2.0 scene
|
+-- maps/                               # SLAM map outputs
|   +-- occupancy_grid.pgm              # 2D occupancy grid (SLAM Toolbox serialized map)
|   +-- occupancy_grid.yaml             # Map metadata: resolution, origin, occupied/free thresholds
|   +-- occupancy_grid_per_floor/       # Per-floor 2D maps (multi-floor sessions)
|   |   +-- floor_{n}.pgm
|   |   +-- floor_{n}.yaml
|   +-- rtabmap.db                      # RTAB-Map full 3D SLAM database (optional, can be large)
|   +-- costmap_snapshot.pgm            # Global costmap at export time (inflation + voxel + keepout layers)
|   +-- costmap_metadata.yaml           # Costmap layer config, inflation parameters, keepout zone definitions
|
+-- pointclouds/                        # 3D point cloud data
|   +-- full_map.ply                    # Complete RTAB-Map colored 3D point cloud
|   +-- full_map.pcd                    # PCL format (Visualization profile)
|   +-- layers/                         # Per-sensor-fusion-view data layers
|   |   +-- photorealistic.ply          # View 1: ZED sRGB vertex colors + segmentation class scalar field
|   |   +-- voxel_segmented.ply         # View 2: L1 SDK class-colored (ground/static/dynamic) + class ID scalar
|   |   +-- compound_labeled.ply        # View 3: Fused cloud with VLM region ID + confidence scalar fields
|   +-- sequence/                       # Temporal 4D: timestamped cloud snapshots (optional)
|       +-- sequence_index.json         # Timestamp -> filename -> robot pose mapping
|       +-- {timestamp_ns}.ply          # Individual cloud snapshots at configurable interval
|
+-- meshes/                             # Poisson-reconstructed triangle meshes
|   +-- full_map.obj                    # Complete environment mesh
|   +-- full_map.mtl                    # Material library
|   +-- regions/                        # Per-semantic-region meshes (Isaac Sim prims, Gazebo models)
|       +-- {region_label}.obj          # e.g., kitchen.obj, hallway.obj, bedroom.obj
|       +-- {region_label}.mtl
|
+-- textures/                           # Visual appearance data from ZED Mini sRGB
|   +-- atlas.png                       # UV-mapped texture atlas composited from ZED frames
|   +-- regions/                        # Per-region texture maps
|   |   +-- {region_label}.png
|   +-- keyframes/                      # ZED Mini RGB keyframes at SLAM keyframe rate
|       +-- keyframe_index.json         # Timestamp -> filename -> 6-DOF camera pose -> intrinsics
|       +-- {timestamp_ns}.jpg          # Individual JPEG keyframes (720p)
|
+-- depth/                              # ZED Mini depth data
|   +-- depth_keyframes/                # Depth maps at SLAM keyframe rate
|       +-- keyframe_index.json         # Timestamp -> filename -> camera pose
|       +-- {timestamp_ns}.png          # 16-bit PNG depth maps (units: millimeters)
|
+-- segmentation/                       # All segmentation source outputs
|   +-- zed_sdk/                        # ZED SDK neural-network object segmentation
|   |   +-- detections.json             # All detections: instance masks, 3D bounding boxes, classes, confidence
|   |   +-- masks/                      # Per-keyframe instance segmentation masks (PNG, class-ID-encoded pixels)
|   |       +-- {timestamp_ns}.png
|   +-- l1_sdk/                         # L1 SDK geometric voxel segmentation
|   |   +-- voxel_labels.json           # Per-point classification statistics (ground/static/dynamic)
|   +-- tensorrt/                       # Custom TensorRT model detections
|   |   +-- detections.json             # All custom detections: bounding box, class, confidence, 3D centroid
|   +-- segmentation_classes.json       # Unified class ID -> label -> RGB color mapping across all sources
|
+-- semantic/                           # VLM and LLM intelligence layer annotations
|   +-- semantic_map.yaml               # Full VLM-annotated semantic map (regions, waypoints, aliases, boundaries)
|   +-- region_descriptions.json        # Per-region VLM natural-language descriptions with timestamps and confidence
|   +-- vlm_context_log.json            # Complete VLM query/response history with timestamps and map coordinates
|   +-- llm_reasoning_log.json          # LLM task plans, scene QA responses, reasoning chains, action plan JSON
|
+-- tracking/                           # Multi-object tracking history
|   +-- tracked_objects.json            # Full TrackedObjectArray time-series: per-track ID, semantic class,
|   |                                   #   3D position, velocity vector, bounding radius, confidence,
|   |                                   #   evasion status, lifecycle state (tentative/confirmed/lost/deleted)
|   +-- tracking_summary.json           # Per-class aggregate statistics: count, mean velocity, spatial distribution
|
+-- navigation/                         # Path planning and navigation decision history
|   +-- planned_paths.json              # All planned paths: planner type (A*/RRT*/Voronoi), timestamps, waypoints
|   +-- executed_paths.json             # Actual executed trajectories with interpolated cmd_vel commands
|   +-- evasion_events.json             # Reactive evasion events: trigger track, TTC, response tier, duration, outcome
|   +-- exploration_progress.json       # Frontier exploration log: coverage % over time, frontier queue, termination reason
|
+-- trajectory/                         # Robot 6-DOF pose history
|   +-- poses.csv                       # TUM trajectory format: timestamp tx ty tz qx qy qz qw
|   +-- poses.json                      # Same data with additional metadata (floor, GPS, velocity)
|   +-- trajectory.kml                  # GPS trajectory for Google Earth (if GPS data available)
|   +-- slam_graph.json                 # SLAM pose graph: keyframe nodes (6-DOF poses) + loop closure edges
|
+-- sensors/                            # Raw sensor data from all hardware sources
|   +-- lidar_l1/                       # Unitree L1 4D LiDAR
|   |   +-- l1_cloud_stats.json         # Per-scan statistics: point count, effective rate, hemisphere coverage
|   |   +-- l1_imu.json                 # L1 onboard 250 Hz IMU data (accel + gyro, motion compensation input)
|   |   +-- l1_intensity.json           # Grayscale intensity channel metadata (the "4th dimension" in 4D LiDAR)
|   |   +-- l1_sdk_config.json          # SDK version, scan mode, voxel segmentation parameters
|   +-- camera_zed/                     # Stereolabs ZED Mini stereo camera
|   |   +-- zed_config.json             # SDK version, resolution, framerate, depth mode, confidence threshold
|   |   +-- zed_imu.json                # ZED onboard 800 Hz IMU data (raw samples, consumed internally by VIO)
|   |   +-- zed_vio_confidence.json     # Visual-inertial odometry tracking confidence time-series
|   |   +-- zed_calibration.json        # Stereo calibration: intrinsics (fx,fy,cx,cy), extrinsics, baseline, distortion
|   +-- imu_icm20948/                   # SparkFun ICM-20948 9-axis IMU (Arduino I2C)
|   |   +-- accel_gyro.json             # Raw accelerometer + gyroscope data at 100 Hz with timestamps
|   |   +-- bias_calibration.json       # Gyroscope bias estimate from startup 5-second static calibration
|   |   +-- imu_config.json             # Range settings (dps, g), DMP configuration, magnetometer state
|   +-- ultrasonic_hcsr04/              # HC-SR04 x4 ultrasonic proximity sensors (Arduino GPIO)
|   |   +-- ranges.json                 # All range readings: timestamp, sensor_id (front/rear/left/right), range_m
|   |   +-- sensor_config.json          # Beam angle (15 deg), stagger timing (50ms), cross-talk mitigation
|   +-- encoders/                       # TS-25GA370 quadrature wheel encoders (Arduino ISR)
|   |   +-- raw_ticks.json              # Raw encoder tick counts (left/right channels) with timestamps
|   |   +-- encoder_config.json         # PPR (540 output), wheel diameter, ticks_per_meter, track width
|   +-- phone/                          # Docked smartphone sensor package
|   |   +-- gps_fixes.json              # All NavSatFix messages: lat, lon, alt, horizontal accuracy, fix type
|   |   +-- gps_anchor.json             # Session GPS anchor point for map georeferencing
|   |   +-- barometer.json              # Barometric pressure / altitude readings (~0.1m resolution, floor detection)
|   |   +-- wifi_signal.json            # WiFi signal strength (RSSI) over time
|   |   +-- phone_config.json           # Phone model, OS version, sensor capabilities, hotspot state
|   +-- arduino/                        # Arduino Mega 2560 co-processor aggregate data
|       +-- battery_voltage.json        # Analog A0 voltage divider readings time-series (4S LiPo monitoring)
|       +-- serial_health.json          # rosserial connection health: latency, dropped messages, reconnect events
|
+-- odometry/                           # All odometry sources and EKF fusion state
|   +-- wheel_odom.json                 # Raw wheel encoder odometry (/odom_raw, 50 Hz)
|   +-- zed_vio.json                    # ZED Mini visual-inertial odometry (/zed/odom, 30 Hz)
|   +-- fused_odom.json                 # EKF-fused canonical odometry (/odom, 50 Hz)
|   +-- ekf_state.json                  # Full EKF state history: pose, velocity, 15x15 covariance matrices
|   +-- ekf_config.json                 # Per-sensor fusion configuration (odomN_config, imuN_config matrices)
|
+-- motor/                              # Motor and actuator telemetry (Arduino-sourced)
|   +-- motor_health.json               # /motor/health: per-motor RPM, PWM duty, current estimate, stall/fault flags
|   +-- motor_state.json                # /motor/state: commanded vs actual velocity, PID/MPC tracking error
|   +-- cmd_vel_history.json            # All /cmd_vel commands with timestamps (full command history)
|   +-- motor_config.json               # PID gains (Kp/Ki/Kd per wheel), MPC parameters, motion envelope limits
|
+-- audio/                              # Voice interaction data (optional, privacy-sensitive, explicit opt-in)
|   +-- voice_transcripts.json          # Whisper STT transcripts + LLM responses with timestamps
|   +-- audio_clips/                    # Raw audio WAV clips from phone microphone (if bag-recorded)
|
+-- jetson/                             # Jetson Orin Nano Super compute platform telemetry
|   +-- tegrastats.json                 # Time-series: CPU/GPU utilization %, RAM usage, GPU memory, power (W), per-zone temps
|   +-- gpu_memory_breakdown.json       # Per-process GPU memory allocation: ZED SDK, TensorRT, RTAB-Map, SDK segmentation
|   +-- thermal_history.json            # Temperature time-series: CPU, GPU, board, ambient (for thermal analysis)
|   +-- power_mode.json                 # NVPModel power mode transitions (7W / 15W / 25W) over session
|
+-- materials/                          # Material definitions for rendering engines
|   +-- materials.mdl                   # NVIDIA MDL material definitions (Isaac Sim profile)
|   +-- materials.mtl                   # Wavefront MTL material library (Gazebo/Blender profiles)
|
+-- robot/                              # Robot kinematic model (optional toggle)
|   +-- alberc.urdf                     # URDF/xacro-generated kinematic model
|   +-- alberc.usd                      # USD-converted robot asset (Isaac Sim profile, via urdf_to_usd)
|   +-- alberc.sdf                      # SDF-converted robot model (Gazebo profile)
|   +-- tf_tree.json                    # Complete TF2 frame tree snapshot at export time
|
+-- bags/                               # ROS2 bag clips (optional toggle)
|   +-- session_clip.mcap               # MCAP-format bag of user-selected time range and topic set
|   +-- topic_list.json                 # Topics included in the bag clip with message counts
|
+-- diagnostics/                        # System state context at capture time
|   +-- node_status.json                # ROS2 node list with health status and uptime
|   +-- topic_rates.json                # Per-topic Hz measurements at export time
|   +-- system_resources.json           # System resource snapshot: CPU, GPU, RAM, disk, network
|   +-- battery_log.json                # Battery voltage history over the full mapping session
```

**`manifest.json` schema** includes: format version, export profile, map session ID, ISO 8601 timestamp, GPS anchor coordinates, floor indices, coordinate frame convention (ROS REP-105, Z-up, meters), active sensor sources, map coverage percentage, total point count, mesh face count, semantic region count, per-data-layer inclusion flags, sensor configurations at capture time (L1 scan rate, ZED resolution and depth mode, IMU sample rates), export options used (mesh quality, decimation, coordinate frame), and a complete file inventory with relative paths, byte sizes, and SHA-256 checksums.

**Per-profile inclusion:** Not every directory is populated for every profile — the Isaac Sim profile omits PCD files, the Web Viewer profile omits raw sensor telemetry. The manifest declares which directories are populated. The Export Wizard's "Custom" mode allows independent toggling of each data category. Privacy-sensitive data (audio recordings, voice transcripts) requires explicit user opt-in and is excluded by default across all profiles.

#### Three Sensor Fusion Views as Export Data Layers

Each of the three sensor fusion views (see [Three Sensor Fusion Views](#three-sensor-fusion-views)) is exported as a separate point cloud file in `pointclouds/layers/`, enabling independent visualization and analysis of each perceptual modality:

| View | Export File | Vertex Color Encoding | Per-Vertex Scalar Fields |
|---|---|---|---|
| Photorealistic | `photorealistic.ply` | RGB from ZED Mini sRGB stereo imagery | Segmentation class ID (ZED SDK) |
| Voxel / Point Cloud | `voxel_segmented.ply` | Class-coded: ground (green), static structure (gray), dynamic clusters (red) | L1 SDK voxel class ID, grayscale intensity (4th D) |
| Compound + VLM | `compound_labeled.ply` | ZED sRGB where stereo coverage exists, L1 class colors elsewhere | Segmentation class ID, VLM semantic region ID, classification confidence |

For mesh-based exports (Isaac Sim, Gazebo, Blender profiles), segmentation classes are represented as separate OBJ groups or glTF mesh primitives, enabling per-class visibility toggling in the target tool. In USD exports, each semantic region is a distinct `Mesh` prim under a named `Xform`, with Isaac Sim semantic schema tags applied per-prim.

#### Mesh Reconstruction Pipeline

RTAB-Map produces dense colored 3D point clouds, but simulation environments (Isaac Sim, Gazebo) and 3D editors (Blender) require triangle mesh geometry for rendering, physics simulation, and material application. The export pipeline reconstructs meshes from point clouds via Open3D:

- **Method:** Poisson surface reconstruction with configurable octree depth, producing watertight manifold meshes from oriented point clouds. Normal estimation via PCA with tangent plane propagation.
- **Quality tiers:** Low (octree depth 8, ~30s on Jetson), Medium (depth 10, ~2 min), High (depth 12, ~10 min). Selectable in the Export Wizard.
- **UV parametrization:** xatlas or Open3D's parametrize_atlas generates UV coordinates. ZED Mini sRGB keyframes are projected onto mesh faces via the known camera poses and intrinsics, producing per-region PNG texture atlases.
- **Segmentation-aware meshing:** The point cloud is partitioned by semantic region labels before reconstruction, producing per-region meshes that become independent models in Gazebo SDF or distinct prims in USD.
- **Decimation:** Optional quadric mesh simplification (50% or 25% face count) for large environments, preserving mesh topology and UV mapping.

#### Isaac Sim USD Export

The Isaac Sim export profile produces a complete OpenUSD scene that loads directly in NVIDIA Isaac Sim or any Omniverse-compatible application:

**USD Scene Graph Structure:**
```
/ALBERCExport                           (Xform - root)
  /floor_1                              (Xform - per-floor grouping)
    /kitchen                            (Mesh + UsdPreviewSurface material)
    /hallway                            (Mesh + UsdPreviewSurface material)
    /bedroom                            (Mesh + UsdPreviewSurface material)
    /pointcloud                         (PointInstancer - raw colored cloud)
  /robot                                (Reference to alberc.usd - optional)
```

- **Materials:** `UsdPreviewSurface` shaders with `diffuseColor` connected to the ZED-derived texture atlas PNGs. For point-cloud-only regions, `primvars:displayColor` preserves vertex RGB.
- **Semantic Tags:** Isaac Sim's Replicator semantic schema (`semantics:Semantics:params:semanticType` = `"class"`, `semantics:Semantics:params:semanticData` = `"kitchen"`) is applied per-prim, matching the VLM-derived semantic map classifications. This enables Isaac Sim's built-in semantic segmentation synthetic data sensor to produce ground-truth annotations consistent with ALBERC's perception pipeline.
- **Robot Asset:** The ALBERC URDF is converted to USD via Isaac Sim's `urdf_to_usd` asset converter (or the open-source `urdfpy` + `usd-core` pipeline). The robot USD is placed at the robot's last known 6-DOF pose as a USD `<reference>`, ready for Isaac Sim's articulation and sensor simulation APIs.
- **No Isaac Sim dependency at export time:** The export pipeline uses only `usd-core` (Pixar's open-source Python package). The output is standard OpenUSD, loadable in Isaac Sim, Omniverse, Blender (with USD plugin), Apple Reality Composer, and any OpenUSD-compatible tool.

**Sim-to-real-to-sim round-trip:** Map a real environment with ALBERC -> export as USD via the dashboard -> load the real-world-derived scene in Isaac Sim -> run a simulated ALBERC in the actual mapped space -> train RL policies (evasion, social navigation) via Isaac Gym in the real environment's geometry -> deploy learned policies back to the physical robot. This closes the loop between the RL Integration Path (see [Reinforcement Learning Integration Path](#reinforcement-learning-integration-path)) and real-world data collection.

#### Gazebo SDF World Export

The Gazebo export profile generates a complete SDF world file that reconstructs the mapped environment as a Gazebo Fortress simulation:

- **World structure:** A `<world>` element containing per-semantic-region `<model>` entries (each with `<visual>` and `<collision>` meshes from the segmented OBJ files), `<light>` elements estimated from ZED Mini auto-exposure metadata, the ALBERC robot via `<include>` referencing the existing URDF-derived SDF, and ground plane / physics plugins.
- **Per-region models:** Each VLM-labeled region (kitchen, hallway, etc.) is a separate Gazebo model with its own `model.config`, enabling selective loading and independent physics interaction.
- **Sensor alignment:** Simulated sensor plugins in the SDF world produce topic names and frame IDs identical to the physical hardware, so the same ALBERC launch files work in both real and reconstructed-world simulation.

#### Temporal and 4D Data Export

For time-series analysis and temporal visualization of the mapping session:

- **Timestamped point cloud sequences:** The `pointclouds/sequence/` directory contains PLY (or PCD) files at configurable intervals (default: every 10 seconds of the mapping session). The `sequence_index.json` maps each timestamp to its filename, the robot's 6-DOF pose at that instant, and the incremental coverage percentage. This enables replay of the environment's geometric evolution in tools like CloudCompare's temporal viewer or custom Three.js animations.
- **ROS bag clips:** The Export Wizard allows selecting a time range from recorded bag data (leveraging the existing circular buffer and mission-triggered recording systems). The selected range is extracted as an MCAP file in `bags/`, with configurable topic selection (default: core sensors + navigation). MCAP is the standard ROS2 Humble bag storage format and has reader libraries in Python, C++, and TypeScript.
- **Trajectory in TUM format:** Robot poses at the SLAM keyframe rate are exported in TUM trajectory format (`timestamp tx ty tz qx qy qz qw`), the de facto standard for trajectory evaluation tools (`evo`, `rpg_trajectory_evaluation`). This makes ALBERC's mapping data directly usable for SLAM benchmarking and odometry accuracy analysis.

#### Dashboard Export UI

**Export Wizard** — A multi-step dialog accessible from the Map Manager panel and a dedicated "Export" tab:

| Step | Function | Details |
|---|---|---|
| 1. Select Data | Choose map session, floor(s), time range | Miniature Three.js 3D preview of selected data. Displays point count, coverage %, semantic region count, and estimated export size. |
| 2. Choose Profile | Select target environment | Five profile cards (Isaac Sim / Gazebo / Visualization / Blender / Web Viewer) with format and content summaries. "Custom" option opens per-category toggles. |
| 3. Configure | Toggle data inclusions | Per-category controls: 3D clouds, 2D grids, meshes (quality tier), textures, depth maps, all raw sensor data (L1/ZED/IMU/ultrasonic/encoders/phone/Arduino), segmentation, semantic annotations, tracking history, navigation state, trajectory + odometry, motor telemetry, Jetson telemetry, diagnostics, robot model, bag clip (+ time range), audio (privacy opt-in). |
| 4. Preview & Export | Review and download | Full file tree preview with per-file size estimates. Total ZIP estimate. Progress bar: querying data -> reconstructing meshes -> converting formats -> assembling metadata -> compressing. Download link on completion. |

**Export History** — A panel below the Export Wizard displaying past exports:

| Column | Content |
|---|---|
| Timestamp | ISO 8601 export creation time |
| Name / Tags | User-assigned name and tags for organization |
| Profile | Target environment profile used |
| Map ID | Source map session identifier |
| Size | ZIP file size |
| Actions | Re-download (if still on NVMe SSD), re-export (same configuration), delete (free storage), rename/tag |

Storage: export files are retained on the 1TB NVMe SSD with configurable retention (default 7 days, adjustable via `ALBERC_EXPORT_RETENTION_DAYS`). Maximum storage allocation configurable via `ALBERC_EXPORT_MAX_STORAGE_GB` (default 50 GB). The Export History panel displays current storage usage.

#### export_manager_node ROS2 Service Interface

```
# Service: /export/create
# Triggers a new export job. Returns immediately with a tracking ID.
Request:
  string   profile              # isaac_sim | gazebo | visualization | blender | web_viewer | custom
  string   map_id               # Map session identifier
  int32[]  floors               # Floor indices to include (empty = all)
  bool     include_pointclouds  # 3D point clouds (full + per-layer)
  bool     include_maps         # 2D occupancy grids + costmap
  bool     include_meshes       # Poisson-reconstructed meshes
  bool     include_textures     # ZED keyframes + texture atlas
  bool     include_depth        # ZED depth keyframes
  bool     include_segmentation # All segmentation source outputs
  bool     include_semantic     # VLM/LLM annotations
  bool     include_tracking     # Object tracking history
  bool     include_navigation   # Planned/executed paths, evasion events
  bool     include_trajectory   # TUM poses, SLAM graph, odometry
  bool     include_sensors      # Raw sensor data (all hardware sources)
  bool     include_motor        # Motor telemetry + cmd_vel history
  bool     include_jetson       # Jetson compute telemetry
  bool     include_diagnostics  # Node status, topic rates
  bool     include_robot_model  # URDF + converted model + TF tree
  bool     include_bag_clip     # ROS bag clip
  float64  bag_start_time       # Unix timestamp (if include_bag_clip)
  float64  bag_end_time         # Unix timestamp (if include_bag_clip)
  bool     include_audio        # Voice transcripts + audio (privacy-sensitive)
  string   mesh_quality         # low | medium | high
  float32  decimation           # 1.0 = full, 0.5 = half, 0.25 = quarter
  string   coordinate_frame     # map | enu
  string   name                 # User-assigned export name (optional)
  string[] tags                 # User-assigned tags (optional)
Response:
  string   export_id            # UUID for tracking
  bool     success
  string   message

# Service: /export/status
Request:
  string   export_id
Response:
  string   stage                # queued | meshing | converting | compressing | complete | failed
  float32  progress             # 0.0 to 1.0
  string   download_path        # NVMe SSD path (populated on completion)
  uint64   file_size_bytes

# Service: /export/list
Response:
  ExportEntry[] exports         # Array of {export_id, name, profile, map_id, timestamp, size, status}

# Service: /export/delete
Request:
  string   export_id
Response:
  bool     success

# Topic: /export/progress (std_msgs/String, JSON-encoded)
# Published at 1 Hz during active export for dashboard progress bar
```

---

## Hardware

### Component Summary

| Component | Qty | Spec Summary | Tier | Interface |
|---|---|---|---|---|
| Jetson Orin Nano Super | 1 | 67 TOPS, 1024 CUDA, 8GB LPDDR5 102GB/s | L2 | DC jack (5V buck) |
| Arduino Mega 2560 | 1 | ATmega2560 16MHz, 54 DIO, 6 ISR pins | L2 | USB from Jetson |
| Unitree L1 4D LiDAR | 1 | 360 deg x 90 deg, 21.6K pts/s, 0.05-30m, 6W/12V | L3 | 12V barrel + USB data |
| ZED Mini | 1 | 2x4MP stereo, 63mm, VIO +/-1mm, 800Hz IMU | L3 | USB 3.0 |
| ICM-20948 (SparkFun) | 1 | 9-axis, DMP, gyro +/-2000dps, accel +/-16g | L2 | I2C D20/D21 (3.3V) |
| HC-SR04 | 4 | 2-400cm, 15 deg beam, 40kHz | L2 | Arduino GPIO D22-D29 |
| TS-25GA370 + encoder | 2 | 12V, 1:45, 130RPM, 540PPR output | L1 | L298N + ISR D2/3/18/19 |
| L298N | 2 | 2A/ch, ~2.4V drop, per-motor | L1 | Arduino D4-D9 |
| 4S LiPo 4000mAh 60C | 1 | 14.8V nom, 59.2 Wh, 240A burst | L1 | Both bucks |
| 12V Buck | 1 | 4S->12V, L298N + LiDAR | L2 | -- |
| 5V Buck-Boost | 1 | 4S->5V, LCD, reverse prot. | L2 | Jetson DC jack |
| Smartphone (docked) | 1 | Android/iOS, GPS, mic, speaker, hotspot, BT | L3 | WiFi WebSocket |
| 1TB NVMe SSD | 1 | PCIe Gen3 x4, M.2 Key-M | L2 | Jetson M.2 slot |

All SDKs (JetPack, ZED SDK, Unitree L1 SDK) and Arduino serial communication are configured and operational.

**Charging:** The 4S LiPo is accessible for removal and external balance charging (standard RC workflow). An onboard XT60 charge connector on the L1 chassis side panel allows charging without battery removal - the balance charger connects directly through the chassis. Both charge paths use the same balance lead for cell-level monitoring. The robot must be powered off during charging (SW1 off).

**Networking:** The Jetson Orin Nano Super Dev Kit includes an M.2 Key-E WiFi/BT module providing WiFi (802.11ac/ax) and Bluetooth 5.x. The Jetson connects to infrastructure WiFi for dashboard access, laptop offload DDS, and cloud LLM API calls. When no infrastructure WiFi is available, the docked phone's hotspot provides network connectivity. Automatic failover: if infrastructure WiFi drops, the Jetson reconnects to the phone hotspot within 10s.

### Power Architecture

```
              +-------------------------------------------------------+
              |       4S LiPo BATTERY (Level 1)                       |
              |       4000mAh 60C -- 14.8V nom / 16.8V full / 59.2Wh |
              +------+----------------------+-------------------------+
                     |                      |
                     |    +-----------------+
                     |    |   SAFETY
                     |    |   +-- SW1: Main power (battery disconnect)
                     |    |   +-- SW2: Motor enable (12V rail)
                     |    |   +-- SW3: Compute enable (5V rail)
                     |    |   +-- Tether loop: emergency kill (all rails)
                     |    +-----------------+
                     |                      |
                     v                      v
            +----------------+    +------------------------+
            | 12V Buck Conv. |    | 5V Buck-Boost Conv.    |
            |                |    | (LCD V/I display,      |
            |                |    |  reverse protection)   |
            +--+-----+----+-+    +--+----------------------+
               |     |    |         |
               v     v    v         v
           +-----++-----++----+  +---------------------------+
           |L298N||L298N|| L1 |  | Jetson Orin Nano Super    |
           | #1  || #2  ||12V |  | (7-25W)                   |
           +--+--++--+--+|bar |  |                           |
              v      v   +----+  | USB power from Jetson:    |
          +------++------+       | +-- Arduino Mega 2560     |
          |Mot L ||Mot R |       | +-- ZED Mini (1.9W)       |
          +------++------+       | +-- L1 USB data           |
                                 |                           |
                                 | Arduino GPIO power:       |
                                 | +-- ICM-20948 (3.3V)      |
                                 | +-- HC-SR04 x4 (5V)       |
                                 +---------------------------+

    12V rail: ~1.5A typ (LiDAR 0.5A + motors 0.5A + L298N)
    5V rail:  ~2.5A typ (Jetson + USB devices)
    Total:    ~21W typical / ~42W peak
    Runtime:  ~2.8h typical / ~1.4h peak (59.2 Wh battery)
```

### Mechanical Design

ALBERC is a three-tier 3D-printed PLA chassis (~25 x 20 x 25 cm) manufactured on a Prusa MK3S+/MK4 FDM printer, designed in Autodesk Fusion 360, and validated with ANSYS Mechanical for static structural integrity under full component weight plus dynamic forces during acceleration and braking, and ANSYS thermal transient analysis confirming no structural member exceeds PLA's glass transition temperature (~60 deg C) under sustained worst-case thermal load from the Jetson (25W), L1 4D LiDAR (6W), L298N drivers, and buck converters. Level 1 (bottom) houses the drivetrain: TS-25GA370 motors, ball caster, L298N boards, and 4S LiPo centrally mounted for low CG. Level 2 (middle) is the electronics deck: Arduino Mega, Jetson with active fan, buck converters, four flush-mounted HC-SR04, ICM-20948 centered away from magnetic sources, and ventilation slots. Level 3 (top) mounts the ZED Mini forward-facing, the Unitree L1 4D LiDAR on a 15 deg forward-inclined platform with full hemispherical clearance (the incline improves close-range floor-level detection, compensated in the URDF static TF), and the smartphone dock.

### Grounding

All grounds common: battery negative, 12V buck, 5V buck, Jetson, Arduino, both L298N, LiDAR, ultrasonics, IMU. Without a common ground reference, logic signals become unreliable and sensor interfaces fail in unpredictable ways.

### Safety Switches

ALBERC implements a four-element manual safety system:

| Switch | Function | Location | Effect |
|---|---|---|---|
| SW1 - Main Power | Battery disconnect | L1 chassis side, accessible | Kills all power. Full system shutdown. |
| SW2 - Motor Enable | 12V rail isolate | L1 chassis side, next to SW1 | Disables motors and LiDAR motor. Compute stays live. Safe for development. |
| SW3 - Compute Enable | 5V rail isolate | L2 deck edge | Disables Jetson and all USB-powered devices. Battery and 12V rail remain connected for motor testing. |
| Tether Loop | Emergency kill | Lanyard attached to battery connector | Physical pull disconnects battery from both rails. Unconditional all-power kill. For runaway scenarios. |

SW1 + SW2 + SW3 are toggle switches with visual indicators (LED or mechanical flag). The tether loop is a physical connector in the battery positive lead that separates under tension - no electronics involved, no failure mode other than mechanical.

### Phone Integration

The docked smartphone is mounted on the L3 tier via a velcro pad and elastic retention strap - a universal mount that accommodates any phone size without custom hardware. The phone serves as ALBERC's face, audio interface, and auxiliary sensor package. It connects to the Jetson via WiFi (infrastructure or phone's own hotspot) and communicates over WebSocket.

| Phone Capability | ROS2 Integration | Use |
|---|---|---|
| GPS | -> /gps/fix (NavSatFix) via gps_bridge_node | Outdoor localization, coarse indoor position |
| Microphone | -> audio_bridge_node (16 kHz WebSocket) | Voice commands, wake word detection |
| Speaker | <- tts_node (synthesized speech WebSocket) | TTS output, alerts, announcements |
| WiFi Hotspot | Network source for Jetson | Field operation without infrastructure WiFi |
| Bluetooth | Future peripheral bridge | BLE beacons, external sensors |
| Display | <- phone_face_node (PWA WebSocket) | Animated face, status bar, chat transcription |

---

## Docker

### Container Architecture

ALBERC uses a multi-stage Docker build to produce lean, reproducible deployment images for both Jetson (ARM64) and x86 development environments.

```
+----------------------------------------------------------+
|  alberc-base                                              |
|  Ubuntu 22.04 + ROS2 Humble + Nav2 + common deps         |
+----------------------------+-----------------------------+
|  alberc-jetson              |  alberc-dev-x86             |
|  JetPack 6.x + CUDA 12.x   |  CUDA 12.x (x86)           |
|  ZED SDK (ARM64)            |  ZED SDK (x86 sim)          |
|  L1 SDK (ARM64)             |  L1 SDK (x86 sim)           |
|  TensorRT (ARM64)           |  Gazebo Fortress             |
+----------------------------+-----------------------------+
|  alberc-app                 |  alberc-app                  |
|  ALBERC ROS2 packages      |  ALBERC ROS2 packages        |
|  Web dashboard build        |  Web dashboard build          |
|  TensorRT .engine models    |  PyTorch model fallback       |
+----------------------------+-----------------------------+
```

### Jetson Deployment

```bash
# Build on Jetson (or cross-compile with QEMU)
docker compose -f docker/docker-compose.jetson.yml build

# Launch with full device access
docker compose -f docker/docker-compose.jetson.yml up

# Override to offload specific nodes to laptop
ROS_DOMAIN_ID=42 OFFLOAD_NODES="rtabmap_node,vlm_context_node" \
  docker compose -f docker/docker-compose.jetson.yml up
```

The Jetson container runs with `--runtime=nvidia`, mounts `/dev` for USB device access (Arduino, ZED, L1), and uses host networking for ROS2 DDS discovery.

### x86 Development

```bash
# Build development container
docker compose -f docker/docker-compose.dev.yml build

# Launch with Gazebo simulation
docker compose -f docker/docker-compose.dev.yml up

# Attach to running container for development
docker exec -it alberc-dev bash
```

Two simulation backends are supported:

**Gazebo Fortress (Default, Free):**
The x86 container replaces physical sensor drivers with Gazebo Fortress simulation plugins. The simulation environment includes:
- **World:** Configurable apartment model (default: single-floor 3-room apartment with furniture, doors, and household objects). Additional worlds: empty room (testing), multi-room office, outdoor courtyard.
- **Robot model:** URDF-derived SDF with accurate collision geometry, differential drive plugin, and simulated weight/inertia from ANSYS data.
- **Simulated sensors:** L1 4D LiDAR (GPU-accelerated ray casting, 360 x 90 deg, 21.6K pts/s, +-2cm noise), ZED Mini stereo camera (640x480 simulated stereo pair with depth), ICM-20948 IMU (100 Hz with configurable noise/bias), HC-SR04 ultrasonics (4x ray-based range sensors), wheel encoders (tick-accurate with configurable slip).
- **Dynamic actors:** Simulated walking persons and pets for evasion testing.

The full ROS2 graph, dashboard, and LLM stack run identically to hardware deployment. Sim-to-real transfer is validated by comparing navigation metrics (path tracking error, goal success rate) between simulation and hardware.

**NVIDIA Isaac Sim (High-Fidelity, GPU-Accelerated):**
For research requiring photorealistic rendering, physics-accurate contact dynamics, or domain randomization for RL training:
- **Renderer:** RTX-accelerated ray tracing. Photorealistic indoor scenes with accurate lighting, reflections, and materials.
- **Physics:** PhysX 5 with accurate differential drive dynamics, wheel-surface contact, and object interaction.
- **Sensor models:** Physically-based LiDAR simulation (accounts for material reflectivity, multi-path), stereo camera with realistic noise model, IMU with temperature-dependent drift.
- **RL integration:** Isaac Gym interface for massively parallel RL training (evasion policy, social navigation) with thousands of simultaneous environments on a single GPU.
- **ROS2 bridge:** Isaac Sim's `ros2_bridge` extension publishes identical topics to the real hardware stack. The same ALBERC launch files work with both Gazebo and Isaac Sim - only the simulator backend changes.

Isaac Sim requires an NVIDIA RTX GPU (3070+) on the development laptop. Gazebo runs on any x86 machine.

**Real-World Map Import:** ALBERC's [Export & File Management](#export--file-management) system produces USD scene packages from real-world mapping sessions that load directly into Isaac Sim. The export includes Poisson-reconstructed meshes with ZED Mini sRGB textures, Isaac Sim semantic schema tags matching VLM-derived classifications, and optionally the ALBERC robot model as a referenced USD asset. This enables a sim-to-real-to-sim round-trip: map a real environment with ALBERC, export as USD, load in Isaac Sim, and train RL policies (evasion, social navigation) in the real-world-derived scene geometry via Isaac Gym — then deploy the learned policies back to the physical robot.

### Environment Variables

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | ROS2 DDS domain for multi-machine isolation |
| `ALBERC_LLM_PROVIDER` | `local` | Active LLM backend: local / openai / anthropic / google |
| `ALBERC_EXPORT_RETENTION_DAYS` | `7` | Days to retain export ZIP files on NVMe SSD before auto-deletion |
| `ALBERC_EXPORT_MAX_STORAGE_GB` | `50` | Maximum SSD space (GB) allocated to export files |
| `ALBERC_LLM_MODEL` | `qwen2-vl-7b` | Model name within the selected provider |
| `ALBERC_PLANNER` | `astar` | Default global planner: astar / rrtstar / voronoi |
| `ALBERC_VLM_ENABLED` | `true` | Enable/disable VLM compound view labeling |
| `ALBERC_VLM_RATE` | `1.0` | VLM query rate in Hz |
| `OFFLOAD_NODES` | `""` | Comma-separated list of nodes to skip on this machine |
| `OPENAI_API_KEY` | -- | OpenAI API key (if using OpenAI provider) |
| `ANTHROPIC_API_KEY` | -- | Anthropic API key (if using Claude provider) |
| `GOOGLE_API_KEY` | -- | Google API key (if using Gemini provider/VLM) |

---

## Distributed Computing

### Jetson-First Design

All ROS2 nodes are designed to run on the Jetson Orin Nano as the default deployment target. The Jetson is the single-machine reference configuration - if no laptop is available, the full system runs on the Jetson alone within its 8GB memory and 67 TOPS compute budget.

However, the ROS2 DDS communication layer is inherently distributed. Any node can be launched on any machine on the same network with the same `ROS_DOMAIN_ID`, with zero code changes. This is a launch-time configuration decision, not an architectural fork.

### Multi-Machine Launch

To offload compute-heavy nodes to a laptop:

```bash
# On Jetson: launch all nodes except offloaded ones
OFFLOAD_NODES="rtabmap_node,vlm_context_node,detection_node" \
  ros2 launch alberc_bringup alberc.launch.py

# On Laptop: launch only the offloaded nodes
ROS_DOMAIN_ID=0 \
  ros2 launch alberc_bringup alberc_offload.launch.py \
    nodes:="rtabmap_node,vlm_context_node,detection_node"
```

Typical offload candidates:
- `rtabmap_node` - memory-intensive 3D SLAM (benefits from >8GB RAM)
- `vlm_context_node` - cloud API calls (benefits from lower-latency internet on laptop)
- `detection_node` - can run on laptop GPU if Jetson GPU is saturated
- `vr_stream_node` - Draco compression is CPU-intensive

The laptop receives sensor data over WiFi/Ethernet via DDS and publishes results back. Latency overhead is typically 1-5ms on a local network, acceptable for all non-safety-critical nodes. Safety-critical nodes (`reactive_evasion`, `arduino_bridge`) always run on the Jetson.

### Testing Playground

The testing playground is a development environment that provides access to live robot sensor data for experimentation without risk to the production ROS2 graph.

**Architecture:** A ROS2 topic bridge republishes all sensor topics (`/cloud`, `/zed/*`, `/odom`, `/map`, `/tracked_objects`) to a namespaced `/playground/*` topic space. Development code subscribes to `/playground/*` topics and publishes to `/playground/cmd_vel` (which is NOT bridged back to the real `/cmd_vel` by default - explicit opt-in required).

**Modes:**
- **Live on-bot:** Playground nodes run on the Jetson alongside production nodes. Useful for testing new perception algorithms on real sensor data.
- **Offloaded to laptop:** Playground topics are bridged to a laptop over DDS. Full access to live data with laptop compute resources (larger GPU, more RAM, internet access).
- **Recorded playback:** ROS2 bag files recorded on the bot are replayed on a laptop. Fully offline development.

### Bot Projects

Bot projects are reusable, deployable bundles of code that extend ALBERC's capabilities. A project is a directory containing any combination of:

- **Scripted missions:** YAML or Python-defined sequences of Nav2 goals, LLM queries, and sensor captures. Example: "patrol these 5 waypoints, photograph each, ask LLM for anomalies."
- **ROS2 packages:** Custom nodes that plug into the existing graph. Example: a custom object detector for a specific use case.
- **Jupyter notebooks:** Interactive analysis running on the testing playground data. Example: visualizing SLAM pose graph convergence.
- **Custom code:** Arbitrary Python/C++ that uses the ROS2 API. Example: a data collection script for ML training.
- **Export profiles:** Pre-configured export configurations bundled with projects. A project can define what data to export and in which format upon mission completion. Example: a room scanning mission that automatically exports a USD scene of the scanned area for Isaac Sim import.

Projects are deployed via the dashboard or CLI:

```bash
# Deploy a project
ros2 run alberc_projects deploy --project ~/projects/kitchen_inventory

# List running projects
ros2 run alberc_projects list

# Stop a project
ros2 run alberc_projects stop kitchen_inventory
```

---

## Integration Notes

### Known Risks and Mitigations

The following table consolidates all identified risks across power, sensors, software, and mechanical domains.

| Risk | Severity | Mitigation | Status |
|---|---|---|---|
| No cliff sensor - LiDAR and ultrasonics cannot detect downward drop-offs | CRITICAL | Add downward IR cliff sensor to L1 tier | Requires hardware addition |
| 4S LiPo 12V buck dropout - below 12.8V pack voltage, buck loses regulation; LiDAR resets, motors lose torque | HIGH | Software cutoff at 13.2V (3.3V/cell). Hardware voltage alarm on balance lead | Design complete |
| PLA thermal near Jetson/L1 - PLA Tg ~60 deg C, Jetson 25W + L1 6W + L298N + bucks all generate heat | HIGH | ANSYS thermal transient validated. Ventilation slots in L2/L3. Active Jetson fan | Validation pending |
| Arduino power loss on Jetson reboot - USB-powered Arduino dies, all motors/sensors/safety lost | MEDIUM | Consider independent 5V backup. Implement watchdog timeout (motors stop if no command for 500ms) | Design consideration |
| ZED SDK proprietary lock-in - no open-source depth path; JetPack update can break SDK | MEDIUM | Pin ZED SDK version. Test updates on spare SD before deploying | Process established |
| LLM hallucination - spatial claims not grounded in sensor data | MEDIUM | All LLM spatial assertions verified against depth data before navigation. LLM never in control loop | Design complete |
| 8GB GPU memory pressure - ZED SDK + TensorRT + RTAB-Map + SDK segmentation compete for GPU memory | MEDIUM | Profile per-node. Set RTAB-Map memory limits. Disable RTAB-Map during LLM-heavy tasks. Offload to laptop | Design complete |
| RTAB-Map RAM - 3D SLAM can consume unbounded memory on long sessions | MEDIUM | WM/LTM memory management. Hard memory limit. Session checkpointing | Design complete |
| LiDAR 12V rail sag on motor transients - sudden motor acceleration drops 12V rail, L1 resets | MEDIUM | 470uF+ bulk capacitor at L1 barrel input. Short, thick power wiring | Design complete |
| MPPI with dense 3D costmap - L1's 360 x 90 deg projection creates denser costmap than 2D LiDAR | MEDIUM | Tune MPPI sample count and costmap resolution from Nav2 defaults. Profile incrementally | Process established |
| 5V buck-boost sag to Jetson - 25W peak = 5A at 5V; sag below 4.75V causes throttling or reboot | MEDIUM | Solder pot after adjustment. Monitor via LCD. Adequate current rating | Design complete |
| No bump sensor - no physical contact detection | LOW | Add perimeter microswitches or FSR in future revision | Future hardware |
| ZED rolling shutter - skew during fast rotation degrades depth + VIO | LOW | Limit angular velocity during mapping. L1 mechanical scan more reliable during fast motion | Design complete |
| ICM-20948 magnetometer unreliable near motors/LiPo/converters | LOW | Use accel+gyro only for EKF. Heading from SLAM + VIO. Magnetometer disabled by default | Design complete |
| L298N voltage drop ~2.4V - reduces effective motor terminal voltage | LOW | Acceptable for current phase. Future upgrade to MOSFET-based driver | Known limitation |
| HC-SR04 diagonal blind spots - cardinal-only coverage; 45 deg approach undetected | LOW | L1 360 deg compensates above ultrasonic plane. Ultrasonic is near-field safety only | Mitigated by L1 |
| L1 near-blind-zone below 0.05m - accuracy degrades | LOW | 15 deg incline improves floor coverage. Verify front near-field overlap with ultrasonic experimentally | Validation pending |

### Sensor Calibration

- **L1 4D LiDAR:** 15 deg forward incline is defined as a static TF in the URDF (`lidar_link` -> `base_link`). Verify incline angle physically and in TF with `ros2 run tf2_tools view_frames`.
- **ZED Mini:** Factory-calibrated stereo. Run ZED SDK's self-calibration on first deploy and after any physical remount. Extrinsic TF (`camera_link` -> `base_link`) must match physical mount.
- **ICM-20948:** Gyro bias calibrated on startup (hold still for 5s). Accelerometer calibrated via 6-position static calibration. Magnetometer disabled (unreliable near motors).
- **Wheel odometry:** Measure actual wheel diameter and track width. Calibrate by driving known distances and comparing encoder-derived odometry to ground truth.
- **Ultrasonic:** Verify each sensor individually against known distances. Confirm no cross-talk with staggered timing (minimum 50ms between triggers).

### Thermal Management

The Jetson Orin Nano (7-25W), Unitree L1 4D LiDAR (6W), L298N drivers (1-2W each), and buck converters generate cumulative heat within a compact PLA chassis (Tg ~60 deg C). Thermal management strategy:

- Jetson active fan (always on above 15W TDP)
- L2 and L3 ventilation slots validated by ANSYS thermal transient analysis
- L1 mounted on L3 with open airflow - not enclosed
- L298N positioned on L1 tier with direct access to chassis ventilation
- Software thermal monitoring: Jetson `tegrastats` + Arduino temp sense -> dashboard alerts at 55 deg C enclosure temp
- If thermal limits are approached: reduce Jetson TDP to 15W mode, reduce MPPI sample count, throttle VLM query rate

### Graceful Shutdown

Three shutdown modes are available, configurable from the dashboard:

| Mode | Trigger | Behavior |
|---|---|---|
| **Full Graceful** | Battery < 12.8V, or user command, or dashboard button | 1. Announce via TTS: "Low battery, shutting down." 2. Save SLAM Toolbox 2D map (serialize). 3. Flush RTAB-Map 3D database. 4. Stop bag recording and flush buffers. 5. Stop all motor output (zero PWM). 6. Stop all ROS2 nodes in reverse dependency order. 7. Execute `sudo shutdown -h now`. |
| **Save + Warn** | Configurable battery threshold (default 13.0V) | Save SLAM state and publish low-battery warnings on dashboard and phone. Robot continues operating at reduced speed (0.2 m/s max). User decides when to power off. Risk: unclean shutdown if battery reaches critical. |
| **Return to Dock** | Configurable battery threshold (default 13.5V) | Navigate to the nearest "dock" or "home" waypoint in the semantic map. On arrival, execute Full Graceful shutdown. If navigation fails (goal unreachable), fall back to Full Graceful at current position. |

SW1 (physical power switch) always overrides software shutdown - it immediately kills all power with no graceful sequence. The tether loop has the same effect.

### Data Logging and Bag Recording

Three recording modes are available, all configurable from the dashboard:

| Mode | Behavior | Storage |
|---|---|---|
| **Circular Buffer (Default)** | Continuously records the last N minutes (configurable, default 5 min) of all sensor topics to NVMe SSD. Old data overwritten in ring buffer. Dashboard "Save" button preserves current buffer as a permanent bag file. | ~2 GB / 5 min at full sensor rate |
| **Mission-Triggered** | Automatically starts recording when an active mission/task begins. Stops when mission completes or is cancelled. Each mission gets its own timestamped bag file. | Variable per mission |
| **On-Demand** | Recording starts/stops via dashboard button, voice command ("ALBERC, start recording"), or ROS2 service call. | User-controlled |

**Recorded Topics (configurable):**

| Topic Group | Topics | Default |
|---|---|---|
| Core sensors | /cloud, /zed/rgb/image, /zed/depth, /odom, /imu/arduino | Always |
| Navigation | /map, /cmd_vel, /plan, /tracked_objects | Always |
| Motor | /motor/health, /motor/state | Always |
| Audio | /audio/raw (16 kHz) | Mission-triggered only |
| Full sensor | /zed/left/image_raw, /zed/right/image_raw, all ultrasonics | On-demand only |

Storage is managed on the 1TB NVMe SSD with configurable retention: default keeps last 30 days of bag recordings (oldest auto-deleted), unlimited for saved/named recordings. Dashboard displays storage usage and allows manual deletion.

### Clock Synchronization

Arduino, Jetson, ZED, and L1 all maintain independent clocks. The architecture handles this as follows:

- **Arduino -> Jetson:** rosserial stamps messages on Jetson receipt (2-5ms jitter). EKF process noise covariance accounts for this jitter.
- **ZED Mini:** Internal 800 Hz IMU and camera frames are timestamped by the ZED SDK's internal clock, synchronized to Jetson system time on SDK initialization.
- **Unitree L1 4D LiDAR:** Internal 250 Hz IMU and point clouds are timestamped by the L1 SDK, synchronized to Jetson system time on SDK initialization.
- **Laptop (distributed):** ROS2 DDS uses system time on each machine. If laptop clock drifts, use `chrony` or `ntpdate` to synchronize both machines to the same NTP server.
- **Phone:** GPS timestamps are used for the /gps/fix topic. Phone-to-Jetson audio latency (WebSocket) is measured on connection and compensated in the audio pipeline.

---

## Repository Structure

```
alberc/
+-- docker/
|   +-- Dockerfile.jetson          # ARM64 Jetson deployment image
|   +-- Dockerfile.dev             # x86 development/simulation image
|   +-- docker-compose.jetson.yml
|   +-- docker-compose.dev.yml
+-- src/
|   +-- alberc_bringup/            # Launch files, configs, URDF
|   +-- alberc_perception/         # detection_node, object_tracker, view_compositor
|   +-- alberc_motor/              # motor_mpc_node, motor health monitoring
|   +-- alberc_nav/                # reactive_evasion, planner plugins (RRT*, Voronoi)
|   +-- alberc_exploration/        # explore_lite integration, custom frontier scorer
|   +-- alberc_msgs/               # Custom message types (MotorHealth, MotorState, TrackedObjectArray, etc.)
|   +-- alberc_ai/                 # llm_planner_node, vlm_context_node
|   +-- alberc_voice/              # stt_node, tts_node
|   +-- alberc_phone/              # audio_bridge_node, phone_face_node, gps_bridge_node
|   +-- alberc_ui/                 # web_dashboard_node (React), mobile_ui_node
|   +-- alberc_vr/                 # vr_stream_node (WebXR + Draco)
|   +-- alberc_maps/               # map_manager_node
|   +-- alberc_export/             # export_manager_node, profile configs, format converters (USD, SDF, FBX)
|   +-- alberc_projects/           # project_manager_node, bot project runtime
+-- firmware/
|   +-- arduino_mega/              # Arduino firmware (motor, IMU, ultrasonic, encoder)
+-- models/
|   +-- tensorrt/                  # Compiled .engine model files
|   +-- training/                  # Model training scripts and data
+-- web/
|   +-- dashboard/                 # React dashboard source
|   |   +-- src/components/ExportWizard/   # Export wizard multi-step dialog
|   |   +-- src/components/ExportHistory/  # Export history and file management panel
|   +-- mobile/                    # Mobile-optimized UI
|   +-- phone-face/               # PWA animated face
+-- projects/                      # Bot project templates and examples
+-- docs/
|   +-- datasheets/                # Component datasheets
+-- LICENSE                        # MIT License
+-- README.md                      # This document
```

---

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Follow existing code style and ROS2 package conventions
4. Test on x86 (Docker simulation) before requesting Jetson hardware testing
5. Submit a pull request with:
   - Description of changes and motivation
   - Test evidence (simulation results, logs, or screenshots)
   - Any new dependencies documented in package.xml and Dockerfile

For bug reports and feature requests, open an issue on the GitHub repository.

---

## License

This project is licensed under the **MIT License** - see [LICENSE](LICENSE) for details.

---

## Appendix A: Component Datasheet Specifications

### A.1 Jetson Orin Nano Super Developer Kit

| Parameter | Value |
|---|---|
| AI Performance | 67 TOPS |
| GPU | Ampere, 1024 CUDA cores, 32 Tensor Cores |
| CPU | 6-core Arm Cortex-A78AE v8.2, 1.5MB L2 + 4MB L3 |
| Memory | 8GB 128-bit LPDDR5, 102 GB/s |
| Storage | SD card + M.2 Key-M NVMe (PCIe Gen3 x4) |
| Video Decode | 1x 4K60, 2x 4K30, 5x 1080p60 (H.265) |
| USB | 4x USB 3.2 Gen2 Type-A, USB-C (UFP) |
| Networking | GbE, M.2 Key-E (WiFi) |
| Display | DP 1.2 (+MST) |
| I/O | 40-pin header (UART, SPI, I2S, I2C, GPIO) |
| Power | 7-25W |
| Dimensions | 103 x 90.5 x 34.77mm |

### A.2 Arduino Mega 2560

| Parameter | Value |
|---|---|
| MCU | ATmega2560, 16 MHz |
| Memory | 256KB flash, 8KB SRAM, 4KB EEPROM |
| I/O | 54 digital (15 PWM), 16 analog, 4 UARTs |
| Interrupts | 6 pins (D2, D3, D18, D19, D20, D21) |
| Logic | 5V native |
| Size | 101.5 x 53.3mm |

### A.3 Unitree L1 4D LiDAR

| Parameter | Value |
|---|---|
| Laser | 905nm, Class 1 IEC-60825 |
| Range | 0.05m-30m (90% reflectivity) |
| FOV | 360 deg H x 90 deg V |
| Effective Sampling | 21,600 pts/sec |
| Scanning | Contactless brushless mirror |
| H/V Scan Frequency | 11 Hz / 180 Hz |
| Accuracy / Resolution | +/-2.0cm / 8mm |
| 4D Data | 3D position + 1D grayscale |
| Onboard IMU | 3-axis accel + gyro, 250 Hz |
| Communication | TTL UART 2 Mbps via USB adapter |
| Power | 6W, 12V DC barrel |
| Protection | IP54 |
| Temperature | -10 deg C to 60 deg C |
| Size / Weight | 75x75x65mm / 230g |

### A.4 Stereolabs ZED Mini

| Parameter | Value |
|---|---|
| Sensors | 2x 1/3" 4MP CMOS (2688x1520) |
| Resolutions | 2208x1242@15, 1920x1080@30, 1280x720@60, 672x376@100 |
| FOV | 102 deg H x 57 deg V x 118 deg D |
| Baseline | 63mm |
| Depth | 0.1-15m, <1% @2m, <1.8% @4m |
| IMU | Gyro + Accel, 800 Hz |
| VI-SLAM | +/-1mm position, 0.1 deg orientation, 100 Hz |
| Interface | USB 3.0 Type-C |
| Power | 380mA / 5V (1.9W) |
| Size / Weight | 124.5x30.5x26.5mm / 62.9g |

### A.5 ICM-20948 (SparkFun Breakout)

| Parameter | Value |
|---|---|
| Gyroscope | +/-250/500/1000/2000 dps |
| Accelerometer | +/-2/4/8/16g |
| Magnetometer | +/-4900 uT |
| DMP | Onboard Digital Motion Processor |
| Interface | I2C 400 kHz (breakout level-shifts to 3.3V) |
| Power | 2.5mW |
| Package | QFN-24, 3x3x1mm |

### A.6 HC-SR04

| Parameter | Value |
|---|---|
| Range / Resolution | 2-400cm / 3mm |
| Beam | ~15 deg |
| Formula | cm = echo_us / 58 |
| Supply | 5V, 15mA active |

### A.7 TS-25GA370 Motor + Encoder

| Parameter | Value |
|---|---|
| Voltage / Gear Ratio | 12V / 1:45 |
| Speed / Torque | 130 RPM / 1.2 kg-cm rated |
| Current | <=0.15A no-load, 0.5A max |
| Encoder | Quadrature Hall, 12 PPR motor, 540 PPR output |

### A.8 L298N Motor Driver

| Parameter | Value |
|---|---|
| Supply / Current | 5-35V / 2A continuous, 3A peak per channel |
| Voltage Drop | ~2.4V across bridge |
| Regulator | 78M05 5V/500mA onboard |

### A.9 4S LiPo Battery

| Parameter | Value |
|---|---|
| Configuration | 4S (4 cells series) |
| Capacity | 4000mAh |
| Discharge Rate | 60C continuous (240A burst) |
| Voltage | 14.8V nominal / 16.8V full / 12.0V cutoff |
| Energy | 59.2 Wh |
| Software Cutoff | 13.2V (3.3V/cell) |

### A.10 Buck Converters

| | 12V Buck | 5V Buck-Boost |
|---|---|---|
| Input | 4S LiPo | 4S LiPo |
| Output | 12V regulated | 5V regulated (adjustable) |
| Loads | L298N x2 + L1 4D LiDAR | Jetson Orin Nano Super |
| Features | Standard | LCD display, reverse protection, anti-backflow |

---

## Appendix B: Power Budget

| Component | Typical (W) | Peak (W) | Rail |
|---|---|---|---|
| Jetson Orin Nano Super | 10 | 25 | 5V buck-boost |
| Unitree L1 4D LiDAR | 6 | 6 | 12V buck |
| ZED Mini | 1.9 | 1.9 | 5V (Jetson USB) |
| Arduino Mega 2560 | 0.5 | 0.5 | 5V (Jetson USB) |
| ICM-20948 | 0.003 | 0.003 | 3.3V (Arduino) |
| HC-SR04 x4 | 0.06 | 0.06 | 5V (Arduino) |
| TS-25GA370 x2 (50% duty) | 1.8 | 6.0 | 12V via L298N |
| L298N x2 overhead | 1.0 | 2.0 | 12V buck |
| **TOTAL** | **~21** | **~42** | -- |

| Battery | Energy | Typical Runtime | Peak Runtime |
|---|---|---|---|
| 4S 4000mAh 60C | 59.2 Wh | ~2.8 hours | ~1.4 hours |

---

<div align="center">

**A.L.B.E.R.C.** - Autonomous Learning Bot to Explore, React and Collaborate

Built at LOGOS Robotics Lab, Arizona State University

</div>
