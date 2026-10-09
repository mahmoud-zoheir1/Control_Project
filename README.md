# ARL Autonomous Vehicle Control Track

**Autotronics Research Lab (ARL) — Ain Shams University**  
*Course: Autonomous Vehicles & Drive-by-Wire Systems | Individual Project*

<p align="center">
  <img src="assets/demo.gif" alt="Autonomous Vehicle Simulation Demo" width="100%" />
</p>

---

## 🧠 What Is This Project About?

In this project you will build the control system for a self-driving car in a ROS 2 simulation. The car drives around a racetrack and your job is to make it stay on the path, control its speed, and complete laps as fast and accurately as possible.

You will work through a series of milestones, each building on the last:

1. **Explore the system** — Learn what topics the car publishes and subscribes to.
2. **Bring the car to life** — Implement the physics equations that describe how the car moves.
3. **Drive it manually** — Build a keyboard teleoperation node to drive the car yourself.
4. **Add cruise control** — Implement a PID speed controller so the car holds a steady speed.
5. **Make it autonomous** — Implement three different steering controllers (Lateral PID, Pure Pursuit, and MPC) so the car drives itself around the track.
6. **Monitor performance** — Build a lap analyzer that logs lap times, tracking error, and shows live graphs and 3D overlays in RViz.
7. **Report your results** — Compare your controllers and document your findings.

The car model is realistic: it has velocity as a state (not a direct input), meaning it accelerates and decelerates due to drag and friction — just like a real vehicle.



---



---

## 🚀 Quickstart

### 1. Build the Workspace
```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Install build, simulation, and controller dependencies
sudo apt update && sudo apt install -y python3-colcon-common-extensions \
  python3-numpy python3-scipy ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-xacro

# Build the workspace (bicycle_sim, bicycle_control, track_environment)
cd /path/to/bicycle_gym-main
colcon build --symlink-install
source install/setup.bash
```

In every new terminal, source the ROS distribution and built workspace:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/bicycle_gym-main
source install/setup.bash
```

### 2. Launch Modes

| Mode | Launch Command | Section |
|---|---|---|
| **Base Simulation (CLI Testing)** | `ros2 launch bicycle_sim bicycle_sim.launch.py` | Milestones 1 & 2 |
| **Interactive Keyboard Teleop** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop` | Milestones 3 & 4 |
| **Lateral PID (Reactive)** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid` | Milestone 5.2 |
| **Pure Pursuit (Geometric Preview)**| `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit` | Milestone 5.3 |
| **Extended Kinematic MPC (Optimal Preview)** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=mpc` | Milestone 5.4 |

For keyboard teleoperation, start the keyboard driver in a second sourced terminal:

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To request closed-loop cruise control in teleoperation mode:

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop use_cruise_control:=true
```

The launch file also supports `rviz:=false` and `analyzer:=false` to disable those nodes. For example:

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit rviz:=false
```

### 3. Inspecting the ROS Graph

Run these commands from a second sourced terminal while the simulation is running:

```bash
ros2 node list
ros2 topic list
ros2 topic info /state
ros2 topic info /throttle
ros2 topic info /steer
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show std_msgs/msg/Float32
ros2 interface show geometry_msgs/msg/Twist
ros2 topic echo /state
```

### 4. Direct Actuator Testing

With the base simulation running, send actuator commands from another sourced terminal. Throttle/brake uses `[-1.0, 1.0]`; steering is in radians, with positive values turning left.

```bash
ros2 topic pub --once /throttle std_msgs/msg/Float32 "{data: 0.5}"
ros2 topic pub --once /steer std_msgs/msg/Float32 "{data: 0.30}"
ros2 topic pub --once /throttle std_msgs/msg/Float32 "{data: -1.0}"
```

### 5. Real-Time Telemetry & Graphing
```bash
# Install plotting and telemetry visualization tools
sudo apt update && sudo apt install -y ros-humble-plotjuggler-ros rqt-plot

# Inspect live signals in rqt_plot:
ros2 run rqt_plot rqt_plot /telemetry/cte /telemetry/speed

# Or launch PlotJuggler for multi-topic time-series analysis:
ros2 run plotjuggler plotjuggler
```

Some telemetry topics are only available after the corresponding analyzer work is complete.

---

## 🎯 Milestones at a Glance

- **Milestone 1**: Topic Discovery, Graph Inspection & Telemetry Plotting (`ros2 topic list / info`, `rqt_plot`, `plotjuggler`)
- **Milestone 2**: Extended Kinematic Bicycle Model & Euler Integration (`src/bicycle_sim/bicycle_sim/bicycle_model.py`)
- **Milestone 3**: Teleoperation Bridge & Open-Loop Driving (`src/bicycle_control/bicycle_control/teleop_bridge.py`)
- **Milestone 4**: Low-Level Powertrain Cruise Control (`src/bicycle_control/bicycle_control/longitudinal_pid.py`)
- **Milestone 5**: Autonomous Path Tracking — It's Time to Get the Car to Drive Autonomously!
  - **5.1**: High-Level Velocity Profiler & Path Curvature (`src/bicycle_control/bicycle_control/velocity_profiler.py`)
  - **5.2**: Steer Using Reactive Feedback (`src/bicycle_control/bicycle_control/lateral_pid.py`)
  - **5.3**: Steer Using Geometric Preview (`src/bicycle_control/bicycle_control/pure_pursuit.py`)
  - **5.4**: Steer Using Constrained Optimal Preview (Extended Kinematic MPC) (`src/bicycle_control/bicycle_control/mpc.py`)
  - **5.5**: Real-Time Telemetry, Graphing & RViz Dashboard Engineering (`src/track_environment/track_environment/lap_analyzer.py`)
- **Milestone 6**: Free Exploration & Reference Resources (Ackermann Kinematics, 3D Simulation, Nav2 MPPI)
- **Milestone 7**: Deliverable 1 — Repository Documentation (`README.md` Benchmark Report)
- **Milestone 8**: Deliverable 2 — Technical Video Walkthrough (3–5 Minute Demo)

---
## Milestone 6: Free Exploration & Reference Resources

To connect your work in this lab to industrial autonomous vehicle systems, modern simulators, and production ROS 2 frameworks, explore the following organized learning resources. These materials illustrate how the 2D planar kinematic bicycle model extends into multi-body dynamics, 3D physics engines, and advanced sampling-based predictive control.

---

### 1. Four-Wheel Ackermann Kinematics & `ros2_control`
*Explore multi-body steering geometry and industrial ROS 2 controller architectures.*

In a physical 4-wheel vehicle navigating a turn, the inside front wheel must turn sharper than the outside wheel because it follows a smaller turning radius ($R - W/2$ vs $R + W/2$). Forcing both wheels to the same angle causes tire scrub, tread wear, and energy loss.

$$\tan\delta_{inner} = \frac{L}{R - \frac{W}{2}}, \quad \tan\delta_{outer} = \frac{L}{R + \frac{W}{2}}$$

#### Curated Resources:
- [ROS 2 Control Mobile Robot Kinematics Guide](https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html) — Guide on modeling 4-wheel kinematics and visualizing full car models instead of simplified bicycle models.
- [ROS 2 Steering Controllers Library](https://control.ros.org/kilted/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html) — Official documentation for Ackermann and bicycle steering controllers in `ros2_control`.
- [ros2_control_demos Example 11: Steered Wheel Base](https://control.ros.org/humble/doc/ros2_control_demos/example_11/doc/userdoc.html) — Industrial demonstration of steered-wheel bases and hardware interfaces.
- [ros2_control_demos Repository](https://github.com/ros-controls/ros2_control_demos) — Comprehensive reference suite for `ros2_control` implementations.
- [ROS 2 Controllers Official Repository](https://github.com/ros-controls/ros2_controllers/tree/master) — Upstream implementations of vehicle and chassis controllers.
- [Four-Wheel AMR Reference Implementation](https://github.com/abubakar-mughal97/four_wheel_amr) — 4-wheel mobile robot package with Ackermann steering.

---

### 2. Modern 3D Simulation Environments (Gazebo & MVSim)
*Bridge the gap between 2D planar kinematics and full 3D physics engines with tire friction dynamics.*

While kinematic models assume zero tire slip, physical vehicles experience tire deflection and friction saturation (Pacejka Magic Formula). 3D physics engines simulate suspension compliance, tire contact patches, sensor noise, and terrain.

#### Curated Resources:
- [Ackermann Vehicle in Modern Gazebo (Gz-Sim) & ROS 2](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2) ([Main Branch](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2/tree/main)) — Autonomous Ackermann vehicle simulation using modern Gazebo (Gz-Sim / Ignition) and ROS 2.
- [Classic Gazebo Ackermann Simulation](https://github.com/lucasmazzetto/gazebo_ackermann_steering_vehicle) — Classic Gazebo simulation showcasing physical Ackermann steering linkages.
- [Ackermann Autonomous Car Simulation](https://github.com/armando-genis/Ackermann-Autonomous-Car-Simulation) — Autonomous driving stack with Ackermann kinematics in simulation.
- [MVSim — Multi-Vehicle Simulator for ROS 2 Humble](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/MVSim/Simulation-MVSim.html) — Lightweight, fast multi-vehicle dynamic simulator tailored for mobile robots and autonomous vehicles.

---

### 3. Stochastic Sampling-Based Predictive Control (Nav2 MPPI)
*Explore model predictive path integral control for non-linear vehicle systems.*

Model Predictive Path Integral (MPPI) control is an advanced algorithm that generates thousands of randomized candidate trajectories in parallel (using GPU or multi-core CPU) and averages them using path integral weighting to produce optimal controls without needing gradient-based solvers.

#### Curated Resources:
- [Nav2 MPPI Controller](https://index.ros.org/p/nav2_mppi_controller/) — Production real-time MPPI controller package in the ROS 2 Navigation stack with dynamic obstacle avoidance and customizable cost functions.

---

### 💡 Synthesis Task for Your Report:
In your `README.md` report, synthesize your takeaways from exploring these organized resources:
1. **Kinematics vs Multi-Body**: How 4-wheel Ackermann kinematics accounts for differing inner and outer wheel turning radii ($\delta_{inner}$ vs $\delta_{outer}$), and how this is modeled in `ros2_control`.
2. **2D vs 3D Simulation**: The computational and modeling trade-offs between lightweight 2D kinematic simulation and full 3D physics engines (Gazebo / MVSim).
3. **Deterministic vs Sampling Control**: How modern sampling-based controllers (Nav2 MPPI) differ in flexibility, obstacle handling, and compute requirements compared to deterministic optimization (MPC).

---


## 🏆 Telemetry Benchmark Leaderboard

I tested each autonomous controller for three laps using the Lap Analyzer.

| Controller | Best Lap (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps |
|---|---:|---:|---:|---:|---:|---:|
| Lateral PID | 138.80 | 4.07 | 0.2613 | 1.966 | 0.3447 | 3 |
| Pure Pursuit | 130.60 | 4.16 | 0.0367 | 0.380 | 0.0607 | 3 |
| MPC | 122.10 | 4.22 | 0.1010 | 0.539 | 0.1377 | 3 |

MPC achieved the fastest recorded lap, while Pure Pursuit had the lowest tracking error. Lateral PID showed larger errors and some steering oscillations.

The results are based on laps detected by the analyzer. The reported distance per lap differs from the reference track length, so the lap detection still needs verification.



---

# Individual Project Report
**Student:** Mahmoud Zoheir Abdelkader

**Environment:** Ubuntu 26.04, ROS 2 Lyrical, Python

## 1. Project Overview

In this project, I worked on controlling a simulated car using ROS 2. The main goal was to make the car follow a racing track while controlling its speed and steering.

I completed Milestones 1 to 5, including the vehicle model, keyboard control, speed PID, path tracking controllers, and lap analyzer.

Milestone 6 was not completed due to time constraints and the approaching submission deadline. However, I plan to complete it independently after the deadline to further improve my skills and understanding.

## 2. Vehicle Model

I used the extended kinematic bicycle model to simulate the car's motion.

The main equations are:

- x_dot = v cos(yaw)
- y_dot = v sin(yaw)
- yaw_dot = (v / L) tan(steering)
- v_dot = acceleration

The model uses Euler integration to update the vehicle position, heading, and speed.

## 3. Speed Control

I implemented a longitudinal PID controller to maintain the target speed.

The PID uses the speed error to calculate the throttle command, with integral clamping to reduce windup.

During testing, the car tracked a target speed of 3 m/s, with an observed speed of about 3.03 m/s.

I also used a velocity profiler to reduce the target speed in sharp corners based on path curvature.

## 4. Path Tracking Controllers

### Lateral PID

The Lateral PID uses cross-track error (CTE) and heading error to calculate steering.

After tuning the gains, the car followed the track, but some steering oscillations were still noticeable.

### Pure Pursuit

Pure Pursuit selects a point ahead of the car on the reference path and calculates the steering angle needed to reach it.

The lookahead distance changes with vehicle speed.

This controller gave smooth tracking during my tests, with CTE samples around 0.02–0.07 m in one observed section.

### MPC

MPC uses the bicycle model to predict future vehicle motion and select steering and acceleration commands.

I implemented a cost function based on tracking errors, speed error, and control effort, using SLSQP optimization and warm start.

The MPC controller ran successfully in the simulation and followed the track during visual testing.

## 5. Lap Analyzer

The Lap Analyzer monitors:

- Lap time and best lap
- Cross-track error
- Vehicle speed
- Heading error
- Mean, maximum, and RMS CTE

I also added a live telemetry HUD and CTE visualization in RViz.

The telemetry topics were tested successfully. Full-lap statistics still need further verification.

## 6. Controller Comparison

| Feature | Lateral PID | Pure Pursuit | MPC |
|---|---|---|---|
| Method | Feedback | Geometric | Predictive |
| Path preview | No | Yes | Yes |
| Complexity | Low | Low | High |
| Observed behavior | Some oscillations | Smooth tracking | Followed the track |

MPC can potentially achieve better tracking because it predicts future motion and considers vehicle constraints. However, it requires more computation and proper tuning.

Pure Pursuit gave the smoothest visible tracking in my tests.

## 7. Benchmark Results

The three-lap results are shown in the Telemetry Benchmark Leaderboard above.


## 8. Running the Project

I used Ubuntu 26.04 and ROS 2 Lyrical.

Build the workspace:

```bash
cd ~/Control_Project
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run one of the controllers:

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid
```

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit
```

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=mpc
```

Check telemetry:

```bash
ros2 topic echo /lap/metrics
```

## 9. Conclusion

I learned how to connect vehicle modeling, PID control, path tracking, and performance monitoring in ROS 2.

Pure Pursuit showed smooth tracking, while MPC introduced predictive control and optimization. More testing is needed for a full performance comparison.




VIDEO LINK 

Control Project — Video Walkthrough
https://drive.google.com/file/d/17BcCibOjiemiFBWGAGoACcOy7p1OSgfC/view?usp=sharing
## Project Walkthrough Video

[Watch the Control Project Demo](https://drive.google.com/file/d/17BcCibOjiemiFBWGAGoACcOy7p1OSgfC/view?usp=sharing)

## Project Walkthrough Video

[Watch the Control Project Demo](https://drive.google.com/file/d/17BcCibOjiemiFBWGAGoACcOy7p1OSgfC/view?usp=sharing)
