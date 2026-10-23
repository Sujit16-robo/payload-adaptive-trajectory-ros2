
# Payload-Adaptive Trajectory Scaling for a 3-DOF Robotic Arm

A ROS2 Humble + Gazebo Harmonic demonstration of **payload-adaptive trajectory time-scaling**: a middleware node that computes joint torques in real time using closed-form rigid-body dynamics (inertia, Coriolis, gravity terms) and automatically stretches a trajectory's duration just enough to keep every joint within its torque limit — no manual retuning when the payload changes.

## Demo

| Empty Payload | 5 kg Payload |
|---|---|
| ![empty](media/empty_payload.mkv) | ![heavy](media/with_payload.mkv) |

> GitHub does not play `.mkv` inline — download the files from `media/` to view, or see the Results section below for plots.

**Result:** the same trajectory automatically runs slower when carrying a heavier payload, keeping joint torques under their rated limits instead of risking overload at fixed speed.

| Payload | Peak Torque (Baseline) | Peak Torque (Adaptive) | Reduction | Scale factor k |
|---|---|---|---|---|
| Empty (0.001 kg) | 1328.05 N·m | 1102.46 N·m | 17.0% | 1.176 |
| Light (1.5 kg) | 1376.86 N·m | 1120.78 N·m | 18.6% | 1.198 |
| Heavy (5.0 kg) | 1490.83 N·m | 1163.25 N·m | 22.0% | 1.246 |

Live ROS2+Gazebo runs matched these offline-computed values almost exactly (empty k=1.176, heavy k=1.246).

## How it works

1. A trajectory publisher sends a fixed 3-waypoint joint trajectory (same for every run).
2. An **interceptor node** sits between the publisher and the robot. It:
   - Fits a cubic polynomial per trajectory segment (matching position/velocity boundary conditions)
   - Densely samples position, velocity, and acceleration along each segment
   - Computes joint torques at every sample using a hand-derived closed-form dynamics model (no external dynamics library) that includes inertia, Coriolis coupling, and gravity terms, with the actual payload mass as a parameter
   - Finds the worst-case torque/limit ratio across all joints and samples
   - If that ratio exceeds 1.0, computes a time-scale factor `k = sqrt(max_ratio)` and stretches the trajectory's timing by `k` (velocities divided by `k`, time stamps multiplied by `k`)
3. The time-scaled trajectory is sent to Gazebo Harmonic, driving the arm via native joint-position-controller plugins.

This means: same path, same waypoints — only the *speed* changes based on how much the arm is carrying.

## Dynamics model

Derived by hand from the Lagrangian formulation for a 3-DOF arm with the actual CAD-derived mass/inertia properties of each link (see `offline_sim.py` for the full equations). Key terms:

- **Inertia term** — effective inertia changes with arm configuration and payload mass
- **Coriolis term** — velocity-coupling effects between joints during simultaneous multi-joint motion
- **Gravity term** — configuration-dependent gravitational torque (held constant under scaling as a stated approximation — see Limitations)

Torque limits are set to 80% of the URDF's rated effort limits as a safety margin.

## Repository structure
payload_arm/
├── config/ # controller gains (unused directly by Gazebo native plugins, kept for reference)
├── launch/
│ └── gazebo.launch.py # launches Gazebo Harmonic + spawns the robot
├── media/ # demo videos + torque plots
├── meshes/ # CAD-exported STL meshes
├── payload_arm/ # Python nodes
│ ├── interceptor_node.py # core adaptive time-scaling logic
│ ├── trajectory_executor.py # relays scaled trajectory to joint position topics
│ ├── traj_publisher.py # publishes the fixed test trajectory
│ └── data_logger.py
├── offline_sim.py # standalone offline dynamics/torque analysis + plot generation
├── urdf/
│ └── payload_arm.urdf.xacro
├── package.xml
├── setup.py / setup.cfg
└── test/

## Requirements

- ROS2 Humble
- Gazebo Harmonic (`gz-sim` 8.x) — launched via `gz sim`, **not** `ign gazebo`
- `ros_gz_bridge`
- Python 3, NumPy, Matplotlib (for `offline_sim.py`)

## Running the demo

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/payload_arm_ws/install/payload_arm/share

# Terminal 1 — launch Gazebo + spawn robot
ros2 launch payload_arm gazebo.launch.py

# Terminal 2 — interceptor node (set payload mass here)
ros2 run payload_arm interceptor_node --ros-args -p payload_mass:=0.001 -p adaptive_enabled:=true

# Terminal 3 — relay node
ros2 run payload_arm trajectory_executor

# Terminal 4 — send the trajectory (run last)
ros2 run payload_arm traj_publisher
Change payload_mass (e.g. 5.0 for a 5 kg payload) and restart Gazebo between runs to compare scaling behavior. Expected scale factor is logged as:
=== SCALE FACTOR k = 1.176 (adaptive=True, mass=0.001) ===

## Offline analysis
python3 offline_sim.py
Regenerates the torque-vs-time plots in media/ and prints the summary table above.

## Architecture note: why native Gazebo plugins instead of ros2_control
An earlier version used ros2_control + gz_ros2_control, but this segfaulted due to an ABI/symbol mismatch: the system had both Ignition Fortress (gz-sim6) and Gazebo Harmonic (gz-sim8) installed, and the locally built gz_ros2_control plugin was compiled against ignition::gazebo::v6 symbols while Gazebo Harmonic's runtime is gz::sim::v8. This was permanently resolved by switching to Gazebo Harmonic's native JointPositionController and JointStatePublisher system plugins, bridged via ros_gz_bridge with std_msgs/Float64 topics.

## Limitations
1.Validated in simulation only — not yet tested on physical hardware.
2.Control is feed-forward (trajectory pre-scaled before execution), not closed-loop torque monitoring during motion.
3.Payload mass is supplied as a known parameter, not sensed or estimated online.
4.The gravity term is held constant under time-scaling as an approximation; a more rigorous per-joint scaling law (separating gravity-dependent and velocity-dependent torque components) is possible but not implemented here.
5.Mesh visuals were manually fitted in Blender (no CAD-native joint-axis export was available) and may show a minor cosmetic gap at extreme joint angles at the very end of the demo trajectory. This does not affect collision geometry, inertial properties, or any reported torque/dynamics results — all quantitative results are computed independently of the visual mesh.

## Future work
1.Online payload mass estimation via torque residuals (cf. Slotine & Li, 1987, adaptive manipulator control)
2.Closed-loop variant with real-time torque feedback
3.Rigorous per-joint scaling law separating gravity and velocity-dependent torque terms
4.Physical hardware validation
5.CAD-native joint-axis extraction for exact mesh alignment through full range of motion


## License

© 2026 Sujit. All rights reserved. This repository is shared publicly for portfolio and demonstration purposes. Please do not copy, redistribute, or reuse this code without permission.
