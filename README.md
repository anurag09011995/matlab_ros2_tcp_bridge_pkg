<p align="center">
  <img src="https://raw.githubusercontent.com/beetle_09/matlab_ros2_tcp_bridge_pkg/main/media/banner.png" width="100%" alt="MATLAB ROS2 TCP Bridge Banner">
</p>

# MATLAB–ROS 2 TCP Bridge with Gazebo Simulation
*A full-stack robotics project integrating design, simulation, and remote control through a custom TCP bridge.*

---

## Project Overview

This repository showcases a **cross-platform robotic system** built using **ROS 2 (Humble)**, **Gazebo Classic**, and **MATLAB**, running seamlessly between **WSL2 (Ubuntu)** and **Windows 11**.  
The system enables real-time remote control of a simulated robot using MATLAB via a custom **TCP communication bridge** — without relying on ROS2 DDS.

It demonstrates:
- End-to-end robot development — from CAD to simulation.  
- Networked control and sensor data streaming across environments.  
- Real-time MATLAB-driven actuation and telemetry in Gazebo.  

*The project highlights how advanced robotics workflows can integrate simulation, design, and AI/analytics tools in a single connected ecosystem.*

---

## Engineering Workflow

1. **Design Phase (Fusion 360)**  
   - Modeled the robot body and wheel system from scratch.  
   - Exported precise `.stl` meshes for base link, wheels, and LiDAR.  

2. **Simulation Phase (ROS 2 + Gazebo)**  
   - Created URDF/XACRO files defining links, joints, and materials.  
   - Tuned physical parameters and differential drive motion.  
   - Integrated LiDAR scanning and visualized it in RViz.  

3. **Connectivity Phase (Networking)**  
   - Attempted DDS connection between WSL2 & MATLAB.  
   - Encountered Windows firewall and DDS security conflicts.  
   - Designed a **custom TCP-based bridge** using Python sockets for reliable bidirectional data flow.  

4. **Control Phase (MATLAB Integration)**  
   - Developed MATLAB client scripts to connect, parse telemetry, and send velocity commands.  
   - Validated real-time control — MATLAB commands caused movement in Gazebo.  

*This demonstrates end-to-end control of a ROS2 robot simulation from MATLAB — proving networked robot operation across platforms.*

---

## Tech Stack

| Layer | Technology | Purpose |
|-------|-------------|----------|
| Modeling | Fusion 360 | 3D design and STL export |
| Simulation | Gazebo Classic 11, RViz2 | Physics + visualization |
| Framework | ROS 2 Humble Hawksbill | Node orchestration |
| Control | MATLAB R2022b | TCP client and command generation |
| OS Environment | Windows 11 + WSL 2 Ubuntu 22.04 | Cross-platform setup |
| Networking | Custom TCP sockets | MATLAB ↔ ROS2 communication |
| Languages | Python, MATLAB | Node + client logic |

---

## System Architecture

```
+---------------------------+       TCP        +-----------------------------+
|         MATLAB (Win)      | <--------------> |      ROS 2 Node (WSL2)      |
|---------------------------|   9090 / 9091    |-----------------------------|
| connect_bridge.m          |   bi-directional | tcp_ros_bridge.py           |
| read_telemetry.m          | <---- Telemetry  | Publishes: /odom, /scan     |
| send_cmd_vel.m            | ----> Commands   | Subscribes: /cmd_vel        |
+---------------------------+                  +-----------------------------+
                                                      |
                                                      |
                                                      v
                                              +------------------+
                                              |  Gazebo Simulator |
                                              |  (Robot + LiDAR)  |
                                              +------------------+
```

---

## Project Structure

```
matlab_ros2_tcp_bridge_pkg/
├── config/
│   └── display.rviz
├── launch/
│   ├── gazebo.launch.py
│   ├── display.launch.py
│   ├── rviz.launch.py
│   └── state_publisher.launch.py
├── matlab/
│   ├── connect_bridge.m
│   ├── read_telemetry.m
│   └── send_cmd_vel.m
├── matlab_ros2_tcp_bridge_pkg/
│   ├── __init__.py
│   └── tcp_ros_bridge.py
├── meshes/
│   ├── base_link.stl
│   ├── left_wheel.stl
│   ├── right_wheel.stl
│   └── lidar.stl
├── urdf/
│   ├── materials.xacro
│   ├── mobile2_bot_macro.urdf.xacro
│   ├── mobile2_bot.gazebo
│   └── mobile2_bot.trans
├── worlds/
│   └── room.sdf
├── setup.py
├── package.xml
├── setup.cfg
├── .gitignore
└── README.md
```

---

## Build and Run Instructions

### Build the Package
```bash
cd ~/<your_workspace>
colcon build --packages-select matlab_ros2_tcp_bridge_pkg --symlink-install
source install/setup.bash
```

### Launch Gazebo Simulation
```bash
ros2 launch matlab_ros2_tcp_bridge_pkg gazebo.launch.py
```
Spawns the robot inside `room.sdf` world and initializes LiDAR scanning.

### Start the TCP Bridge Node
```bash
ros2 run matlab_ros2_tcp_bridge_pkg tcp_ros_bridge
```

Expected output:
```
[INFO] [tcp_ros_bridge]: [telemetry] waiting on 0.0.0.0:9090
[INFO] [tcp_ros_bridge]: [commands]  waiting on 0.0.0.0:9091
tcp_ros_bridge up: telemetry@9090  commands@9091
```

### Run MATLAB Control Scripts

Open MATLAB (Windows side) and run the following in order:

First run --> **connect_bridge.m** (connects the bridge of Windows and WSL using TCP connection with an IP address)  
Then --> **read_telemetry.m** (subscribes to `/odom` and `/scan` of the robot)  
Last --> **send_cmd_vel.m** (publishes velocity commands to move the robot)

**Note:**  
Run all the scripts in one MATLAB window.  
For the telemetry file, use a parallel execution method in MATLAB:

```matlab
telemetryThread = parfeval(@read_telemetry, 0);
```

This allows the command window to later execute `send_cmd_vel.m`.

---

## Key Learning Outcomes

Area | What Was Implemented  
-----|-----------------------  
3D Design & CAD | Custom mobile robot modeled and exported from Fusion 360  
Simulation Engineering | URDF/XACRO, Gazebo plugins, and RViz visualization  
Control Systems | Differential-drive and LiDAR data interpretation  
Software Integration | ROS 2 Python node ↔ MATLAB TCP client  
Network Engineering | Cross-platform TCP/IP socket communication  
System Debugging | Mesh path resolution, DDS troubleshooting, and firewall tuning  
Interoperability | MATLAB (Windows) ↔ ROS (WSL2) real-time connectivity  

---

## Real-World Applications

- Cloud robotics and remote operation  
- Digital twin synchronization  
- Networked robotic systems (IoT + AI)  
- Simulation-driven research for Industry 4.0  
- Education in cross-platform robotic communication  

---

## Troubleshooting Tips

Issue | Cause | Fix  
------|--------|-----  
Mesh not loading | Wrong path or `GAZEBO_MODEL_PATH` | Re-export path in `.bashrc`  
Connection closed | TCP socket timed out | Restart bridge + MATLAB script  
MATLAB timeout | Wrong WSL IP | Run `hostname -I` inside WSL  
Gazebo crash | Missing STL | Verify all mesh files exist  
Robot not moving | Global `t_cmd` empty | Run `connect_bridge.m` first  

---

## Repository Metadata

**Repository Name:** `matlab_ros2_tcp_bridge_pkg`

**Short Description:**  
Cross-platform mobile robot simulation in ROS 2 + Gazebo + MATLAB — designed in Fusion 360, controlled over a custom TCP bridge for real-time telemetry and motion.

**Topics:**  
ros2, gazebo, matlab, robotics, tcp-communication, robot-simulation, wsl2, fusion360, urdf, lidar, automation, networked-robots, python, differential-drive, cad-to-simulation, cross-platform-integration, ai-robotics, digital-twin, control-systems

**Keywords:**  
ROS2 Humble · MATLAB TCP bridge · Gazebo simulation · Fusion 360 robot design · WSL2 robotics · LiDAR visualization · Cross-platform control · Networked robot systems

---

## Author

**Anurag Pandey**  
Robotics Engineer | AI & Automation Researcher  
New York, USA  
[anurag.532f@gmail.com](mailto:anurag.532f@gmail.com)  
[LinkedIn](https://www.linkedin.com/in/anuragpandey09/)

---

## License

MIT License © 2025 Anurag Pandey  
Use freely for academic, research, and development purposes.

