# SIT Airborne — Meituan Drone Simulation (80 kg Class)

> Welcome to the **SIT Airborne** repository — a home for a Meituan-style **80 kg-class drone** simulation built on PX4 + Gazebo with ROS 2 integration. This repo focuses on safe teleoperation, endurance modeling (battery/hybrid), and wind-aware hold behavior so you can prototype and test quickly.

---

## ✨ What’s inside
- **ROS 2 teleop node** (Jazzy) using **MAVROS2** and PX4 **OFFBOARD** velocity control
- **Endurance guard**: auto-lands at 30% remaining (battery or hybrid fuel)
- **Wind modes**: `ideal` (0 m/s) or `random` (0–20 m/s, re-roll every 5 s) with a short visual gust
- **Position hold**: when you’re not commanding, the vehicle fights wind to stay put
- **CSV logging**: wind, pose/velocity, energy/fuel, and events for analysis

---

## 🧱 Tech stack
- **ROS 2 Jazzy** + **MAVROS2**
- **PX4 SITL** (Gazebo, `gz_x500` model)
- Optional **MAVSDK** direct path (no ROS)

---

## 🚀 Quick start (sim)
Open **three terminals**:

**1) PX4 SITL (Gazebo)**
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

**2) MAVROS2 bridge**
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@ gcs_url:=udp://@
```

**3) Teleop node**
```bash
source /opt/ros/jazzy/setup.bash
python3 teleop_ros2_jazzy.py
# choose: system (battery/hybrid) and wind (ideal/random)
# press: T to arm + OFFBOARD
```

**Controls**
- `W/A/S/D` — North/West/South/East  
- `R/F` — Up/Down · `J/L` — Yaw  
- `SPACE` — zero command + reset hold anchor  
- `B`/`H` — snapshots (battery/hybrid) · `G` — land · `Q` — quit

> If you don’t use ROS 2, run the optional **MAVSDK** version: `python3 teleop.py` (after PX4 is up).

---

## 📂 Project layout
```
.
├── teleop_ros2_jazzy.py   # ROS 2 teleop (MAVROS2 + PX4)
├── teleop.py              # optional MAVSDK teleop
├── requirements.txt       # pip deps for MAVSDK path
└── README.md
```

---

## 🧪 Logging
A CSV is created automatically (default `teleop_ros2_log.csv`). If you pass a nested path, make sure the parent folder exists. Columns include time, wind, pose/vel, SoC or fuel, and key events.

---

## 🛠 Troubleshooting (quick hits)
- **GeographicLib error**: `sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh`
- **Not connected**: check `/mavros/state`; use the right port (14540/14550/…)
- **OFFBOARD fails**: stream setpoints ≥ 2 Hz before switching (the teleop does this on **T**)
- **Arming denied** (SITL): at `pxh>` run  
  `param set COM_ARM_WO_GPS 1` · `param set COM_PREARM_MODE 0` · `param set CBRK_USB_CHK 197848` · `param save`
- **Why denied?** echo `/mavros/statustext/recv` or use `commander check` in the PX4 console

---
