# Drone Teleop + Endurance Guard (ROS 2 **Jazzy** + MAVROS2 + PX4 gz_x500)

End-to-end guide to install, run, and troubleshoot a **PX4** simulator driven by a **ROS 2 Jazzy** teleop node via **MAVROS2**.  
Also ships an optional **non‑ROS MAVSDK** teleop for a lighter setup.

**Highlights**
- Keyboard **teleop** with **position‑hold** (stabilizes against wind when you’re not commanding).
- **Battery** and **Hybrid (generator)** endurance models with **30% auto‑land**.
- **Ideal** (0 m/s) or **Random** (0–20 m/s) wind every 5 s + short **visual gust** nudges so you *see* a shove; hold drives back.
- Background **CSV logging** of wind, pose/vel, energy/fuel and events.
- Works with **PX4 SITL Gazebo** `gz_x500` model.

> TL;DR (3 terminals): **PX4 gz_x500 → MAVROS2 → `teleop_ros2_jazzy.py`** → press **T** to arm + OFFBOARD.

---

## Repository layout
```
.
├── teleop_ros2_jazzy.py     # ROS 2 teleop node (MAVROS2 + PX4) – main script
├── teleop.py                 # Non-ROS MAVSDK teleop (optional)
├── hover_common.py           # Shared helpers (if present)
├── requirements.txt          # pip deps for MAVSDK path
├── README.md                 # this file
└── logs/                     # (optional) your log folder
```

---

## 1) Install

### 1.1 ROS 2 **Jazzy** + MAVROS2
```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-mavros ros-jazzy-mavros-extras geographiclib-tools

# make ROS 2 available in each terminal
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
Verify:
```bash
ros2 --version
ros2 pkg prefix mavros
ls $(ros2 pkg prefix mavros)/share/mavros/launch
# You should see launch files like: px4.launch, node.launch, etc.
```

### 1.2 GeographicLib datasets (required by MAVROS)
If missing, `mavros_node` crashes with `GeographicLib exception ... egm96-5.pgm`.
```bash
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
# quick check:
ls /usr/share/GeographicLib/geoids/egm96-5.pgm
```

### 1.3 PX4 SITL (Gazebo, **gz_x500**)
```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh      # first time only
make px4_sitl gz_x500           # launch Gazebo X500 + MAVLink on UDP 14540
```
> Multiple instances increment ports: 14540, 14550, 14560, …

---

## 2) Run (ROS 2 Jazzy path)

Open **three terminals**.

### Terminal #1 – PX4 SITL
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
Optional (recommended in SITL): relax a few checks at the `pxh>` prompt:
```bash
param set COM_ARM_WO_GPS 1
param set COM_PREARM_MODE 0
param set CBRK_USB_CHK 197848
param set NAV_DLL_ACT 0
param save
```

### Terminal #2 – MAVROS2 bridge
```bash
source /opt/ros/jazzy/setup.bash

# Minimal launch (works on Jazzy where 'px4.launch' exists):
ros2 launch mavros px4.launch fcu_url:=udp://:14540@ gcs_url:=udp://@
```
If your install is different (no `px4.launch`), use one of these:

**Option A – Generic MAVROS launch you do have**
```bash
ros2 pkg prefix mavros
ls $(ros2 pkg prefix mavros)/share/mavros/launch  # find what's available
ros2 launch mavros mavros.launch fcu_url:=udp://:14540@ gcs_url:=udp://@
```

**Option B – Run the node directly (no launch)**
```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14540@ -p gcs_url:=udp://@ \
  -p tgt_system:=1 -p tgt_component:=1
```

**Option C – Tiny local launch file**
Create `mavros_px4.launch.py` with a Node running `mavros_node` and params above, then:
```bash
ros2 launch /full/path/to/mavros_px4.launch.py
```

Sanity check MAVROS connection:
```bash
ros2 topic echo /mavros/state --once   # expect: connected: true
```

### Terminal #3 – Teleop node
```bash
source /opt/ros/jazzy/setup.bash
python3 teleop_ros2_jazzy.py
```
Prompts:
- System: `battery` or `hybrid`
- Wind: `ideal` (0 m/s) or `random` (0–20 m/s, **re-roll every 5 s**)

Controls:
- **W/A/S/D** → North/West/South/East (internally ENU ±y/±x)
- **R/F** → Up/Down
- **J/L** → Yaw left/right
- **SPACE** → Zero command **and** reset hold anchor (re‑centers hold)
- **T** → Arm + switch to **OFFBOARD** (script primes setpoints first)
- **G** → Land (manual)
- **B** → Battery snapshot
- **H** → Hybrid snapshot
- **Q** → Quit

**CSV logging**
- Default file printed on startup (e.g., `teleop_ros2_log.csv`).  
  The file is **created if missing**; if you pass a nested path, create the parent folder.
- Columns:
  ```
  event,wall_time,mode,wind_mode,wind_mps,wind_deg,
  sim_time_s,in_air,pos_n,pos_e,vel_n,vel_e,cmd_xy,
  soc,batt_v,batt_a,batt_kw,fuel_L,fuel_pct,gen_kw,lph
  ```
- Events include: `wind_config_selected`, `wind_change`, `offboard_started`,
  `auto_land_trigger`, `land_complete`, `manual land`.

---

## 3) What the teleop models

### Battery (electric)
- 24S 34Ah pack, open‑circuit + sag model.
- U15II KV80 + ~40″ props; hover power from rotor momentum theory.
- Extra draw with **wind** and **XY command** (tilt proxy).
- **Auto‑land at 30%** SoC (armed after a 5 s grace from OFFBOARD).

### Hybrid (generator + fuel)
- 14 kW continuous generator + fuel tank (15 L default).
- Burn rate from electrical power (or optional fixed L/h).
- **Auto‑land at 30%** fuel.

### Wind
- `ideal`: 0 m/s.
- `random`: 0–20 m/s with a new random direction every 5 s.
- A short 1 s **visual gust** nudges the vehicle so drift is visible; position‑hold brings it back.

---

## 4) Troubleshooting (copy‑paste friendly)

### A) MAVROS crashes with GeographicLib error
```
GeographicLib exception: File not readable ... egm96-5.pgm
```
**Fix**
```bash
sudo apt install geographiclib-tools
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
ls /usr/share/GeographicLib/geoids/egm96-5.pgm   # should exist now
```

### B) Arming failed
Often caused by pre‑flight checks, EKF not ready, GPS requirement, or PX4 stuck in `AUTO.LAND`.

1) **Relax checks (SITL only)** – at the PX4 shell (`pxh>`):
```bash
param set COM_ARM_WO_GPS 1
param set COM_PREARM_MODE 0
param set CBRK_USB_CHK 197848
param set NAV_DLL_ACT 0
param save
```
2) **Ensure not in AUTO.LAND** – from a ROS 2 terminal:
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'POSCTL'}"
```
3) **Prime OFFBOARD setpoints** – PX4 requires setpoints before OFFBOARD:
```bash
ros2 topic pub -r 20 /mavros/setpoint_velocity/cmd_vel geometry_msgs/TwistStamped "{}" &
sleep 1
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
```
4) **See the exact reason**:
```bash
# via MAVROS (requires sys_status plugin)
ros2 topic echo /mavros/statustext/recv

# or directly in the PX4 console:
commander check
commander arm
```
5) **Wait for EKF** – give the sim 5–10 s after world start before arming.

### C) `/mavros/statustext/recv` shows nothing
- Make sure the shell is sourced: `source /opt/ros/jazzy/setup.bash`.
- Launch MAVROS with plugin/config YAMLs to ensure `sys_status` loads:
```bash
ros2 launch mavros px4.launch \
  fcu_url:=udp://:14540@ gcs_url:=udp://@ \
  pluginlists_yaml:=$(ros2 pkg prefix mavros)/share/mavros/launch/px4_pluginlists.yaml \
  config_yaml:=$(ros2 pkg prefix mavros)/share/mavros/launch/px4_config.yaml
```
- Then keep `ros2 topic echo /mavros/statustext/recv` running and attempt to arm.

### D) MAVROS launch file not found (`px4.launch.py`)
Different distros ship different names. On Jazzy you typically have **`px4.launch`** (no `.py`). List what you have:
```bash
ls $(ros2 pkg prefix mavros)/share/mavros/launch
```
Use the name you see (e.g., `px4.launch`), or run the node directly:
```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@ -p gcs_url:=udp://@
```

### E) MAVROS not connected
Check:
```bash
ros2 topic echo /mavros/state --once
# connected: false  → wrong port
```
If PX4 is the **second** instance, use **14550**:
```bash
ros2 launch mavros px4.launch fcu_url:=udp://:14550@ gcs_url:=udp://@
```

### F) OFFBOARD won’t engage
- Ensure setpoints are streaming **≥ 2 Hz** before switching to OFFBOARD.  
  The teleop primes them when you press **T**. Test manually with:
  ```bash
  ros2 topic pub -r 20 /mavros/setpoint_velocity/cmd_vel geometry_msgs/TwistStamped "{}"
  ```

### G) Drone “flies away” in wind
- **Position‑hold** is active only when your XY command magnitude is below a small deadband.  
  Hit **SPACE** to zero commands and reset the anchor at the current spot.
- If Gazebo world wind is also enabled, you might be double‑pushing.  
  In the script, set `VISUAL_GUSTS = False` to rely solely on Gazebo wind (or disable Gazebo wind).

### H) CSV not created
- The log **file** is created automatically. If you pass a nested path (e.g., `logs/teleop.csv`), ensure the **parent folder exists**.

### I) ROS 2 environment not active
Every terminal must be sourced:
```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO   # should print 'jazzy'
```

---

## 5) Optional: Non‑ROS MAVSDK teleop

This path skips ROS/MAVROS and talks directly to PX4 over UDP.

```bash
# start PX4 first (gazebo gz_x500, as above)
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt      # contains: mavsdk>=2.4.10 (example)
python3 teleop.py                    # prompts for battery/hybrid + wind
# add --port 14540 if needed
```

---

## 6) Publish to GitHub

```bash
# from your project root (where README.md lives)
git init
git add .
git commit -m "Initial commit: ROS2 Jazzy teleop + docs"
git branch -M main
git remote add origin https://github.com/<your-user>/<your-repo>.git
git push -u origin main
```

---

## Safety

Sim first. For real vehicles: configure failsafes, geofence, RC kill/land, and have a trained pilot. You are responsible for compliance with local regulations.

## License

MIT
