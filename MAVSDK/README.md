# Drone Teleop + Endurance Guard (PX4 + MAVSDK)

Interactive teleoperation with **battery** and **hybrid** endurance guards, **wind modes** (ideal / random),
brief **visual gust** nudges so you can *see* shoves, a stabilizing **position-hold**, and continuous **CSV logging**.
Auto-lands at **30%** SoC (battery) or fuel (hybrid).

> You do **not** need ROS 2 for this script. It talks MAVLink via **MAVSDK (Python)**.
> Use ROS 2 only if you want extra nodes/visualization; it’s optional.

---

## Before you run

### Recommended platform
- Ubuntu **22.04** (or 24.04)
- Python **3.10+**

### Install PX4 SITL (JMAVSim or Gazebo)
PX4 SITL provides the simulated vehicle + MAVLink endpoint.

```bash
# 1) Get PX4 source
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# 2) Install deps (Ubuntu)
bash Tools/setup/ubuntu.sh

# 3) Launch a simulator (opens MAVLink UDP on :14540)
make px4_sitl jmavsim      # or: make px4_sitl gazebo
```

> Running multiple instances? PX4 uses 14540 / 14550 / … sequentially.
> Match the port with `--port` when starting `teleop.py`.

### Set up Python env + dependencies
From your project folder (this repo’s root):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
```

`requirements.txt` contains:
```
mavsdk>=2.4.10
```

The first run may download the `mavsdk_server` binary automatically—let it finish.

### Optional: ROS 2
Not required for this project. If you use it, install ROS 2 Humble (22.04) separately.
This repo does not depend on ROS 2 packages.

---

## Files in this repo

```
.
├── teleop.py              # main script
├── requirements.txt       # Python deps (MAVSDK)
├── README.md              # this file
├── .gitignore             # Python/build/log ignores
├── LICENSE                # (MIT)
└── logs/                  # optional: CSV output folder
```

Create the `logs/` folder if you plan to write logs there, or pass another path with `--log-file`.

---

## Running

1) Start **PX4 SITL** in one terminal:
```bash
cd ~/PX4-Autopilot
make px4_sitl jmavsim    # or: make px4_sitl gazebo
```

2) In another terminal, run the teleop:
```bash
cd <this-repo-root>
source .venv/bin/activate
python3 teleop.py
```

You’ll be prompted to choose:
- **System**: `battery` or `hybrid`
- **Wind mode**: `ideal` (0 m/s) or `random` (re-roll every 5 s, 0–20 m/s)

**Useful options:**
```bash
python3 teleop.py --port 14540                   --speed 1                   --log-file logs/teleop_log.csv                   --log-period 0.5
```
- `--port`        UDP port of PX4 (default **14540**)
- `--speed`       energy sim speed multiplier (default 1×)
- `--log-file`    CSV file path (file is created if missing; create parent folders first)
- `--log-period`  period (in **sim** seconds) between CSV rows

---

## Controls

- **W/A/S/D**: North / West / South / East  
- **R/F**: Up / Down (NED: +Down; R = Up)  
- **J/L**: Yaw left / right  
- **SPACE**: Zero your command **and** reset the hold anchor here  
- **T**: Arm + takeoff + start Offboard  
- **B**: Battery snapshot (SoC, V, A, kW, tilt, wind, sim time, position)  
- **H**: Hybrid snapshot (fuel, kW, L/h, tilt, wind, sim time, position)  
- **G**: Land (manual)  
- **Q**: Quit

---

## What the script does

- **Battery mode**
  - 24S 34Ah pack model (open-circuit voltage + internal resistance)
  - U15II KV80 + ~40" props → hover power via rotor momentum theory
  - Power rises with **tilt**, **user XY command**, and **wind**
  - Auto-lands at **30% SoC** (guard activates ~5 s after takeoff)

- **Hybrid mode**
  - 14 kW generator, fuel burn from electrical power (or fixed L/h)
  - Auto-lands at **30% fuel**

- **Wind**
  - `ideal`: always 0 m/s
  - `random`: every 5 s, random speed 0–20 m/s & direction
  - Brief **visual gust** nudge after each change so you *see* a shove; the hold fights it
  - Power/fuel draw increases with wind

- **CSV logging**
  - Background CSV with: wind, tilt, pose/vel, SoC/Volts/Amps/kW **or** fuel/L·h
  - Events: `wind_config_selected`, `wind_change`, `auto_land_trigger`, `land_complete`, `manual land`

**CSV columns:**
```
event,wall_time,mode,wind_mode,wind_mps,wind_deg,
sim_time_s,in_air,roll_deg,pitch_deg,pos_n,pos_e,vel_n,vel_e,cmd_xy,
soc,batt_v,batt_a,batt_kw,fuel_L,fuel_pct,gen_kw,lph
```

---

## Troubleshooting

- **Drone runs away in wind**  
  - If the simulator already has wind, set `VISUAL_GUSTS = False` in `teleop.py` to avoid double pushing.  
  - Increase hold gains `HOLD_KP/HOLD_KD`, and clamp XY with `VEL_MAX_MS`.

- **Immediate landing**  
  - Make sure you pressed **T** to arm + takeoff + enter Offboard.  
  - The 30% guard only arms after ~5 s (`GUARD_ACTIVATE_GRACE_S`).

- **No CSV file**  
  - Ensure the folder in `--log-file` exists (`mkdir -p logs/`). The script creates the **file**, not parent folders.

- **No connection**  
  - Verify SITL is running on **UDP 14540** (or pass `--port` to match). Only one process can bind a given UDP port.

---

## Pushing this repo to GitHub

Use the helper script in this repo:

```bash
# SSH (recommended) — first add your SSH key in GitHub settings
./push_to_github.sh git@github.com:YOUR_USERNAME/YOUR_REPO.git

# or HTTPS
./push_to_github.sh https://github.com/YOUR_USERNAME/YOUR_REPO.git
```

What it does:
- Initializes git (if needed), commits current files, sets branch to `main`,
- adds/updates `origin`, and pushes `main` to GitHub.

---

## Safety

This script sends **offboard velocity** setpoints. Use a simulator first.
For real vehicles, configure failsafes, RC kill/land, geofence, and have a pilot ready.

---

## License

MIT
