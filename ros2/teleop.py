#!/usr/bin/env python3
# ROS 2 (rclpy) teleop for PX4 via MAVROS2
# - Offboard velocity via /mavros/setpoint_velocity/cmd_vel (ENU)
# - Keys: WASD (N/W/S/E), R/F (Up/Down), J/L (yaw rate), SPACE (zero+reset anchor), T (arm+OFFBOARD), G (land), B/H snapshots, Q quit
# - Wind: ideal(0) or random (0..20 m/s, re-roll every 5 s) with 1 s nudge so you *see* the shove; hold fights it
# - Battery: 24S 34Ah U15II model; Hybrid: generator/fuel; both auto-land at 30%
# - CSV logging of wind, pose/vel, SoC/Volts/Amps/kW or Fuel/L·h, events
#
# =============================================================================
# TUNABLES CHEAT SHEET (edit constants in the "Tunables" section below)
# -----------------------------------------------------------------------------
# • Control rates:         COMMAND_HZ, ENERGY_HZ, LOG_HZ
# • Key step sizes:        VEL_STEP_MS, VEL_STEP_UP_MS, YAW_STEP_DEG_S, VEL_MAX_MS
# • Position hold gains:   HOLD_KP, HOLD_KD, CMD_DEADBAND
# • Auto-land behavior:    LAND_AT_PCT, GUARD_ACTIVATE_GRACE_S
# • Wind model:            WIND_MODE (prompted at runtime), WIND_RANDOM_MAX_MPS,
#                          WIND_UPDATE_PERIOD_S, VISUAL_GUSTS, GUST_CMD_GAIN, GUST_BURST_S
# • Energy multipliers:    SPEED_FACTOR, WIND_POWER_PER_MPS, K_VEL_POWER
# • Battery model:         N_CELLS, CAP_AH, CELL_V_NOM, PACK_IR_OHM, DRONE_MASS_BATT_KG,
#                          CELL_V_MAX, CELL_V_MIN, P_IDLE_KW, ETA_PROP, ETA_MOTOR, N_ROTORS, PROP_DIAM_IN
# • Hybrid model:          GEN_CONT_KW, GEN_MASS_KG, FUEL_TANK_L, SFC_KG_PER_KWH, FUEL_DENSITY_KG_PER_L
# • Logging:               default CSV path via --log-file, sample period via --log-period
# =============================================================================

import os, sys, math, time, random, csv, threading, select, termios, tty, contextlib
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped, TwistStamped as TwistStampedMsg
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

# ========================== Tunables ==========================
# ---------- Rates ----------
COMMAND_HZ = 10.0           # Hz: publish /mavros/setpoint_velocity/cmd_vel (increase for tighter control)
ENERGY_HZ  = 20.0           # Hz: battery/hybrid energy integration
LOG_HZ     = 10.0           # Hz: check/write CSV (actual rows respect log_period_s below)

# ---------- Key step sizes (ENU: x=East, y=North, z=Up) ----------
VEL_STEP_MS     = 0.7       # m/s per keypress for E/N axes
VEL_STEP_UP_MS  = 0.7       # m/s per keypress for Up/Down (z)
YAW_STEP_DEG_S  = 15.0      # deg/s per keypress for yaw rate
VEL_MAX_MS      = 3.0       # clamp final horizontal command magnitude (safety/feel)

# ---------- Position-hold gains (used when your XY command is within CMD_DEADBAND) ----------
HOLD_KP = 0.8               # proportional gain (raise to fight wind harder; too high -> oscillation)
HOLD_KD = 1.2               # derivative gain (damping; raise if oscillatory)
CMD_DEADBAND = 0.08         # |cmd_xy| below this engages hold; above it, anchor follows your motion

# ---------- Auto-land guard ----------
LAND_AT_PCT = 0.30          # 30% remaining triggers auto-land
GUARD_ACTIVATE_GRACE_S = 5.0# wait this long after OFFBOARD before enabling auto-land

# ---------- Wind model ----------
WIND_MODE = "ideal"         # default; actual mode is prompted at runtime ("ideal" | "random")
WIND_RANDOM_MAX_MPS = 20.0  # random wind range upper bound
WIND_UPDATE_PERIOD_S = 5.0  # re-roll cadence for random wind
VISUAL_GUSTS = True         # show a 1s nudge right after each re-roll (to visualize shove)
GUST_CMD_GAIN = 0.20        # fraction of wind injected into cmd_vel during the gust
GUST_BURST_S  = 1.0         # seconds: gust display duration

# ---------- Energy multipliers (how wind/command increase power draw) ----------
SPEED_FACTOR    = 1.0       # scales SIM time for energy/fuel; 1.0 = real-time
WIND_POWER_PER_MPS = 0.02   # +2% power per 1 m/s of wind (raise for stronger penalty)
K_VEL_POWER        = 0.05   # +5% per 1 m/s of user XY command (tilt proxy)

# ---------- Battery model (24S 34Ah, U15II ~40" props) ----------
N_CELLS, CAP_AH, CELL_V_NOM = 24, 34.0, 3.7
PACK_KWH = (N_CELLS * CELL_V_NOM * CAP_AH) / 1000.0
CELL_V_MAX, CELL_V_MIN = 4.10, 3.30     # cell open-circuit voltages at 100%/0% SoC
PACK_IR_OHM = 0.03                      # pack internal resistance (affects sag under load)
DRONE_MASS_BATT_KG = 80.0               # all-up mass for battery configuration (kg)
P_IDLE_KW = 0.15                        # power when not flying/offboard (electronics idle)
ETA_PROP, ETA_MOTOR = 0.75, 0.90        # efficiencies -> ETA_TOTAL
ETA_TOTAL = ETA_PROP * ETA_MOTOR
N_ROTORS = 4
PROP_DIAM_IN = 40.0
PROP_DIAM_M  = PROP_DIAM_IN * 0.0254
PER_MOTOR_POWER_CAP_W = None            # e.g., 3000.0 W per motor; None disables
PER_MOTOR_CURRENT_CAP = None            # e.g., 120.0 A per motor; None disables

# ---------- Hybrid model ----------
GEN_CONT_KW, GEN_MASS_KG = 14.0, 29.5
SFC_KG_PER_KWH, FUEL_DENSITY_KG_PER_L = 0.600, 0.74
HOVER_BURN_LPH_FIXED, USE_FIXED_HOVER_BURN = 12.5, False
BASE_AIRFRAME_MASS_KG, FUEL_TANK_L = 50.5, 15.0

# ---------- Air density (ISA sea-level). Change temp/alt to simulate hot/high ----------
def isa_density(temp_C: float = 15.0, pressure_alt_m: float = 0.0) -> float:
    T0,P0,L,g,R = 288.15,101325.0,0.0065,9.80665,287.05
    h = max(0.0, pressure_alt_m)
    P = P0 * (1.0 - L*h/T0) ** (g/(R*L))
    T = temp_C + 273.15
    return P / (R * T)
RHO = isa_density()

# ========================== Helpers ==========================
def hover_mech_power_disk(mass_kg: float, rho: float) -> float:
    T = mass_kg * 9.80665
    A_one = math.pi * (PROP_DIAM_M/2.0)**2
    A_tot = N_ROTORS * A_one
    return (T**1.5) / max(1e-6, math.sqrt(2.0*rho*A_tot))

def hover_electrical_kW(mass_kg: float, rho: float) -> float:
    return hover_mech_power_disk(mass_kg, rho) / max(1e-6, ETA_TOTAL) / 1000.0

def ocv_from_soc(soc: float) -> float:
    soc = max(0.0, min(1.0, soc))
    return (CELL_V_MIN + (CELL_V_MAX - CELL_V_MIN)*soc) * N_CELLS

def solve_current_with_sag(p_elec_w: float, soc: float) -> Tuple[float,float]:
    v_ocv = ocv_from_soc(soc); V = max(1e-3, v_ocv)
    for _ in range(8):
        I = p_elec_w / max(1e-3, V)
        V = max(1e-3, v_ocv - I*PACK_IR_OHM)
    return V, I

def battery_hover_power_kW(soc: float, rho: float) -> Tuple[float,float,float]:
    p_mech = hover_mech_power_disk(DRONE_MASS_BATT_KG, rho)
    p_elec = p_mech / max(1e-6, ETA_TOTAL)
    if PER_MOTOR_POWER_CAP_W is not None:
        p_elec = min(p_elec, PER_MOTOR_POWER_CAP_W*N_ROTORS)
    V, I = solve_current_with_sag(p_elec, soc)
    if PER_MOTOR_CURRENT_CAP is not None:
        I_cap = PER_MOTOR_CURRENT_CAP*N_ROTORS
        if I > I_cap:
            I = I_cap
            V = max(1e-3, ocv_from_soc(soc) - I*PACK_IR_OHM)
            p_elec = V*I
    return p_elec/1000.0, V, I

def fuel_burn_lph_from_power(power_kW: float) -> float:
    return (power_kW * SFC_KG_PER_KWH) / max(1e-6, FUEL_DENSITY_KG_PER_L)

def mmss(tsec: float) -> str:
    tsec = max(0.0, tsec); return f"{int(tsec//60):02d}:{int(tsec%60):02d}"

# ========================== Data classes ==========================
@dataclass
class WindState:
    mode: str = "ideal"
    speed_mps: float = 0.0
    dir_deg: float = 0.0        # TO direction (0=N, 90=E)
    gust_expire: float = 0.0

@dataclass
class BatterySim:
    pack_kWh: float = PACK_KWH
    energy_kWh: float = PACK_KWH
    soc: float = 1.0
    v_load: float = ocv_from_soc(1.0)
    i_load: float = 0.0
    p_kw: float = 0.0
    time_s: float = 0.0

@dataclass
class HybridState:
    fuel_L: float = FUEL_TANK_L
    lph: float = 0.0
    rho: float = RHO
    power_kW: float = 0.0
    time_s: float = 0.0

# ========================== Keyboard ==========================
class KeyInput:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd); tty.setcbreak(self.fd); return self
    def __exit__(self,*_): termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def getch(self):
        dr,_,_ = select.select([sys.stdin],[],[],0)
        return sys.stdin.read(1) if dr else None

# ========================== ROS2 Node ==========================
class TeleopNode(Node):
    def __init__(self, mode: str, wind_mode: str, log_path: str = "teleop_ros2_log.csv",
                 log_period_s: float = 1.0):
        super().__init__("teleop_mavros2")
        self.mode = mode                    # "battery" or "hybrid"
        self.wind = WindState(mode=wind_mode)
        self.rho = RHO

        # MAVROS state and local pose/vel (ENU)
        self.state = State()
        self.pos_e = 0.0   # East  (ROS x)
        self.pos_n = 0.0   # North (ROS y)
        self.vel_e = 0.0
        self.vel_n = 0.0

        # Hold anchor (position we try to maintain when your XY command is small)
        self.anchor_e = 0.0
        self.anchor_n = 0.0

        # Manual command accumulators (key inputs)
        self.vx_e = 0.0
        self.vy_n = 0.0
        self.vz_u = 0.0
        self.yaw_rate = 0.0

        self.offboard_active = False
        self.flight_started_wall = None
        self.landing_triggered = False

        # Energy sims
        self.batt = BatterySim()
        self.hy = None
        if self.mode == "hybrid":
            # Initialize hybrid based on full tank hover power estimate
            fuel_mass = FUEL_TANK_L * FUEL_DENSITY_KG_PER_L
            mass = BASE_AIRFRAME_MASS_KG + GEN_MASS_KG + fuel_mass
            p_hover = hover_electrical_kW(mass, self.rho)
            p_used  = min(p_hover, GEN_CONT_KW)
            lph = HOVER_BURN_LPH_FIXED if USE_FIXED_HOVER_BURN else fuel_burn_lph_from_power(p_used)
            self.hy = HybridState(fuel_L=FUEL_TANK_L, lph=lph, rho=self.rho, power_kW=p_used)

        # CSV logger
        self.log_path = log_path
        os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)  # creates parent if needed
        self.log_period_s = max(1e-3, log_period_s)
        self._log_last_sim = -1.0
        self._log_f = open(self.log_path, "a", buffering=1, newline="")
        self._log_csv = csv.writer(self._log_f)
        if self._log_f.tell() == 0:
            self._log_csv.writerow([
                "event","wall_time","mode","wind_mode","wind_mps","wind_deg",
                "sim_time_s","in_air","pos_n","pos_e","vel_n","vel_e","cmd_xy",
                "soc","batt_v","batt_a","batt_kw","fuel_L","fuel_pct","gen_kw","lph"
            ])
        self.log_event("wind_config_selected")

        # QoS (BEST_EFFORT is sufficient for sim; switch to RELIABLE if needed)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        # Pubs/Subs (topics follow MAVROS defaults)
        self.pub_cmd = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.sub_state = self.create_subscription(State, "/mavros/state", self.cb_state, 10)
        self.sub_pose  = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.cb_pose, qos)
        self.sub_vel   = self.create_subscription(TwistStampedMsg, "/mavros/local_position/velocity_local", self.cb_vel, qos)

        # Services
        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_mode= self.create_client(SetMode, "/mavros/set_mode")
        self.cli_land= self.create_client(CommandTOL, "/mavros/cmd/land")

        # Timers
        self.timer_cmd   = self.create_timer(1.0/COMMAND_HZ, self.on_cmd_timer)   # control loop
        self.timer_energy= self.create_timer(1.0/ENERGY_HZ,  self.on_energy_timer)# energy/fuel
        self.timer_wind  = self.create_timer(0.1,            self.on_wind_timer)  # random wind re-roll watcher
        self.timer_log   = self.create_timer(1.0/LOG_HZ,     self.on_log_timer)   # csv write guard

        # Keyboard thread (non-blocking stdin)
        self._stop = False
        self._key_thread = threading.Thread(target=self.key_loop, daemon=True)
        self._key_thread.start()

        self.get_logger().info(f"Mode={self.mode}, Wind={self.wind.mode}, log={self.log_path}")

    # --------- Callbacks ---------
    def cb_state(self, msg: State):
        self.state = msg

    def cb_pose(self, msg: PoseStamped):
        # ENU: x=East, y=North
        self.pos_e = float(msg.pose.position.x)
        self.pos_n = float(msg.pose.position.y)

    def cb_vel(self, msg: TwistStampedMsg):
        self.vel_e = float(msg.twist.linear.x)
        self.vel_n = float(msg.twist.linear.y)

    # --------- Timers ---------
    def on_cmd_timer(self):
        # Position-hold when user command is small (uses HOLD_KP/KD & CMD_DEADBAND)
        cmd_e = self.vx_e
        cmd_n = self.vy_n
        if math.hypot(self.vx_e, self.vy_n) < CMD_DEADBAND:
            err_e = self.anchor_e - self.pos_e
            err_n = self.anchor_n - self.pos_n
            corr_e = HOLD_KP * err_e - HOLD_KD * self.vel_e
            corr_n = HOLD_KP * err_n - HOLD_KD * self.vel_n
            cmd_e += corr_e
            cmd_n += corr_n
        else:
            # User actively moving: slide anchor so hold doesn't fight you
            self.anchor_e = self.pos_e
            self.anchor_n = self.pos_n

        # Optional visual gust shove (adds a fraction of wind speed for GUST_BURST_S seconds)
        if VISUAL_GUSTS and time.time() < self.wind.gust_expire:
            wn = self.wind.speed_mps * math.cos(math.radians(self.wind.dir_deg))  # North component
            we = self.wind.speed_mps * math.sin(math.radians(self.wind.dir_deg))  # East component
            cmd_e += GUST_CMD_GAIN * we
            cmd_n += GUST_CMD_GAIN * wn

        # Clamp XY to avoid excessive velocities
        mag = math.hypot(cmd_e, cmd_n)
        if mag > VEL_MAX_MS:
            scale = VEL_MAX_MS / mag
            cmd_e *= scale; cmd_n *= scale

        # Publish ENU setpoint
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = cmd_e               # ENU: x = East
        msg.twist.linear.y = cmd_n               # ENU: y = North
        msg.twist.linear.z = self.vz_u           # ENU: z = Up
        msg.twist.angular.z = math.radians(self.yaw_rate)  # rad/s
        self.pub_cmd.publish(msg)

    def on_energy_timer(self):
        # Energy/fuel background update (tuned by SPEED_FACTOR, WIND_POWER_PER_MPS, K_VEL_POWER)
        dt_sim = (1.0/ENERGY_HZ) * SPEED_FACTOR
        if self.mode == "battery":
            self.batt.time_s += dt_sim
            if self.offboard_active:
                base_kw, v, i = battery_hover_power_kW(self.batt.soc, self.rho)
                tilt_mult = 1.0  # If you subscribe to attitude, replace with 1/(cos(roll)*cos(pitch))
                vel_extra = 1.0 + K_VEL_POWER * min(5.0, math.hypot(self.vx_e, self.vy_n))
                wind_mult = 1.0 + WIND_POWER_PER_MPS * max(0.0, self.wind.speed_mps)
                p_kw = base_kw * tilt_mult * vel_extra * wind_mult
            else:
                p_kw, v, i = P_IDLE_KW, ocv_from_soc(self.batt.soc), 0.0

            self.batt.energy_kWh = max(0.0, self.batt.energy_kWh - p_kw * dt_sim / 3600.0)
            self.batt.soc = self.batt.energy_kWh / max(1e-6, self.batt.pack_kWh)
            self.batt.p_kw, self.batt.v_load, self.batt.i_load = p_kw, v, i

            # Auto-land at 30% (after GUARD_ACTIVATE_GRACE_S in OFFBOARD)
            if self.flight_started_wall and (time.time()-self.flight_started_wall)>=GUARD_ACTIVATE_GRACE_S:
                if (not self.landing_triggered) and self.batt.soc <= LAND_AT_PCT + 1e-6:
                    self.landing_triggered = True
                    self.log_event("auto_land_trigger")
                    self.get_logger().warn("Battery <= 30% — LAND")
                    self.do_land()

        else:  # hybrid
            if self.hy is None: return
            self.hy.time_s += dt_sim

            fuel_mass = self.hy.fuel_L * FUEL_DENSITY_KG_PER_L
            mass = BASE_AIRFRAME_MASS_KG + GEN_MASS_KG + fuel_mass
            p_hover = hover_electrical_kW(mass, self.rho)

            tilt_mult = 1.0
            vel_extra = 1.0 + K_VEL_POWER * min(5.0, math.hypot(self.vx_e, self.vy_n))
            wind_mult = 1.0 + WIND_POWER_PER_MPS * max(0.0, self.wind.speed_mps)

            p_adj = min(p_hover * tilt_mult * vel_extra * wind_mult, GEN_CONT_KW)
            self.hy.power_kW = p_adj
            self.hy.lph = HOVER_BURN_LPH_FIXED if USE_FIXED_HOVER_BURN else fuel_burn_lph_from_power(p_adj)

            self.hy.fuel_L = max(0.0, self.hy.fuel_L - (self.hy.lph/3600.0)*dt_sim)

            if (not self.landing_triggered) and (self.hy.fuel_L / FUEL_TANK_L) <= LAND_AT_PCT + 1e-6:
                if self.flight_started_wall and (time.time()-self.flight_started_wall)>=GUARD_ACTIVATE_GRACE_S:
                    self.landing_triggered = True
                    self.log_event("auto_land_trigger")
                    self.get_logger().warn("Fuel <= 30% — LAND")
                    self.do_land()

    def on_wind_timer(self):
        # Re-roll random wind every WIND_UPDATE_PERIOD_S; sets gust window for the visual shove
        if self.wind.mode != "random":
            return
        if not hasattr(self, "_wind_last"): self._wind_last = 0.0
        now = time.time()
        if now - self._wind_last >= WIND_UPDATE_PERIOD_S:
            self._wind_last = now
            self.wind.speed_mps = random.uniform(0.0, WIND_RANDOM_MAX_MPS)
            self.wind.dir_deg   = random.uniform(0.0, 360.0)
            self.wind.gust_expire = now + GUST_BURST_S
            self.log_event("wind_change")

    def on_log_timer(self):
        # Periodic CSV logging; actual row rate is governed by log_period_s
        if self.mode == "battery":
            sim_t = self.batt.time_s
            if sim_t - self._log_last_sim >= self.log_period_s:
                self._log_last_sim = sim_t
                self._log_csv.writerow([
                    "", time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
                    "battery", self.wind.mode, f"{self.wind.speed_mps:.3f}", f"{self.wind.dir_deg:.3f}",
                    f"{sim_t:.3f}", "1" if self.offboard_active else "0",
                    f"{self.pos_n:.6f}", f"{self.pos_e:.6f}", f"{self.vel_n:.6f}", f"{self.vel_e:.6f}",
                    f"{math.hypot(self.vx_e,self.vy_n):.6f}",
                    f"{self.batt.soc:.6f}", f"{self.batt.v_load:.6f}", f"{self.batt.i_load:.6f}", f"{self.batt.p_kw:.6f}",
                    "", "", "", ""
                ])
        else:
            sim_t = self.hy.time_s if self.hy else 0.0
            if sim_t - self._log_last_sim >= self.log_period_s:
                self._log_last_sim = sim_t
                self._log_csv.writerow([
                    "", time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
                    "hybrid", self.wind.mode, f"{self.wind.speed_mps:.3f}", f"{self.wind.dir_deg:.3f}",
                    f"{sim_t:.3f}", "1" if self.offboard_active else "0",
                    f"{self.pos_n:.6f}", f"{self.pos_e:.6f}", f"{self.vel_n:.6f}", f"{self.vel_e:.6f}",
                    f"{math.hypot(self.vx_e,self.vy_n):.6f}",
                    "", "", "", "",
                    f"{self.hy.fuel_L:.6f}", f"{self.hy.fuel_L/FUEL_TANK_L:.6f}", f"{self.hy.power_kW:.6f}", f"{self.hy.lph:.6f}"
                ])

    # --------- Actions ---------
    def do_offboard_start(self):
        # PX4 requires setpoints streaming BEFORE OFFBOARD. Prime for ~1s @20Hz.
        self.get_logger().info("Priming OFFBOARD...")
        start = time.time()
        rate = 20.0
        while time.time() - start < 1.0:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd.publish(msg)
            time.sleep(1.0/rate)

        # Arm
        if not self.cli_arm.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Arming service not available")
            return
        req = CommandBool.Request(value=True)
        resp = self.cli_arm.call(req)
        if not resp.success:
            self.get_logger().error("Arming failed")
            return

        # OFFBOARD
        if not self.cli_mode.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetMode service not available")
            return
        sm = SetMode.Request()
        sm.custom_mode = "OFFBOARD"
        mresp = self.cli_mode.call(sm)
        if not mresp.mode_sent:
            self.get_logger().error("OFFBOARD mode switch failed")
            return

        self.offboard_active = True
        self.flight_started_wall = time.time()
        # lock anchor at current position
        self.anchor_e = self.pos_e
        self.anchor_n = self.pos_n
        self.log_event("offboard_started")
        self.get_logger().info("OFFBOARD active.")

    def do_land(self):
        # Try AUTO.LAND first; fallback to CommandTOL
        with contextlib.suppress(Exception):
            if self.cli_mode.wait_for_service(timeout_sec=1.0):
                req = SetMode.Request(); req.custom_mode = "AUTO.LAND"
                resp = self.cli_mode.call(req)
                if resp.mode_sent:
                    self.get_logger().info("AUTO.LAND sent")
                    self.log_event("land_complete")
                    return
        with contextlib.suppress(Exception):
            if self.cli_land.wait_for_service(timeout_sec=1.0):
                tol = CommandTOL.Request()
                tol.min_pitch = 0.0; tol.yaw = 0.0; tol.latitude=0.0; tol.longitude=0.0; tol.altitude=0.0
                r = self.cli_land.call(tol)
                if r.success:
                    self.get_logger().info("CommandTOL land sent")
        self.log_event("land_complete")

    # --------- Logging ---------
    def log_event(self, name: str):
        sim_t = (self.batt.time_s if self.mode=="battery" else (self.hy.time_s if self.hy else 0.0))
        self._log_csv.writerow([
            name, time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
            self.mode, self.wind.mode, f"{self.wind.speed_mps:.3f}", f"{self.wind.dir_deg:.3f}",
            f"{sim_t:.3f}", "1" if self.offboard_active else "0",
            f"{self.pos_n:.6f}", f"{self.pos_e:.6f}", f"{self.vel_n:.6f}", f"{self.vel_e:.6f}",
            f"{math.hypot(self.vx_e,self.vy_n):.6f}",
            (f"{self.batt.soc:.6f}" if self.mode=="battery" else ""),
            (f"{self.batt.v_load:.6f}" if self.mode=="battery" else ""),
            (f"{self.batt.i_load:.6f}" if self.mode=="battery" else ""),
            (f"{self.batt.p_kw:.6f}" if self.mode=="battery" else ""),
            (f"{self.hy.fuel_L:.6f}" if self.mode=="hybrid" and self.hy else ""),
            (f"{(self.hy.fuel_L/FUEL_TANK_L):.6f}" if self.mode=="hybrid" and self.hy else ""),
            (f"{self.hy.power_kW:.6f}" if self.mode=="hybrid" and self.hy else ""),
            (f"{self.hy.lph:.6f}" if self.mode=="hybrid" and self.hy else ""),
        ])

    # --------- Keyboard loop ---------
    def key_loop(self):
        print("\nControls: WASD (N/W/S/E), R/F (Up/Down), J/L (yaw), SPACE (hover+reset anchor),")
        print("          T (arm+OFFBOARD), G (land), B (battery snap), H (hybrid snap), Q (quit)\n")
        with KeyInput() as keys:
            while not self._stop:
                k = keys.getch()
                if not k:
                    time.sleep(0.01); continue
                k = k.lower()
                if   k=='w': self.vy_n += VEL_STEP_MS     # North+
                elif k=='s': self.vy_n -= VEL_STEP_MS     # North-
                elif k=='d': self.vx_e += VEL_STEP_MS     # East+
                elif k=='a': self.vx_e -= VEL_STEP_MS     # East-
                elif k=='r': self.vz_u += VEL_STEP_UP_MS  # Up+
                elif k=='f': self.vz_u -= VEL_STEP_UP_MS  # Up- (Down)
                elif k=='j': self.yaw_rate -= YAW_STEP_DEG_S
                elif k=='l': self.yaw_rate += YAW_STEP_DEG_S
                elif k==' ':
                    self.vx_e=self.vy_n=self.vz_u=self.yaw_rate=0.0
                    self.anchor_e = self.pos_e
                    self.anchor_n = self.pos_n
                    print("[hover] zero cmd & anchor reset")
                elif k=='t':
                    self.do_offboard_start()
                elif k=='g':
                    print("[land] manual")
                    self.log_event("manual land")
                    self.do_land()
                elif k=='b' and self.mode=="battery":
                    sim=self.batt
                    print(f"[battery] {sim.soc*100:6.2f}%  V={sim.v_load:5.1f}  I={sim.i_load:5.0f}  P={sim.p_kw:4.2f} kW"
                          f" | wind={self.wind.speed_mps:.1f} m/s @{int(self.wind.dir_deg)%360}°  t={mmss(sim.time_s)} sim"
                          f"  pos(N,E)=({self.pos_n:+.1f},{self.pos_e:+.1f}) m")
                elif k=='h' and self.mode=="hybrid" and self.hy:
                    hs=self.hy
                    print(f"[hybrid] fuel={hs.fuel_L:5.2f} L  {hs.fuel_L/FUEL_TANK_L*100:5.2f}%  "
                          f"hover≈{hs.power_kW:.2f} kW  burn≈{hs.lph:.2f} L/h"
                          f" | wind={self.wind.speed_mps:.1f} m/s @{int(self.wind.dir_deg)%360}°  t={mmss(hs.time_s)} sim"
                          f"  pos(N,E)=({self.pos_n:+.1f},{self.pos_e:+.1f}) m")
                elif k=='q':
                    print("[quit]")
                    self._stop = True
                    rclpy.shutdown()
                    break

    # --------- Shutdown ---------
    def destroy_node(self):
        self._stop = True
        try: self._key_thread.join(timeout=0.5)
        except Exception: pass
        try: self._log_f.close()
        except Exception: pass
        super().destroy_node()

# ========================== Main ==========================
def prompt_mode() -> str:
    while True:
        s = input("Select system [battery/hybrid] (default battery): ").strip().lower()
        if s in ("","battery"): return "battery"
        if s == "hybrid": return "hybrid"
        print("Type 'battery' or 'hybrid'.")

def prompt_wind() -> str:
    s = input("Wind mode [ideal/random] (default ideal): ").strip().lower()
    return "random" if s in ("r","random") else "ideal"

def main():
    global SPEED_FACTOR
    import argparse
    ap = argparse.ArgumentParser(description="ROS2 teleop (MAVROS2 + PX4)")
    ap.add_argument("--log-file", default="teleop_ros2_log.csv")        # change default CSV path here
    ap.add_argument("--log-period", type=float, default=1.0)           # seconds of *sim* time between rows
    ap.add_argument("--speed", type=float, default=1.0, help="Energy sim speed multiplier")
    args = ap.parse_args()
    SPEED_FACTOR = max(0.0, args.speed)

    mode = prompt_mode()
    wind_mode = prompt_wind()

    rclpy.init()
    node = TeleopNode(mode=mode, wind_mode=wind_mode, log_path=args.log_file, log_period_s=args.log_period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
