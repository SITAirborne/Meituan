#!/usr/bin/env python3
"""
Teleop with wind modes (ideal/random) + stabilizing hold + endurance sims + CSV logging
- Battery: U15II KV80 + 24S 34Ah model; auto-lands at 30% SoC.
- Hybrid : generator/fuel model; auto-lands at 30% fuel.
- Wind modes:
    • ideal  : 0 m/s entire flight
    • random : re-roll every 5 s, speed ∈ [0,20] m/s, direction ∈ [0,360)°
- Visual gusts: brief nudges after each wind step so you *see* a shove; hold fights it.

Keys: W/A/S/D (N/W/S/E), R/F (Up/Down), J/L (yaw), SPACE (zero + reset anchor),
      T (arm+takeoff+offboard), B/H (snapshots), G (manual land), Q (quit)
"""

import asyncio, contextlib, math, random, select, sys, termios, time, tty
from argparse import ArgumentParser
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Tuple
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

# -------- General knobs --------
TAKEOFF_ALT_M      = 5.0
COMMAND_HZ         = 10.0
VEL_STEP_MS        = 0.7
VEL_STEP_UP_MS     = 0.7
YAW_STEP_DEG_S     = 15.0
LAND_AT_PCT        = 0.30
GUARD_ACTIVATE_GRACE_S = 5.0
SPEED_FACTOR       = 1.0
K_VEL_POWER        = 0.05   # +5% power per 1 m/s of *user* XY command

# Wind → power multiplier (applies in both battery & hybrid)
WIND_POWER_PER_MPS     = 0.02
WIND_UPDATE_PERIOD_S   = 5.0
WIND_RANDOM_MAX_MPS    = 20.0

# ---- Visual gusts + hold (to see the shove & recovery even if SITL wind=0) ----
VISUAL_GUSTS = True       # change if don't want see the gust
GUST_CMD_GAIN = 0.20      # only 20% of wind speed is used as a commanded nudge
GUST_BURST_S  = 1.0       # shove lasts 1 s after each wind change
HOLD_KP = 0.8             # stronger hold to fight the shove
HOLD_KD = 1.2
CMD_DEADBAND = 0.08       # below this, we treat as hover (enable hold)
VEL_MAX_MS = 3.0          # clamp final XY command so it can’t run away

# -------- Weather / wind --------
@dataclass
class Weather:
    wind_mode: str          # "ideal" or "random"
    wind_speed_mps: float
    wind_dir_deg: float     # TO direction (0=N, 90=E)
    rho: float              # kg/m^3
    gust_expire_ts: float = 0.0  # internal: when the brief nudge ends

def isa_density(temp_C: float, pressure_alt_m: float) -> float:
    T0,P0,L,g,R = 288.15,101325.0,0.0065,9.80665,287.05
    h = max(0.0, pressure_alt_m)
    P = P0 * (1.0 - L*h/T0) ** (g/(R*L))
    T = temp_C + 273.15
    return P / (R * T)

# -------- Airframe / rotor --------
ETA_PROP, ETA_MOTOR = 0.75, 0.90
ETA_TOTAL = ETA_PROP * ETA_MOTOR
N_ROTORS = 4
PROP_DIAM_IN = 40.0
PROP_DIAM_M  = PROP_DIAM_IN * 0.0254

def hover_mech_power_disk(mass_kg: float, rho: float) -> float:
    T = mass_kg * 9.80665
    A_one = math.pi * (PROP_DIAM_M/2.0)**2
    A_tot = N_ROTORS * A_one
    return (T**1.5) / max(1e-6, math.sqrt(2.0*rho*A_tot))

def hover_electrical_kW(mass_kg: float, rho: float) -> float:
    return hover_mech_power_disk(mass_kg, rho) / max(1e-6, ETA_TOTAL) / 1000.0

# -------- Battery model (24S 34Ah, U15II) --------
N_CELLS, CAP_AH, CELL_V_NOM = 24, 34.0, 3.7
PACK_KWH = (N_CELLS * CELL_V_NOM * CAP_AH) / 1000.0
CELL_V_MAX, CELL_V_MIN = 4.10, 3.30
PACK_IR_OHM = 0.03
DRONE_MASS_BATT_KG = 80.0
P_IDLE_KW = 0.15
PER_MOTOR_POWER_CAP_W = None
PER_MOTOR_CURRENT_CAP = None

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

@dataclass
class BatterySim:
    pack_kWh: float
    energy_kWh: float
    soc: float
    v_load: float
    i_load: float
    p_kw: float
    time_s: float = 0.0

# -------- Hybrid model --------
GEN_CONT_KW, GEN_MASS_KG = 14.0, 29.5
SFC_KG_PER_KWH, FUEL_DENSITY_KG_PER_L = 0.600, 0.74
HOVER_BURN_LPH_FIXED, USE_FIXED_HOVER_BURN = 12.5, False
BASE_AIRFRAME_MASS_KG, FUEL_TANK_L = 50.5, 15.0

def fuel_burn_lph_from_power(power_kW: float) -> float:
    return (power_kW * SFC_KG_PER_KWH) / max(1e-6, FUEL_DENSITY_KG_PER_L)

@dataclass
class HybridState:
    fuel_L: float
    lph: float
    rho: float
    power_kW: float
    time_s: float = 0.0

# -------- Telemetry watchers --------
@dataclass
class TelemetryState:
    in_air: Optional[bool] = None
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    vel_n: float = 0.0
    vel_e: float = 0.0
    pos_n: float = 0.0
    pos_e: float = 0.0

async def watch_in_air(drone: System, st: TelemetryState, stop: asyncio.Event):
    try:
        async for ia in drone.telemetry.in_air():
            if stop.is_set(): break
            st.in_air = bool(ia)
    except asyncio.CancelledError:
        pass

async def watch_attitude(drone: System, st: TelemetryState, stop: asyncio.Event):
    try:
        async for e in drone.telemetry.attitude_euler():
            if stop.is_set(): break
            st.roll_rad  = math.radians(getattr(e,"roll_deg",0.0))
            st.pitch_rad = math.radians(getattr(e,"pitch_deg",0.0))
    except asyncio.CancelledError:
        pass

async def watch_velocity(drone: System, st: TelemetryState, stop: asyncio.Event):
    try:
        async for v in drone.telemetry.velocity_ned():
            if stop.is_set(): break
            st.vel_n = getattr(v,"north_m_s",0.0)
            st.vel_e = getattr(v,"east_m_s",0.0)
    except asyncio.CancelledError:
        pass

# -------- Keyboard --------
class KeyInput:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd); tty.setcbreak(self.fd); return self
    def __exit__(self,*_): termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def getch(self):
        dr,_,_ = select.select([sys.stdin],[],[],0)
        return sys.stdin.read(1) if dr else None

# -------- MAVSDK helpers --------
async def connect_any(port: int) -> System:
    d = System(); await d.connect(system_address=f"udpin://0.0.0.0:{port}")
    async for s in d.core.connection_state():
        if s.is_connected: break
    return d

async def arm_takeoff(drone: System, alt: float):
    await drone.action.set_takeoff_altitude(alt)
    await drone.action.arm(); await asyncio.sleep(0.3)
    await drone.action.takeoff()
    t0 = time.time()
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= alt-0.5 or time.time()-t0 > 12: break

async def start_offboard_with_zero(drone: System):
    zero = VelocityNedYaw(0.0,0.0,0.0,0.0)
    await drone.offboard.set_velocity_ned(zero); await asyncio.sleep(0.05)
    try: await drone.offboard.start()
    except OffboardError: raise

async def land(drone: System):
    await drone.action.land()
    t0 = time.time()
    async for ia in drone.telemetry.in_air():
        if not ia or time.time()-t0 > 60: break

# -------- CSV logger --------
class CsvLogger:
    def __init__(self, path: str):
        import os
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.f = open(path, "a", buffering=1)
        if self.f.tell() == 0:
            self.f.write("event,wall_time,mode,wind_mode,wind_mps,wind_deg,"
                         "sim_time_s,in_air,roll_deg,pitch_deg,pos_n,pos_e,vel_n,vel_e,cmd_xy,"
                         "soc,batt_v,batt_a,batt_kw,fuel_L,fuel_pct,gen_kw,lph\n")
    def _fmt(self,x):
        if x is None: return ""
        if isinstance(x,float): return f"{x:.6f}"
        return str(x)
    def row(self, **kw):
        keys=("event","wall_time","mode","wind_mode","wind_mps","wind_deg",
              "sim_time_s","in_air","roll_deg","pitch_deg","pos_n","pos_e","vel_n","vel_e","cmd_xy",
              "soc","batt_v","batt_a","batt_kw","fuel_L","fuel_pct","gen_kw","lph")
        self.f.write(",".join(self._fmt(kw.get(k)) for k in keys) + "\n")
    def event(self, mode, wind_mode, wind_mps, wind_deg, sim_t, name, **extras):
        self.row(event=name, wall_time=datetime.now().isoformat(timespec="milliseconds"),
                 mode=mode, wind_mode=wind_mode, wind_mps=wind_mps, wind_deg=wind_deg,
                 sim_time_s=sim_t, **extras)
    def close(self):
        try: self.f.close()
        except Exception: pass

# -------- Central landing trigger --------
async def trigger_land_once(drone: System, offboard_ref: dict,
                            landing_ev: asyncio.Event, stop: asyncio.Event,
                            reason: str, logger: Optional[CsvLogger],
                            mode: str, sim_time_s: float,
                            weather: Weather,
                            roll_rad: float, pitch_rad: float,
                            st=None):
    if landing_ev.is_set(): return
    landing_ev.set()
    if logger:
        logger.event(mode, weather.wind_mode, weather.wind_speed_mps, weather.wind_dir_deg,
                     sim_time_s, "auto_land_trigger",
                     in_air=True, roll_deg=math.degrees(roll_rad), pitch_deg=math.degrees(pitch_rad),
                     pos_n=(st.pos_n if st else ""), pos_e=(st.pos_e if st else ""),
                     vel_n=(st.vel_n if st else ""), vel_e=(st.vel_e if st else ""))
    with contextlib.suppress(Exception):
        if offboard_ref.get("active"):
            await drone.offboard.stop(); offboard_ref["active"]=False
    print(f"[trigger] {reason} — landing now.")
    await land(drone)
    if logger:
        logger.event(mode, weather.wind_mode, weather.wind_speed_mps, weather.wind_dir_deg,
                     sim_time_s, "land_complete", in_air=False,
                     roll_deg=math.degrees(roll_rad), pitch_deg=math.degrees(pitch_rad),
                     pos_n=(st.pos_n if st else ""), pos_e=(st.pos_e if st else ""),
                     vel_n=(st.vel_n if st else ""), vel_e=(st.vel_e if st else ""))
    stop.set()

# -------- Wind driver (random steps + mark nudge window) --------
async def wind_driver(weather: Weather, stop: asyncio.Event, logger: Optional[CsvLogger],
                      mode: str):
    if logger:
        logger.event(mode, weather.wind_mode, weather.wind_speed_mps, weather.wind_dir_deg,
                     0.0, "wind_config_selected")
    if weather.wind_mode == "ideal":
        weather.wind_speed_mps = 0.0
        weather.wind_dir_deg   = 0.0
        weather.gust_expire_ts = 0.0
        return
    last_change = -1.0
    start = time.time()
    while not stop.is_set():
        now = time.time()
        if last_change < 0 or (now - last_change) >= WIND_UPDATE_PERIOD_S:
            last_change = now
            weather.wind_speed_mps = random.uniform(0.0, WIND_RANDOM_MAX_MPS)
            weather.wind_dir_deg   = random.uniform(0.0, 360.0)
            weather.gust_expire_ts = now + GUST_BURST_S   # brief shove window
            if logger:
                sim_t = (now - start) * SPEED_FACTOR
                logger.event(mode, weather.wind_mode, weather.wind_speed_mps, weather.wind_dir_deg,
                             sim_t, "wind_change")
        try:
            await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            break

# -------- Background: battery --------
async def battery_sim_updater(sim: BatterySim, weather: Weather,
                              st: TelemetryState, horiz_mag_ref: dict,
                              stop: asyncio.Event):
    last = time.time()
    while not stop.is_set():
        now = time.time()
        dt_sim = max(0.0, (now-last)) * SPEED_FACTOR
        last = now; sim.time_s += dt_sim

        if bool(st.in_air):
            base_kw, v, i = battery_hover_power_kW(sim.soc, weather.rho)
            tilt_mult = 1.0 / max(1e-3, math.cos(st.roll_rad)*math.cos(st.pitch_rad))
            vel_extra = 1.0 + K_VEL_POWER * min(5.0, horiz_mag_ref.get("mag_xy",0.0))
            wind_mult = 1.0 + WIND_POWER_PER_MPS * max(0.0, weather.wind_speed_mps)
            p_kw = base_kw * tilt_mult * vel_extra * wind_mult
        else:
            p_kw, v, i = P_IDLE_KW, ocv_from_soc(sim.soc), 0.0

        sim.energy_kWh = max(0.0, sim.energy_kWh - p_kw * dt_sim / 3600.0)
        sim.soc = sim.energy_kWh / max(1e-6, sim.pack_kWh)
        sim.p_kw, sim.v_load, sim.i_load = p_kw, v, i
        try:
            await asyncio.sleep(0.05)
        except asyncio.CancelledError:
            break

async def battery_guard(drone: System, sim: BatterySim,
                        flight_started_at_ref: dict, offboard_ref: dict,
                        landing_ev: asyncio.Event, stop: asyncio.Event,
                        logger: Optional[CsvLogger], mode: str, weather: Weather,
                        st: TelemetryState):
    while not stop.is_set():
        fs = flight_started_at_ref.get("t", None)
        if fs and (time.time()-fs) >= GUARD_ACTIVATE_GRACE_S:
            if sim.soc <= LAND_AT_PCT + 1e-6:
                await trigger_land_once(drone, offboard_ref, landing_ev, stop,
                                        "capacity <= 30%", logger, mode, sim.time_s,
                                        weather, st.roll_rad, st.pitch_rad, st)
                return
        try:
            await asyncio.sleep(0.05)
        except asyncio.CancelledError:
            break

# -------- Background: hybrid --------
async def hybrid_updater(h: HybridState, weather: Weather,
                         st: TelemetryState, horiz_mag_ref: dict,
                         drone: System, offboard_ref: dict,
                         landing_ev: asyncio.Event, stop: asyncio.Event,
                         logger: Optional[CsvLogger], mode: str):
    last = time.time()
    while not stop.is_set():
        now = time.time()
        dt_sim = max(0.0, (now-last)) * SPEED_FACTOR
        last = now; h.time_s += dt_sim

        fuel_mass = h.fuel_L * FUEL_DENSITY_KG_PER_L
        mass = BASE_AIRFRAME_MASS_KG + GEN_MASS_KG + fuel_mass
        p_hover = hover_electrical_kW(mass, weather.rho)

        tilt_mult = 1.0 / max(1e-3, math.cos(st.roll_rad)*math.cos(st.pitch_rad))
        vel_extra = 1.0 + K_VEL_POWER * min(5.0, horiz_mag_ref.get("mag_xy",0.0))
        wind_mult = 1.0 + WIND_POWER_PER_MPS * max(0.0, weather.wind_speed_mps)

        p_adj = min(p_hover * tilt_mult * vel_extra * wind_mult, GEN_CONT_KW)
        h.power_kW = p_adj
        h.lph = HOVER_BURN_LPH_FIXED if USE_FIXED_HOVER_BURN else fuel_burn_lph_from_power(p_adj)

        h.fuel_L = max(0.0, h.fuel_L - (h.lph / 3600.0) * dt_sim)

        if (h.fuel_L / FUEL_TANK_L) <= LAND_AT_PCT + 1e-6:
            await trigger_land_once(drone, offboard_ref, landing_ev, stop,
                                    "fuel <= 30%", logger, mode, h.time_s,
                                    weather, st.roll_rad, st.pitch_rad, st)
            return
        try:
            await asyncio.sleep(0.05)
        except asyncio.CancelledError:
            break

# -------- Periodic CSV logger --------
async def csv_periodic_logger(mode: str, logger: CsvLogger, log_period_s: float,
                              st: TelemetryState, battery_sim: Optional[BatterySim],
                              hstate: Optional[HybridState], horiz_mag_ref: dict,
                              stop: asyncio.Event, weather: Weather):
    last_written = -1.0
    while not stop.is_set():
        try:
            if mode == "battery" and battery_sim:
                sim_t = battery_sim.time_s
                if sim_t - last_written >= log_period_s:
                    last_written = sim_t
                    logger.row(
                        event="", wall_time=datetime.now().isoformat(timespec="milliseconds"),
                        mode="battery", wind_mode=weather.wind_mode,
                        wind_mps=weather.wind_speed_mps, wind_deg=weather.wind_dir_deg,
                        sim_time_s=sim_t, in_air=bool(st.in_air),
                        roll_deg=math.degrees(st.roll_rad), pitch_deg=math.degrees(st.pitch_rad),
                        pos_n=st.pos_n, pos_e=st.pos_e, vel_n=st.vel_n, vel_e=st.vel_e,
                        cmd_xy=horiz_mag_ref.get("mag_xy",0.0),
                        soc=battery_sim.soc, batt_v=battery_sim.v_load,
                        batt_a=battery_sim.i_load, batt_kw=battery_sim.p_kw,
                        fuel_L="", fuel_pct="", gen_kw="", lph=""
                    )
            elif mode == "hybrid" and hstate:
                sim_t = hstate.time_s
                if sim_t - last_written >= log_period_s:
                    last_written = sim_t
                    logger.row(
                        event="", wall_time=datetime.now().isoformat(timespec="milliseconds"),
                        mode="hybrid", wind_mode=weather.wind_mode,
                        wind_mps=weather.wind_speed_mps, wind_deg=weather.wind_dir_deg,
                        sim_time_s=sim_t, in_air=bool(st.in_air),
                        roll_deg=math.degrees(st.roll_rad), pitch_deg=math.degrees(st.pitch_rad),
                        pos_n=st.pos_n, pos_e=st.pos_e, vel_n=st.vel_n, vel_e=st.vel_e,
                        cmd_xy=horiz_mag_ref.get("mag_xy",0.0),
                        fuel_L=hstate.fuel_L, fuel_pct=(hstate.fuel_L/FUEL_TANK_L),
                        gen_kw=hstate.power_kW, lph=hstate.lph,
                        soc="", batt_v="", batt_a="", batt_kw=""
                    )
            await asyncio.sleep(0.05)
        except asyncio.CancelledError:
            break

# -------- Teleop main --------
async def teleop(mode: str, port: int, weather: Weather, log_path: str, log_period_s: float):
    logger = CsvLogger(log_path)
    drone = await connect_any(port)

    st = TelemetryState()
    stop = asyncio.Event()
    landing_ev = asyncio.Event()

    watchers = [
        asyncio.create_task(watch_in_air(drone, st, stop)),
        asyncio.create_task(watch_attitude(drone, st, stop)),
        asyncio.create_task(watch_velocity(drone, st, stop)),
    ]

    wind_task = asyncio.create_task(wind_driver(weather, stop, logger, mode))

    vx = vy = vz = yaw_rate = 0.0
    dt = 1.0 / COMMAND_HZ
    horiz_mag_ref = {"mag_xy": 0.0}
    hold_anchor = {"n": 0.0, "e": 0.0}

    offboard_ref = {"active": False}
    flight_started_at_ref = {"t": None}
    tasks = []

    battery_sim = None
    hstate = None
    if mode == "battery":
        battery_sim = BatterySim(PACK_KWH, PACK_KWH, 1.0, ocv_from_soc(1.0), 0.0, 0.0)
        tasks += [
            asyncio.create_task(battery_sim_updater(battery_sim, weather, st, horiz_mag_ref, stop)),
            asyncio.create_task(battery_guard(drone, battery_sim, flight_started_at_ref,
                                              offboard_ref, landing_ev, stop, logger,
                                              mode, weather, st))
        ]
    else:
        fuel_mass = FUEL_TANK_L * FUEL_DENSITY_KG_PER_L
        mass = BASE_AIRFRAME_MASS_KG + GEN_MASS_KG + fuel_mass
        p_hover = hover_electrical_kW(mass, weather.rho)
        p_used  = min(p_hover, GEN_CONT_KW)
        lph = HOVER_BURN_LPH_FIXED if USE_FIXED_HOVER_BURN else fuel_burn_lph_from_power(p_used)
        hstate = HybridState(FUEL_TANK_L, lph, weather.rho, p_used)
        tasks += [asyncio.create_task(hybrid_updater(hstate, weather, st, horiz_mag_ref,
                                                     drone, offboard_ref, landing_ev, stop,
                                                     logger, mode))]

    tasks += [asyncio.create_task(csv_periodic_logger(mode, logger, log_period_s,
                                                      st, battery_sim, hstate,
                                                      horiz_mag_ref, stop, weather))]

    def mmss(tsec: float) -> str:
        tsec = max(0.0, tsec); return f"{int(tsec//60):02d}:{int(tsec%60):02d}"

    try:
        with KeyInput() as keys:
            last_loop = time.time()
            while not stop.is_set():
                now = time.time()
                dt_loop = max(1e-3, now - last_loop)
                last_loop = now

                # integrate rough local position
                st.pos_n += st.vel_n * dt_loop
                st.pos_e += st.vel_e * dt_loop

                k = keys.getch()
                if k:
                    k = k.lower()
                    if   k=='w': vx += VEL_STEP_MS
                    elif k=='s': vx -= VEL_STEP_MS
                    elif k=='d': vy += VEL_STEP_MS
                    elif k=='a': vy -= VEL_STEP_MS
                    elif k=='r': vz -= VEL_STEP_UP_MS
                    elif k=='f': vz += VEL_STEP_UP_MS
                    elif k=='j': yaw_rate -= YAW_STEP_DEG_S
                    elif k=='l': yaw_rate += YAW_STEP_DEG_S
                    elif k==' ':
                        vx=vy=vz=yaw_rate=0.0
                        hold_anchor["n"] = st.pos_n
                        hold_anchor["e"] = st.pos_e
                    elif k=='t':
                        await arm_takeoff(drone, TAKEOFF_ALT_M)
                        flight_started_at_ref["t"] = time.time()
                        await start_offboard_with_zero(drone)
                        offboard_ref["active"] = True
                        # lock anchor at start point
                        hold_anchor["n"] = st.pos_n
                        hold_anchor["e"] = st.pos_e
                    elif k=='b' and mode=="battery":
                        sim = battery_sim
                        pct = sim.soc*100.0; V=sim.v_load; I=sim.i_load; PkW=sim.p_kw
                        rdeg,pdeg = math.degrees(st.roll_rad), math.degrees(st.pitch_rad)
                        print(f"[battery] {pct:6.2f}%  {V:5.1f} V  {I:5.0f} A  {PkW:4.2f} kW  "
                              f"| tilt≈({rdeg:+.1f}°, {pdeg:+.1f}°)  wind={weather.wind_speed_mps:.1f} m/s @{int(weather.wind_dir_deg)%360}°  "
                              f"t={mmss(sim.time_s)} sim  pos=({st.pos_n:+.1f},{st.pos_e:+.1f}) m")
                    elif k=='h' and mode=="hybrid":
                        hs = hstate
                        frac = hs.fuel_L / FUEL_TANK_L
                        rdeg,pdeg = math.degrees(st.roll_rad), math.degrees(st.pitch_rad)
                        print(f"[hybrid] fuel={hs.fuel_L:5.2f} L  {frac*100:5.2f}%  "
                              f"| tilt≈({rdeg:+.1f}°, {pdeg:+.1f}°)  wind={weather.wind_speed_mps:.1f} m/s @{int(weather.wind_dir_deg)%360}°  "
                              f"hover≈{hs.power_kW:.2f} kW  burn≈{hs.lph:.2f} L/h  t={mmss(hs.time_s)} sim  "
                              f"pos=({st.pos_n:+.1f},{st.pos_e:+.1f}) m")
                    elif k=='g':
                        await trigger_land_once(drone, offboard_ref, landing_ev, stop,
                                                "manual land", logger, mode,
                                                battery_sim.time_s if battery_sim else (hstate.time_s if hstate else 0.0),
                                                weather, st.roll_rad, st.pitch_rad, st)
                    elif k=='q':
                        stop.set()

                # track user horizontal command magnitude (manual)
                horiz_mag_ref["mag_xy"] = math.hypot(vx, vy)

                # position-hold correction if user not commanding much
                cmd_n = vx
                cmd_e = vy
                if math.hypot(vx, vy) < CMD_DEADBAND:
                    err_n = hold_anchor["n"] - st.pos_n
                    err_e = hold_anchor["e"] - st.pos_e
                    corr_n = HOLD_KP * err_n - HOLD_KD * st.vel_n
                    corr_e = HOLD_KP * err_e - HOLD_KD * st.vel_e
                    cmd_n += corr_n
                    cmd_e += corr_e
                else:
                    # move anchor with the user so hold doesn't fight sticks
                    hold_anchor["n"] = st.pos_n
                    hold_anchor["e"] = st.pos_e

                # brief visual gust nudge right after each wind change
                if VISUAL_GUSTS and time.time() < weather.gust_expire_ts:
                    wn = weather.wind_speed_mps * math.cos(math.radians(weather.wind_dir_deg))
                    we = weather.wind_speed_mps * math.sin(math.radians(weather.wind_dir_deg))
                    cmd_n += GUST_CMD_GAIN * wn
                    cmd_e += GUST_CMD_GAIN * we

                # clamp XY to keep it controllable
                cmd_n = max(-VEL_MAX_MS, min(VEL_MAX_MS, cmd_n))
                cmd_e = max(-VEL_MAX_MS, min(VEL_MAX_MS, cmd_e))

                if offboard_ref["active"] and not landing_ev.is_set():
                    await drone.offboard.set_velocity_ned(VelocityNedYaw(cmd_n, cmd_e, vz, yaw_rate))

                await asyncio.sleep(1.0 / COMMAND_HZ)
    finally:
        with contextlib.suppress(Exception):
            if offboard_ref["active"]:
                await drone.offboard.stop()
        # cancel background tasks cleanly (silence CancelledError)
        for t in tasks+[wind_task]+watchers:
            t.cancel()
        for t in tasks+[wind_task]+watchers:
            try: await t
            except asyncio.CancelledError: pass
            except Exception: pass
        logger.close()

# -------- Prompts --------
def prompt_mode() -> str:
    while True:
        s = input("Select system [battery/hybrid] (default battery): ").strip().lower()
        if s in ("","battery"): return "battery"
        if s == "hybrid": return "hybrid"
        print("Please type 'battery' or 'hybrid'.")

def prompt_wind() -> Weather:
    rho = isa_density(15.0, 0.0)  # ISA SL
    s = input("Wind mode [ideal/random] (default ideal): ").strip().lower()
    if s in ("r","random"):
        speed = random.uniform(0.0, WIND_RANDOM_MAX_MPS)
        direction = random.uniform(0.0, 360.0)
        return Weather("random", speed, direction, rho, gust_expire_ts=time.time()+GUST_BURST_S)
    return Weather("ideal", 0.0, 0.0, rho, gust_expire_ts=0.0)

# -------- CLI --------
def main():
    ap = ArgumentParser(description="Teleop with wind modes (ideal/random) + stabilizing hold")
    ap.add_argument("--port", type=int, default=14540, help="UDP port PX4 is on")
    ap.add_argument("--speed", type=float, default=1.0, help="Energy sim speed multiplier")
    ap.add_argument("--log-file", type=str, default="teleop_log.csv", help="CSV output file")
    ap.add_argument("--log-period", type=float, default=1.0, help="Sim seconds between CSV rows")
    args = ap.parse_args()

    global SPEED_FACTOR; SPEED_FACTOR = max(0.0, args.speed)

    mode = prompt_mode()
    weather = prompt_wind()
    print(f"Wind mode: {weather.wind_mode.upper()}  |  start wind={weather.wind_speed_mps:.1f} m/s @ {int(weather.wind_dir_deg)%360}°")

    asyncio.run(teleop(mode, args.port, weather, args.log_file, max(1e-3, args.log_period)))

if __name__ == "__main__":
    main()
