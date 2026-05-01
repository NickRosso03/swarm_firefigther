"""
drone_agent.py — Agente autonomo per un singolo drone dello swarm.

Topic DDS per drone i (prefisso "drone_{i}/"):
  RICEVUTI da Godot:
    drone_{i}/X, Y, Z         : posizione [m]
    drone_{i}/VX, VY, VZ      : velocità lineare [m/s]
    drone_{i}/TX, TY, TZ      : Euler angles roll/pitch/yaw [rad]
    drone_{i}/WX, WY, WZ      : velocità angolari [rad/s]
    drone_{i}/tick            : impulso di sync ogni physics frame
    drone_{i}/connected       : 1 quando Godot è pronto
    drone_{i}/fire_spotted    : 1 quando il drone i vede un fuoco
        
        -drone_{i}/fire_spotted_id : id fuoco avvistato
        -drone_{i}/fire_spotted_x
        -drone_{i}/fire_spotted_y
        -drone_{i}/fire_spotted_z
        -drone_{i}/fire_extinguished : id fuoco spento  

  PUBBLICATI verso Godot:
    drone_{i}/f1..f4          : forze propulsori [N]
  
  PUBBLICATI per monitor (non letti da nessun agente):
    drone_{i}/z_tgt, vz_tgt       : setpoint quota e velocità verticale
    drone_{i}/x_tgt, y_tgt        : setpoint posizione XY
    drone_{i}/vx_tgt, vy_tgt      : setpoint velocità XY
    drone_{i}/roll_tgt, pitch_tgt : setpoint assetto (rad)

  CONDIVISI tra agenti Python :
    drone_{i}/status          : codice stato FSM (float)
    drone_{i}/sx, sy, sz      : posizione pubblicata dagli altri
    drone_{i}/fire_x,y,z      : target incendio corrente


"""

import time
import math
import threading
import logging

from dds import DDS, Time
from multirotor_controller import MultirotorController
from coverage_planner import CoveragePlanner
from trajectory import WaypointPath2D, StraightLine2DMotion
from collision_avoidance import CollisionAvoidance

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(message)s",
    datefmt="%H:%M:%S"
)

# ---------------------------------------------------------------------------
# Parametri operativi
# ---------------------------------------------------------------------------
AREA_SIZE       = 200.0  # [mxm]
TAKEOFF_ALT     = 15.0   # [m]
WAYPOINT_RADIUS = 2.0    # [m]

FIRE_RADIUS     = 10.0   # [m] — deve essere >= D_SAFE del potential field
FIRE_STANDOFF_RADIUS = 9.0
SUPPRESS_TIME   = 10.0    # [s] base; scala con intensità in _suppress_time_for
INTENSITY_MIN_THRESHOLD = 0.05 # INTENSITY_MIN_THRESHOLD: soglia minima di intensità per considerare un fuoco valido.

TRAJ_VMAX      = 8.0    # [m/s]  velocità di crociera in esplorazione
TRAJ_ACC       = 2.0    # [m/s²] accelerazione
TRAJ_DEC       = 3.0    # [m/s²] decelerazione
TRAJ_THRESHOLD = 5.0    # [m]    distanza drone-waypoint per commutare
 
FIRE_VMAX      = 7.0    # [m/s]  velocità di avvicinamento al fuoco
FIRE_ACC       = 2.0    # [m/s²] accelerazione
FIRE_DEC       = 2.0    # [m/s²] decelerazione

# ---------------------------------------------------------------------------
# Parametri acqua / ricarica
# ---------------------------------------------------------------------------
WATER_MAX          = 100.0   # unità arbitrarie
WATER_MIN_THRESH   =  10.0   # sotto questa soglia non intraprende nuove soppressioni
WATER_CONSUME_RATE =   1.0   # unità/s consumate durante SUPPRESSING
RECHARGE_RATE      =  10.0   # unità/s durante REFUELING
STATION_LAND_ALT   =   0.2  # [m] quota "a terra" per fine atterraggio sulla stazione
STATION_RADIUS     =   2.5   # [m] distanza massima per considerarsi sopra la stazione


#----------------------------------------------------------------------------
N_DRONES        = 8
DDS_HOST        = '127.0.0.1'
DDS_PORT        = 4444


# ---------------------------------------------------------------------------
# Codici numerici degli stati (servono perché DDS trasporta solo float)
# ---------------------------------------------------------------------------
class StateCode:
    IDLE        = 0.0
    TAKEOFF     = 0.0
    HOVERING    = 0.0   
    EXPLORING   = 1.0
    MOVING      = 2.0
    SUPPRESSING = 4.0
    RETURNING   = 3.0
    GOING_TO_STATION = 5.0   # ← nuovo
    REFUELING   = 6.0   # ← nuovo

class State:
    IDLE        = "IDLE"
    TAKEOFF     = "TAKEOFF"
    HOVERING    = "HOVERING"    # stabilizzazione in quota prima di esplorare
    EXPLORING   = "EXPLORING"
    MOVING      = "MOVING"
    SUPPRESSING = "SUPPRESSING"
    RETURNING   = "RETURNING"
    GOING_TO_STATION = "GOING_TO_STATION"   # ← nuovo
    REFUELING        = "REFUELING"           # ← nuovo

FREE_STATES = {State.EXPLORING, State.RETURNING}

AVOIDANCE_STATES = {State.EXPLORING, State.MOVING, State.RETURNING, State.GOING_TO_STATION}


class DroneAgent:
    """
    Agente autonomo per un singolo drone.
    Ogni istanza gira nel proprio thread (main.py).
    """

    def __init__(self, drone_id: int, n_drones: int = N_DRONES):
        self.id      = drone_id
        self.n       = n_drones
        self.log     = logging.getLogger(f"D{drone_id}")
        self._p      = f"drone_{drone_id}"   # prefisso topic

        self.dds     = DDS(DDS_HOST, DDS_PORT)
        self.ctrl    = MultirotorController()
        self.timer   = Time()

        # Stato fisico corrente
        self.x  = self.y  = self.z  = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.tx = self.ty = self.tz = 0.0   # roll, pitch, yaw
        self.wx = self.wy = self.wz = 0.0   # roll_rate, pitch_rate, yaw_rate

        # FSM
        self.state          = State.IDLE
        self.target_fire    = None
        self.fire_id        = None
        self._suppress_t    = 0.0
        self._hover_timer   = 0.0
        self._tick_count    = 0 
        
        # set accumulativo degli id fuoco risolti: usato perché Godot chiama
        # DDS.clear("world/fire_resolved") subito dopo lo spegnimento.
        self._resolved_fire_ids: set = set()
        # topic world/fire_intensity_{id} sottoscritti on-demand
        self._subscribed_intensity: set = set()
        
        # Swarm awareness (aggiornata ogni ciclo)
        self._swarm: dict[int, dict] = {}
        self._swarm_lock = threading.Lock()

        # Piano di perlustrazione
        self.waypoints = CoveragePlanner.get_sector(drone_id, n_drones, area_size=AREA_SIZE, altitude=TAKEOFF_ALT)

        self._traj      = WaypointPath2D(TRAJ_VMAX, TRAJ_ACC, TRAJ_DEC, TRAJ_THRESHOLD)

        self._fire_traj = StraightLine2DMotion(FIRE_VMAX, FIRE_ACC, FIRE_DEC)

        # Sistema anticollisione — potential field 
        self._avoidance = CollisionAvoidance()

        self._standoff_x = 0.0
        self._standoff_z = 0.0

        # Acqua / ricarica
        self.water_level        = WATER_MAX
        self._station_pos       = None          # [x, z] ricevuto via DDS da Godot
        self._refuel_timer      = 0.0

        self._active_drone_ids  = set(range(self.n))   # tutti attivi all'avvio

        # Quota di decollo per il controller
        self.ctrl.set_target(z=TAKEOFF_ALT)

    # =======================================================================
    # Entry point del thread
    # =======================================================================

    def run(self):
        self._setup_dds()
        self.timer.start()

        self.log.info("In attesa di Godot (variabile 'start')...")
        self.dds.wait(f"{self._p}/connected")
        self.log.info("Godot connesso. Inizio loop di controllo.")
        self.state = State.TAKEOFF
        _dbg = 0

        while True:
            # Sincronizzazione: attende il tick di Godot
            tick = self.dds.wait(f"{self._p}/tick")
            delta_t = self.timer.elapsed()

            self._tick_count += 1

            if delta_t <= 0:
                delta_t = 1.0 / 60.0
            elif delta_t > 0.1:
                delta_t = 0.1

            # 1. Leggi sensori
            self._read_state()

            _dbg += 1
            
            # log ogni 2.0s — solo drone 0.
            if self.id == 0:
                if _dbg % 120 == 0:
                    state_str = f"[{self.state}]".ljust(15)
                    self.log.info(
                        f"{state_str}"# | "
                    )

            # 2. Aggiorna stato swarm
            self._update_swarm()

            # 3. FSM
            self._update_fsm(delta_t)

            # 4. Controller fisico → pubblica forze
            self._control_and_publish(delta_t)

            # 5. Pubblica il proprio stato per gli altri agenti
            self._publish_own_state()

    # =======================================================================
    # Setup DDS
    # =======================================================================

    def _setup_dds(self):
        self.dds.start()

        p = self._p
        own_vars = [
            f"{p}/X", f"{p}/Y", f"{p}/Z",
            f"{p}/VX", f"{p}/VY", f"{p}/VZ",
            f"{p}/TX", f"{p}/TY", f"{p}/TZ",
            f"{p}/WX", f"{p}/WY", f"{p}/WZ",
            f"{p}/tick", f"{p}/connected",
            f"{p}/fire_spotted",
            f"{p}/fire_spotted_id",
            f"{p}/fire_spotted_x", f"{p}/fire_spotted_y", f"{p}/fire_spotted_z",
            f"world/station_{self.id}_pos_x",
            f"world/station_{self.id}_pos_z",
        ]

        swarm_vars = []
        for i in range(self.n):
            if i != self.id:
                swarm_vars += [
                    f"drone_{i}/status",
                    f"drone_{i}/sx", f"drone_{i}/sy", f"drone_{i}/sz",
                    f"drone_{i}/svx", f"drone_{i}/svz",
                    f"drone_{i}/fire_x", f"drone_{i}/fire_y", f"drone_{i}/fire_z",
                    f"drone_{i}/fire_spotted",
                    f"drone_{i}/fire_spotted_id",
                    f"drone_{i}/fire_spotted_x",
                    f"drone_{i}/fire_spotted_y",
                    f"drone_{i}/fire_spotted_z",
                    f"drone_{i}/fire_id",
                    f"drone_{i}/fire_extinguished",  
                    f"drone_{i}/is_active",
                ]

        self.dds.subscribe(own_vars + swarm_vars )

    # =======================================================================
    # Lettura sensori
    # =======================================================================

    def _read_state(self):
        p = self._p
        self.x  = self.dds.read(f"{p}/X")  or 0.0
        self.y  = self.dds.read(f"{p}/Y")  or 0.0
        self.z  = self.dds.read(f"{p}/Z")  or 0.0
        self.vx = self.dds.read(f"{p}/VX") or 0.0
        self.vy = self.dds.read(f"{p}/VY") or 0.0
        self.vz = self.dds.read(f"{p}/VZ") or 0.0
        self.tx = self.dds.read(f"{p}/TX") or 0.0
        self.ty = self.dds.read(f"{p}/TY") or 0.0
        self.tz = self.dds.read(f"{p}/TZ") or 0.0
        self.wx = self.dds.read(f"{p}/WX") or 0.0
        self.wy = self.dds.read(f"{p}/WY") or 0.0
        self.wz = self.dds.read(f"{p}/WZ") or 0.0
        
        sx = self.dds.read(f"world/station_{self.id}_pos_x")
        sz = self.dds.read(f"world/station_{self.id}_pos_z")
        if sx is not None and sz is not None:
            self._station_pos = [sx, sz]

    def _update_swarm(self):
        with self._swarm_lock:
            for i in range(self.n):
                if i == self.id:
                    continue
                self._swarm[i] = {
                    "status": self.dds.read(f"drone_{i}/status") or 0.0,
                    "pos":   [self.dds.read(f"drone_{i}/sx") or 0.0,
                              self.dds.read(f"drone_{i}/sy") or 0.0,
                              self.dds.read(f"drone_{i}/sz") or 0.0],
                    "vel":   [self.dds.read(f"drone_{i}/svx") or 0.0,
                              self.dds.read(f"drone_{i}/svz") or 0.0],
                    "fire":  [self.dds.read(f"drone_{i}/fire_x") or 0.0,
                              self.dds.read(f"drone_{i}/fire_y") or 0.0,
                              self.dds.read(f"drone_{i}/fire_z") or 0.0],
                    "fire_id": self.dds.read(f"drone_{i}/fire_id") or 0.0,          
                }

                #Ascolto messaggi "Fuoco spento" da altri droni
                ext_id = self.dds.read(f"drone_{i}/fire_extinguished") or 0.0
                if ext_id != 0.0:
                    self._resolved_fire_ids.add(ext_id)
            
            # Aggiorna set droni attivi e triggera replanning se cambia
            new_active = {self.id}   # io sono sempre nel set
            for i in range(self.n):
                if i == self.id:
                    continue
                val = self.dds.read(f"drone_{i}/is_active")
                if val is None or val == 1.0:   # None = non ancora pubblicato → assumi attivo
                    new_active.add(i)

            if new_active != self._active_drone_ids:
                self._active_drone_ids = new_active
                self._replan_coverage()

    # =======================================================================
    # FSM
    # =======================================================================

    def _update_fsm(self, dt: float):
        if self.state == State.TAKEOFF:
            self._do_takeoff(dt)
        elif self.state == State.HOVERING:
            self._do_hovering(dt)
        elif self.state == State.EXPLORING:
            self._do_exploring(dt)
            self._check_sensors()
        elif self.state == State.MOVING:
            self._do_moving(dt)
        elif self.state == State.SUPPRESSING:
            self._do_suppressing(dt)
        elif self.state == State.RETURNING:
            self._do_returning(dt)
            self._check_sensors()
        elif self.state == State.GOING_TO_STATION:    # ← nuovo
            self._do_going_to_station(dt)
        elif self.state == State.REFUELING:           # ← nuovo
            self._do_refueling(dt)   


    def _do_takeoff(self, dt: float):
        self.ctrl.set_target(z=TAKEOFF_ALT)
        if abs(self.y - TAKEOFF_ALT) < 0.5:
            self.ctrl.set_target(x=self.x, y=self.z)
            self.ctrl.reset_velocity_integrators()
            self._hover_timer = 0.0
            self.state = State.HOVERING

    def _do_hovering(self,  dt: float):
        self.ctrl.set_target(x=self.x, y=self.z)
        self._hover_timer += dt
        if self._hover_timer > 2.0:
          #  self.log.info("Hover stabile. Inizio perlustrazione.")
            self._traj.load(self.waypoints)
            nearest = self._nearest_waypoint()
            self._traj.start((self.x, self.z), start_idx=nearest)
            self.ctrl.set_target(yaw=self._traj.heading)
            self.state = State.EXPLORING

    def _do_exploring(self, dt: float):
        (x_tgt, z_tgt) = self._traj.evaluate(dt, (self.x, self.z))
        self.ctrl.set_target(x=x_tgt, y=z_tgt, yaw=self._traj.heading)
        
        if self.water_level <= WATER_MIN_THRESH:
            self._start_going_to_station()
            return

    def _do_moving(self, dt: float):
        if self.target_fire is None:
            self.state = State.EXPLORING
            return
    
        if not self._is_fire_valid(self.fire_id):
            self.target_fire = None
            self.fire_id     = None
            self._start_returning()
            return
    
        # Misura la distanza dal proprio spot di arrivo
        dist_to_standoff = self._dist2d([self.x, self.z], [self._standoff_x, self._standoff_z])

    
        (x_tgt, z_tgt) = self._fire_traj.evaluate(dt)
        self.ctrl.set_target(x=x_tgt, y=z_tgt, yaw=self._fire_traj.heading)
    
        # Selettore di arrivo molto stretto, il drone è arrivato in posizione
        v_horiz = math.sqrt(self.vx**2 + self.vz**2)          
        if dist_to_standoff < 2.0 and self._fire_traj.is_done and v_horiz < 1.5:  
            self._start_suppressing()


    def _do_suppressing(self, dt: float):
        if not self._is_fire_valid(self.fire_id):
            self.target_fire = None
            self.fire_id     = None
            self._start_returning()
            return
    
        fx, fy, fz = self.target_fire
        
        target_yaw = math.atan2(fx - self._standoff_x, fz - self._standoff_z)
        self.ctrl.set_target(x=self._standoff_x, y=self._standoff_z, yaw=target_yaw)

        self._avoidance.reset()

    
        intensity = self._get_fire_intensity(self.fire_id)
        n_suppressing = 1
        with self._swarm_lock:
            for info in self._swarm.values():
                if info["status"] == StateCode.SUPPRESSING:
                    if self._dist3d(info["fire"], [fx, fy, fz]) < FIRE_RADIUS * 2:
                        n_suppressing += 1
    
        self._suppress_t += dt
        
        self.water_level = max(0.0, self.water_level - WATER_CONSUME_RATE * dt)
        if self.water_level == 0.0:
            # Acqua esaurita durante soppressione — abbandona
            self.target_fire = None
            self.fire_id     = None
            self._start_going_to_station()
            return
        
        if self._suppress_t >= self._suppress_time_for(intensity, n_suppressing):
            
            # Dice a Godot di spegnere visivamente il fuoco
            self.dds.publish(f"world/fire_resolved_{int(self.fire_id)}", 1.0)
            
            # MESSAGGIO RADIO ALLO SCIAME: "Ho spento questo ID"
            self.dds.publish(f"{self._p}/fire_extinguished", self.fire_id)
            
            # Lo aggiunge alla propria memoria
            self._resolved_fire_ids.add(self.fire_id)
            
            self.target_fire = None
            self.fire_id     = None
            self._start_returning()

    def _start_suppressing(self):
        """Inizializza la soppressione."""
        self._suppress_t = 0.0

        self.ctrl.set_target(x=self._standoff_x, y=self._standoff_z)
        self.ctrl.reset_velocity_integrators()
        
        self._avoidance.reset() # Disabilita l'EMA anticollisione
        
        # I valori _standoff_x e _standoff_z sono già stati calcolati alla partenza
        self.state = State.SUPPRESSING

    def _start_returning(self):
        """Inizializza traiettoria di ritorno. """
        nearest = self._nearest_waypoint()
       # self._traj.start((self.ctrl.x_target, self.ctrl.y_target), start_idx=nearest)
        self._traj.start((self.x, self.z), start_idx=nearest)
        self.ctrl.set_yaw_direct(self._traj.heading)
        self.ctrl.reset_velocity_integrators()
        self._avoidance.reset() 
        self.state = State.RETURNING

    def _do_returning(self, dt: float):
        (x_tgt, z_tgt) = self._traj.evaluate(dt, (self.x, self.z))
        self.ctrl.set_target(x=x_tgt, y=z_tgt, yaw=self._traj.heading)

        cx, cz = self._traj.current_target
        if self._dist2d([self.x, self.z], [cx, cz]) < WAYPOINT_RADIUS * 3:
            self._avoidance.reset()
            self.state = State.EXPLORING
    
    # =======================================================================
    # Ricarica e Gestione Stazione
    # =======================================================================

    def _start_going_to_station(self):
        if self._station_pos is None:
            return
        self.target_fire = None
        self.fire_id     = None
        sx, sz = self._station_pos
        self._fire_traj.start_motion(
            (self.ctrl.x_target, self.ctrl.y_target),
            (sx, sz)
        )
        self.ctrl.reset_velocity_integrators()
        self._avoidance.reset()
        self.state = State.GOING_TO_STATION


    def _do_going_to_station(self, dt: float):
        sx, sz = self._station_pos
        x_tgt, z_tgt = self._fire_traj.evaluate(dt)
        self.ctrl.set_target(x=x_tgt, y=z_tgt, yaw=self._fire_traj.heading)
        dist    = self._dist2d([self.x, self.z], [sx, sz])
        v_horiz = math.sqrt(self.vx**2 + self.vz**2)
        if dist < STATION_RADIUS and self._fire_traj.is_done and v_horiz < 1.0:
            self.ctrl.set_target(x=sx, y=sz)
            self.ctrl.reset_velocity_integrators()
            self.ctrl.reset_altitude_integrators()
            self._avoidance.reset()
            self.state = State.REFUELING

    def _do_refueling(self, dt: float):
        sx, sz = self._station_pos
        # Mantiene x e z fissi sulla stazione
        self.ctrl.set_target(x=sx, y=sz)
        
        # 1. RAMP-DOWN: Abbassiamo il setpoint di quota gradualmente
        DESCENT_SPEED = 3.0  # m/s (regola questo valore per discese più o meno dolci)
        current_z_target = self.ctrl.z_target
        
        # Finché il target del controller è sopra la quota di atterraggio, lo abbassiamo
        if current_z_target > STATION_LAND_ALT:
            new_z_target = max(STATION_LAND_ALT, current_z_target - DESCENT_SPEED * dt)
            self.ctrl.set_target(z=new_z_target)

        # 2. CONTROLLO FISICO: Il drone è fisicamente arrivato a terra?
        if self.y > STATION_LAND_ALT + 0.3:
            # Continua ad aspettare che scenda
            pass 
        else:
            # È a terra (o quasi), procediamo con la ricarica
            self.water_level = min(WATER_MAX, self.water_level + RECHARGE_RATE * dt)
            
            # Ricarica completata
            if self.water_level >= WATER_MAX:
                self.ctrl.reset_velocity_integrators()
                self.ctrl.reset_altitude_integrators()
                self._avoidance.reset()
                # Deleghiamo la risalita al normale stato di decollo
                self.state = State.TAKEOFF
        
    def _is_fire_valid(self, fire_id: float, fire_pos: list = None) -> bool:
        """Determina se un fuoco è valido usando la conoscenza condivisa dello sciame."""
        if fire_id is None or fire_id == 0.0:
            return False
        
        # 1. È già nei nostri registri di squadra come spento?
        if fire_id in self._resolved_fire_ids:
            return False
        
        # 2. Controllo fisico: è un fuoco appena nato (intensità bassissima)?
        intensity = self._get_fire_intensity(fire_id)
        if intensity < INTENSITY_MIN_THRESHOLD:
            return False

        # 3. Controllo coordinate
        if fire_pos and (fire_pos[0] == 0.0 and fire_pos[2] == 0.0):
            return False

        return True

    # =======================================================================
    # Logica swarm 
    # =======================================================================

    def _get_fire_intensity(self, fire_id: float) -> float:
        """Legge l'intensità (pura lettura del sensore, nessuna deduzione logica qui)."""
        topic = f"world/fire_intensity_{int(fire_id)}"
        if fire_id not in self._subscribed_intensity:
            self.dds.subscribe([topic])
            self._subscribed_intensity.add(fire_id)
        val = self.dds.read(topic)
        return float(val) if val is not None else 0.0
    
    @staticmethod
    def _needed_drones(intensity: float) -> int:
        """Droni necessari in base all'intensità del fuoco."""
        if intensity >= 0.8: return 3
        if intensity >= 0.4: return 2
        return 1
    
    
    @staticmethod
    def _suppress_time_for(intensity: float, n_suppressing: int) -> float:
        """Tempo base scalato per intensità e numero di droni cooperanti."""
        if intensity >= 0.8:   base = SUPPRESS_TIME * 3.0
        elif intensity >= 0.4: base = SUPPRESS_TIME * 2.0
        else:                  base = SUPPRESS_TIME
        return base / max(1, n_suppressing)
    
    
    def _should_respond(self, fire_id: float, fire_pos: list) -> bool:
        if self.state not in FREE_STATES:
            return False
        
        if self.water_level < WATER_MIN_THRESH:
            return False

        intensity = self._get_fire_intensity(fire_id)
        if intensity < INTENSITY_MIN_THRESHOLD:
            return False

        needed = self._needed_drones(intensity)

        already_committed = 0
        closer_free       = 0   # droni liberi più vicini al fuoco di me (o stessa dist, ID minore)

        my_dist = self._dist3d([self.x, self.y, self.z], fire_pos)

        with self._swarm_lock:
            for drone_id, info in self._swarm.items():
                status = info["status"]

                if status in (StateCode.MOVING, StateCode.SUPPRESSING):
                    if info.get("fire_id", 0.0) == fire_id:   # stesso fuoco, non stessa zona
                        already_committed += 1

                if status in (StateCode.EXPLORING, StateCode.RETURNING):
                    other_dist = self._dist3d(
                        [info["pos"][0], info["pos"][1], info["pos"][2]],
                        fire_pos
                    )
                    # Tiebreak: a parità di distanza, ID minore ha priorità
                    if other_dist < my_dist or (other_dist == my_dist and drone_id < self.id):
                        closer_free += 1

        slots_remaining = needed - already_committed
        return slots_remaining > 0 and closer_free < slots_remaining
    
       
    def _check_sensors(self):
        """Legge fire_spotted proprio + quello dei compagni."""
        # Sensore proprio
        # I droni valutano i sensori solo 1 volta ogni 10 frame, in momenti diversi
        if self._tick_count % 10 != self.id:
            return
        
        if self.dds.read(f"{self._p}/fire_spotted") == 1.0:
            fire_id = self.dds.read(f"{self._p}/fire_spotted_id") or 0.0
            if fire_id != 0.0:
                fx = self.dds.read(f"{self._p}/fire_spotted_x") or 0.0
                fy = self.dds.read(f"{self._p}/fire_spotted_y") or 0.0
                fz = self.dds.read(f"{self._p}/fire_spotted_z") or 0.0
                self._evaluate_fire(fire_id, [fx, fy, fz])
    
        # Sensori dei compagni — propagazione della scoperta nello swarm
        for i in range(self.n):
            if i == self.id:
                continue
            if self.dds.read(f"drone_{i}/fire_spotted") == 1.0:
                fire_id = self.dds.read(f"drone_{i}/fire_spotted_id") or 0.0
                if fire_id != 0.0:
                    fx = self.dds.read(f"drone_{i}/fire_spotted_x") or 0.0
                    fy = self.dds.read(f"drone_{i}/fire_spotted_y") or 0.0
                    fz = self.dds.read(f"drone_{i}/fire_spotted_z") or 0.0
                    self._evaluate_fire(fire_id, [fx, fy, fz])
    
    
    def _evaluate_fire(self, fire_id: float, fire_pos: list):
        """Decide se rispondere a un fuoco rilevato."""
        if self.state not in FREE_STATES:
            return
        if fire_id == self.fire_id:
            return
        if not self._is_fire_valid(fire_id, fire_pos):
            return
        if self._should_respond(fire_id, fire_pos):
            self.target_fire = fire_pos
            self.fire_id     = fire_id
            
            # Calcola il punto di soppressione
            self._standoff_x, self._standoff_z = self._calculate_standoff_pos(fire_pos)
            
            # Punta la rotta verso il punto di standoff
            dx = self._standoff_x - self.x
            dz = self._standoff_z - self.z
            self.ctrl.set_yaw_direct(math.atan2(dx, dz))

            self.ctrl.reset_velocity_integrators()
            
            # Inizia il movimento diretto verso il posto assegnato (non verso il centro!)
            self._fire_traj.start_motion(
                (self.ctrl.x_target, self.ctrl.y_target),
                (self._standoff_x, self._standoff_z)
            )
            self.state = State.MOVING

    # =======================================================================
    # Controller e pubblicazione forze
    # =======================================================================
    def _control_and_publish(self, dt: float):
        altitude_only = self.state == State.TAKEOFF

        dx, dz = 0.0, 0.0

        if self.state in AVOIDANCE_STATES:
            with self._swarm_lock:
                snap = dict(self._swarm)          # copia reale, non alias
            dx, dz = self._avoidance.compute(
                my_pos=(self.x, self.z),
                my_vel=(self.vx, self.vz),
                swarm=snap,
            )

        # --- FIX: applica l'offset solo per questa evaluate ---
        base_x = self.ctrl.x_target          # salva il target pulito
        base_y = self.ctrl.y_target

        self.ctrl.x_target += dx             # offset temporaneo
        self.ctrl.y_target += dz

        f1, f2, f3, f4 = self.ctrl.evaluate(
            delta_t=dt,
            z=self.y,  vz=self.vy,
            x=self.x,  vx=self.vx,
            y=self.z,  vy=self.vz,
            roll=self.tz,   roll_rate=self.wz,
            pitch=self.tx,  pitch_rate=self.wx,
            yaw=self.ty,    yaw_rate=self.wy,
            altitude_only=altitude_only,
        )

        self.ctrl.x_target = base_x          # ripristina il target pulito
        self.ctrl.y_target = base_y
        # ------------------------------------------------------

        p = self._p
        self.dds.publish(f"{p}/f1", f1)
        self.dds.publish(f"{p}/f2", f2)
        self.dds.publish(f"{p}/f3", f3)
        self.dds.publish(f"{p}/f4", f4)

        self._publish_monitored_vars()

    def _publish_monitored_vars(self):
        p = self._p
        #Pubblicazione variabili monitoring
        self.dds.publish(f"{p}/z_tgt",     self.ctrl.z_target)
        self.dds.publish(f"{p}/vz_tgt",    self.ctrl.vz_target)
        self.dds.publish(f"{p}/x_tgt",     self.ctrl.x_target)
        self.dds.publish(f"{p}/y_tgt",     self.ctrl.y_target)
        self.dds.publish(f"{p}/vx_tgt",    self.ctrl.vx_target)
        self.dds.publish(f"{p}/vy_tgt",    self.ctrl.vy_target)
        self.dds.publish(f"{p}/roll_tgt",  self.ctrl.roll_target)
        self.dds.publish(f"{p}/pitch_tgt", self.ctrl.pitch_target)

    def _publish_own_state(self):
        p = self._p
        sc = {
            State.IDLE:        StateCode.IDLE,
            State.TAKEOFF:     StateCode.TAKEOFF,
            State.HOVERING:    StateCode.HOVERING,
            State.EXPLORING:   StateCode.EXPLORING,
            State.MOVING:      StateCode.MOVING,
            State.SUPPRESSING: StateCode.SUPPRESSING,
            State.RETURNING:   StateCode.RETURNING,
            State.GOING_TO_STATION: StateCode.GOING_TO_STATION,
            State.REFUELING:        StateCode.REFUELING,
        }.get(self.state, 0.0)
        
        self.dds.publish(f"{p}/status", sc)
        self.dds.publish(f"{p}/sx", self.x)
        self.dds.publish(f"{p}/sy", self.y)
        self.dds.publish(f"{p}/sz", self.z)
        self.dds.publish(f"{p}/svx", self.vx)
        self.dds.publish(f"{p}/svz", self.vz)

        fx, fy, fz = self.target_fire if self.target_fire else (0.0, 0.0, 0.0)
        self.dds.publish(f"{p}/fire_x", fx)
        self.dds.publish(f"{p}/fire_y", fy)
        self.dds.publish(f"{p}/fire_z", fz)
        self.dds.publish(f"{p}/fire_id", self.fire_id if self.fire_id is not None else 0.0)

        self.dds.publish(f"{p}/tgt_x", self.ctrl.x_target)
        self.dds.publish(f"{p}/tgt_z", self.ctrl.y_target)

        self.dds.publish(f"{self._p}/water_level", self.water_level)

        is_active = self.state not in (State.GOING_TO_STATION, State.REFUELING)
        self.dds.publish(f"{self._p}/is_active", 1.0 if is_active else 0.0)

        # --- NUOVA LOGICA ROBUSTA PER L'ANIMAZIONE ---
        # È a 1.0 solo se lo stato è REFUELING e il drone è fisicamente a terra
        is_refueling_now = 1.0 if (self.state == State.REFUELING and self.y <= STATION_LAND_ALT + 0.3) else 0.0
        self.dds.publish(f"{self._p}/refueling", is_refueling_now)

    # =======================================================================
    # Utility
    # =======================================================================

    def _nearest_waypoint(self) -> int:
        """Restituisce l'indice del waypoint più vicino alla posizione corrente."""
        best_idx  = 0
        best_dist = float('inf')
        for i, wp in enumerate(self.waypoints):
            d = self._dist2d([self.x, self.z], wp)
            if d < best_dist:
                best_dist = d
                best_idx  = i
        #self.log.info(f"Waypoint più vicino: #{best_idx} {self.waypoints[best_idx]} (dist={best_dist:.1f}m)")
        return best_idx
    
    def _calculate_standoff_pos(self, fire_pos: list) -> tuple:
        """Calcola la posizione ottimale attorno al fuoco usando il golden angle."""
        GOLDEN = math.pi * (3.0 - math.sqrt(5.0))  # ≈ 137.5°
        angle = self.id * GOLDEN
        fx, _, fz = fire_pos
        sx = fx + FIRE_STANDOFF_RADIUS * math.cos(angle)
        sz = fz + FIRE_STANDOFF_RADIUS * math.sin(angle)
        return sx, sz

    def _replan_coverage(self):
        """Ricalcola il settore di perlustrazione basandosi sui droni attivi."""
        active_sorted = sorted(self._active_drone_ids)
        if self.id not in active_sorted:
            return   # io sono in ricarica, niente da ricalcolare
        my_index  = active_sorted.index(self.id)
        n_active  = len(active_sorted)
        self.waypoints = CoveragePlanner.get_sector(
            my_index, n_active, area_size=AREA_SIZE, altitude=TAKEOFF_ALT
        )
        # Se sto già esplorando, riparte dal waypoint più vicino nel nuovo settore
        if self.state in (State.EXPLORING, State.RETURNING):
            nearest = self._nearest_waypoint()
            self._traj.load(self.waypoints)
            self._traj.start((self.x, self.z), start_idx=nearest)
            #self.log.info(f"Replanning: {n_active} droni attivi, idx={my_index}, "
            #            f"wp più vicino #{nearest}")
    

    @staticmethod
    def _dist2d(a, b) -> float:
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    @staticmethod
    def _dist3d(a, b) -> float:
        return math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))