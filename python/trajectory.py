"""
trajectory.py — Generatore di traiettoria :

  VirtualRobot        — moto 1D con profilo trapezoidale (ACCEL/CRUISE/DECEL/TARGET)
  StraightLine2DMotion — segmento rettilineo 2D nel piano X/Z di Godot
  WaypointPath2D      — sequenza ciclica di segmenti, commutazione basata sul drone reale

Adattamenti rispetto all'originale:
  1. Piano di lavoro: X/Z Godot invece di X/Y
     --> heading usa atan2(dx, dz) per la convenzione yaw di Godot (0 = avanti su Z)
  2. Path2D --> WaypointPath2D: waypoint ciclici (loop) invece di lista lineare
  3. Metodo start_motion accetta tuple (x, z) coerente col resto del sistema
"""

import math


# ============================================================
# VirtualRobot 
# ============================================================

class VirtualRobot:
    """
    Generatore di profilo di velocità trapezoidale 1D.
    Fasi: ACCEL --> CRUISE --> DECEL --> TARGET
    
    Parametri:
        _p_target : distanza totale da percorrere [m]
        _vmax     : velocità di crociera [m/s]
        _acc      : accelerazione [m/s²]  (positiva)
        _dec      : decelerazione [m/s²]  (positiva)
    
    Stato interno: p (posizione) e v (velocità).
    evaluate(delta_t) aggiorna lo stato; leggere p e v dopo la chiamata.
    """

    ACCEL  = 0
    CRUISE = 1
    DECEL  = 2
    TARGET = 3

    def __init__(self, _p_target: float, _vmax: float,
                 _acc: float, _dec: float):
        self.p_target = _p_target
        self.vmax     = _vmax
        self.accel    = _acc
        self.decel    = _dec
        self.v        = 0.0   # velocità corrente [m/s]
        self.p        = 0.0   # posizione corrente [m]
        self.phase    = VirtualRobot.ACCEL
        # Distanza di frenata dalla cinematica: D = v²_max / (2·dec)
        self.decel_distance = 0.5 * _vmax * _vmax / _dec

    def evaluate(self, delta_t: float) -> None:
        if self.phase == VirtualRobot.ACCEL:
            self.p = self.p + self.v * delta_t \
                     + self.accel * delta_t * delta_t / 2.0
            self.v = self.v + self.accel * delta_t
            distance = max(0.0, self.p_target - self.p)

            if self.v >= self.vmax:
                self.v = self.vmax
                self.phase = VirtualRobot.CRUISE
            elif distance <= self.decel_distance:
                # Gestione phase overlap: entra in DECEL solo se la velocità
                # attuale supera quella cinematicamente "giusta" per frenare
                # entro la distanza residua (formula vd = sqrt(2·dec·d))
                v_exp = math.sqrt(2.0 * self.decel * distance)
                if v_exp < self.v:
                    self.phase = VirtualRobot.DECEL

        elif self.phase == VirtualRobot.CRUISE:
            self.p = self.p + self.vmax * delta_t
            distance = self.p_target - self.p
            if distance <= self.decel_distance:
                self.phase = VirtualRobot.DECEL

        elif self.phase == VirtualRobot.DECEL:
            self.p = self.p + self.v * delta_t \
                     - self.decel * delta_t * delta_t / 2.0
            v_new = self.v - self.decel * delta_t
            if v_new >= 0.0:
                self.v = v_new
            if self.p >= self.p_target:
                self.v = 0.0
                self.p = self.p_target
                self.phase = VirtualRobot.TARGET

        elif self.phase == VirtualRobot.TARGET:
            self.v = 0.0
            self.p = self.p_target

    @property
    def speed(self) -> float:
        return self.v

    @property
    def position(self) -> float:
        return self.p

    @property
    def is_done(self) -> bool:
        return self.phase == VirtualRobot.TARGET


# ============================================================
# StraightLine2DMotion — segmento rettilineo nel piano X/Z
# ============================================================

class StraightLine2DMotion:
    """
    Adattamento di StraightLine2DMotion per il piano X/Z di Godot.

    Il VirtualRobot percorre la distanza euclidea tra start e end.
    La posizione 2D del VR viene ricostruita proiettando la progressione
    scalare (vr.p) lungo la direzione normalizzata del segmento.

        - Coordinate (x, z) invece di (x, y)
        - heading = atan2(dx, dz) per convenzione yaw Godot
    """

    def __init__(self, _vmax: float, _acc: float, _dec: float):
        self.vmax  = _vmax
        self.accel = _acc
        self.decel = _dec
        # Inizializzati in start_motion
        self.xs = self.zs = 0.0
        self.xe = self.ze = 0.0
        self.heading  = 0.0
        self.distance = 0.0
        self._vr: VirtualRobot = None

    def start_motion(self, start: tuple, end: tuple) -> None:
        """
        Avvia il moto dal punto start al punto end.
        :param start: (x, z) in coordinate Godot
        :param end:   (x, z) in coordinate Godot
        """
        self.xs, self.zs = start
        self.xe, self.ze = end

        dx = self.xe - self.xs
        dz = self.ze - self.zs
        self.distance = math.sqrt(dx * dx + dz * dz)

        if self.distance > 1e-3:
            # Yaw Godot: 0 = avanti su Z+, positivo verso X+
            # Stesso angolo usato in multirotor_controller per puntare il drone
            self.heading = math.atan2(dx, dz)
        else:
            self.heading = 0.0

        self._vr = VirtualRobot(self.distance, self.vmax, self.accel, self.decel)

    def evaluate(self, delta_t: float) -> tuple:
        """
        Aggiorna il VR e restituisce (x_tgt, z_tgt).
        Quando il VR è in TARGET, restituisce il punto di arrivo esatto.
        """
        if self._vr is None:
            return (self.xe, self.ze)

        self._vr.evaluate(delta_t)

        # Proiezione della progressione scalare sulla direzione del segmento
        #  xt = xs + vr.p * cos(heading)
        # ma con angolo calcolato rispetto a Z 
        if self.distance > 1e-3:
            ratio = self._vr.p / self.distance
            xt = self.xs + ratio * (self.xe - self.xs)
            zt = self.zs + ratio * (self.ze - self.zs)
        else:
            xt, zt = self.xe, self.ze

        return (xt, zt)

    @property
    def current_speed(self) -> float:
        """Velocità corrente del VR [m/s]."""
        return self._vr.speed if self._vr else 0.0

    @property
    def is_done(self) -> bool:
        """True quando il VR ha raggiunto il TARGET."""
        return self._vr is None or self._vr.is_done


# ============================================================
# WaypointPath2D — percorso ciclico di waypoint
# ============================================================

class WaypointPath2D:
    """
    Analoga a Path2D del prof, adattata per navigazione ciclica (lawnmower).

    Differenza chiave rispetto alla Path2D originale:
        - La lista dei waypoint è ciclica (loop infinito).
        - La commutazione al waypoint successivo avviene quando il DRONE REALE
          è dentro threshold metri dal waypoint corrente (come nel prof),
          NON quando il VR finisce il segmento.
        - Questo garantisce che il drone raggiunga effettivamente ogni
          waypoint prima di procedere al successivo.

    Schema di funzionamento:

        WaypointPath2D
              │
              ▼
        StraightLine2DMotion  <-- start_motion(prev_wp, next_wp)
              │
              ▼
          VirtualRobot 1D  --> p(t), v(t)  (profilo trapezoidale)
              │
              ▼
        (x_tgt, z_tgt) = xs + (p/L)·(xe-xs),  zs + (p/L)·(ze-zs)
              │
              ▼
        ctrl.set_target(x=x_tgt, y=z_tgt)

        Commutazione waypoint: target_distance(drone_reale, wp_corrente) < threshold
    """

    def __init__(self, _vmax: float, _acc: float, _dec: float,
                 _threshold: float = 4.0):
        """
        :param _vmax:      velocità di crociera [m/s]
        :param _acc:       accelerazione [m/s²]
        :param _dec:       decelerazione [m/s²]
        :param _threshold: distanza (drone reale - waypoint) per commutare [m]
        """
        self.vmax      = _vmax
        self.accel     = _acc
        self.decel     = _dec
        self.threshold = _threshold

        self._wps: list      = []
        self._idx: int       = 0
        self._traj           = StraightLine2DMotion(_vmax, _acc, _dec)
        self._current_target = (0.0, 0.0)

    # ------------------------------------------------------------------
    def load(self, waypoints: list) -> None:
        """Carica la lista di waypoint (verranno percorsi in loop)."""
        self._wps = list(waypoints)
        self._idx = 0

    def start(self, start_pos: tuple, start_idx: int = 0) -> None:
        """
        Avvia il percorso dalla posizione start_pos verso il waypoint start_idx.
        :param start_pos: (x, z) posizione attuale del drone
        :param start_idx: indice del primo waypoint da raggiungere
        """
        if not self._wps:
            return
        self._idx            = start_idx % len(self._wps)
        self._current_target = self._wps[self._idx]
        self._traj.start_motion(start_pos, self._current_target)

    # ------------------------------------------------------------------
    def evaluate(self, delta_t: float, drone_pose: tuple) -> tuple:
        """
        Aggiorna la traiettoria e restituisce (x_tgt, z_tgt).

        Logica di commutazione :
          1. Calcola la distanza del drone reale dal waypoint corrente.
          2. Se < threshold: il drone è vicino --> avanza al prossimo waypoint
             e ri-avvia StraightLine2DMotion dalla posizione corrente del VR
             (non dalla posizione del drone, per continuità del target).
          3. Altrimenti: aggiorna il VR e restituisce la posizione del VR.

        :param delta_t:    passo temporale [s]
        :param drone_pose: (x, z) posizione reale del drone
        :return:           (x_tgt, z_tgt) setpoint per il controller
        """
        if not self._wps:
            return (0.0, 0.0)

        # Posizione corrente del VR (prima di aggiornare)
        (x_vr, z_vr) = self._traj.evaluate(delta_t)

        # Distanza del DRONE REALE dal waypoint corrente
        cx, cz = self._current_target
        dx = drone_pose[0] - cx
        dz = drone_pose[1] - cz
        target_distance = math.sqrt(dx * dx + dz * dz)

        if target_distance < self.threshold:
            # Drone vicino al waypoint --> avanza al successivo
            self._idx = (self._idx + 1) % len(self._wps)
            self._current_target = self._wps[self._idx]
            # Ri-avvia dal punto corrente del VR (continuità del target)
            self._traj.start_motion((x_vr, z_vr), self._current_target)
            # Primo passo sul nuovo segmento
            (x_vr, z_vr) = self._traj.evaluate(delta_t)

        return (x_vr, z_vr)

    # ------------------------------------------------------------------
    @property
    def heading(self) -> float:
        """Heading Godot del segmento corrente [rad]."""
        return self._traj.heading

    @property
    def current_speed(self) -> float:
        """Velocità corrente del VR [m/s]."""
        return self._traj.current_speed

    @property
    def current_waypoint_idx(self) -> int:
        """Indice del waypoint attualmente puntato."""
        return self._idx

    @property
    def current_target(self) -> tuple:
        """Coordinate (x, z) del waypoint corrente."""
        return self._current_target
