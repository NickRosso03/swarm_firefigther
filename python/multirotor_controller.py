import math
from controllers import P_Controller, PI_Controller, PID_Controller

# Stimato dai log: hover stabile -> f_per_motor ≈ 3.59 N
# Aumenta se il drone scende lentamente in hover, riduci se sale.
HOVER_FF = 3.59


class MultirotorController:

    def __init__(self, hover_ff: float = HOVER_FF):
        self.hover_ff = hover_ff

        # QUOTA
        self.z_control  = P_Controller(kp=1.5, sat=2.0)
        self.vz_control = PID_Controller(kp=4.0, ki=0.3, kd=0.1, sat=5.0)

        # ── POSIZIONE XY ───────────────────────────────────────────────────────
        # kp=0.4: può essere alzato rispetto al precedente 0.25 perché il loop
        self.x_control  = P_Controller(kp=0.6, sat=10.0)
        self.y_control  = P_Controller(kp=0.6, sat=10.0)

        # ── VELOCITÀ -> TILT ────────────────────────────────────────────────────
        # L'anti-windup del PI_Controller congela l'integrale a saturazione
        self.vx_control = PID_Controller(kp=0.3, ki=0.1, kd=0.2, sat=math.radians(35))
        self.vy_control = PID_Controller(kp=0.3, ki=0.1, kd=0.2, sat=math.radians(35))

        # ── ATTITUDE (angolo -> rate) ────────────────────────────────────────────
        self.roll_control  = P_Controller(kp=2.4, sat=1.5)
        self.pitch_control = P_Controller(kp=2.4, sat=1.5)

        # ── RATE (rate -> coppia motori) ────────────────────────────────────────
        self.w_pitch_control = PID_Controller(kp=0.25, ki=0.1, kd=0.025, sat=0.6)
        self.w_roll_control  = PID_Controller(kp=0.25, ki=0.1, kd=0.025, sat=0.6)
        
        # YAW
        self.yaw_control   = P_Controller(kp=4.0, sat=0.6)
        self.w_yaw_control = PID_Controller(kp=1.0, ki=0.1, kd=0.1, sat=0.4)
        self.yaw_target    = 0.0
        
        # Setpoint
        self.z_target = 1.0
        self.x_target = 0.0
        self.y_target = 0.0

        # Valori intermedi per debug
        self.vz_target    = 0.0
        self.vx_target    = 0.0
        self.vy_target    = 0.0
        self.roll_target  = 0.0
        self.pitch_target = 0.0

    def reset_velocity_integrators(self) -> None:
        """Azzera gli integratori dei loop velocità XY.
        Da chiamare alle transizioni di stato che interrompono il moto
        (HOVERING, SUPPRESSING, inizio MOVING) per evitare che l'integrale
        accumulato nella direzione precedente causi overshoot nella nuova.
        """
        self.vx_control.reset()
        self.vy_control.reset()


    def evaluate(self,
                 delta_t: float,
                 z: float,  vz: float,
                 x: float,  vx: float,
                 y: float,  vy: float,
                 roll: float,  roll_rate: float,
                 pitch: float, pitch_rate: float,
                 yaw: float, yaw_rate: float, 
                 altitude_only: bool = False) -> tuple:
        """
        Calcola (f1, f2, f3, f4) in Newton.
        altitude_only=True: usare SOLO durante TAKEOFF (non HOVERING).
        """

        # QUOTA (sempre attiva)
        self.vz_target = self.z_control.evaluate(delta_t, self.z_target - z)
        f_corr = self.vz_control.evaluate(delta_t, self.vz_target - vz)
        f_base = self.hover_ff + f_corr

        if altitude_only:
            self.w_roll_control.reset()
            self.w_pitch_control.reset()
            self.w_yaw_control.reset()
            self.vx_control.reset()   # evita windup durante salita verticale
            self.vy_control.reset()
            return f_base, f_base, f_base, f_base

        # VELOCITÀ SETPOINT: il P corregge le deviazioni dal target;
        # il ki del vx/vy_control accumula la velocità di crociera in stazionario.
        self.vx_target = self.x_control.evaluate(delta_t, self.x_target - x)
        self.vy_target = self.y_control.evaluate(delta_t, self.y_target - y)
        
        err_vx_glob = self.vx_target - vx
        err_vy_glob = self.vy_target - vy

        # ATTITUDE TARGET nel frame mondo — i controller P lavorano direttamente
        # sugli errori di velocità mondiali, senza rotazione di frame sull'ingresso.
        # La trasformazione mondo->drone viene applicata SOLO qui, all'ultimo stadio,
        # così i due canali X/Z mondo restano disaccoppiati durante le virate.
        tilt_x_world = -self.vx_control.evaluate(delta_t, err_vx_glob)
        tilt_z_world =  self.vy_control.evaluate(delta_t, err_vy_glob)

        # rot(-yaw): tilt mondo -> tilt frame drone  (roll e pitch locali)
        self.roll_target  =  tilt_x_world * math.cos(yaw) + tilt_z_world * math.sin(yaw)
        self.pitch_target = -tilt_x_world * math.sin(yaw) + tilt_z_world * math.cos(yaw)

        # ATTITUDE RATE CONTROL (incluso YAW)
        pitch_rate_tgt = self.pitch_control.evaluate(delta_t, self.pitch_target - pitch)
        pitch_cmd      = self.w_pitch_control.evaluate(delta_t, pitch_rate_tgt - pitch_rate)

        roll_rate_tgt  = self.roll_control.evaluate(delta_t, self.roll_target - roll)
        roll_cmd       = self.w_roll_control.evaluate(delta_t, roll_rate_tgt - roll_rate)

        # --- FIX WRAP-AROUND DELLO YAW ---
        yaw_err = self.yaw_target - yaw
        # Normalizza l'errore tra -PI e +PI (prende sempre la via più breve)
        yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi
        
        yaw_rate_tgt   = self.yaw_control.evaluate(delta_t, yaw_err)
        yaw_cmd        = self.w_yaw_control.evaluate(delta_t, yaw_rate_tgt - yaw_rate)

        # TILT COMPENSATION
        tilt  = math.sqrt(roll * roll + pitch * pitch)
        cos_t = math.cos(tilt)
        f = min(f_base / cos_t, f_base * 1.3) if cos_t > 0.5 else f_base

        # MIXER
        f1 = f + roll_cmd - pitch_cmd + yaw_cmd
        f2 = f - roll_cmd - pitch_cmd - yaw_cmd
        f3 = f - roll_cmd + pitch_cmd + yaw_cmd
        f4 = f + roll_cmd + pitch_cmd - yaw_cmd

        # Clamp: i motori reali non producono spinta negativa
        f1 = max(0.0, f1)
        f2 = max(0.0, f2)
        f3 = max(0.0, f3)
        f4 = max(0.0, f4)

        return f1, f2, f3, f4

    def set_target(self, x=None, y=None, z=None,yaw=None):
        if x is not None: self.x_target = x
        if y is not None: self.y_target = y
        if z is not None: self.z_target = z
        #if yaw is not None: self.yaw_target = yaw
        if yaw is not None:
            # Filtro passa-basso: prende la via più breve (-π, +π)
            err = (yaw - self.yaw_target + math.pi) % (2 * math.pi) - math.pi
            self.yaw_target += 0.1 * err  # alpha=0.1 -> risposta lenta e smooth

    def set_yaw_direct(self, yaw: float) -> None:
        """Imposta yaw_target istantaneamente, bypassando il filtro passa-basso.
        Da usare SOLO all'inizio di un nuovo task (fuoco o waypoint iniziale)
        per evitare che il drone parta con un heading sbagliato.
        Durante il task continuare a usare set_target(yaw=...) per il filtraggio.
        """
        self.yaw_target = (yaw + math.pi) % (2 * math.pi) - math.pi