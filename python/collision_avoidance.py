"""
Sistema anticollisione basato su Potential Fields 

    Frep(q) = krep * (1/rho - 1/rho0) * (1/rho²) * (q - q_obs)/rho    se rho ≤ rho0
    Frep(q) = 0                                                       se rho > rho0

Estensione con termine derivativo (velocity damping):
    Fdamp(q) = kdamp * v_approach / rho_eff²  * û_away

dove v_approach è la componente radiale (positiva) della velocità relativa
di avvicinamento, che reagisce alla velocità di chiusura prima ancora che 
la distanza scenda sotto D_SAFE.

Opera esclusivamente nel piano orizzontale XZ (coordinate Godot).
La quota Y è gestita dal controller di quota separato e non viene perturbata.

"""

import math


# ---------------------------------------------------------------------------
# Parametri di default (possono essere sovrascritti nel costruttore)
# ---------------------------------------------------------------------------

# Distanza minima di sicurezza [m].
# Sotto questo valore la magnitudine della forza è clampata (evita 1/rho²→∞).
D_SAFE = 4.0

# Raggio di influenza [m].
# Oltre questa distanza la forza repulsiva è esattamente zero.
D_INFLUENCE = 16.0

# Guadagno repulsivo statico (termine Khatib).
K_REP = 350.0

# Guadagno del termine derivativo (velocity damping) [m²/s → m].
# Produce un offset aggiuntivo proporzionale alla velocità radiale di
# avvicinamento; si attiva solo quando i droni si stanno avvicinando.
K_DAMP = 300.0

# Offset massimo applicabile al setpoint [m].
MAX_OFFSET = 25.0

MAX_NEIGHBORS = 4


# ---------------------------------------------------------------------------
# Classe principale
# ---------------------------------------------------------------------------

class CollisionAvoidance:
    """
    Calcola l'offset repulsivo (dx, dz) da sommare al setpoint XZ del
    MultirotorController per allontanare il drone dagli altri dello swarm.
    """

    def __init__(self,
                 d_safe: float      = D_SAFE,
                 d_influence: float = D_INFLUENCE,
                 k_rep: float       = K_REP,
                 k_damp: float      = K_DAMP,
                 max_offset: float  = MAX_OFFSET,
                 max_neighbors: int = MAX_NEIGHBORS):
        """
        :param d_safe:      distanza di sicurezza — sotto questa la forza è massima [m]
        :param d_influence: raggio di influenza — oltre questo la forza è zero [m]
        :param k_rep:       guadagno repulsivo statico 
        :param k_damp:      guadagno del termine derivativo (velocity damping)
        :param max_offset:  clamp del vettore offset totale [m]
        """
        if d_safe >= d_influence:
            raise ValueError("d_safe deve essere < d_influence")

        self.d_safe        = d_safe
        self.d_influence   = d_influence
        self.k_rep         = k_rep
        self.k_damp        = k_damp
        self.max_offset    = max_offset
        self.max_neighbors = max_neighbors

        self.smooth_dx = 0.0
        self.smooth_dz = 0.0
    # ------------------------------------------------------------------
    def compute(self, my_pos: tuple, my_vel: tuple, swarm: dict) -> tuple:
        """
        Calcola il vettore repulsivo totale sommando i contributi di ogni drone.

        Per ogni drone j a distanza rho da me:

          Termine statico :
            F_stat = k_rep · (1/rho_eff - 1/rho0) · (1/rho_eff²)

          Termine derivativo :
            v_app  = -[(dx·(vx_me-vx_j) + dz·(vz_me-vz_j)) / rho]
            v_app  = max(0, v_app)   ← solo in avvicinamento
            F_damp = k_damp · v_app / rho_eff²

          Magnitudine totale: F_tot = F_stat + F_damp
          Direzione: versore û_away = [dx, dz] / rho  (da j verso me)

        :param my_pos: (x, z) del drone corrente in coordinate Godot/controller
        :param my_vel: (vx, vz) velocità del drone corrente (frame mondo Godot)
        :param swarm:  snapshot del dict _swarm di DroneAgent:
                       {drone_id: {"pos": [sx, sy, sz],
                                   "vel": [svx, svz],
                                   ...}, ...}
        :return:       (dx, dz) offset da sommare al setpoint [m]
        """
        total_x = 0.0
        total_z = 0.0

        mx, mz   = my_pos
        mvx, mvz = my_vel

        # 1. Raccogliamo i vicini e filtriamo quelli fuori dal raggio
        valid_neighbors = []
        d_influence_sq = self.d_influence * self.d_influence

        for drone_id, info in swarm.items():
            pos = info.get("pos")
            if pos is None:
                continue

            ox, oz = pos[0], pos[2]
            dx = mx - ox
            dz = mz - oz
            rho_sq = dx * dx + dz * dz

            # Scartiamo subito chi è fuori dal raggio (ottimizzazione senza math.sqrt)
            if rho_sq >= d_influence_sq:
                continue

            # Droni degenerati (quasi coincidenti)
            if rho_sq < 1e-6:
                total_x += self.max_offset * 0.1
                continue

            # Se è dentro il raggio, calcoliamo la radice vera e lo aggiungiamo alla lista
            rho = math.sqrt(rho_sq)
            valid_neighbors.append((rho, dx, dz, info))

        # 2. Ordiniamo la lista per distanza crescente (il primo elemento della tupla è rho)
        valid_neighbors.sort(key=lambda x: x[0])

        # 3. Applichiamo la repulsione SOLO sui K droni più vicini
        for rho, dx, dz, info in valid_neighbors[:self.max_neighbors]:
            rho_eff  = max(rho, self.d_safe)
            rho_eff2 = rho_eff * rho_eff

            # -- Termine statico Khatib 
            mag = (self.k_rep
                   * (1.0 / rho_eff - 1.0 / self.d_influence)
                   * (1.0 / rho_eff2))

            # -- Termine derivativo (velocity damping)
            vel_j = info.get("vel")
            if vel_j is not None and self.k_damp > 0.0:
                jvx, jvz  = vel_j[0], vel_j[1]
                v_approach = -((dx * (mvx - jvx) + dz * (mvz - jvz)) / rho)
                if v_approach > 0.3:
                    mag += self.k_damp * v_approach / rho_eff2

            # -- Proiezione sul versore unitario
            inv_rho = 1.0 / rho
            total_x += mag * dx * inv_rho
            total_z += mag * dz * inv_rho

        # --- Clamp della magnitudine totale ---
        mag_total = math.sqrt(total_x * total_x + total_z * total_z)
        if mag_total > self.max_offset:
            scale    = self.max_offset / mag_total
            total_x *= scale
            total_z *= scale

        # Filtro Esponenziale  
        alpha = 0.45
        self.smooth_dx += alpha * (total_x - self.smooth_dx)
        self.smooth_dz += alpha * (total_z - self.smooth_dz)

        return self.smooth_dx, self.smooth_dz
    
    def reset(self) -> None:
        """Azzera lo stato del filtro. Chiamare alle transizioni FSM rilevanti."""
        self.smooth_dx = 0.0
        self.smooth_dz = 0.0


