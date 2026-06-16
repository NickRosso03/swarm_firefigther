"""
monitor.py — Processo monitor per Swarm Firefighter.

════════════════════════════════════════════════════════════════════════
ARCHITETTURA GENERALE
════════════════════════════════════════════════════════════════════════

Il monitor è un processo Python SEPARATO e INDIPENDENTE dal sistema
di controllo (main.py + agenti). Non scrive nessuna variabile DDS, si
limita a leggere. Non può interferire con il controllo dei droni.

Schema dei due processi in parallelo:

  ┌─────────────────────────────────┐
  │  main.py (processo 1)           │
  │  ├── thread Drone-0             │
  │  ├── thread Drone-1             │
  │  │   ...  drone_agent.py        │──── pubblica DDS ──►┐
  │  └── thread Drone-N             │                     │
  └─────────────────────────────────┘                     │
                                                          ▼
                                                   [DDS broker in Godot]
                                                          │
  ┌─────────────────────────────────┐                     │
  │  monitor.py (processo 2)        │◄─── legge DDS ──────┘
  │  ├── main thread (sleep)        │
  │  └── reader thread (daemon)     │
  │        wait() + read() DDS      │
  │        push() → DroneBuffer     │
  │        calcola SwarmMetrics     │
  └─────────────────────────────────┘

════════════════════════════════════════════════════════════════════════
FLUSSO DATI INTERNO
════════════════════════════════════════════════════════════════════════

  Godot pubblica X,Y,Z,VX... ogni physics frame (60 Hz)
  drone_agent pubblica f1..f4, status, *_tgt, water_level ogni loop
  world.gd pubblica world/active_fires_count ogni frame
          │
          ▼
  DDS.wait("drone_0/X")     ← il reader thread si sincronizza
  DDS.read("drone_i/VAR")   ← legge tutte le altre variabili
          │
          ▼
  DroneBuffer.push()        ← applica remapping assi Godot→Python
                               scrive sulle deque circolari
  SwarmMetrics.push()       ← metriche aggregate di sciame:
                               distanza minima inter-drone,
                               livelli acqua, conteggio fuochi
          │
          └──► salva PNG su Ctrl+C / on-exit

════════════════════════════════════════════════════════════════════════
THREAD SAFETY
════════════════════════════════════════════════════════════════════════

Il reader thread scrive sui DroneBuffer e SwarmMetrics continuamente.
Il main thread li legge solo al momento del salvataggio.
Protezione: threading.Lock() in SwarmMonitor._lock.

════════════════════════════════════════════════════════════════════════
AVVIO
════════════════════════════════════════════════════════════════════════

  python monitor.py          →  modalità batch (salva su Ctrl+C)

════════════════════════════════════════════════════════════════════════
MODIFICA RICHIESTA IN world.gd
════════════════════════════════════════════════════════════════════════

In _update_hud(), dopo la riga che legge _fire_manager._active_fires,
aggiungere:

    DDS.publish("world/active_fires_count",
                DDS.DDS_TYPE_INT,
                _fire_manager._active_fires.size())

Questo pubblica ogni frame il numero di fuochi attivi, permettendo
al monitor di loggarlo senza toccare GDScript altrove.
"""

import os
import sys
import math
import signal
import time
import logging
import argparse
import threading
from collections import deque
from datetime import datetime

import matplotlib
matplotlib.use("Agg")   # backend headless — nessuna finestra, solo file
import matplotlib.pyplot as plt

# ─────────────────────────────────────────────────────────────────────────────
# PARAMETRI DI CONFIGURAZIONE
# ─────────────────────────────────────────────────────────────────────────────

DDS_HOST = '127.0.0.1'
DDS_PORT = 4444

# N_DRONES è impostato a runtime da --n (default 5).
# Non modificare questa riga: viene sovrascritta in _parse_args().
N_DRONES = 5

# Soglia di sicurezza dell'avoidance — usata come riferimento nel grafico.
D_SAFE = 4.0


def _parse_args():
    """
    Legge --n N e --label da riga di comando.
    Esempio: python monitor.py --n 8 --label damp_off
    Restituisce una tupla (numero_droni, etichetta).
    """
    parser = argparse.ArgumentParser(description="Monitor Swarm Firefighter")
    parser.add_argument(
        "--n", type=int, default=5,
        help="Numero di droni da monitorare (default: 5)"
    )
    parser.add_argument(
        "--label", type=str, default="",
        help="Etichetta della run (es. 'damp_on', 'damp_off', 'n8', 'seed42_rep1'). "
             "Viene appesa al nome della cartella di output per identificare la run."
    )
    args, _ = parser.parse_known_args()
    return args.n, args.label

# BUFFER_LEN: quanti campioni tenere in memoria per drone.
# A 60 Hz ogni secondo produce 60 campioni. Con BUFFER_LEN = 3600*5 = 18000
# teniamo ~5 minuti di storia. Quando il buffer è pieno, i campioni più vecchi
# vengono scartati automaticamente (comportamento della deque con maxlen).
BUFFER_LEN = 3600 * 5

# Cartella radice dove creare le sottocartelle run_YYYYMMDD_HHMMSS/
PLOTS_ROOT = "plots"

# ─────────────────────────────────────────────────────────────────────────────
# LOGGING
# ─────────────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [MONITOR] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("monitor")

# ─────────────────────────────────────────────────────────────────────────────
# IMPORT DIPENDENZE PROGETTO
# ─────────────────────────────────────────────────────────────────────────────
try:
    from dds import DDS
    from dataplot import DataPlotter, plot_multiple_save
except ImportError as e:
    log.error(
        f"Import fallito: {e}. "
        "Assicurati che monitor.py sia nella stessa cartella di dds.py e dataplot.py."
    )
    sys.exit(1)


# ═════════════════════════════════════════════════════════════════════════════
# DroneBuffer — buffer circolare per un singolo drone
# ═════════════════════════════════════════════════════════════════════════════

class DroneBuffer:
    """
    Contenitore di tutte le serie temporali monitorate per un drone.

    Usa collections.deque con maxlen fisso: quando il buffer è pieno
    e si aggiunge un nuovo elemento, il più vecchio viene rimosso
    automaticamente (politica FIFO). Non serve gestire la memoria a mano.

    Per ogni grandezza fisica teniamo DUE serie:
      - la variabile "corrente" (misurata da Godot o calcolata dal controller)
      - la variabile "target" (setpoint impostato dalla FSM del drone)
    Questo permette i grafici curr vs target (es. position control).
    """

    def __init__(self, drone_id: int, maxlen: int):
        self.id = drone_id
        mk = lambda: deque(maxlen=maxlen)

        # Asse temporale: tempo in secondi dall'avvio del monitor.
        self.t = mk()

        # ── Posizione [m] ─────────────────────────────────────────────────────
        # Convenzione: x/y = orizzontali, z = altitudine (remappata da Godot Y)
        self.x = mk();  self.x_tgt = mk()
        self.y = mk();  self.y_tgt = mk()
        self.z = mk();  self.z_tgt = mk()

        # ── Velocità [m/s] ────────────────────────────────────────────────────
        self.vx = mk();  self.vx_tgt = mk()
        self.vy = mk();  self.vy_tgt = mk()
        self.vz = mk();  self.vz_tgt = mk()

        # ── Assetto [rad] ─────────────────────────────────────────────────────
        self.roll  = mk();  self.roll_tgt  = mk()
        self.pitch = mk();  self.pitch_tgt = mk()
        self.yaw   = mk()

        # ── Forze motori [N] ──────────────────────────────────────────────────
        self.f1 = mk();  self.f2 = mk()
        self.f3 = mk();  self.f4 = mk()

        # ── Stato FSM (codice numerico float) ─────────────────────────────────
        self.status = mk()

        # ── Livello acqua [0..100] ────────────────────────────────────────────
        self.water = mk()

    def push(self, t: float, vals: dict):
        """
        Aggiunge un campione alle deque, applicando il remapping degli assi.

        Tabella di remapping Godot → buffer Python:
          Grandezza fisica   │ Topic DDS Godot │ Attributo buffer
          ───────────────────┼─────────────────┼─────────────────
          Altitudine curr    │ drone_i/Y       │ self.z
          Altitudine target  │ drone_i/z_tgt   │ self.z_tgt
          Posizione X curr   │ drone_i/X       │ self.x
          Posizione Y curr   │ drone_i/Z       │ self.y
          Vel. verticale     │ drone_i/VY      │ self.vz
          Vel. X             │ drone_i/VX      │ self.vx
          Vel. Y (oriz.)     │ drone_i/VZ      │ self.vy
          Roll curr          │ drone_i/TZ      │ self.roll
          Pitch curr         │ drone_i/TX      │ self.pitch
          Yaw curr           │ drone_i/TY      │ self.yaw

        :param t:    tempo in secondi dall'avvio del monitor
        :param vals: dict {nome_variabile_short: valore_float}
        """
        self.t.append(t)

        def g(k): return vals.get(k, 0.0)

        # Altitudine: Y_Godot → z buffer
        self.z.append(g("Y"));      self.z_tgt.append(g("z_tgt"))
        # Posizione X: X_Godot → x buffer
        self.x.append(g("X"));      self.x_tgt.append(g("x_tgt"))
        # Posizione Y orizzontale: Z_Godot → y buffer
        self.y.append(g("Z"));      self.y_tgt.append(g("y_tgt"))
        # Velocità verticale: VY_Godot → vz buffer
        self.vz.append(g("VY"));    self.vz_tgt.append(g("vz_tgt"))
        # Velocità X: VX_Godot → vx buffer
        self.vx.append(g("VX"));    self.vx_tgt.append(g("vx_tgt"))
        # Velocità Y orizzontale: VZ_Godot → vy buffer
        self.vy.append(g("VZ"));    self.vy_tgt.append(g("vy_tgt"))
        # Assetto
        self.roll.append(g("TZ"));  self.roll_tgt.append(g("roll_tgt"))
        self.pitch.append(g("TX")); self.pitch_tgt.append(g("pitch_tgt"))
        self.yaw.append(g("TY"))
        # Forze e stato FSM: nessun remap necessario
        self.f1.append(g("f1"));    self.f2.append(g("f2"))
        self.f3.append(g("f3"));    self.f4.append(g("f4"))
        self.status.append(g("status"))
        # Acqua
        self.water.append(g("water_level"))

    def has_data(self) -> bool:
        """True se almeno un campione è stato ricevuto."""
        return len(self.t) > 0


# ═════════════════════════════════════════════════════════════════════════════
# SwarmMetrics — metriche aggregate dello sciame
# ═════════════════════════════════════════════════════════════════════════════

class SwarmMetrics:
    """
    Serie temporali che descrivono il comportamento collettivo dello sciame,
    non riducibili al singolo drone.

    Calcolate nel reader loop ad ogni campione:
      - min_inter_drone_dist : distanza minima istantanea tra qualsiasi coppia
                               di droni nel piano XZ. Valida il sistema
                               anticollisione: non deve mai scendere sotto D_SAFE.
      - active_fires_count   : numero di fuochi attivi nell'ambiente, pubblicato
                               da world.gd via DDS. Mostra la dinamica della
                               missione nel tempo.

    Struttura: ogni metrica è una deque di tuple (t, valore).
    Il salvataggio converte le deque in due liste separate (t_list, val_list)
    prima di passarle a DataPlotter.
    """

    def __init__(self, maxlen: int):
        mk = lambda: deque(maxlen=maxlen)
        self.t_min_dist      = mk()   # timestamp per min_inter_drone_dist
        self.min_dist        = mk()   # distanza minima inter-drone [m]
        self.min_pair_status = mk()   # (status_i, status_j) della coppia più vicina
        self.t_fires         = mk()   # timestamp per active_fires_count
        self.active_fires    = mk()   # numero fuochi attivi

        # Conteggio incendi spenti durante la sessione.
        # _seen_ids evita di contare lo stesso fire_id più volte
        # (il topic fire_extinguished rimane a un valore non-zero per
        # più frame consecutivi prima che il drone lo azzeri).
        self.fires_extinguished_count: int = 0
        self._seen_ext_ids: set = set()

    def push_distances(self, t: float, positions: list, statuses: list = None):
        """
        Calcola e registra la distanza minima tra tutti i droni, ricordando
        anche gli stati FSM della coppia che realizza il minimo.

        Questo permette in fase di analisi di classificare le incursioni sotto
        D_SAFE in due categorie:
          - "transito":  nessuno dei due droni è in SUPPRESSING → anticollisione
                         attiva su entrambi → l'episodio è attribuibile al campo
                         repulsivo (rilevante per l'ablazione del damping);
          - "standoff":  almeno uno dei due è in SUPPRESSING → anticollisione
                         disattivata → la separazione è affidata alla geometria
                         ad angolo aureo, non al campo repulsivo.

        :param t:         timestamp corrente [s]
        :param positions: lista di tuple (x, z) — coordinate orizzontali
                          dei droni che hanno già ricevuto almeno un campione.
                          Droni non ancora inizializzati sono esclusi.
        :param statuses:  lista degli stati FSM (codici float), nello stesso
                          ordine di positions. Opzionale per retrocompatibilità.
        """
        if len(positions) < 2:
            return
        min_d    = float('inf')
        min_pair = (None, None)
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                d = math.dist(positions[i], positions[j])
                if d < min_d:
                    min_d = d
                    if statuses is not None:
                        min_pair = (statuses[i], statuses[j])
        self.t_min_dist.append(t)
        self.min_dist.append(min_d)
        self.min_pair_status.append(min_pair)

    def push_fires(self, t: float, count: float):
        """
        Registra il numero di fuochi attivi al tempo t.

        :param t:     timestamp corrente [s]
        :param count: valore letto da world/active_fires_count via DDS.
                      0.0 se il topic non è ancora arrivato.
        """
        self.t_fires.append(t)
        self.active_fires.append(count)

    def push_extinguished(self, fire_id: float):
        """
        Registra un incendio spento se il suo ID non è già stato contato.
        Chiamare ogni frame per ogni drone che pubblica fire_extinguished != 0.

        :param fire_id: identificativo del fuoco spento (float != 0.0)
        """
        if fire_id != 0.0 and fire_id not in self._seen_ext_ids:
            self._seen_ext_ids.add(fire_id)
            self.fires_extinguished_count += 1

    def has_dist_data(self) -> bool:
        return len(self.min_dist) > 0

    def has_fire_data(self) -> bool:
        return len(self.active_fires) > 0


# ═════════════════════════════════════════════════════════════════════════════
# SwarmMonitor — cuore del sistema di monitoring
# ═════════════════════════════════════════════════════════════════════════════

class SwarmMonitor:
    """
    Gestisce la connessione DDS, il reader thread e il salvataggio PNG.

    Attributi pubblici:
      self.buffers      → dict {drone_id: DroneBuffer}
      self.swarm_metrics → SwarmMetrics
      self._lock        → threading.Lock da acquisire prima di leggere i buffer
    """

    def __init__(self, n_drones: int, label: str = ""):
        self.n_drones = n_drones
        self.label    = label
        self.dds = DDS(DDS_HOST, DDS_PORT)

        self.buffers       = {i: DroneBuffer(i, BUFFER_LEN) for i in range(n_drones)}
        self.swarm_metrics = SwarmMetrics(BUFFER_LEN)

        self._lock    = threading.Lock()
        self._running = False
        self._t0      = time.time()

        # Cartella di output con timestamp: ogni run ha la propria cartella.
        # Se è fornita una label, viene appesa per identificare la run
        # (utile per le run dell'ablazione, es. run_..._damp_off).
        suffix = f"_{label}" if label else ""
        self._run_dir = os.path.join(
            PLOTS_ROOT,
            datetime.now().strftime("run_%Y%m%d_%H%M%S") + suffix
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Setup DDS
    # ─────────────────────────────────────────────────────────────────────────

    def _setup_dds(self):
        """
        Avvia il client DDS e si iscrive a tutte le variabili di interesse.

        Variabili per drone:
          - Stato fisico (da Godot):   X,Y,Z, VX,VY,VZ, TX,TY,TZ
          - Forze (da drone_agent):    f1,f2,f3,f4
          - Stato FSM:                 status
          - Target interni:            z_tgt, vz_tgt, x_tgt, y_tgt,
                                       vx_tgt, vy_tgt, roll_tgt, pitch_tgt
          - Acqua:                     water_level

        Variabili di sciame:
          - world/active_fires_count   (da world.gd — vedi istruzioni in cima)
        """
        self.dds.start()
        all_vars = []
        for i in range(self.n_drones):
            p = f"drone_{i}"
            all_vars += [
                f"{p}/X",  f"{p}/Y",  f"{p}/Z",
                f"{p}/VX", f"{p}/VY", f"{p}/VZ",
                f"{p}/TX", f"{p}/TY", f"{p}/TZ",
                f"{p}/f1", f"{p}/f2", f"{p}/f3", f"{p}/f4",
                f"{p}/status",
                f"{p}/z_tgt",    f"{p}/vz_tgt",
                f"{p}/x_tgt",    f"{p}/y_tgt",
                f"{p}/vx_tgt",   f"{p}/vy_tgt",
                f"{p}/roll_tgt", f"{p}/pitch_tgt",
                f"{p}/water_level",
                f"{p}/fire_extinguished",
            ]
        all_vars.append("world/active_fires_count")
        self.dds.subscribe(all_vars)
        log.info(f"Iscritto a {len(all_vars)} variabili DDS per {self.n_drones} droni.")

    # ─────────────────────────────────────────────────────────────────────────
    # Reader thread
    # ─────────────────────────────────────────────────────────────────────────

    def _read_loop(self):
        """
        Loop di lettura eseguito nel thread "monitor-reader" (daemon).

        SINCRONIZZAZIONE: usa dds.wait("drone_0/X") come punto di sync.
        Godot pubblica drone_0/X ogni physics frame (60 Hz): il reader
        si sveglia ~60 volte al secondo, legge tutte le variabili di tutti
        i droni, calcola le metriche di sciame, e torna ad aspettare.

        PER LE METRICHE DI SCIAME:
          1. Distanza minima inter-drone: calcolata sulle posizioni XZ
             dei droni che hanno già almeno un campione nel buffer.
             Serve per validare il sistema anticollisione.
          2. Livello acqua: già incluso nel DroneBuffer (campo water),
             aggregato in SwarmMetrics per il grafico multi-drone.
          3. Fuochi attivi: letto da world/active_fires_count, topic
             pubblicato da world.gd (vedi istruzioni in cima al file).
        """
        log.info("Read loop avviato. In attesa del primo campione drone_0...")
        while self._running:
            try:
                self.dds.wait("drone_0/X")
            except Exception:
                time.sleep(0.05)
                continue

            t = time.time() - self._t0

            with self._lock:
                # ── Lettura telemetria per singolo drone ──────────────────────
                # ── Lettura telemetria per singolo drone ──────────────────────
                for i in range(self.n_drones):
                    p = f"drone_{i}"
                    
                    # FIX: Ignora l'aggiornamento se Godot non ha ancora inviato la posizione di questo drone.
                    # Questo evita di iniettare (0.0, 0.0) fittizi prima del vero spawn.
                    if self.dds.read(f"{p}/X") is None:
                        continue

                    vals = {}
                    for var in [
                        "X", "Y", "Z", "VX", "VY", "VZ", "TX", "TY", "TZ",
                        "f1", "f2", "f3", "f4", "status",
                        "z_tgt", "vz_tgt", "x_tgt", "y_tgt",
                        "vx_tgt", "vy_tgt", "roll_tgt", "pitch_tgt",
                        "water_level",
                    ]:
                        raw = self.dds.read(f"{p}/{var}")
                        vals[var] = float(raw) if raw is not None else 0.0
                    self.buffers[i].push(t, vals)

                # ── Metrica 1: distanza minima inter-drone ────────────────────
                # Insieme alle posizioni raccoglie gli stati FSM (stesso ordine)
                # per classificare le incursioni in transito vs standoff.
                positions_xz = []
                statuses     = []
                for i in range(self.n_drones):
                    buf = self.buffers[i]
                    if buf.has_data():
                        positions_xz.append((buf.x[-1], buf.y[-1]))
                        statuses.append(buf.status[-1])
                self.swarm_metrics.push_distances(t, positions_xz, statuses)

                # ── Metrica 2: fuochi attivi ──────────────────────────────────
                raw_fires = self.dds.read("world/active_fires_count")
                fire_count = float(raw_fires) if raw_fires is not None else 0.0
                self.swarm_metrics.push_fires(t, fire_count)

                # ── Metrica 3: incendi spenti (conteggio cumulativo) ──────────
                # Ogni drone pubblica il fire_id appena spento su fire_extinguished.
                # push_extinguished() usa un set interno per evitare doppi conteggi
                # (lo stesso ID può rimanere sul topic per più frame consecutivi).
                for i in range(self.n_drones):
                    raw_ext = self.dds.read(f"drone_{i}/fire_extinguished")
                    if raw_ext is not None:
                        self.swarm_metrics.push_extinguished(float(raw_ext))

    # ─────────────────────────────────────────────────────────────────────────
    # Salvataggio PNG
    # ─────────────────────────────────────────────────────────────────────────

    def save_all(self):
        """
        Salva i grafici PNG di tutta la sessione per tutti i droni
        e i grafici di sciame aggregati.

        Struttura output:
          plots/run_YYYYMMDD_HHMMSS/
            drone_0/
              altitude.png
              velocity_z.png
              position_x.png
              position_y.png
              velocity_xy.png
              attitude.png
              forces.png
              water.png          ← nuovo
            drone_1/ ...
            swarm/
              min_inter_drone_dist.png
              resources.png              (acqua di tutti i droni + fuochi attivi)
              trajectories_xy.png        ← nuovo: vista dall'alto delle traiettorie
              summary.txt                (metriche chiave, incl. % tempo < D_SAFE)
        """
        with self._lock:
            n_saved = 0
            for i in range(self.n_drones):
                buf = self.buffers[i]
                if not buf.has_data():
                    log.warning(f"Drone {i}: nessun dato ricevuto, skip.")
                    continue
                drone_dir = os.path.join(self._run_dir, f"drone_{i}")
                os.makedirs(drone_dir, exist_ok=True)
                t = list(buf.t)
                self._save_drone(buf, t, drone_dir, i)
                n_saved += 1

            # Grafici di sciame
            swarm_dir = os.path.join(self._run_dir, "swarm")
            os.makedirs(swarm_dir, exist_ok=True)
            self._save_swarm(swarm_dir)
            self._save_trajectories_xy(swarm_dir)

            # Summary testuale
            self._save_summary(swarm_dir)

        log.info(f"Salvati grafici per {n_saved} droni in: {self._run_dir}/")

    def _save_drone(self, buf: DroneBuffer, t: list, out_dir: str, drone_id: int):
        """
        Costruisce e salva tutti i grafici per un singolo drone.
        """
        prefix = f"Drone {drone_id}"

        # ── QUOTA ─────────────────────────────────────────────────────────────
        d = DataPlotter(); d.set_x("t [s]")
        d.add_y("z", "z curr [m]"); d.add_y("zt", "z target [m]")
        for k, tv in enumerate(t):
            d.append_x(tv)
            d.append_y("z", buf.z[k])
            d.append_y("zt", buf.z_tgt[k])
        d.save(os.path.join(out_dir, "altitude.png"), title=f"{prefix} — Quota")

        # ── VELOCITÀ VERTICALE ────────────────────────────────────────────────
        d = DataPlotter(); d.set_x("t [s]")
        d.add_y("vz", "vz curr [m/s]"); d.add_y("vzt", "vz target [m/s]")
        for k, tv in enumerate(t):
            d.append_x(tv)
            d.append_y("vz", buf.vz[k])
            d.append_y("vzt", buf.vz_tgt[k])
        d.save(os.path.join(out_dir, "velocity_z.png"),
               title=f"{prefix} — Velocita verticale")

        # ── POSIZIONE X ───────────────────────────────────────────────────────
        d = DataPlotter(); d.set_x("t [s]")
        d.add_y("x", "x curr [m]"); d.add_y("xt", "x target [m]")
        for k, tv in enumerate(t):
            d.append_x(tv)
            d.append_y("x", buf.x[k])
            d.append_y("xt", buf.x_tgt[k])
        d.save(os.path.join(out_dir, "position_x.png"),
               title=f"{prefix} — Posizione X")

        # ── POSIZIONE Y ───────────────────────────────────────────────────────
        d = DataPlotter(); d.set_x("t [s]")
        d.add_y("y", "y curr [m]"); d.add_y("yt", "y target [m]")
        for k, tv in enumerate(t):
            d.append_x(tv)
            d.append_y("y", buf.y[k])
            d.append_y("yt", buf.y_tgt[k])
        d.save(os.path.join(out_dir, "position_y.png"),
               title=f"{prefix} — Posizione Y")

        # ── VELOCITÀ XY ───────────────────────────────────────────────────────
        dvx = DataPlotter(); dvx.set_x("t [s]")
        dvx.add_y("vx", "vx curr [m/s]"); dvx.add_y("vxt", "vx target [m/s]")
        dvy = DataPlotter(); dvy.set_x("t [s]")
        dvy.add_y("vy", "vy curr [m/s]"); dvy.add_y("vyt", "vy target [m/s]")
        for k, tv in enumerate(t):
            dvx.append_x(tv); dvy.append_x(tv)
            dvx.append_y("vx",  buf.vx[k]);  dvx.append_y("vxt", buf.vx_tgt[k])
            dvy.append_y("vy",  buf.vy[k]);  dvy.append_y("vyt", buf.vy_tgt[k])
        plot_multiple_save([dvx, dvy], os.path.join(out_dir, "velocity_xy.png"),
                           title=f"{prefix} — Velocita XY")
        
        # in _save_drone, dopo il grafico velocity_xy
        speed = [math.hypot(a, b) for a, b in zip(buf.vx, buf.vy)]
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(t, speed, 'r-', linewidth=1.2, label='|v| [m/s]')
        ax.axhline(5.0, color='gray', ls='--', lw=0.8, label='TRAJ_VMAX = 5 m/s')
        ax.axhline(6.0, color='gray', ls=':',  lw=0.8, label='FIRE_VMAX = 6 m/s')
        ax.set_xlabel("t [s]"); ax.set_ylabel("velocità [m/s]")
        ax.legend(); ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, "speed.png"), dpi=100)
        plt.close(fig)

        # ── ASSETTO ───────────────────────────────────────────────────────────
        dr = DataPlotter(); dr.set_x("t [s]")
        dr.add_y("r", "roll curr [deg]"); dr.add_y("rt", "roll target [deg]")
        dp = DataPlotter(); dp.set_x("t [s]")
        dp.add_y("p", "pitch curr [deg]"); dp.add_y("pt", "pitch target [deg]")
        for k, tv in enumerate(t):
            dr.append_x(tv); dp.append_x(tv)
            dr.append_y("r",  math.degrees(buf.roll[k]))
            dr.append_y("rt", math.degrees(buf.roll_tgt[k]))
            dp.append_y("p",  math.degrees(buf.pitch[k]))
            dp.append_y("pt", math.degrees(buf.pitch_tgt[k]))
        plot_multiple_save([dr, dp], os.path.join(out_dir, "attitude.png"),
                           title=f"{prefix} — Assetto")

        # ── FORZE MOTORI ──────────────────────────────────────────────────────
        d = DataPlotter(); d.set_x("t [s]")
        d.add_y("f1", "f1 [N]"); d.add_y("f2", "f2 [N]")
        d.add_y("f3", "f3 [N]"); d.add_y("f4", "f4 [N]")
        for k, tv in enumerate(t):
            d.append_x(tv)
            d.append_y("f1", buf.f1[k]); d.append_y("f2", buf.f2[k])
            d.append_y("f3", buf.f3[k]); d.append_y("f4", buf.f4[k])
        d.save(os.path.join(out_dir, "forces.png"),
               title=f"{prefix} — Forze motori")

        # ── ACQUA (singolo drone) ─────────────────────────────────────────────
        # Mostra il ciclo carica/scarica del drone: scende durante SUPPRESSING,
        # risale durante REFUELING. Utile per verificare la gestione delle risorse.
        d = DataPlotter(); d.set_x("t [s]")
        d.add_y("w", "water [%]")
        for k, tv in enumerate(t):
            d.append_x(tv)
            d.append_y("w", buf.water[k])
        d.save(os.path.join(out_dir, "water.png"),
               title=f"{prefix} — Livello acqua")

    def _save_swarm(self, out_dir: str):
        """
        Salva i grafici aggregati dello sciame.

        Due grafici:
          1. min_inter_drone_dist.png — distanza minima istantanea tra droni,
             con linea tratteggiata rossa a D_SAFE come soglia di riferimento.

          2. resources.png — grafico unificato a due subplot verticali:
             (sopra) livelli acqua di tutti i droni sovrapposti;
             (sotto) numero di fuochi attivi nel tempo.
             Lo stesso asse temporale permette di correlare le fasi di
             ricarica con la dinamica degli incendi.
        """
        sm = self.swarm_metrics

        # ── 1. DISTANZA MINIMA INTER-DRONE ────────────────────────────────────
        if sm.has_dist_data():
            t_d  = list(sm.t_min_dist)
            dist = list(sm.min_dist)

            fig, ax = plt.subplots(figsize=(12, 4))
            ax.plot(t_d, dist, 'b-', linewidth=0.8, label="dist. min [m]")
            ax.axhline(y=D_SAFE, color='r', linestyle='--', linewidth=1.2,
                       label=f"D_SAFE = {D_SAFE} m")
            ax.set_xlabel("t [s]")
            ax.set_ylabel("distanza [m]")
            ax.set_title(f"Sciame ({self.n_drones} droni) — Distanza minima inter-drone")
            ax.legend(fontsize=9)
            ax.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig(os.path.join(out_dir, "min_inter_drone_dist.png"), dpi=100)
            plt.close(fig)
        else:
            log.warning("SwarmMetrics: nessun dato distanza, skip.")

        # ── 2. GRAFICO RISORSE UNIFICATO ──────────────────────────────────────
        drones_with_data = [
            (i, self.buffers[i]) for i in range(self.n_drones)
            if self.buffers[i].has_data()
        ]

        has_water = bool(drones_with_data)
        has_fires = sm.has_fire_data()

        if has_water or has_fires:
            fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
            fig.suptitle(
                f"Sciame ({self.n_drones} droni) — Gestione delle risorse",
                fontsize=11, fontweight='bold'
            )

            # ── Subplot superiore: livelli acqua ──────────────────────────────
            ax_water = axes[0]
            if has_water:
                min_len = min(len(buf.t) for _, buf in drones_with_data)
                t_ref   = list(self.buffers[drones_with_data[0][0]].t)[:min_len]
                colors  = plt.cm.tab10.colors
                for idx, (i, buf) in enumerate(drones_with_data):
                    water_vals = list(buf.water)[:min_len]
                    ax_water.plot(
                        t_ref, water_vals,
                        color=colors[idx % len(colors)],
                        linewidth=0.9,
                        label=f"drone {i}"
                    )
                ax_water.set_ylabel("acqua [%]")
                ax_water.set_ylim(0, 105)
                ax_water.legend(fontsize=8, ncol=min(self.n_drones, 5))
                ax_water.grid(True, alpha=0.3)
            else:
                ax_water.text(0.5, 0.5, "nessun dato acqua",
                              ha='center', va='center', transform=ax_water.transAxes)

            # ── Subplot inferiore: fuochi attivi ──────────────────────────────
            ax_fires = axes[1]
            if has_fires:
                t_f   = list(sm.t_fires)
                fires = list(sm.active_fires)
                ax_fires.plot(t_f, fires, 'r-', linewidth=0.9, label="fuochi attivi")
                ax_fires.set_xlabel("t [s]")
                ax_fires.set_ylabel("fuochi attivi")
                ax_fires.legend(fontsize=9)
                ax_fires.grid(True, alpha=0.3)
            else:
                ax_fires.text(0.5, 0.5, "nessun dato fuochi",
                              ha='center', va='center', transform=ax_fires.transAxes)
                ax_fires.set_xlabel("t [s]")

            plt.tight_layout()
            plt.savefig(os.path.join(out_dir, "resources.png"), dpi=100)
            plt.close(fig)
        else:
            log.warning("SwarmMetrics: nessun dato per il grafico risorse, skip.")

    def _save_trajectories_xy(self, out_dir: str):
        """
        Vista dall'alto (top-down) delle traiettorie nel piano XY di tutti i droni.

        Ogni drone ha un colore distinto; un marcatore tondo segna la posizione
        iniziale. Questo grafico rende immediatamente leggibili due aspetti che le
        serie temporali non mostrano:
          - la suddivisione boustrophedon dell'arena in settori (una striscia per
            drone, percorsa a serpentina);
          - la convergenza dei droni sugli slot di standoff ad angolo aureo durante
            la soppressione cooperativa di un incendio.

        Usa i buffer x (Godot X) e y (Godot Z) già rimappati, quindi rappresenta
        correttamente il piano orizzontale visto dall'alto. L'asse è in scala
        isometrica (aspect='equal') per non deformare le distanze.
        """
        drones = [
            (i, self.buffers[i]) for i in range(self.n_drones)
            if self.buffers[i].has_data()
        ]
        if not drones:
            log.warning("Trajectory XY: nessun dato, skip.")
            return

        fig, ax = plt.subplots(figsize=(8, 8))
        colors = plt.cm.tab10.colors
        for idx, (i, buf) in enumerate(drones):
            xs, ys = list(buf.x), list(buf.y)
            c = colors[idx % len(colors)]
            ax.plot(xs, ys, '-', color=c, linewidth=0.7, alpha=0.85,
                    label=f"drone {i}")
            # Marcatore sulla posizione iniziale
            ax.plot(xs[0], ys[0], 'o', color=c, markersize=6,
                    markeredgecolor='k', markeredgewidth=0.5)

        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(f"Sciame ({self.n_drones} droni) — Traiettorie (vista dall'alto)")
        ax.set_aspect('equal', adjustable='box')
        ax.legend(fontsize=8, ncol=min(self.n_drones, 5))
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, "trajectories_xy.png"), dpi=100)
        plt.close(fig)

    def _save_summary(self, out_dir: str):
        """
        Salva summary.txt con le metriche chiave della sessione.
        Usato per costruire la tabella di confronto N-droni nel Cap. 4.

        La metrica "Tempo sotto D_SAFE" quantifica l'affermazione del Cap. 2
        secondo cui i droni scendono sotto la distanza di sicurezza solo per
        brevi istanti: riporta la percentuale di campioni con distanza minima
        inferiore a D_SAFE e il numero di incursioni (episodi consecutivi
        distinti sotto soglia). Va letta insieme alla traccia status per
        distinguere gli avvicinamenti in transito (anticollisione attiva) da
        quelli allo standoff (anticollisione disattivata, separazione garantita
        dalla geometria ad angolo aureo).
        """
        sm   = self.swarm_metrics
        dur  = time.time() - self._t0

        # Codice FSM di SUPPRESSING (deve coincidere con drone_agent.py / world.gd)
        SUPPRESSING = 4.0

        if sm.has_dist_data():
            dists        = list(sm.min_dist)
            pairs        = list(sm.min_pair_status)
            min_dist_val = min(dists)
            n_below      = sum(1 for d in dists if d < D_SAFE)
            pct_below    = 100.0 * n_below / len(dists)

            # Incursioni: episodi consecutivi distinti sotto soglia, classificati:
            #   standoff → in almeno un campione dell'episodio la coppia più
            #              vicina include un drone in SUPPRESSING (anticollisione
            #              disattivata, separazione affidata alla geometria);
            #   transito → in nessun campione compare SUPPRESSING (anticollisione
            #              attiva su entrambi → attribuibile al campo repulsivo).
            # Criterio conservativo: basta un campione SUPPRESSING per marcare
            # l'episodio come standoff, così le incursioni attribuite al damping
            # nell'ablazione sono stimate per difetto.
            inc_transit, inc_standoff = 0, 0
            in_episode, ep_has_supp   = False, False
            for d, pair in zip(dists, pairs):
                below = d < D_SAFE
                if below:
                    if not in_episode:
                        in_episode, ep_has_supp = True, False
                    if pair and SUPPRESSING in pair:
                        ep_has_supp = True
                elif in_episode:
                    if ep_has_supp:
                        inc_standoff += 1
                    else:
                        inc_transit += 1
                    in_episode = False
            if in_episode:  # episodio ancora aperto a fine run
                if ep_has_supp:
                    inc_standoff += 1
                else:
                    inc_transit += 1
            incursions = inc_transit + inc_standoff
        else:
            min_dist_val, pct_below = float('nan'), float('nan')
            incursions, inc_transit, inc_standoff = 0, 0, 0

        avg_fires = (sum(sm.active_fires) / len(sm.active_fires)
                     if sm.has_fire_data() else float('nan'))

        lines = [
            f"Label               : {self.label or '-'}",
            f"N_DRONES            : {self.n_drones}",
            f"Durata sessione     : {dur:.1f} s  ({dur/60:.1f} min)",
            f"Incendi spenti      : {sm.fires_extinguished_count}",
            f"Dist. min osservata : {min_dist_val:.2f} m  (D_SAFE = {D_SAFE} m)",
            f"Tempo sotto D_SAFE  : {pct_below:.2f} %  ({incursions} incursioni: "
            f"{inc_transit} transito, {inc_standoff} standoff)",
            f"Media fuochi attivi : {avg_fires:.2f}",
        ]

        path = os.path.join(out_dir, "summary.txt")
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
        log.info("summary.txt salvato:")
        for l in lines:
            log.info(f"  {l}")

    # ─────────────────────────────────────────────────────────────────────────
    # Avvio / arresto
    # ─────────────────────────────────────────────────────────────────────────

    def start(self):
        """
        Avvia il client DDS e il reader thread (daemon).
        """
        self._setup_dds()
        self._running = True
        threading.Thread(
            target=self._read_loop, name="monitor-reader", daemon=True
        ).start()

    def stop(self):
        """Ferma il reader thread e chiude il socket DDS."""
        self._running = False
        try:
            self.dds.stop()
        except Exception:
            pass


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════

def _run_monitor():
    """
    Avvio in modalità batch.

    Uso: python monitor.py --n 8

    Il main thread dorme in un loop infinito (time.sleep(1)).
    Tutto il lavoro avviene nel reader thread in background.
    Il salvataggio avviene su SIGINT (Ctrl+C) o SIGTERM.
    """
    n_drones, label = _parse_args()
    log.info(f"Avvio monitor per {n_drones} droni."
             + (f" Label: '{label}'." if label else ""))

    monitor = SwarmMonitor(n_drones, label)
    monitor.start()

    def _on_sigint(sig, frame):
        log.info("SIGINT — salvo grafici...")
        monitor.save_all()
        monitor.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _on_sigint)
    signal.signal(signal.SIGTERM, _on_sigint)

    log.info(f"Monitor batch attivo. Output: {monitor._run_dir}/  |  Ctrl+C per salvare.")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    _run_monitor()