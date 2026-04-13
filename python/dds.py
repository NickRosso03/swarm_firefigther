"""
dds.py — Data Distribution Service, protocollo binario.

differenze:
  - publish() non richiede più di specificare DDS_TYPE_*: lo deduce
    automaticamente dal tipo Python (int → DDS_TYPE_INT, altrimenti FLOAT)
  - Aggiunto keep_alive() automatico in background per evitare TTL expiry
    (il server Godot disconnette i peer dopo 2 secondi senza keep-alive)

Formato pacchetti (identico al prof):
  SUBSCRIBE : [0x81, n_vars, len, name, len, name, ...]
  PUBLISH   : [0x82, type, len, name, value_4bytes]
  KEEP_ALIVE: [0x80]
"""

import socket
import threading
import select
import io
import time
import struct


# ---------------------------------------------------------------------------
# Costanti di protocollo 
# ---------------------------------------------------------------------------
COMMAND_KEEP_ALIVE = 0x80
COMMAND_SUBSCRIBE  = 0x81
COMMAND_PUBLISH    = 0x82

DDS_TYPE_UNKNOWN = 0
DDS_TYPE_INT     = 1
DDS_TYPE_FLOAT   = 2

KEEP_ALIVE_INTERVAL = 1.0   # secondi — deve essere < TIME_TO_LIVE (2s) in dds.gd


class _MonitoredVariable:
    """Variabile thread-safe con supporto wait/notify """

    def __init__(self):
        self._lock      = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self.value      = None

    def get(self):
        with self._lock:
            return self.value

    def wait_value(self):
        with self._condition:
            self._condition.wait()
            return self.value

    def notify(self, val):
        with self._condition:
            self.value = val
            self._condition.notify_all()


class DDS(threading.Thread):
    """
    Client DDS che parla con il broker centrale in Godot (dds.gd).

    Uso :
        dds = DDS()
        dds.start()                         # avvia thread ricezione + keep-alive
        dds.subscribe(['Z', 'VZ', 'tick'])
        dds.wait('tick')                    # blocca finché Godot non pubblica 'tick'
        z = dds.read('Z')
        dds.publish('f1', 3.14)             # tipo dedotto automaticamente
    """

    # Ri-esporta le costanti per compatibilità con codice del prof
    DDS_TYPE_UNKNOWN = DDS_TYPE_UNKNOWN
    DDS_TYPE_INT     = DDS_TYPE_INT
    DDS_TYPE_FLOAT   = DDS_TYPE_FLOAT

    def __init__(self, host: str = '127.0.0.1', port: int = 4444):
        super().__init__(daemon=True)
        self._host      = host
        self._port      = port
        self._variables: dict[str, _MonitoredVariable] = {}
        self._running   = False

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind su porta effimera per ricevere le pubblicazioni dal broker
        self._sock.bind(('', 0))

    # ------------------------------------------------------------------
    # Overload start() per accettare host/port opzionali 
    # ------------------------------------------------------------------

    def start(self, remote_host: str = None, remote_port: int = None):  # type: ignore[override]
        if remote_host is not None:
            self._host = remote_host
        if remote_port is not None:
            self._port = remote_port
        self._running = True
        super().start()

    def stop(self):
        self._running = False

    # ------------------------------------------------------------------
    # API pubblica
    # ------------------------------------------------------------------

    def subscribe(self, var_list: list[str]):
        """
        Informa il broker che vogliamo ricevere le variabili in var_list.
        Può essere chiamato più volte (accumula le sottoscrizioni).
        """
        buf = io.BytesIO()
        buf.write(bytes([COMMAND_SUBSCRIBE, len(var_list)]))
        for name in var_list:
            encoded = name.encode('utf-8')
            buf.write(bytes([len(encoded)]))
            buf.write(encoded)
            self._variables[name] = _MonitoredVariable()
        self._sock.sendto(buf.getvalue(), (self._host, self._port))

    def publish(self, name: str, value, dtype: int = None):
        """
        Pubblica una variabile verso il broker Godot.

        dtype può essere omesso: se value è int → DDS_TYPE_INT,
        altrimenti DDS_TYPE_FLOAT.
        """
        if dtype is None:
            dtype = DDS_TYPE_INT if isinstance(value, int) else DDS_TYPE_FLOAT

        encoded = name.encode('utf-8')
        buf = io.BytesIO()
        buf.write(bytes([COMMAND_PUBLISH, dtype, len(encoded)]))
        buf.write(encoded)

        if dtype == DDS_TYPE_INT:
            buf.write(struct.pack('<i', int(value)))
        else:
            buf.write(struct.pack('<f', float(value)))

        self._sock.sendto(buf.getvalue(), (self._host, self._port))

    def read(self, name: str):
        """Legge l'ultimo valore ricevuto (None se non ancora arrivato)."""
        var = self._variables.get(name)
        return var.get() if var else None

    def wait(self, name: str):
        """Blocca finché il broker non pubblica 'name'. Restituisce il valore."""
        var = self._variables.get(name)
        return var.wait_value() if var else None

    # ------------------------------------------------------------------
    # Thread loop
    # ------------------------------------------------------------------

    def run(self):
        last_ka = time.monotonic()

        while self._running:
            # Keep-alive periodico (evita TTL expiry del broker)
            now = time.monotonic()
            if now - last_ka >= KEEP_ALIVE_INTERVAL:
                self._sock.sendto(bytes([COMMAND_KEEP_ALIVE]),
                                  (self._host, self._port))
                last_ka = now

            ready, _, _ = select.select([self._sock], [], [], 0.1)
            if not ready:
                continue

            data, _ = self._sock.recvfrom(4096)
            if not data:
                continue

            if data[0] == COMMAND_PUBLISH:
                self._on_publish(data)

        self._sock.close()

    def _on_publish(self, data: bytes):
        """Decodifica un pacchetto PUBLISH ricevuto dal broker."""
        dtype  = data[1]
        n_len  = data[2]
        name   = data[3: 3 + n_len].decode('utf-8')
        val_start = 3 + n_len

        if dtype == DDS_TYPE_FLOAT:
            value = struct.unpack('<f', data[val_start: val_start + 4])[0]
        elif dtype == DDS_TYPE_INT:
            value = struct.unpack('<i', data[val_start: val_start + 4])[0]
        else:
            return

        var = self._variables.get(name)
        if var:
            var.notify(value)


# ---------------------------------------------------------------------------
# Helper Time 
# ---------------------------------------------------------------------------

class Time:
    def __init__(self):
        self._start = None
        self._last  = None

    def start(self):
        self._start = time.monotonic()
        self._last  = self._start

    def get(self) -> float:
        return time.monotonic() - self._start

    def elapsed(self) -> float:
        now = time.monotonic()
        dt  = now - self._last
        self._last = now
        return dt
