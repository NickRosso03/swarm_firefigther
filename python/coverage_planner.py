"""
coverage_planner.py — Generazione percorsi di perlustrazione.

NOTA ASSI GODOT:
  X → orizzontale (destra)
  Y → verticale (su)
  Z → orizzontale (avanti/indietro)

I waypoint hanno quindi forma [X, Z] per il piano orizzontale.
La quota Y è gestita separatamente dal controller di altitudine.
"""


class CoveragePlanner:

    @staticmethod
    def get_sector(drone_id: int,
                   n_drones: int,
                   area_size: float = 200.0,
                   altitude: float  = 15.0,
                   row_spacing: float = 10.0) -> list:
        """
        Restituisce waypoint [X, Z] per il settore assegnato al drone.
        L'area totale [0, area_size] x [0, area_size] viene divisa in
        n_drones strisce verticali (lungo X).
        """
        offset = area_size / 2.0  #
        sw      = area_size / n_drones
        x_start = (drone_id * sw + sw * 0.1)-offset
        x_end   = (drone_id * sw + sw * 0.9)-offset

        n_rows    = max(1, int(area_size / row_spacing))
        waypoints = []

        for row in range(n_rows):
            z = (row * row_spacing)-offset #M
            if row % 2 == 0:
                waypoints.append([x_start, z])
                waypoints.append([x_end,   z])
            else:
                waypoints.append([x_end,   z])
                waypoints.append([x_start, z])

        return waypoints

    @staticmethod
    def start_position(drone_id: int, n_drones: int,
                       area_size: float = 80.0) -> list:
        """Posizione iniziale al suolo [X, Y, Z] per il drone drone_id."""
        spacing = area_size / n_drones
        x = drone_id * spacing + spacing / 2.0
        return [x, 0.0, 0.0]
