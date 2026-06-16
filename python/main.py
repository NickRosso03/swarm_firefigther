"""
main.py — Entry point del sistema swarm.
Avvia un thread per ciascun drone.

Uso:
  python main.py            # 5 droni (default)
  python main.py --n 8      # 8 droni

Il valore di --n DEVE coincidere con n_drones nella scena Godot
e con il --n passato a monitor.py.
"""

import threading, time, sys, argparse
from drone_agent import DroneAgent


def _parse_args():
    parser = argparse.ArgumentParser(description="Swarm Firefighter — agenti drone")
    parser.add_argument(
        "--n", type=int, default=5,
        help="Numero di droni dello sciame (default: 5). "
             "Deve coincidere con n_drones in Godot e con --n del monitor."
    )
    args, _ = parser.parse_known_args()
    return args.n


def main():
    n_drones = _parse_args()

    print(f"\n{'='*48}")
    print(f"  Swarm Firefighter — {n_drones} droni")
    print(f"{'='*48}")
    print("Avvio agenti... assicurarsi che la scena Godot sia aperta")
    print(f"con n_drones = {n_drones} nell'Inspector.\n")

    agents  = [DroneAgent(i, n_drones) for i in range(n_drones)]
    threads = [threading.Thread(target=a.run, name=f"Drone-{a.id}", daemon=True)
               for a in agents]

    for t in threads:
        t.start()
        time.sleep(0.15)   # piccolo offset per non sovraccaricare il broker

    print(f"{n_drones} agenti avviati. Ctrl+C per fermare.\n")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nArresto.")
        for a in agents:
            a.dds.stop()
        sys.exit(0)


if __name__ == "__main__":
    main()