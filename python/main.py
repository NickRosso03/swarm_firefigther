"""
main.py — Entry point del sistema swarm.
Avvia un thread per ciascuno dei 5 droni.
"""

import threading, time, sys
from drone_agent import DroneAgent, N_DRONES


def main():
    print(f"\n{'='*48}")
    print(f"  Swarm Firefighter — {N_DRONES} droni")
    print(f"{'='*48}")
    print("Avvio agenti... assicurarsi che la scena Godot sia aperta.\n")

    agents  = [DroneAgent(i, N_DRONES) for i in range(N_DRONES)]
    threads = [threading.Thread(target=a.run, name=f"Drone-{a.id}", daemon=True)
               for a in agents]

    for t in threads:
        t.start()
        time.sleep(0.15)   # piccolo offset per non sovraccaricare il broker

    print(f"{N_DRONES} agenti avviati. Ctrl+C per fermare.\n")
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
