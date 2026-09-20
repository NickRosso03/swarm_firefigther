# Swarm Firefighter

A Python + Godot 4.6 simulation of an autonomous drone swarm that searches for, coordinates around, and suppresses dynamically spreading fires. The project combines a 3D Godot physics environment with Python-based flight control, mission logic, swarm coordination, collision avoidance, refuelling, telemetry, and experiment analysis.

## Features

- Multi-drone simulation with configurable swarm size.
- Distributed fire detection and cooperative fire suppression.
- Sector-based lawnmower coverage planning for exploration.
- Fire propagation across the terrain and to combustible plants.
- Water consumption, dedicated charging stations, landing, and refuelling.
- Potential-field collision avoidance with velocity damping and smoothing.
- Cascaded P/PI/PID flight-control loops and quadrotor motor mixing.
- Custom UDP-based DDS-style broker shared by Godot and Python.
- Optional headless monitoring with PNG plots and run summaries.
- Aggregate analysis across multiple experimental runs.

## Architecture

The simulation is split into two cooperating processes:

```text
Godot 4.6
  ├── 3D physics world and drone bodies
  ├── fire spawning, propagation, visuals, and plant damage
  ├── charging stations and HUD
  └── DDS UDP broker on 127.0.0.1:4444
                 ▲                 │
                 │ telemetry       │ motor forces, targets, commands
                 │                 ▼
Python
  ├── one DroneAgent thread per drone
  ├── flight controllers and motor-force calculation
  ├── coverage planning and trajectories
  ├── fire assignment and swarm coordination
  ├── collision avoidance
  └── optional monitor.py telemetry recorder
```

Godot publishes each drone's position, velocity, attitude, fire detections, and physics tick. Python waits for the tick, updates the agent state machine, computes control forces, and publishes the four motor forces back to Godot.

The Python and Godot processes must use the same number of drones. The default is **5**.

## Repository layout

```text
python/
  main.py                    Start one control thread per drone
  drone_agent.py             Drone FSM, fire response, resources, and swarm logic
  dds.py                     UDP DDS client and binary packet protocol
  multirotor_controller.py   Cascaded flight controller and motor mixer
  controllers.py             P, PI, PID, integrator, and differentiator classes
  coverage_planner.py        Sector allocation and lawnmower waypoints
  trajectory.py              1D trapezoidal and 2D waypoint trajectories
  collision_avoidance.py     Potential-field avoidance with velocity damping
  monitor.py                 Telemetry recorder and PNG/summary output
  dataplot.py                Matplotlib plotting helpers
  aggregate.py               Aggregate summary.txt files across runs

swarm-fighter/
  project.godot              Godot 4.6 project configuration and DDS autoload
  scenes/world.tscn           Main world, drones, fires, stations, obstacles, and HUD
  scenes/drone.tscn           Drone rigid body, sensors, model, and water cannon
  scenes/fire_zone.tscn       Dynamic fire instance
  scenes/charging_station.tscn Charging station scene
  scripts/world.gd            Spawns entities and updates the HUD
  scripts/drone.gd            Applies motor forces and publishes Godot telemetry
  scripts/fire_manager.gd     Spawns and tracks ground and plant fires
  scripts/fire_zone.gd        Fire intensity, spread, plant ignition, and extinguishing
  autoloads/dds.gd            Godot-side UDP DDS broker
  fire_config.tres            Shared fire-behaviour resource
```

## Requirements

- Godot **4.6** with the Forward+ renderer.
- Python **3.10+** recommended.
- Python package:
  - `matplotlib` — required by `monitor.py`, `dataplot.py`, and `aggregate.py`.
- A local UDP loopback connection on port `4444`.

The control path itself uses Python's standard library. There is currently no `requirements.txt` or `pyproject.toml`; install the plotting dependency manually or create a virtual environment as shown below.

## Setup

Clone the repository and create a Python environment:

```bash
git clone https://github.com/NickRosso03/swarm_firefigther.git
cd swarm_firefigther

python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\\Scripts\\activate
python -m pip install --upgrade pip
python -m pip install matplotlib
```

Import the Godot project by opening the `swarm-fighter/` directory in Godot 4.6. The main scene is configured in `swarm-fighter/project.godot` as `scenes/world.tscn`.

## Run the simulation

Start Godot first so that the DDS broker is listening and the world can publish drone ticks:

```bash
# From the repository root
godot --editor swarm-fighter/project.godot
```

Run the project from the Godot editor, or use the command line from the project directory:

```bash
cd swarm-fighter
godot --path . --editor
# In another terminal, from the repository root:
cd ../python
python main.py
```

For a different swarm size, set `n_drones` in the `World` node Inspector and pass the same value to Python:

```bash
python main.py --n 8
```

The Python process waits for each corresponding Godot drone to publish `connected`, then synchronizes its control loop to that drone's `tick` topic. Press `Ctrl+C` to stop the Python agents.

## Monitor a run

`monitor.py` is an independent read-only DDS client. Start it while the simulation and control agents are running:

```bash
cd python
python monitor.py --n 5
```

Add a label to identify an experiment:

```bash
python monitor.py --n 8 --label damp_off_rep1
```

Press `Ctrl+C` to save the recorded data. Output is written under a timestamped directory in `plots/`:

```text
plots/run_YYYYMMDD_HHMMSS[_label]/
  drone_0/
    altitude.png
    velocity_z.png
    position_x.png
    position_y.png
    velocity_xy.png
    speed.png
    attitude.png
    forces.png
    water.png
  ...
  swarm/
    min_inter_drone_dist.png
    resources.png
    trajectories_xy.png
    summary.txt
```

The monitor records controller targets, physical state, motor forces, water levels, active fires, minimum inter-drone distance, and fire-extinguishing statistics. The reference collision distance is `D_SAFE = 4.0 m`.

## Aggregate experiment results

After collecting multiple monitor runs, aggregate all `summary.txt` files below `plots/`:

```bash
cd python
python aggregate.py
```

Useful options:

```bash
python aggregate.py --plots-dir ../plots
python aggregate.py --plots-dir ../plots --verbose
python aggregate.py --plots-dir ../plots --warmup 30
```

The command prints grouped statistics by drone count and normalized run condition, and saves `aggregate_results.txt` in the selected plots directory.

## Simulation controls

The free camera in `swarm-fighter/scripts/free_camera.gd` supports:

- `WASD` — move horizontally
- `Q` / `E` — move down / up
- Hold `Shift` — move four times faster
- Hold the right mouse button — capture the mouse and look around
- Hold `Alt` — look around while keeping the mouse visible
- Mouse-wheel / trackpad scroll — change camera speed
- `F` — reset to the overview position
- `P` — toggle debug flight paths

The world also contains a reset button node, although it is hidden by default in `scenes/world.tscn`.

## Configuration

Most runtime parameters are intentionally exposed in the source or Godot Inspector:

- `python/drone_agent.py` — arena size, flight speeds, fire response, water capacity, suppression time, DDS host/port, and state-machine behaviour.
- `python/multirotor_controller.py` — hover feed-forward and controller gains.
- `python/collision_avoidance.py` — safe distance, influence radius, repulsion, damping, neighbour limit, and maximum offset.
- `swarm-fighter/scripts/fire_manager.gd` — spawn interval, fire caps, random seed, and adaptive spawn slowdown.
- `swarm-fighter/fire_config.tres` — shared fire spread, detection, plant ignition, and burn parameters.
- `swarm-fighter/scripts/world.gd` and the `World` node — drone count and arena size.

Keep these values synchronized where noted in the code. In particular, `n_drones` and `area_size` must agree between `world.gd` and the Python control system.

## DDS protocol

The Godot autoload `autoloads/dds.gd` opens UDP port `4444` and implements three packet types:

- `0x80` — keep-alive
- `0x81` — subscribe to variables
- `0x82` — publish a typed value

Python's `dds.py` mirrors this protocol, sends keep-alives every second, and provides `subscribe()`, `publish()`, `read()`, and `wait()` helpers. Topics use prefixes such as `drone_0/X`, `drone_0/f1`, `drone_0/tick`, and `world/fire_intensity_1`.

## Troubleshooting

### Python waits for Godot indefinitely

- Start the Godot scene before starting `python/main.py`.
- Confirm that Godot is listening on UDP port `4444`.
- Check that `main.py --n`, the Godot `World.n_drones`, and `monitor.py --n` match.
- Make sure no other process is using port `4444`.

### Drones do not move correctly

- Verify that Python is running from the `python/` directory, or that the directory is on `PYTHONPATH`.
- Check the Godot Output panel and Python logs for DDS connection errors.
- Confirm that the scene's `drone_scene`, `charging_station_scene`, `fire_manager.config`, and `fire_zone_scene` properties are assigned.

### No plots are produced

- Install `matplotlib` in the active Python environment.
- Stop `monitor.py` with `Ctrl+C`; plots are saved during shutdown.
- Ensure the monitor's `--n` value matches the running swarm.

## Status

This is an active simulation and experimentation project. The control, fire-management, monitoring, and analysis components are implemented, but there is no automated test suite or packaged release workflow in the repository yet.

## License

No license file is currently included. Unless a license is added, the repository remains under the default copyright of its author.
