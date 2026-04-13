## world.gd — Script della scena principale.
##
## Responsabilità:
##   - Istanzia i 5 droni a posizioni iniziali sfalsate
##   - Espone il bottone di reset
##   - Mostra un HUD minimale con lo stato di ogni drone via Label
##
## Struttura della scena world.tscn:
##   World  (Node3D, questo script)
##   ├── DDS           (dds.gd — Autoload, non serve aggiungerlo qui
##   │                  se è registrato come Autoload in Project Settings)
##   ├── Environment   (WorldEnvironment + DirectionalLight3D)
##   ├── Ground        (StaticBody3D con CollisionShape3D piatta)
##   ├── FireManager   (Node3D, fire_manager.gd)
##   ├── Drones        (Node3D — contenitore, figli aggiunti dinamicamente)
##   └── UI            (CanvasLayer)
##       ├── ResetButton (Button)
##       └── StatusPanel (VBoxContainer con 5 Label)

extends Node3D

# ---------------------------------------------------------------------------
# Parametri
# ---------------------------------------------------------------------------
@export var drone_scene    : PackedScene          # assegna drone.tscn
@export var n_drones       : int   = 5
@export var area_size      : float = 200.0         # deve coincidere con Python
@export var start_altitude : float = 0.3          # piccolo offset da terra

# ---------------------------------------------------------------------------
# Riferimenti
# ---------------------------------------------------------------------------
@onready var _drones_root : Node3D    = $Drones
@onready var _status_panel: VBoxContainer = $UI/StatusPanel
@onready var _reset_btn   : Button    = $UI/ResetButton

var _drones    : Array = []
var _labels    : Array = []


# Mapping codice numerico → stringa leggibile (deve coincidere con drone_agent.py)
const STATUS_NAMES := {
	0.0: "IDLE/TAKEOFF",
	1.0: "EXPLORING",
	2.0: "MOVING",
	3.0: "RETURNING",
	4.0: "SUPPRESSING",
}

#Debug percorso droni
var _show_debug_paths : bool = false
var _path_meshes : Array = []
var _sector_waypoints : Array = [] # Conterrà i waypoint pre-calcolati per ogni drone
const FLIGHT_ALT = 15.0

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
	if drone_scene == null:
		push_error("world.gd: drone_scene non assegnata!")
		return

	_spawn_drones()
	_build_hud()

	_reset_btn.pressed.connect(_on_reset_pressed)
	
	# Registra le variabili di stato dei droni per l'HUD e i path
	for i in n_drones:
		DDS.subscribe("drone_%d/status" % i)
		DDS.subscribe("drone_%d/tgt_x" % i)
		DDS.subscribe("drone_%d/tgt_z" % i)
		DDS.subscribe("drone_%d/fire_x" % i)
		DDS.subscribe("drone_%d/fire_y" % i)
		DDS.subscribe("drone_%d/fire_z" % i)
		
		# --- SETUP DEBUG MESH PER I PERCORSI ---
		var mi := MeshInstance3D.new()
		var im := ImmediateMesh.new()
		mi.mesh = im
		var mat := StandardMaterial3D.new()
		mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		# Diamo un colore diverso ad ogni drone usando l'HSV
		mat.albedo_color = Color.from_hsv(float(i) / n_drones, 1.0, 1.0)
		mi.material_override = mat
		add_child(mi)
		_path_meshes.append(mi)
		
		# Pre-calcola il percorso esplorativo 
		_sector_waypoints.append(_generate_sector(i))





func _process(_delta: float) -> void:
	_update_hud()
	
	if _show_debug_paths:
		_draw_debug_paths()




# ---------------------------------------------------------------------------
# Spawn droni
# ---------------------------------------------------------------------------

func _spawn_drones() -> void:
	var offset  :=area_size / 2
	var spacing := area_size / n_drones
	for i in n_drones:
		var drone : RigidBody3D = drone_scene.instantiate()
		drone.drone_id = i
		drone.name     = "Drone%d" % i
		_drones_root.add_child(drone)
		# global_position va impostata dopo add_child(),
		# solo quando il nodo è dentro l'albero della scena
		drone.global_position = Vector3(
			(i * spacing + spacing * 0.5)-offset,
			start_altitude,
			2.0-offset    # leggermente dentro l'area
		)
		_drones.append(drone)
		print("World: Drone %d spawned in (%.1f, %.1f, %.1f)" \
			  % [i, drone.global_position.x,
					drone.global_position.y,
					drone.global_position.z])


# ---------------------------------------------------------------------------
# HUD
# ---------------------------------------------------------------------------

func _build_hud() -> void:
	for i in n_drones:
		var lbl := Label.new()
		lbl.text             = "D%d: --" % i
		lbl.add_theme_font_size_override("font_size", 14)
		_status_panel.add_child(lbl)
		_labels.append(lbl)


func _update_hud() -> void:
	for i in n_drones:
		var code   : float  = DDS.read("drone_%d/status" % i)
		var _name   : String = STATUS_NAMES.get(code, "?")
		var drone  : Node3D = _drones[i]
		var pos    : Vector3 = drone.global_position
		_labels[i].text = "D%d [%s] (%.0f, %.0f, %.0f)" \
			% [i, _name, pos.x, pos.y, pos.z]
			

# ---------------------------------------------------------------------------
# Debug Paths Visualization
# ---------------------------------------------------------------------------
func _generate_sector(id: int) -> Array:
	var wps : Array = []
	var offset = area_size / 2
	var sw = area_size / n_drones
	var x_start = (id * sw + sw * 0.1) - offset
	var x_end   = (id * sw + sw * 0.9) - offset
	var row_spacing = 10.0
	var n_rows  = max(1, int(area_size / row_spacing))
	
	# La quota di default dell'esplorazione è 8 metrip
	for row in range(n_rows):
		var z = (row * row_spacing) - offset
		if row % 2 == 0:
			wps.append(Vector3(x_start, FLIGHT_ALT, z))
			wps.append(Vector3(x_end, FLIGHT_ALT, z))
		else:
			wps.append(Vector3(x_end, FLIGHT_ALT, z))
			wps.append(Vector3(x_start, FLIGHT_ALT, z))
	return wps


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and event.keycode == KEY_P:
		_show_debug_paths = not _show_debug_paths
		print("Debug Paths: ", _show_debug_paths)
		# Se li spengo, pulisco subito le mesh
		if not _show_debug_paths:
			for mi in _path_meshes:
				mi.mesh.clear_surfaces()


func _draw_debug_paths() -> void:
	for i in n_drones:
		var im : ImmediateMesh = _path_meshes[i].mesh
		im.clear_surfaces()
		im.surface_begin(Mesh.PRIMITIVE_LINES)
		
		var drone_pos = _drones[i].global_position
		var status = DDS.read("drone_%d/status" % i)
		
		# 1. Disegna l'intero settore (linea spezzata) per il drone
		var wps = _sector_waypoints[i]
		for j in range(wps.size() - 1):
			im.surface_add_vertex(wps[j])
			im.surface_add_vertex(wps[j+1])
			
		# 2. Disegna un segmento "dinamico" che unisce il drone al suo target effettivo
		if status == 2.0: # 2.0 = MOVING / SUPPRESSING (va verso il fuoco)
			var fx = DDS.read("drone_%d/fire_x" % i)
			var fy = DDS.read("drone_%d/fire_y" % i)
			var fz = DDS.read("drone_%d/fire_z" % i)
			im.surface_add_vertex(drone_pos)
			im.surface_add_vertex(Vector3(fx, fy, fz))
		else: # Altri stati (EXPLORING, RETURNING) -> va verso un waypoint
			var tx = DDS.read("drone_%d/tgt_x" % i)
			var tz = DDS.read("drone_%d/tgt_z" % i)
			#Alla quota attuale del drone per vedere bene lo spostamento XY
			im.surface_add_vertex(drone_pos)
			im.surface_add_vertex(Vector3(tx, drone_pos.y, tz))
			
		im.surface_end()

# ---------------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------------

func _on_reset_pressed() -> void:
	for drone in _drones:
		drone.reset()
	print("World: tutti i droni resettati.")
	
