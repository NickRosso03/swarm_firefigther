## world.gd — Script della scena principale.
##
## Responsabilità:
##   - Istanzia i 5 droni a posizioni iniziali sfalsate
##   - Mostra un HUD minimale con lo stato di ogni drone via Label


extends Node3D

# ---------------------------------------------------------------------------
# Parametri
# ---------------------------------------------------------------------------
@export var drone_scene    : PackedScene          # assegna drone.tscn
@export var n_drones       : int   = 5
@export var area_size      : float = 200.0         # deve coincidere con Python
@export var start_altitude : float = 0.3          # piccolo offset da terra
@export var charging_station_scene : PackedScene   # ← assegna charging_station.tscn

# ---------------------------------------------------------------------------
# Riferimenti
# ---------------------------------------------------------------------------
@onready var _drones_root : Node3D    = $Drones
@onready var _status_panel: VBoxContainer = $UI/StatusPanel
@onready var _reset_btn   : Button    = $UI/ResetButton

@onready var _fire_manager : Node3D = $FireManager
var _plant_label : Label
var _fire_label : Label
var _time_label : Label
var _sim_time := 0.0

var _drones    : Array = []
var _labels    : Array = []
var _stations  : Array = []


# Mapping codice numerico → stringa leggibile (deve coincidere con drone_agent.py)
const STATUS_NAMES := {
	0.0: "IDLE/TAKEOFF",
	1.0: "EXPLORING",
	2.0: "MOVING",
	3.0: "RETURNING",
	4.0: "SUPPRESSING",
	5.0: "→ STATION",   # ← nuovo
	6.0: "REFUELING",   # ← nuovo
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
	_spawn_stations()
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
		DDS.subscribe("drone_%d/water_level" % i)
		DDS.subscribe("drone_%d/is_active" % i)
		
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
		#_sector_waypoints.append(_generate_sector(i))





func _process(_delta: float) -> void:
	_sim_time += _delta
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

func _spawn_stations() -> void:
	if charging_station_scene == null:
		push_error("world.gd: charging_station_scene non assegnata!")
		return
	var offset  := area_size / 2.0
	var spacing := area_size / n_drones
	for i in n_drones:
		var station : Node3D = charging_station_scene.instantiate()
		station.station_id = i
		station.name = "Station%d" % i
		
		# AGGIUNGI QUESTA RIGA: Inserisce la stazione nel gruppo appena creata
		station.add_to_group("charging_stations") 
		
		_drones_root.add_child(station)   
		station.global_position = Vector3(
			(i * spacing + spacing * 0.5) - offset,
			0.0,
			-(area_size / 2.0) + 4.0   
		)
		_stations.append(station)
		print("World: Station %d spawned at (%.1f, 0, %.1f)" \
			% [i, station.global_position.x, station.global_position.z])
# ---------------------------------------------------------------------------
# HUD
# ---------------------------------------------------------------------------

func _build_hud() -> void:
	
	_time_label = Label.new()
	_time_label.add_theme_font_size_override("font_size", 20)
	_apply_shadow_and_outline(_time_label)
	_status_panel.add_child(_time_label)
	
	for i in n_drones:
		var lbl := Label.new()
		lbl.text             = "D%d: --" % i
		lbl.add_theme_font_size_override("font_size", 18)
		_apply_shadow_and_outline(lbl)
		_status_panel.add_child(lbl)
		_labels.append(lbl)
		
	# Separatore + label piante
	var sep := Label.new()
	sep.text = "────────────────────"
	sep.add_theme_font_size_override("font_size", 18)
	_status_panel.add_child(sep)

	_plant_label = Label.new()
	_plant_label.add_theme_font_size_override("font_size", 20)
	_apply_shadow_and_outline(_plant_label)
	_status_panel.add_child(_plant_label)
	
	
	_fire_label = Label.new()
	_fire_label.add_theme_font_size_override("font_size", 20)
	_apply_shadow_and_outline(_fire_label)
	_status_panel.add_child(_fire_label)
	


func _update_hud() -> void:
	for i in n_drones:
		var code   : float  = DDS.read("drone_%d/status" % i)
		var _name   : String = STATUS_NAMES.get(code, "?")
		var drone  : Node3D = _drones[i]
		var pos    : Vector3 = drone.global_position
		var water : float = DDS.read("drone_%d/water_level" % i)
		if water == null: water = 100.0
		
		_labels[i].text = "D%d [%s] (%.0f, %.0f, %.0f) W:%.0f%%" \
			% [i, _name, pos.x, pos.y, pos.z, water]
		var colors := {0.0: Color.GRAY, 1.0: Color.LAWN_GREEN, 2.0: Color.ORANGE,
					   3.0: Color.CYAN, 4.0: Color.RED,5.0: Color.CORNFLOWER_BLUE,6.0:
						 Color.DEEP_PINK,}
		_labels[i].modulate = colors.get(code, Color.WHITE)
		if _fire_manager:
			var ignited = _fire_manager.plants_ignited
			var burned  = _fire_manager.plants_burned
			var saved   = _fire_manager.plants_saved
			_plant_label.text = "Piante: %d accese | %d bruciate | %d salvate" \
				% [ignited, burned, saved]
			# Colore
			_plant_label.modulate = Color("#99FF66")
			
			_time_label.text = "Tempo trascorso: %02d:%02d" % [int(_sim_time)/60, int(_sim_time)%60]
			
			var af = _fire_manager._active_fires.size()
			var gf = _fire_manager._ground_fire_ids.size()
			var pf = _fire_manager._burning_plants.size()
			_fire_label.text = "Fuochi attivi: %d (terra: %d | piante: %d)" % [af, gf, pf]
			
			_fire_label.modulate = Color(Color.ORANGE_RED)
			
		
			 
			

# ---------------------------------------------------------------------------
# Debug Paths Visualization
# ---------------------------------------------------------------------------
func _generate_sector(my_index: int, n_active: int) -> Array:
	var wps : Array = []
	var offset = area_size / 2
	var sw = area_size / n_active          # ← n_active invece di n_drones
	var x_start = (my_index * sw + sw * 0.1) - offset   # ← my_index
	var x_end   = (my_index * sw + sw * 0.9) - offset
	var row_spacing = 10.0
	var n_rows  = max(1, int(area_size / row_spacing))

	for row in range(n_rows):
		var z = (row * row_spacing) - offset
		if row % 2 == 0:
			wps.append(Vector3(x_start, FLIGHT_ALT, z))
			wps.append(Vector3(x_end,   FLIGHT_ALT, z))
		else:
			wps.append(Vector3(x_end,   FLIGHT_ALT, z))
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
	# Costruisci la lista dei droni attivi (stesso algoritmo di Python)
	var active_sorted : Array = []
	for i in n_drones:
		# is_active == 0.0 solo se esplicitamente pubblicato come inattivo
		# Prima che Python pubblichi, read restituisce 0.0 → trattalo come attivo
		var val := DDS.read("drone_%d/is_active" % i)
		if val != 0.0 or not DDS.read("drone_%d/status" % i) in [5.0, 6.0]:
			active_sorted.append(i)
	var n_active := active_sorted.size()
	if n_active == 0:
		return

	for i in n_drones:
		var im : ImmediateMesh = _path_meshes[i].mesh
		im.clear_surfaces()

		var status := DDS.read("drone_%d/status" % i)
		var is_charging := (status == 5.0 or status == 6.0)

		im.surface_begin(Mesh.PRIMITIVE_LINES)

		if not is_charging:
			# Calcola l'indice remappato per questo drone
			var my_index := active_sorted.find(i)
			if my_index == -1:
				im.surface_end()
				continue
			var wps := _generate_sector(my_index, n_active)

			# Disegna settore
			for j in range(wps.size() - 1):
				im.surface_add_vertex(wps[j])
				im.surface_add_vertex(wps[j + 1])

			# Segmento drone → target corrente
			var drone_pos = _drones[i].global_position
			var tx := DDS.read("drone_%d/tgt_x" % i)
			var tz := DDS.read("drone_%d/tgt_z" % i)
			im.surface_add_vertex(drone_pos)
			im.surface_add_vertex(Vector3(tx, drone_pos.y, tz))
		else:
			# Drone in ricarica: disegna solo una croce sulla sua posizione
			var p = _drones[i].global_position
			im.surface_add_vertex(p + Vector3(-2, 0, 0))
			im.surface_add_vertex(p + Vector3( 2, 0, 0))
			im.surface_add_vertex(p + Vector3(0, 0, -2))
			im.surface_add_vertex(p + Vector3(0, 0,  2))

		im.surface_end()

# Funzione rapida per applicare bordi e ombre
func _apply_shadow_and_outline(lbl: Label):
	#  Aggiunge il bordo 
	lbl.add_theme_constant_override("outline_size", 3)
	lbl.add_theme_color_override("font_outline_color", Color.BLACK)
	
	#  Aggiunge l'ombra 
	lbl.add_theme_constant_override("shadow_offset_x", 2)
	lbl.add_theme_constant_override("shadow_offset_y", 2)
	lbl.add_theme_constant_override("shadow_outline_size", 2)
	lbl.add_theme_color_override("font_shadow_color", Color(0, 0, 0, 0.7)) # Nero semi-trasparente

# ---------------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------------

func _on_reset_pressed() -> void:
	for drone in _drones:
		drone.reset()
	print("World: tutti i droni resettati.")
	
