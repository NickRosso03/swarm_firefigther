## fire_zone.gd — Singola zona di incendio nella scena.
##
## Non ha più parametri @export propri per i valori operativi:
## tutto arriva da FireConfig tramite activate(). Gli unici @export
## rimasti sono quelli strutturali (detection, auto_extinguish) che
## non appartengono alla config condivisa.
##
## Flusso di vita:
##   FireManager._spawn_fire_at()
##     → zone.activate(id, pos, config)   ← config copiata nei campi locali
##     → _process() ogni frame
##     → _extinguish() → emit extinguished(id) → queue_free()

extends Node3D

# ---------------------------------------------------------------------------
# Parametri strutturali (non in FireConfig: sono specifici dell'istanza)
# ---------------------------------------------------------------------------

@export var auto_extinguish : bool = false

# ── Parametri fuoco su pianta: impostati da FireManager.ignite_plant ─────────
## Questi due campi vengono scritti da FireManager PRIMA di activate(),
## così activate() sa già che tipo di fuoco sta inizializzando.
var is_plant_fire : bool   = false
var plant_node    : Node3D = null

# ---------------------------------------------------------------------------
# Parametri operativi (copiati da FireConfig in activate())
# ---------------------------------------------------------------------------
## Documentati in fire_config.gd. Non modificare direttamente.

var spread_time           : float = 45.0
var spread_radius         : float = 16.0
var _detection_radius_min : float =  4.0
var _detection_radius_max : float = 10.0
var plant_ignition_delay  : float = 10.0
var plant_burn_time       : float = 60.0
var plant_spread_radius   : float =  2.0

# ---------------------------------------------------------------------------
# Stato runtime
# ---------------------------------------------------------------------------

var fire_id       : int   = 0
var intensity     : float = 0.0
var _spread_done  : bool  = false
var _burn_elapsed : float = 0.0
var _active       : bool  = true
var _drones_near  : int   = 0

## detection_radius effettivo (interpolato con l'intensità).
var detection_radius : float = 4.0

## { Node3D → float }: tempo di contatto accumulato per ogni pianta vicina.
var _plant_contact_time : Dictionary = {}
var _nearby_plants      : Array      = []

var _mesh_material : StandardMaterial3D = null

# ---------------------------------------------------------------------------
# Nodi figli
# ---------------------------------------------------------------------------
@onready var _particles : GPUParticles3D   = $Particles
@onready var _area      : Area3D           = $DetectionArea
@onready var _shape     : CollisionShape3D = $DetectionArea/CollisionShape3D
@onready var _label     : Label3D          = $Label3D
@onready var _light     : OmniLight3D      = $OmniLight3D

# ---------------------------------------------------------------------------
# Segnali
# ---------------------------------------------------------------------------
signal extinguished(id: int)

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
	_area.body_entered.connect(_on_body_entered)
	_area.body_exited.connect(_on_body_exited)

	if _label:
		_label.text = "FIRE #%d" % fire_id

	DDS.subscribe("world/fire_resolved")

	if _particles:
		_particles.emitting = true
		_particles.process_material = _particles.process_material.duplicate()
		var mesh := _particles.draw_pass_1
		if mesh and mesh.surface_get_material(0) is StandardMaterial3D:
			_mesh_material = mesh.surface_get_material(0).duplicate()
			_mesh_material.emission_enabled = true
			var mesh_copy := mesh.duplicate()
			mesh_copy.surface_set_material(0, _mesh_material)
			_particles.draw_pass_1 = mesh_copy


func _process(delta: float) -> void:
	if not _active:
		DDS.publish("world/fire_intensity_%d" % fire_id, DDS.DDS_TYPE_FLOAT, 0.0)
		return

	# Crescita intensità
	if intensity < 1.0:
		intensity = min(intensity + delta / spread_time, 1.0)
		_update_intensity_visuals()

	# Propagazione a terra: solo fuochi a terra, una sola volta a saturazione.
	if intensity >= 1.0 and not _spread_done and not is_plant_fire:
		_spread_done = true
		_try_propagate()

	# Tracking contatto verso piante vicine (tutti i tipi di fuoco).
	_track_plant_contacts(delta)

	# Countdown combustione (solo fuochi su pianta).
	if is_plant_fire:
		_burn_elapsed += delta
		if _burn_elapsed >= plant_burn_time:
			_destroy_plant()
			return

	DDS.publish("world/fire_intensity_%d" % fire_id, DDS.DDS_TYPE_FLOAT, intensity)

	# Spegnimento da Python (topic dedicato per fire_id + topic generico)
	var resolved_dedicated := DDS.read("world/fire_resolved_%d" % fire_id)
	if resolved_dedicated == 1.0 or int(DDS.read("world/fire_resolved")) == fire_id:
		_extinguish()


# ---------------------------------------------------------------------------
# Tracking contatto fuoco→piante
# ---------------------------------------------------------------------------

func _track_plant_contacts(delta: float) -> void:
	var manager := get_parent()
	if manager == null or not manager.has_method("ignite_plant"):
		return

	## Raggio di ricerca:
	##   fuoco a terra  → detection_radius (raggio effettivo della DetectionArea)
	##   fuoco su pianta → plant_spread_radius (limita la propagazione chioma→chioma)
	var search_radius := plant_spread_radius if is_plant_fire else detection_radius

	for plant in _nearby_plants:
		if not is_instance_valid(plant) or not plant.is_in_group("combustible_plant"):
			continue
		if global_position.distance_to(plant.global_position) <= search_radius:
			_plant_contact_time[plant] = _plant_contact_time.get(plant, 0.0) + delta
			if _plant_contact_time[plant] >= plant_ignition_delay:
				_plant_contact_time.erase(plant)
				manager.ignite_plant(plant)
		else:
			_plant_contact_time.erase(plant)


# ---------------------------------------------------------------------------
# Propagazione a terra: punta verso la pianta più vicina se disponibile
# ---------------------------------------------------------------------------

func _try_propagate() -> void:
	var manager := get_parent()
	if manager == null or not manager.has_method("spawn_child_fire"):
		return

	var best_plant : Node3D = null
	var best_dist  : float  = INF

	for plant in _nearby_plants:
		if not is_instance_valid(plant) or not plant.is_in_group("combustible_plant"):
			continue
		var d := global_position.distance_to(plant.global_position)
		if d < spread_radius and d < best_dist:
			best_dist  = d
			best_plant = plant

	var offset : Vector3
	if best_plant != null:
		var dir  := (best_plant.global_position - global_position).normalized()
		var dist := randf_range(spread_radius * 0.3, spread_radius * 0.6)
		offset    = dir * dist
		offset.y  = 0.0
	else:
		var angle := randf() * TAU
		var dist  := randf_range(spread_radius * 0.3, spread_radius * 0.5)
		offset     = Vector3(cos(angle) * dist, 0.0, sin(angle) * dist)

	manager.spawn_child_fire(global_position + offset)


# ---------------------------------------------------------------------------
# Distruzione pianta per combustione completa
# ---------------------------------------------------------------------------

func _destroy_plant() -> void:
	var manager := get_parent()
	if manager and manager.has_method("spawn_ash") and is_instance_valid(plant_node):
		manager.spawn_ash(plant_node.global_position, plant_node.scale)
		plant_node.queue_free()
	if manager and manager.has_method("on_plant_burned"):
		manager.on_plant_burned(fire_id)
	_extinguish()


# ---------------------------------------------------------------------------
# Visuals
# ---------------------------------------------------------------------------

func _update_intensity_visuals() -> void:
	if _particles:
		_particles.amount_ratio = lerp(0.3, 1.0, intensity)
		var p_mat := _particles.process_material as ParticleProcessMaterial
		if p_mat:
			p_mat.scale_min            = lerp(0.5, 1.2, intensity)
			p_mat.scale_max            = lerp(1.0, 2.0, intensity)
			p_mat.initial_velocity_min = lerp(1.0, 3.5, intensity)
			p_mat.initial_velocity_max = lerp(2.0, 5.0, intensity)
		if _mesh_material:
			_mesh_material.albedo_color               = Color(1.0, lerp(0.8, 0.1, intensity), 0.0, 1.0)
			_mesh_material.emission                   = Color(1.0, lerp(0.5, 0.0, intensity), 0.0, 1.0)
			_mesh_material.emission_energy_multiplier = lerp(2.5, 5.0, intensity)
	if _light:
		_light.light_energy = lerp(0.0, 3.0, intensity)
		_light.omni_range   = lerp(2.0, 12.0, intensity)
	if _label:
		_label.text = "FIRE #%d  %d%%" % [fire_id, int(intensity * 100)]

	# Detection radius cresce con l'intensità (usa i valori da config)
	detection_radius = lerp(_detection_radius_min, _detection_radius_max, intensity)
	if _shape and _shape.shape is SphereShape3D:
		(_shape.shape as SphereShape3D).radius = detection_radius


# ---------------------------------------------------------------------------
# Droni
# ---------------------------------------------------------------------------

func _on_body_entered(body: Node3D) -> void:
	if not _active or not body.has_method("reset"):
		return
	_drones_near += 1

func _on_body_exited(body: Node3D) -> void:
	if not body.has_method("reset"):
		return
	_drones_near = max(0, _drones_near - 1)

func _publish_fire_event() -> void:
	DDS.publish("world/fire_new",   DDS.DDS_TYPE_FLOAT, float(fire_id))
	DDS.publish("world/fire_x",     DDS.DDS_TYPE_FLOAT, global_position.x)
	DDS.publish("world/fire_y",     DDS.DDS_TYPE_FLOAT, global_position.y)
	DDS.publish("world/fire_z",     DDS.DDS_TYPE_FLOAT, global_position.z)


# ---------------------------------------------------------------------------
# Spegnimento
# ---------------------------------------------------------------------------

func _extinguish() -> void:
	if not _active:
		return   # guard: evita doppia chiamata
	intensity = 0.0
	_active   = false
	_plant_contact_time.clear()
	DDS.publish("world/fire_intensity_%d" % fire_id, DDS.DDS_TYPE_FLOAT, 0.0)
	if _particles:
		_particles.emitting = false
	DDS.clear("world/fire_new")
	emit_signal("extinguished", fire_id)
	await get_tree().create_timer(2.0).timeout
	queue_free()


# ---------------------------------------------------------------------------
# API pubblica
# ---------------------------------------------------------------------------

func is_active() -> bool:
	return _active


func activate(id: int, pos: Vector3, config: FireConfig) -> void:
	# Copia tutti i parametri operativi dalla config centralizzata
	spread_time           = config.spread_time
	spread_radius         = config.spread_radius
	_detection_radius_min = config.detection_radius_min
	_detection_radius_max = config.detection_radius_max
	plant_ignition_delay  = config.plant_ignition_delay
	plant_burn_time       = config.plant_burn_time
	plant_spread_radius   = config.plant_spread_radius

	# Inizializzazione stato
	fire_id       = id
	intensity     = 0.0
	_spread_done  = false
	_burn_elapsed = 0.0
	_plant_contact_time.clear()
	_active = true

	_area.set_meta("fire_id", id)
	global_position = pos

	DDS.subscribe("world/fire_resolved_%d" % id)

	if _particles:
		_particles.emitting = true
	_update_intensity_visuals()
	if _label:
		_label.text = "FIRE #%d" % fire_id
	_publish_fire_event()

	# Cache delle piante vicine al momento dello spawn
	var max_search = max(spread_radius, plant_spread_radius)
	for plant in get_tree().get_nodes_in_group("combustible_plant"):
		if is_instance_valid(plant) and global_position.distance_to(plant.global_position) <= max_search:
			_nearby_plants.append(plant)
