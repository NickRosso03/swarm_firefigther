## fire_manager.gd — Gestore degli incendi dinamici.
##
## Responsabilità:
##   - Spawn spontaneo di fuochi a terra con intervallo adattivo
##   - Propagazione a terra (spawn_child_fire) e su piante (ignite_plant)
##   - Gestione ceneri (spawn_ash) e contatori piante
##
## Tutti i parametri dei singoli fuochi sono centralizzati in FireConfig.
## I parametri di spawn (frequenza, cap) sono @export diretti di questo nodo
## così sono modificabili dall'Inspector durante la simulazione.
##
## Logica di spawn adattiva:
##   intervallo = rand(min_interval, max_interval) × (1 + ground_fires × spawn_slowdown)
##   → più fuochi attivi ci sono, più tempo passa prima del prossimo spawn.
##   Solo gli spawn a terra hanno un cap (max_ground_fires).
##   La propagazione pianta→pianta è libera (frenata dalla fisica: plant_spread_radius).

extends Node3D

# ---------------------------------------------------------------------------
# Parametri (modificabili dall'Inspector durante la simulazione)
# ---------------------------------------------------------------------------

## Risorsa centralizzata con tutti i parametri dei singoli fuochi.
## Crea un .tres da FireConfig e assegnalo qui.
@export var config            : FireConfig

@export var fire_zone_scene   : PackedScene
@export var area_min          : Vector2 = Vector2(-90.0,  90.0)
@export var area_max          : Vector2 = Vector2( 90.0, -90.0)

## Intervallo minimo [s] tra spawn spontanei a terra (prima del moltiplicatore adattivo).
@export var min_interval      : float =  8.0

## Intervallo massimo [s] tra spawn spontanei a terra (prima del moltiplicatore adattivo).
@export var max_interval      : float = 18.0

## Cap sugli spawn spontanei a terra. Non limita la propagazione.
@export var max_ground_fires  : int   = 10

## Fattore di rallentamento adattivo. Con N fuochi a terra attivi:
## intervallo × (1 + N × spawn_slowdown).
## 0.0 = nessun rallentamento; 0.5 = rallenta molto.
@export var spawn_slowdown    : float = 0.2

@export var fire_altitude     : float = 0.0

# ---------------------------------------------------------------------------
# Stato interno
# ---------------------------------------------------------------------------

var _next_fire_id    : int        = 1
## Tutti i fuochi attivi: ground + plant. fire_id → FireZone node.
var _active_fires    : Dictionary = {}
## Solo gli id dei fuochi a terra: per il cap e il fattore adattivo.
var _ground_fire_ids : Array      = []
## fire_id → Node3D pianta: evita doppia ignizione e cleanup.
var _burning_plants  : Dictionary = {}

## Contatori piante per HUD.
var plants_ignited          : int   = 0
var plants_burned           : int   = 0   # distrutte dal fuoco
var plants_saved            : int   = 0   # spente dai droni in tempo
var _destroyed_plant_ids    : Array = []

var _spawn_timer   : float = 0.0
var _next_spawn_at : float = 0.0

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
	if config == null:
		push_error("FireManager: 'config' (FireConfig) non assegnata! Crea un .tres da FireConfig.")
	randomize()
	_schedule_next_spawn()
	print("FireManager: pronto. Primo incendio tra %.1f s" % _next_spawn_at)


func _process(delta: float) -> void:
	_spawn_timer += delta
	if _spawn_timer >= _next_spawn_at:
		_spawn_timer = 0.0
		_try_spawn_fire()
		_schedule_next_spawn()


# ---------------------------------------------------------------------------
# Spawn fuochi a terra
# ---------------------------------------------------------------------------

func _try_spawn_fire() -> void:
	if _ground_fire_ids.size() >= max_ground_fires:
		return   # riprova al prossimo intervallo adattivo
	if fire_zone_scene == null:
		push_error("FireManager: fire_zone_scene non assegnata!")
		return
	_spawn_fire_at(_random_position(), false)


func _random_position() -> Vector3:
	var x := randf_range(area_min.x, area_max.x)
	var z := randf_range(area_min.y, area_max.y)
	return Vector3(x, fire_altitude, z)


## Calcola il prossimo intervallo adattivo.
## Con più fuochi a terra attivi l'intervallo cresce, rallentando gli spawn spontanei.
func _schedule_next_spawn() -> void:
	var multiplier := 1.0 + _ground_fire_ids.size() * spawn_slowdown
	_next_spawn_at = randf_range(min_interval, max_interval) * multiplier


## Istanzia e attiva una FireZone, passandole la config centralizzata.
## is_plant = true → non conta nel cap a terra.
func _spawn_fire_at(pos: Vector3, is_plant: bool) -> int:
	if config == null:
		push_error("FireManager: impossibile spawnare fuoco senza FireConfig.")
		return -1

	var id := _next_fire_id
	_next_fire_id += 1

	var zone : Node3D = fire_zone_scene.instantiate()
	add_child(zone)
	zone.extinguished.connect(_on_fire_extinguished)
	zone.activate(id, pos, config)   # ← config passata qui

	_active_fires[id] = zone
	if not is_plant:
		_ground_fire_ids.append(id)
	return id


# ---------------------------------------------------------------------------
# Callback spegnimento
# ---------------------------------------------------------------------------

func _on_fire_extinguished(id: int) -> void:
	_active_fires.erase(id)
	_ground_fire_ids.erase(id)
	var was_plant := id in _burning_plants        # controlla PRIMA di erase
	_burning_plants.erase(id)

	if was_plant:
		if id in _destroyed_plant_ids:
			_destroyed_plant_ids.erase(id)
		else:
			plants_saved += 1

	print("FireManager: #%d spento — attivi: %d (terra: %d, piante: %d)" \
			% [id, _active_fires.size(), _ground_fire_ids.size(), _burning_plants.size()])


# ---------------------------------------------------------------------------
# API pubblica: propagazione a terra → figlio
# (chiamata da fire_zone._try_propagate)
# ---------------------------------------------------------------------------

func spawn_child_fire(pos: Vector3) -> void:
	## Figlio a terra: clampato nell'arena.
	## Ogni fuoco a terra propaga al massimo una volta (_spread_done in fire_zone),
	## quindi il numero è naturalmente limitato senza un cap esplicito.
	if fire_zone_scene == null:
		return
	var x_min = min(area_min.x, area_max.x);  var x_max = max(area_min.x, area_max.x)
	var z_min = min(area_min.y, area_max.y);  var z_max = max(area_min.y, area_max.y)
	pos.x = clamp(pos.x, x_min, x_max)
	pos.z = clamp(pos.z, z_min, z_max)
	pos.y = fire_altitude
	var id := _spawn_fire_at(pos, false)
	if id >= 0:
		print("FireManager: propagazione a terra → #%d" % id)


# ---------------------------------------------------------------------------
# API pubblica: accensione pianta
# (chiamata da fire_zone._track_plant_contacts)
# ---------------------------------------------------------------------------

func ignite_plant(plant: Node3D) -> void:
	## Nessun cap: la propagazione pianta→pianta è libera.
	## Il freno naturale è plant_spread_radius in FireConfig.
	if not is_instance_valid(plant):
		return
	if plant in _burning_plants.values():
		return   # già in fiamme, evita doppia ignizione
	if fire_zone_scene == null or config == null:
		return

	plant.remove_from_group("combustible_plant")

	var id := _next_fire_id
	_next_fire_id += 1

	var zone : Node3D = fire_zone_scene.instantiate()
	add_child(zone)
	zone.extinguished.connect(_on_fire_extinguished)

	# Marca come fuoco su pianta PRIMA di activate() così fire_zone
	# sa già che tipo di fuoco è durante l'inizializzazione.
	zone.is_plant_fire = true
	zone.plant_node    = plant

	# Tutti i parametri vengono da config — nessun valore hardcoded qui.
	zone.activate(id, plant.global_position, config)

	_active_fires[id]   = zone
	_burning_plants[id] = plant
	plants_ignited += 1
	print("FireManager: pianta #%d in fiamme — piante attive: %d" \
		  % [id, _burning_plants.size()])


# ---------------------------------------------------------------------------
# API pubblica: ceneri (chiamata da fire_zone._destroy_plant)
# ---------------------------------------------------------------------------

func spawn_ash(pos: Vector3, plant_scale: Vector3) -> void:
	# Tizzone (tronco bruciato)
	var stump := MeshInstance3D.new()
	add_child(stump)
	stump.global_position = pos

	var box := BoxMesh.new()
	box.size = Vector3(0.15 * plant_scale.x, 0.3 * plant_scale.y, 0.15 * plant_scale.x)
	stump.mesh = box

	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.04, 0.04, 0.04)
	mat.roughness    = 1.0
	stump.material_override = mat

	# Fumo residuo
	var smoke := GPUParticles3D.new()
	stump.add_child(smoke)

	var pmat := ParticleProcessMaterial.new()
	pmat.emission_shape       = ParticleProcessMaterial.EMISSION_SHAPE_BOX
	pmat.emission_box_extents = box.size / 2.0
	pmat.direction            = Vector3(0, 1, 0)
	pmat.spread               = 15.0
	pmat.gravity              = Vector3(0, 0.3, 0)
	pmat.initial_velocity_min = 0.1
	pmat.initial_velocity_max = 0.3
	pmat.color                = Color(0.3, 0.3, 0.3, 0.3)

	smoke.process_material = pmat
	smoke.amount           = 25
	smoke.lifetime         = 3.0

	var s_mesh := BoxMesh.new()
	s_mesh.size        = Vector3(0.1, 0.1, 0.1)
	smoke.draw_pass_1  = s_mesh

	await get_tree().create_timer(60.0).timeout
	if is_instance_valid(smoke):
		smoke.emitting = false
		await get_tree().create_timer(smoke.lifetime).timeout
		smoke.queue_free()


func on_plant_burned(id: int) -> void:
	plants_burned += 1
	_destroyed_plant_ids.append(id)
