## fire_manager.gd — Gestore degli incendi dinamici.
##
## Logica di spawn:
##   - I fuochi a terra nascono spontaneamente con intervallo adattivo:
##     più incendi attivi ci sono, più tempo passa prima del prossimo spawn.
##     Formula: intervallo = base × (1 + ground_fires × SPAWN_SLOWDOWN)
##   - La propagazione pianta→pianta è LIBERA: nessun cap.
##     Una pianta accesa cerca le piante vicine (entro plant_spread_radius)
##     e le accende dopo plant_ignition_delay secondi di contatto.
##   - Solo gli spawn a terra hanno un massimo (max_ground_fires).

extends Node3D

# ---------------------------------------------------------------------------
# Parametri (modificabili dall'Inspector)
# ---------------------------------------------------------------------------
@export var fire_zone_scene   : PackedScene
@export var area_min          : Vector2 = Vector2(-90.0,  90.0)
@export var area_max          : Vector2 = Vector2( 90.0, -90.0)
@export var min_interval      : float   = 8.0    # [s] intervallo base minimo tra spawn a terra
@export var max_interval      : float   = 18.0   # [s] intervallo base massimo
@export var max_ground_fires  : int     = 10      # cap solo sugli spawn spontanei a terra
@export var fire_altitude     : float   = 0.0

## Fattore di rallentamento adattivo dello spawn.
## Con N fuochi a terra attivi: intervallo × (1 + N × SPAWN_SLOWDOWN)
const SPAWN_SLOWDOWN := 0.2

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

var _spawn_timer   : float = 0.0
var _next_spawn_at : float = 0.0

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
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
	var multiplier := 1.0 + _ground_fire_ids.size() * SPAWN_SLOWDOWN
	_next_spawn_at = randf_range(min_interval, max_interval) * multiplier


## Funzione interna condivisa: istanzia e attiva una FireZone.
## is_plant = true → non conta nel cap a terra.
func _spawn_fire_at(pos: Vector3, is_plant: bool) -> int:
	var id := _next_fire_id
	_next_fire_id += 1

	var zone : Node3D = fire_zone_scene.instantiate()
	add_child(zone)
	zone.extinguished.connect(_on_fire_extinguished)
	zone.activate(id, pos)

	_active_fires[id] = zone
	if not is_plant:
		_ground_fire_ids.append(id)
	return id


# ---------------------------------------------------------------------------
# Callback spegnimento
# ---------------------------------------------------------------------------

func _on_fire_extinguished(id: int) -> void:
	_active_fires.erase(id)
	_ground_fire_ids.erase(id)   # noop se era fuoco su pianta
	_burning_plants.erase(id)    # noop se era fuoco a terra
	print("FireManager: #%d spento — attivi: %d (terra: %d, piante: %d)" \
		  % [id, _active_fires.size(), _ground_fire_ids.size(), _burning_plants.size()])


# ---------------------------------------------------------------------------
# API pubblica: propagazione a terra → figlio
# (chiamata da fire_zone._try_propagate)
# ---------------------------------------------------------------------------

func spawn_child_fire(pos: Vector3) -> void:
	## Figlio a terra: clampato nell'arena.
	## Non ha cap proprio: ogni fuoco a terra propaga al massimo una volta
	## (flag _spread_done in fire_zone), quindi il numero è naturalmente limitato.
	if fire_zone_scene == null:
		return
	var x_min = min(area_min.x, area_max.x);  var x_max = max(area_min.x, area_max.x)
	var z_min = min(area_min.y, area_max.y);  var z_max = max(area_min.y, area_max.y)
	pos.x = clamp(pos.x, x_min, x_max)
	pos.z = clamp(pos.z, z_min, z_max)
	pos.y = fire_altitude
	var id := _spawn_fire_at(pos, false)
	print("FireManager: propagazione a terra → #%d" % id)


# ---------------------------------------------------------------------------
# API pubblica: accensione pianta
# (chiamata da fire_zone._track_plant_contacts)
# ---------------------------------------------------------------------------

func ignite_plant(plant: Node3D) -> void:
	## Nessun cap: la propagazione pianta→pianta è libera.
	## Il freno naturale è la distanza fisica (plant_spread_radius in fire_zone).
	if not is_instance_valid(plant):
		return
	if plant in _burning_plants.values():
		return   # già in fiamme, evita doppia ignizione
	if fire_zone_scene == null:
		return

	plant.remove_from_group("combustible_plant")

	var id := _next_fire_id
	_next_fire_id += 1

	var zone : Node3D = fire_zone_scene.instantiate()
	add_child(zone)
	zone.extinguished.connect(_on_fire_extinguished)

	zone.is_plant_fire        = true
	zone.plant_node           = plant
	zone.spread_time          = 60.0
	zone.plant_ignition_delay = 10.0
	zone.plant_spread_radius  = 2.0

	zone.activate(id, plant.global_position)
	_active_fires[id]   = zone
	_burning_plants[id] = plant
	print("FireManager: pianta #%d in fiamme — piante attive: %d" \
		  % [id, _burning_plants.size()])


# ---------------------------------------------------------------------------
# API pubblica: ceneri (chiamata da fire_zone._destroy_plant)
# ---------------------------------------------------------------------------

func spawn_ash(pos: Vector3, plant_scale: Vector3) -> void:
	# 1. Il tizzone (Tronco bruciato)
	var stump := MeshInstance3D.new()
	add_child(stump)
	
	# Posizionamento: lo mettiamo a terra
	stump.global_position = pos
	
	# Mesh: Cubo schiacciato (stile "monolite" o pezzo di legno)
	var box := BoxMesh.new()
	# Usiamo la scala della pianta per proporzionare il tizzone
	box.size = Vector3(0.15 * plant_scale.x, 0.3 * plant_scale.y, 0.15 * plant_scale.x)
	stump.mesh = box
	
	# Materiale: Carbone spento
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.04, 0.04, 0.04)
	mat.roughness = 1.0 # Opaco come il carbone
	stump.material_override = mat

	# 2. Effetto fumo residuo (opzionale e leggerissimo)
	var smoke := GPUParticles3D.new()
	stump.add_child(smoke)
	
	var pmat := ParticleProcessMaterial.new()
	pmat.emission_shape = ParticleProcessMaterial.EMISSION_SHAPE_BOX
	pmat.emission_box_extents = box.size / 2.0
	pmat.direction = Vector3(0, 1, 0)
	pmat.spread = 15.0
	pmat.gravity = Vector3(0, 0.3, 0) # Sale piano
	pmat.initial_velocity_min = 0.1
	pmat.initial_velocity_max = 0.3
	# Colore fumo: grigio scuro che sfuma in trasparenza
	pmat.color = Color(0.3, 0.3, 0.3, 0.3)
	
	smoke.process_material = pmat
	smoke.amount = 25
	smoke.lifetime = 3.0
	
	# Mesh delle particelle (piccoli cubetti di fumo o sfere)
	var s_mesh := BoxMesh.new()
	s_mesh.size = Vector3(0.1, 0.1, 0.1)
	smoke.draw_pass_1 = s_mesh
	
	# 3. Gestione ciclo di vita del fumo
	await get_tree().create_timer(60.0).timeout
	if is_instance_valid(smoke):
		smoke.emitting = false
		# Rimuove il nodo fumo una volta finito, lasciando il tizzone fisso
		await get_tree().create_timer(smoke.lifetime).timeout
		smoke.queue_free()
