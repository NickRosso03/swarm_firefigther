## obstacle_spawner.gd — Spawna ostacoli con configurazioni indipendenti
extends Node3D

@export var obstacle_configs : Array[ObstacleConfig] = []
@export var area_half        : float = 75.0
@export var min_dist_from_center : float = 5.0
@export var exclusion_zones  : Array[Vector3] = []
@export var station_exclusion_radius : float = 20.0

# Non esportiamo più l'array, lo riempiamo da soli!
var charging_stations : Array[Node] = []

func _ready() -> void:
	if obstacle_configs.is_empty():
		push_error("Nessuna configurazione ostacoli assegnata!")
		return

	# FIX 1: Aspettiamo che il motore fisico e la scena si siano "assestati"
	await get_tree().process_frame
	
	# FIX 2: Recuperiamo in automatico tutte le stazioni usando i Gruppi
	charging_stations = get_tree().get_nodes_in_group("charging_stations")
	
	# Un piccolo print di debug per essere sicuri che le trovi
	print("[Spawner] Trovate ", charging_stations.size(), " stazioni per le exclusion zones.")

	for config in obstacle_configs:
		if config == null or config.scene == null:
			continue

		var placed   := 0
		var attempts := 0
		var max_attempts := config.count * 20

		while placed < config.count and attempts < max_attempts:
			attempts += 1
			var x := randf_range(-area_half, area_half)
			var z := randf_range(-area_half, area_half)
			var spawn_pos := Vector3(x, 0, z)

			if _is_valid(spawn_pos):
				var obs : Node3D = config.scene.instantiate()
				add_child(obs)
				obs.global_position = spawn_pos
				obs.rotation.y = randf() * TAU

				var s := randf_range(config.min_scale, config.max_scale)
				obs.scale = Vector3(s, s, s)

				if config.plant:
					obs.add_to_group("combustible_plant")

				placed += 1


func _is_valid(pos: Vector3) -> bool:
	if pos.length() < min_dist_from_center:
		return false
		
	for zone in exclusion_zones:
		if pos.distance_to(zone) < 8.0:
			return false
			
	# Esclusione stazione di ricarica
	for station in charging_stations:
		if station != null:
			var sp = station.global_position
			sp.y = 0.0
			var p2 := pos
			p2.y = 0.0
			if p2.distance_to(sp) < station_exclusion_radius:
				return false
				
	return true
