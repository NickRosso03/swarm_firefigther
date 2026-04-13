## obstacle_spawner.gd — Spawna ostacoli con configurazioni indipendenti
extends Node3D

@export var obstacle_configs : Array[ObstacleConfig] = []
@export var area_half        : float = 75.0
@export var min_dist_from_center : float = 5.0
@export var exclusion_zones  : Array[Vector3] = []

func _ready() -> void:
	if obstacle_configs.is_empty():
		push_error("Nessuna configurazione ostacoli assegnata!")
		return

	for config in obstacle_configs:
		if config == null or config.scene == null:
			continue

		var placed    := 0
		var attempts  := 0
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

				# Le piante vengono aggiunte al gruppo "combustible_plant":
				# fire_zone._track_plant_contacts() le cerca tramite questo gruppo.
				# Quando una pianta prende fuoco viene rimossa dal gruppo da
				# FireManager.ignite_plant() per evitare doppie ignizioni.
				if config.plant:
					obs.add_to_group("combustible_plant")

				placed += 1


func _is_valid(pos: Vector3) -> bool:
	if pos.length() < min_dist_from_center:
		return false
	for zone in exclusion_zones:
		if pos.distance_to(zone) < 8.0:
			return false
	return true
