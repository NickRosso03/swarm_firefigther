## charging_station.gd — Stazione di ricarica dedicata a un singolo drone.
extends Node3D

@export var station_id   : int   = 0
@export var pad_color_idle   : Color = Color(0.2, 0.8, 0.2)
@export var pad_color_active : Color = Color(1.0, 0.7, 0.0)

@onready var _indicator_light : OmniLight3D    = $IndicatorLight
@onready var _particles       : GPUParticles3D = $RechargeParticles
@onready var _pad_mesh        : CSGCylinder3D  = $Pad

var _is_recharging : bool = false

func _ready() -> void:
	DDS.subscribe("drone_%d/refueling" % station_id)
	_set_idle_visuals()


func _physics_process(_delta: float) -> void:
	# Ogni stazione pubblica solo la propria posizione
	DDS.publish("world/station_%d_pos_x" % station_id, DDS.DDS_TYPE_FLOAT, global_position.x)
	DDS.publish("world/station_%d_pos_z" % station_id, DDS.DDS_TYPE_FLOAT, global_position.z)

	var recharging := DDS.read("drone_%d/refueling" % station_id) == 1.0
	if recharging and not _is_recharging:
		_is_recharging = true
		_set_active_visuals()
	elif not recharging and _is_recharging:
		_is_recharging = false
		_set_idle_visuals()


func _set_idle_visuals() -> void:
	if _indicator_light: _indicator_light.light_color = pad_color_idle
	if _particles:       _particles.emitting = false
	if _pad_mesh:
		var mat = _pad_mesh.material
		if mat: mat.albedo_color = pad_color_idle


func _set_active_visuals() -> void:
	if _indicator_light: _indicator_light.light_color = pad_color_active
	if _particles:       _particles.emitting = true
	if _pad_mesh:
		var mat = _pad_mesh.material
		if mat: mat.albedo_color = pad_color_active
