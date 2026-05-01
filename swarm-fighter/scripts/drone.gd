## drone.gd — Corpo fisico del drone in Godot 4.5
##
## Pubblica verso Python ogni physics frame:
##   drone_{id}/X,Y,Z   posizione world [m]
##   drone_{id}/VX,VY,VZ velocità lineare [m/s]
##   drone_{id}/TX,TY,TZ angoli Euler roll/pitch/yaw [rad]
##   drone_{id}/WX,WY,WZ velocità angolare [rad/s]
##   drone_{id}/connected 1.0 ogni frame (Python aspetta questo per partire)
##   drone_{id}/tick     impulso di sync — SEMPRE l'ultimo publish
##
## Legge da Python ogni physics frame:
##   drone_{id}/f1..f4  forze propulsori [N]
##
## Disposizione motori (vista dall'alto):
##   2 --- 1
##   |     |
##   3 --- 4
##   p1=( L,0, L)  p2=(-L,0, L)  p3=(-L,0,-L)  p4=( L,0,-L)

extends RigidBody3D

@export var drone_id   : int   = 0
@export var arm_length : float = 0.195

@onready var mini_hud : Label3D = $MiniHUD
@onready var water_cannon : Node3D = $WaterCannon
@onready var foam_particles : GPUParticles3D = $WaterCannon/FoamParticles


var _fires_in_range : Array = []  # Array di Area3D nodi fuoco in range

const STATUS_NAMES := {
	0.0: "IDLE/TAKEOFF",
	1.0: "EXPLORING",
	2.0: "MOVING",
	3.0: "RETURNING",
	4.0: "SUPPRESSING",
	5.0: "→ STATION",   # ← nuovo
	6.0: "REFUELING",   # ← nuovo
}

var _prefix  : String
var _p1      : Vector3
var _p2      : Vector3
var _p3      : Vector3
var _p4      : Vector3

var _f1      : float = 0.0
var _f2      : float = 0.0
var _f3      : float = 0.0
var _f4      : float = 0.0

var _initial_pos : Vector3
var _initial_rot : Vector3



func _ready() -> void:
	_prefix = "drone_%d" % drone_id
	_p1 = Vector3( arm_length, 0.0,  arm_length)
	_p2 = Vector3(-arm_length, 0.0,  arm_length)
	_p3 = Vector3(-arm_length, 0.0, -arm_length)
	_p4 = Vector3( arm_length, 0.0, -arm_length)

	_initial_pos = global_position
	_initial_rot = global_rotation


	DDS.subscribe("%s/f1" % _prefix)
	DDS.subscribe("%s/f2" % _prefix)
	DDS.subscribe("%s/f3" % _prefix)
	DDS.subscribe("%s/f4" % _prefix)
	
	DDS.subscribe("%s/status" % _prefix)
	DDS.subscribe("%s/fire_x" % _prefix)
	DDS.subscribe("%s/fire_y" % _prefix)
	DDS.subscribe("%s/fire_z" % _prefix)
	
	

func _physics_process(_delta: float) -> void:
	_f1 = DDS.read("%s/f1" % _prefix)
	_f2 = DDS.read("%s/f2" % _prefix)
	_f3 = DDS.read("%s/f3" % _prefix)
	_f4 = DDS.read("%s/f4" % _prefix)


	_apply_motor_force(_f1, _p1)
	_apply_motor_force(_f2, _p2)
	_apply_motor_force(_f3, _p3)
	_apply_motor_force(_f4, _p4)
	
	# Costante aerodinamica di drag 
	var yaw_factor = 0.5
	
	# f1, f2, f3, f4 sono le forze in Newton ricevute da Python tramite DDS
	# Calcola la coppia di Yaw (Torque) 
	# Assumendo eliche 1 e 3 in un senso, 2 e 4 nell'altro
	var yaw_torque = (_f1 + _f3) - (_f2 + _f4)
	yaw_torque *= yaw_factor
	
	# Applica il momento torcente lungo l'asse Y LOCALE del drone
	apply_torque(basis * Vector3(0, yaw_torque , 0))
	
	# CONO — pubblica il fuoco più vicino nel range (o 0 se nessuno)
	_publish_spotted_fire()
	
	_publish_state()


func _publish_state() -> void:
	var wp : Vector3 = global_position
	var wv : Vector3 = linear_velocity
	var wr : Vector3 = global_rotation
	var wa : Vector3 = angular_velocity

	DDS.publish("%s/X"  % _prefix, DDS.DDS_TYPE_FLOAT, wp.x)
	DDS.publish("%s/Y"  % _prefix, DDS.DDS_TYPE_FLOAT, wp.y)
	DDS.publish("%s/Z"  % _prefix, DDS.DDS_TYPE_FLOAT, wp.z)
	DDS.publish("%s/VX" % _prefix, DDS.DDS_TYPE_FLOAT, wv.x)
	DDS.publish("%s/VY" % _prefix, DDS.DDS_TYPE_FLOAT, wv.y)
	DDS.publish("%s/VZ" % _prefix, DDS.DDS_TYPE_FLOAT, wv.z)
	DDS.publish("%s/TX" % _prefix, DDS.DDS_TYPE_FLOAT, wr.x)
	DDS.publish("%s/TY" % _prefix, DDS.DDS_TYPE_FLOAT, wr.y)
	DDS.publish("%s/TZ" % _prefix, DDS.DDS_TYPE_FLOAT, wr.z)
	DDS.publish("%s/WX" % _prefix, DDS.DDS_TYPE_FLOAT, wa.x)
	DDS.publish("%s/WY" % _prefix, DDS.DDS_TYPE_FLOAT, wa.y)
	DDS.publish("%s/WZ" % _prefix, DDS.DDS_TYPE_FLOAT, wa.z)

	# connected ogni frame: Python potrebbe connettersi in qualsiasi momento
	DDS.publish("%s/connected" % _prefix, DDS.DDS_TYPE_FLOAT, 1.0)

	# tick SEMPRE per ultimo: è il segnale che sincronizza il loop Python
	DDS.publish("%s/tick" % _prefix, DDS.DDS_TYPE_FLOAT, 1.0)

func _publish_spotted_fire() -> void:
	# Rimuovi nodi invalidi (fuochi spenti nel frattempo) O già inattivi
	# (FireZone che ha chiamato _extinguish ma non è ancora stato queue_free'd)
	_fires_in_range = _fires_in_range.filter(
		func(a): return is_instance_valid(a) and a.get_parent().is_active()
	)
	
	if _fires_in_range.is_empty():
		DDS.publish("%s/fire_spotted"    % _prefix, DDS.DDS_TYPE_FLOAT, 0.0)
		DDS.publish("%s/fire_spotted_id" % _prefix, DDS.DDS_TYPE_FLOAT, 0.0)
		DDS.publish("%s/fire_spotted_x"  % _prefix, DDS.DDS_TYPE_FLOAT, 0.0)
		DDS.publish("%s/fire_spotted_y"  % _prefix, DDS.DDS_TYPE_FLOAT, 0.0)
		DDS.publish("%s/fire_spotted_z"  % _prefix, DDS.DDS_TYPE_FLOAT, 0.0)
		return
	
	# Trova l'Area3D del fuoco più vicino
	var nearest_area : Area3D = null
	var nearest_dist := INF
	for area in _fires_in_range:
		var d = global_position.distance_to(area.global_position)
		if d < nearest_dist:
			nearest_dist = d
			nearest_area = area
	
	# fire_id letto dal meta impostato da fire_zone.activate() — non da get_parent()
	var real_id : int = nearest_area.get_meta("fire_id") if nearest_area.has_meta("fire_id") else 0
	# Posizione del FireZone (parent dell'Area3D)
	var fire_pos : Vector3 = nearest_area.get_parent().global_position
	
	DDS.publish("%s/fire_spotted"    % _prefix, DDS.DDS_TYPE_FLOAT, 1.0)
	DDS.publish("%s/fire_spotted_id" % _prefix, DDS.DDS_TYPE_FLOAT, float(real_id))
	DDS.publish("%s/fire_spotted_x"  % _prefix, DDS.DDS_TYPE_FLOAT, fire_pos.x)
	DDS.publish("%s/fire_spotted_y"  % _prefix, DDS.DDS_TYPE_FLOAT, fire_pos.y)
	DDS.publish("%s/fire_spotted_z"  % _prefix, DDS.DDS_TYPE_FLOAT, fire_pos.z)


func _apply_motor_force(force_n: float, local_pos: Vector3) -> void:
	var world_pos   : Vector3 = transform.basis * local_pos
	var world_force : Vector3 = transform.basis * Vector3(0.0, force_n, 0.0)
	apply_force(world_force, world_pos)


func reset() -> void:
	global_position  = _initial_pos
	global_rotation  = _initial_rot
	linear_velocity  = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	_f1 = 0.0; _f2 = 0.0; _f3 = 0.0; _f4 = 0.0
	DDS.clear("%s/f1" % _prefix)
	DDS.clear("%s/f2" % _prefix)
	DDS.clear("%s/f3" % _prefix)
	DDS.clear("%s/f4" % _prefix)
	
	
func _process(_delta: float) -> void:
	# 1. Aggiorna il Mini HUD
	var status : float = DDS.read("%s/status" % _prefix)
	mini_hud.text = STATUS_NAMES.get(status, "UNK") + _prefix
	
	# 2. Gestiione le particelle d'acqua
	if status == 4.0: # SUPPRESSING
		if not foam_particles.emitting:
			foam_particles.emitting = true
			
		# Lettura coordinate dell'incendio assegnato al drone
		var fx = DDS.read("%s/fire_x" % _prefix)
		var fy = DDS.read("%s/fire_y" % _prefix)
		var fz = DDS.read("%s/fire_z" % _prefix)
		var fire_pos = Vector3(fx, fy, fz)
		
		if fire_pos.length_squared() > 0.01 and \
		   fire_pos.distance_squared_to(water_cannon.global_position) > 0.25:
			water_cannon.look_at(fire_pos, Vector3.UP)
	else:
		if foam_particles.emitting:
			foam_particles.emitting = false


func _on_vision_cone_area_area_entered(area: Area3D) -> void:
	if not _fires_in_range.has(area):
		_fires_in_range.append(area)



func _on_vision_cone_area_area_exited(area: Area3D) -> void:
	_fires_in_range.erase(area)
