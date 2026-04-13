## obstacle_config.gd
extends Resource
class_name ObstacleConfig

@export var scene : PackedScene
@export var count : int = 20
@export var min_scale : float = 1.0
@export var max_scale : float = 2.0
@export var plant : bool = false
