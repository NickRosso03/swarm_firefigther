## free_camera.gd — Camera free-look per debug della scena.
##
## Controlli:
##   Tasto destro hold   → abilita look (mouse fisico)
##   Alt hold            → abilita look (trackpad: muovi un dito per ruotare)
##   Due dita scroll     → aumenta/diminuisce velocità
##   WASD                → muovi orizzontalmente
##   Q / E               → scendi / sali
##   Shift               → velocità x4
##   F                   → torna alla posizione di overview
extends Camera3D

@export var move_speed   : float = 20.0
@export var look_sens    : float = 0.003
@export var scroll_step  : float = 5.0

var _mouse_active : bool = false   # tasto destro premuto
var _alt_active   : bool = false   # Alt premuto

var _active: bool:
	get: return _mouse_active or _alt_active

func _ready() -> void:
	make_current()

func _unhandled_input(event: InputEvent) -> void:

	# ── Mouse buttons ─────────────────────────────────────────────────────
	if event is InputEventMouseButton:
		match event.button_index:
			MOUSE_BUTTON_RIGHT:
				_mouse_active = event.pressed
				_update_mouse_capture()
			# Due dita scroll su trackpad → stessa cosa dello scroll wheel
			MOUSE_BUTTON_WHEEL_UP:
				move_speed = minf(move_speed + scroll_step, 200.0)
			MOUSE_BUTTON_WHEEL_DOWN:
				move_speed = maxf(move_speed - scroll_step, 1.0)

	# ── Rotazione: MouseMotion attivo se destro OPPURE Alt ───────────────
	if event is InputEventMouseMotion and _active:
		_apply_look(event.relative)

	# ── Tastiera ──────────────────────────────────────────────────────────
	if event is InputEventKey:
		match event.keycode:
			KEY_ALT:
				_alt_active = event.pressed
				_update_mouse_capture()
			KEY_F:
				if event.pressed and not event.echo:
					global_position = Vector3(40.0, 40.0, 80.0)
					rotation        = Vector3(deg_to_rad(-25.0), 0.0, 0.0)
					print("Camera: reset overview")

func _process(delta: float) -> void:
	if not _active:
		return
	var spd := move_speed * (4.0 if Input.is_key_pressed(KEY_SHIFT) else 1.0)
	var dir := Vector3.ZERO
	if Input.is_key_pressed(KEY_W): dir -= transform.basis.z
	if Input.is_key_pressed(KEY_S): dir += transform.basis.z
	if Input.is_key_pressed(KEY_A): dir -= transform.basis.x
	if Input.is_key_pressed(KEY_D): dir += transform.basis.x
	if Input.is_key_pressed(KEY_E): dir += Vector3.UP
	if Input.is_key_pressed(KEY_Q): dir -= Vector3.UP
	if dir != Vector3.ZERO:
		global_position += dir.normalized() * spd * delta

# ── Helpers ───────────────────────────────────────────────────────────────

func _apply_look(relative: Vector2) -> void:
	rotate_y(-relative.x * look_sens)
	rotate_object_local(Vector3.RIGHT, -relative.y * look_sens)
	var euler_rot := rotation
	euler_rot.x = clampf(euler_rot.x, deg_to_rad(-89.0), deg_to_rad(89.0))
	rotation = euler_rot

func _update_mouse_capture() -> void:
	# Con il tasto destro cattura il cursore (comportamento classico FPS)
	# Con Alt lascia il cursore visibile — più comodo sul trackpad
	if _mouse_active:
		Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
	else:
		Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
