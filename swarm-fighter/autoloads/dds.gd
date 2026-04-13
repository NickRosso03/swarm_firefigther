## dds.gd — Broker DDS, Godot 4.6
##
## ARCHITETTURA: un singolo PacketPeerUDP in ascolto sulla porta 4444.
## Tutti i pacchetti arrivano in un posto solo e vengono smistati
## per indirizzo sorgente "ip:port". Questo evita il bug di Godot 4.6
## con UDPServer che non consegnava i pacchetti successivi (keep-alive,
## publish) allo stesso PacketPeerUDP della connessione iniziale,
## causando la scadenza immediata di tutti i peer.
##
## API locale per gli script GDScript (drone.gd, fire_zone.gd, ecc.):
##   DDS.subscribe("varname")
##   DDS.read("varname")       → float
##   DDS.publish("varname", DDS_TYPE_FLOAT, valore)
##   DDS.clear("varname")

extends Node

# ---------------------------------------------------------------------------
# Costanti di protocollo 
# ---------------------------------------------------------------------------
const COMMAND_KEEP_ALIVE := 0x80
const COMMAND_SUBSCRIBE  := 0x81
const COMMAND_PUBLISH    := 0x82

const DDS_TYPE_UNKNOWN := 0
const DDS_TYPE_INT     := 1
const DDS_TYPE_FLOAT   := 2

const SERVER_PORT  := 4444
const TIME_TO_LIVE := 3.0   # secondi — leggermente > 1s keep-alive di Python

# ---------------------------------------------------------------------------
# Socket unico di ascolto
# ---------------------------------------------------------------------------
var _rx : PacketPeerUDP = PacketPeerUDP.new()

# ---------------------------------------------------------------------------
# Strutture dati
# ---------------------------------------------------------------------------

## Mappa "ip:port" → { ttl, send_peer }
## send_peer è un PacketPeerUDP configurato per inviare A quel client.
var _peers : Dictionary = {}

## Mappa nome_variabile → { type, value, subscribers[] }
## subscribers è una lista di chiavi "ip:port"
var _variables : Dictionary = {}

## Variabili locali per gli script GDScript nella scena
var _local_vars : Dictionary = {}

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
	var err := _rx.bind(SERVER_PORT, "0.0.0.0")
	if err != OK:
		push_error("DDS: bind sulla porta %d fallito (err %d)" % [SERVER_PORT, err])
		return
	print("DDS broker: in ascolto sulla porta %d" % SERVER_PORT)


func _process(delta: float) -> void:
	# 1. Legge tutti i pacchetti disponibili
	while _rx.get_available_packet_count() > 0:
		var pkt  : PackedByteArray = _rx.get_packet()
		var ip   : String          = _rx.get_packet_ip()
		var port : int             = _rx.get_packet_port()
		var key  : String          = "%s:%d" % [ip, port]

		# Registra il peer se è nuovo
		if not _peers.has(key):
			var sender := PacketPeerUDP.new()
			sender.set_dest_address(ip, port)
			_peers[key] = { "ttl": 0.0, "peer": sender }
			print("DDS: nuovo client → %s" % key)

		# Reset TTL ad ogni pacchetto ricevuto
		_peers[key]["ttl"] = 0.0

		if pkt.is_empty():
			continue

		match pkt.decode_u8(0):
			COMMAND_KEEP_ALIVE:
				pass   # TTL già resettato sopra
			COMMAND_SUBSCRIBE:
				_handle_subscribe(key, pkt)
			COMMAND_PUBLISH:
				_handle_publish(pkt)

	# 2. Aggiorna TTL e rimuovi peer scaduti
	var expired : Array = []
	for key in _peers.keys():
		_peers[key]["ttl"] += delta
		if _peers[key]["ttl"] > TIME_TO_LIVE:
			expired.append(key)
	for key in expired:
		print("DDS: client scaduto → %s" % key)
		_peers.erase(key)
		# Rimuovi dalle sottoscrizioni
		for var_name in _variables.keys():
			var v : Dictionary = _variables[var_name]
			v["subscribers"].erase(key)

# ---------------------------------------------------------------------------
# Gestione comandi
# ---------------------------------------------------------------------------

func _handle_subscribe(sender_key: String, pkt: PackedByteArray) -> void:
	## Formato: [0x81, n_vars, len, name_bytes, len, name_bytes, ...]
	var n   : int = pkt.decode_u8(1)
	var idx : int = 2
	for _i in n:
		var nlen  : int              = pkt.decode_u8(idx)
		var _name  : String           = pkt.slice(idx + 1, idx + 1 + nlen).get_string_from_utf8()
		idx += 1 + nlen

		if not _variables.has(_name):
			_variables[_name] = { "type": DDS_TYPE_UNKNOWN, "value": 0.0, "subscribers": [] }

		var subs : Array = _variables[_name]["subscribers"]
		if sender_key not in subs:
			subs.append(sender_key)


func _handle_publish(pkt: PackedByteArray) -> void:
	## Formato: [0x82, type, name_len, name_bytes, value_4bytes]
	var dtype   : int    = pkt.decode_u8(1)
	var nlen    : int    = pkt.decode_u8(2)
	var _name    : String = pkt.slice(3, 3 + nlen).get_string_from_utf8()
	var val_off : int    = 3 + nlen

	var value : float = 0.0
	match dtype:
		DDS_TYPE_FLOAT: value = pkt.decode_float(val_off)
		DDS_TYPE_INT:   value = float(pkt.decode_s32(val_off))

	# Aggiorna store
	if not _variables.has(_name):
		_variables[_name] = { "type": dtype, "value": value, "subscribers": [] }
	else:
		_variables[_name]["type"]  = dtype
		_variables[_name]["value"] = value

	# Aggiorna variabile locale se sottoscritta da GDScript
	if _local_vars.has(_name):
		_local_vars[_name] = value

	# Smista ai subscriber remoti (Python)
	_broadcast(_name)


func _broadcast(var_name: String) -> void:
	if not _variables.has(var_name):
		return
	var v    : Dictionary = _variables[var_name]
	var pkt  : PackedByteArray = _build_publish_packet(
		var_name, v["type"], v["value"])

	for key in v["subscribers"]:
		if _peers.has(key):
			_peers[key]["peer"].put_packet(pkt)


func _build_publish_packet(var_name: String, dtype: int, value: float) -> PackedByteArray:
	var name_bytes : PackedByteArray = var_name.to_utf8_buffer()
	var pkt        : PackedByteArray = PackedByteArray()
	pkt.resize(3 + name_bytes.size() + 4)
	pkt.encode_u8(0, COMMAND_PUBLISH)
	pkt.encode_u8(1, dtype)
	pkt.encode_u8(2, name_bytes.size())
	for i in name_bytes.size():
		pkt.encode_u8(3 + i, name_bytes[i])
	var val_off : int = 3 + name_bytes.size()
	match dtype:
		DDS_TYPE_FLOAT: pkt.encode_float(val_off, value)
		DDS_TYPE_INT:   pkt.encode_s32(val_off, int(value))
	return pkt

# ---------------------------------------------------------------------------
# API locale per script GDScript
# ---------------------------------------------------------------------------

func subscribe(var_name: String) -> void:
	if not _local_vars.has(var_name):
		_local_vars[var_name] = 0.0
	if not _variables.has(var_name):
		_variables[var_name] = { "type": DDS_TYPE_UNKNOWN, "value": 0.0, "subscribers": [] }


func read(var_name: String) -> float:
	return float(_local_vars.get(var_name, 0.0))


func publish(var_name: String, dtype: int, value) -> void:
	if not _variables.has(var_name):
		_variables[var_name] = { "type": dtype, "value": float(value), "subscribers": [] }
	else:
		_variables[var_name]["type"]  = dtype
		_variables[var_name]["value"] = float(value)

	if _local_vars.has(var_name):
		_local_vars[var_name] = float(value)

	_broadcast(var_name)


func clear(var_name: String) -> void:
	_local_vars[var_name] = 0.0
	if _variables.has(var_name):
		_variables[var_name]["value"] = 0.0
