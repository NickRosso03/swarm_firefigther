class_name FireConfig
extends Resource
 
# ---------------------------------------------------------------------------
# Fuoco generico (terra)
# ---------------------------------------------------------------------------
 
## Secondi necessari perché l'intensità salga da 0 a 1.
## Valori bassi = fuoco aggressivo; valori alti = sviluppo lento.
@export var spread_time          : float = 45.0
 
## Raggio [m] entro cui viene generato il fuoco figlio quando
## l'intensità raggiunge 1.0. Il figlio appare tra il 30% e il 60%
## di questa distanza, nella direzione della pianta più vicina
## (o in direzione casuale se non ci sono piante).
@export var spread_radius        : float = 16.0
 
# ---------------------------------------------------------------------------
# Detection radius (cresce linearmente con l'intensità)
# ---------------------------------------------------------------------------
 
## Raggio della DetectionArea quando l'intensità è 0 (fuoco appena nato).
@export var detection_radius_min : float = 4.0
 
## Raggio della DetectionArea quando l'intensità è 1 (fuoco pieno).
@export var detection_radius_max : float = 10.0
 
# ---------------------------------------------------------------------------
# Fuoco su pianta
# ---------------------------------------------------------------------------
 
## Secondi di contatto continuo tra fuoco e pianta prima che la pianta
## si accenda. Il contatto deve essere entro plant_spread_radius.
@export var plant_ignition_delay : float = 10.0
 
## Secondi di combustione totale prima che la pianta venga distrutta
## e sostituita con le ceneri.
@export var plant_burn_time      : float = 60.0
 
## Raggio [m] entro cui un fuoco su pianta cerca altre piante da accendere.
## Tenere basso (1–3 m) per limitare la propagazione alle chiome adiacenti.
@export var plant_spread_radius  : float = 2.0
