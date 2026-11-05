## Filtrage de l'ultrason : mediane des dernieres mesures, et une mesure -1 isolee
## (pas d'echo) ne fait pas repartir la voiture.
class_name FiltreUltrason
extends RefCounted

const TAILLE_FILTRE := 3          # mediane sur les 3 dernieres mesures

var portee_max_mm := 1500.0       # au-dela, la mesure est ignoree
var absence_s := 0.15             # duree sans echo avant de considerer qu'il n'y a plus d'obstacle
var zone_proche_mm := 60.0        # sous cette distance, le capteur peut "perdre" un obstacle bien present
var absence_proche_s := 1.0       # duree sans echo exigee quand l'obstacle etait tres proche

var _mesures: Array[float] = []
var _d_filtre := -1.0
var _t_absence := 0.0

func reinitialiser() -> void:
	_mesures.clear()
	_d_filtre = -1.0
	_t_absence = 0.0

## Distance filtree en mm, -1 s'il n'y a pas d'obstacle.
## Un "pas d'echo" n'est accepte qu'apres absence_s
## (absence_proche_s si l'obstacle etait dans la zone ou le capteur devient aveugle).
func filtrer(distance_cm: float, dt: float) -> float:
	var mm := distance_cm * 10.0
	if distance_cm >= 0.0 and mm <= portee_max_mm:
		_mesures.push_back(mm)
		if _mesures.size() > TAILLE_FILTRE:
			_mesures.pop_front()
		_t_absence = 0.0
		var tri := _mesures.duplicate()
		tri.sort()
		# 1 ou 2 mesures : on garde la plus proche (prudent) ; 3 : la mediane
		_d_filtre = tri[1] if tri.size() >= 3 else tri[0]
	else:
		_t_absence += dt
		var delai := absence_proche_s if _d_filtre >= 0.0 and _d_filtre < zone_proche_mm else absence_s
		if _t_absence >= delai:
			_mesures.clear()
			_d_filtre = -1.0
	return _d_filtre
