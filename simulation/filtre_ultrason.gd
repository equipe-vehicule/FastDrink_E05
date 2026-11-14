## Filtrage de l'ultrason : une mesure -1 isolee (pas d'echo) ne fait pas repartir la voiture.
## Deux filtres, au choix (ReglagesRobot.filtre_ultrason, cle filtre_ultrason de reglages_robot.json) :
##   "mediane3"       : mediane des 3 dernieres mesures (filtre d'origine de la simulation ; mode ideal). Il est
##                      appele a chaque tour du programme et ignore mesure_nouvelle (comportement inchange).
##   "mediane_kalman" : filtre retenu par l'equipe (#65, rapport « Filtre tes connaissances » du 10 nov.) : mediane de 3
##                      puis Kalman a vitesse constante. Reproduit 09_Code-PiCar/fastdrink/filtres/mediane_kalman.py et
##                      kalman.py (parametres : config.py). Le Kalman n'est mis a jour qu'a l'arrivee d'une nouvelle
##                      mesure (mesure_nouvelle) ; entre deux, la sortie est projetee avec la vitesse estimee.
## Ecarts restants avec le Python (mediane_kalman.py) : portee_max_mm = 1500 (Python : D_MAX 2000) ; remise a zero
## apres absence_s (0,15 s ; 1 s si l'obstacle etait proche), qui n'existe pas en Python ; horodatage au tour du
## controleur (<= 20 ms) au lieu de l'instant de lecture.
## Les constantes viennent de config.py de 09_Code-PiCar (dependance simulation <-> Pi a garder synchronisee) ;
## SIGMA_A_MM_S2 = 400 est la valeur MEDIANE_KALMAN_SIGMA_A_MM_S2 (et non KALMAN_SIGMA_A_MM_S2 = 200, Kalman seul).
## Aucune dependance a Godot (RefCounted) : se porte tel quel en Python.
class_name FiltreUltrason
extends RefCounted

const TAILLE_FILTRE := 3          # mediane sur les 3 dernieres mesures

# ----- Parametres de "mediane_kalman" (config.py de 09_Code-PiCar/fastdrink ; regles hors ligne le 9 nov.)
const SIGMA_Z_MM := 2.4           # bruit de mesure (mm) : config.py KALMAN_SIGMA_Z_MM (ecart type, robot immobile, 9 nov.)
const SIGMA_A_MM_S2 := 400.0      # acceleration non modelisee (mm/s2) : config.py MEDIANE_KALMAN_SIGMA_A_MM_S2
const SEUIL_REJET := 9.0          # rejet si nu^2 / S > 9 (3 ecarts types) : config.py KALMAN_SEUIL_REJET
const REJETS_AVANT_RESET := 5     # rejets consecutifs avant de repartir de la mesure : config.py KALMAN_REJETS_AVANT_RESET
const D_MIN_MM := 20.0            # plage acceptee par le Kalman : config.py DISTANCE_MIN_MM
const D_MAX_MM := 2000.0          # config.py DISTANCE_MAX_MM (portee_max_mm ci-dessous, plus stricte, s'applique avant)
const V0_ECART_TYPE_MM_S := 1000.0  # P0 = diag(R, 1000^2), v0 = 0 : kalman.py _demarrer

var portee_max_mm := 1500.0       # au-dela, la mesure est ignoree
var absence_s := 0.15             # duree sans echo avant de considerer qu'il n'y a plus d'obstacle
var zone_proche_mm := 60.0        # sous cette distance, le capteur peut "perdre" un obstacle bien present
var absence_proche_s := 1.0       # duree sans echo exigee quand l'obstacle etait tres proche

var _robot: ReglagesRobot
var _mesures: Array[float] = []
var _d_filtre := -1.0
var _t_absence := 0.0
# Etat de "mediane_kalman"
var _t := 0.0                     # temps ecoule (somme des dt)
var _fenetre: Array = []          # [[instant, mesure mm]] des 3 dernieres mesures valides
var _k_demarre := false
var _kd := 0.0                    # distance estimee (mm) a l'instant _kt
var _kv := 0.0                    # vitesse estimee (mm/s, negative si on s'approche)
var _kt := 0.0
var _p11 := 0.0
var _p12 := 0.0
var _p22 := 0.0
var _rejets := 0

func _init(robot: ReglagesRobot = null) -> void:
	_robot = robot

func reinitialiser() -> void:
	_mesures.clear()
	_d_filtre = -1.0
	_t_absence = 0.0
	_fenetre.clear()
	_k_demarre = false
	_kv = 0.0
	_rejets = 0

## Distance filtree en mm, -1 s'il n'y a pas d'obstacle. distance_cm : derniere mesure recue (-1 = pas d'echo) ;
## mesure_nouvelle : vrai si elle est arrivee depuis le tour precedent (ignore par "mediane3").
## Un "pas d'echo" n'est accepte qu'apres absence_s
## (absence_proche_s si l'obstacle etait dans la zone ou le capteur devient aveugle).
func filtrer(distance_cm: float, dt: float, mesure_nouvelle := true) -> float:
	if _robot != null and _robot.filtre_ultrason == "mediane_kalman":
		return _filtrer_kalman(distance_cm, dt, mesure_nouvelle)
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

## Mediane de 3 + Kalman (mediane_kalman.py) : la mediane des 3 dernieres mesures valides est datee au milieu de la
## fenetre (la mediane de 3 mesures d'une distance reguliere vaut celle du milieu), le Kalman la recoit a cet instant,
## puis la sortie est projetee a l'instant present : d = d_est + v_est x (t - t_kalman).
func _filtrer_kalman(distance_cm: float, dt: float, nouvelle: bool) -> float:
	_t += dt
	var mm := distance_cm * 10.0
	# Comme en Python : une mesure hors de [D_MIN, D_MAX] est rejetee avant la fenetre (traitee comme un echo manque)
	if distance_cm >= 0.0 and mm <= portee_max_mm and mm >= D_MIN_MM and mm <= D_MAX_MM:
		_t_absence = 0.0
		if nouvelle:
			_fenetre.push_back([_t, mm])
			if _fenetre.size() > TAILLE_FILTRE:
				_fenetre.pop_front()
			if not _k_demarre:
				_k_mettre_a_jour(mm, _t)   # demarrage sur la premiere mesure valide
			elif _fenetre.size() == TAILLE_FILTRE:
				var tri := [_fenetre[0][1], _fenetre[1][1], _fenetre[2][1]]
				tri.sort()
				var t_milieu: float = _fenetre[1][0]
				if t_milieu > _kt:
					_k_mettre_a_jour(tri[1], t_milieu)
	else:
		_t_absence += dt
		if nouvelle and _k_demarre:
			_k_mettre_a_jour(-1.0, _t)     # echo manque : prediction seule
		var delai := absence_proche_s if _d_filtre >= 0.0 and _d_filtre < zone_proche_mm else absence_s
		if _t_absence >= delai:
			_fenetre.clear()
			_k_demarre = false
			_kv = 0.0
			_rejets = 0
			_d_filtre = -1.0
			return _d_filtre
	if not _k_demarre:
		return -1.0
	_d_filtre = maxf(_kd + _kv * (_t - _kt), 0.0)
	return _d_filtre

## kalman.py KalmanDistance.mettre_a_jour : mesure z (mm, ou -1) prise a l'instant t.
func _k_mettre_a_jour(z: float, t: float) -> void:
	var valide := z != -1.0 and z >= D_MIN_MM and z <= D_MAX_MM
	var r := SIGMA_Z_MM * SIGMA_Z_MM
	if not _k_demarre:
		if valide:
			_k_demarrer(z, t)
		return
	_k_predire(maxf(t - _kt, 0.0))
	_kt = t
	if not valide:
		return
	var y := z - _kd
	var s := _p11 + r
	if y * y / s > SEUIL_REJET:
		_rejets += 1
		if _rejets >= REJETS_AVANT_RESET:
			_k_demarrer(z, t)
		return
	_rejets = 0
	var k1 := _p11 / s
	var k2 := _p12 / s
	_kd += k1 * y
	_kv += k2 * y
	var p11 := (1.0 - k1) * _p11
	var p12 := (1.0 - k1) * _p12
	var p22 := _p22 - k2 * _p12
	_p11 = p11
	_p12 = p12
	_p22 = p22

func _k_demarrer(z: float, t: float) -> void:
	_k_demarre = true
	_kd = z
	_kv = 0.0
	_kt = t
	_p11 = SIGMA_Z_MM * SIGMA_Z_MM
	_p12 = 0.0
	_p22 = V0_ECART_TYPE_MM_S * V0_ECART_TYPE_MM_S
	_rejets = 0

## Prediction sur T secondes : P = F P F^T + Q, F = [[1, T], [0, 1]], Q = sigma_a^2 [[T^4/4, T^3/2], [T^3/2, T^2]].
func _k_predire(tt: float) -> void:
	_kd += _kv * tt
	var q := SIGMA_A_MM_S2 * SIGMA_A_MM_S2
	var n11 := _p11 + tt * (_p12 + _p12) + tt * tt * _p22 + q * pow(tt, 4.0) / 4.0
	var n12 := _p12 + tt * _p22 + q * pow(tt, 3.0) / 2.0
	var n22 := _p22 + q * tt * tt
	_p11 = n11
	_p12 = n12
	_p22 = n22
