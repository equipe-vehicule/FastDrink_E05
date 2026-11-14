## Recherche de la ligne perdue depuis longtemps (virage rate apres un contournement...) : au lieu de s'arreter, la
## voiture va, toujours en avancant, devant le dernier point de ligne vu (dans le sens de marche), puis cherche en
## spirale jusqu'a croiser la ligne. (Journal Godot du 6 novembre, 19 h 17 : la voiture tournait en rond 8 s puis
## s'arretait seule.) Les transitions (ligne revue, obstacle, abandon) sont decidees par le Controleur.
class_name RetrouverLigne
extends RefCounted

var apres_s := 9.0                   # ligne perdue depuis ca en suivi. Pas moins : en virage serre, la voiture retrouve
									 # la ligne seule en tournant (a 4 s, elle bouclait)
var max_s := 90.0                    # ensuite BLOQUE
var approche_max_s := 15.0           # ensuite, ou une fois la cible atteinte : recherche en spirale
var avance_cible_mm := 200.0         # la cible est devant le dernier point vu, dans le sens de marche : jamais en arriere
var rayon_cible_mm := 80.0           # cible consideree comme atteinte
var spirale_s := 8.0                 # plus grand = la spirale s'ouvre plus lentement
var vitesse_mm_s := 72.0    # (= ancienne commande 18 a 4 mm/s par commande)
var gain := 2.0                      # degres de roue par degre d'ecart de direction
var rayon_obstacle_recent_mm := 400.0  # si pres de l'obstacle evite, on s'en ecarte d'abord

var _robot: ReglagesRobot
var _odo: Odometrie
var _cible := Vector2.ZERO
var _t_spirale := -1.0               # < 0 : la voiture va encore vers la cible
var _cote_spirale := 1

func _init(robot: ReglagesRobot, odo: Odometrie) -> void:
	_robot = robot
	_odo = odo

## Ou chercher la ligne : devant le dernier point vu (ou devant l'obstacle qui vient d'etre passe), dans le sens de
## marche. Revenir au dernier point vu ramenait la voiture sur le chemin deja parcouru.
func cible_ligne(memoire: MemoireParcours) -> Vector2:
	var base := memoire.ligne[-1]
	var proches: Array[Vector2] = [memoire.obstacle_hors_ligne]
	if not memoire.obstacles.is_empty():
		proches.append(memoire.obstacles[-1])
	for o in proches:
		if o.is_finite() and o.distance_to(memoire.ligne[-1]) < rayon_obstacle_recent_mm and (o - base).dot(memoire.sens_ligne()) > 0.0:
			base = o
	return base + memoire.sens_ligne() * avance_cible_mm

## derniere_erreur : dernier ecart de la ligne vu par le suivi (cote de la spirale par defaut).
func demarrer(memoire: MemoireParcours, derniere_erreur: float) -> void:
	_cible = cible_ligne(memoire)
	_t_spirale = -1.0
	_cote_spirale = 1 if derniere_erreur >= 0.0 else -1

## Retourne Vector2(angle servo, vitesse cible) pour aller vers la cible, puis tourner en spirale.
func conduire(obstacle_recent: Vector2, t_etat: float, dt: float) -> Vector2:
	var ici := Vector2(_odo.x_global, _odo.y_global)
	var vers := _cible - ici
	var rel := wrapf(atan2(vers.x, vers.y) - _odo.cap_global, -PI, PI)
	if ici.distance_to(obstacle_recent) < rayon_obstacle_recent_mm:
		# Tout pres de l'obstacle evite, l'ultrason ne voit pas ses coins sur le cote : on s'en ecarte d'abord.
		var vers_obstacle := obstacle_recent - ici
		var rel_obstacle := wrapf(atan2(vers_obstacle.x, vers_obstacle.y) - _odo.cap_global, -PI, PI)
		if absf(rel_obstacle) < deg_to_rad(100.0):
			return Vector2(90.0 - 45.0 * signf(rel_obstacle), vitesse_mm_s)
	if _t_spirale < 0.0 and (vers.length() < rayon_cible_mm or t_etat > approche_max_s):
		_t_spirale = 0.0
		_cote_spirale = 1 if rel >= 0.0 else -1
	if _t_spirale >= 0.0:
		_t_spirale += dt
		return Vector2(90.0 + _cote_spirale * 45.0 / (1.0 + _t_spirale / spirale_s), vitesse_mm_s)
	var servo := _robot.roues_vers_servo(rad_to_deg(rel) * gain)
	return Vector2(90.0 + clampf(servo, -45.0, 45.0), vitesse_mm_s)
