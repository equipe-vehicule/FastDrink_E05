## Arret a une distance predeterminee d'un obstacle (precision exigee +/- 30 mm).
## Profil de freinage a deceleration constante ; recule si l'on s'est arrete trop pres.
## La distance d'arret voulue (avant de la voiture -> face de l'obstacle) vient de ReglagesEpreuve.distance_arret_mm.
class_name ArretObstacle
extends RefCounted

const ARRET_TOLERANCE_MM := 1.0   # arret declenche a 1 mm de la cible
const HYSTERESIS_MM := 30.0       # l'obstacle doit s'eloigner de plus que ca pour repartir (= tolerance exigee)

var recul_autorise := true        # recule si on s'est arrete trop pres (depassement)
var seuil_recul_mm := 15.0        # recule si on est plus pres que cible - seuil (moitie de la tolerance)
var vitesse_recul_mm_s := 40.0    # (= ancienne commande 10 a 4 mm/s par commande) vitesse de recul, en valeur absolue

var arrete := false
var en_recul := false

var _robot: ReglagesRobot

func _init(robot: ReglagesRobot) -> void:
	_robot = robot

## Plus d'obstacle (ou retour en suivi) : on peut repartir.
func relacher() -> void:
	arrete = false
	en_recul = false

## Vitesse permettant de s'arreter a d_voulue_mm de l'obstacle (au plus `cible`).
## Retourne une vitesse negative si l'on doit reculer (arret trop pres).
func limiter(d_mm: float, cible: float, d_voulue_mm: float) -> float:
	var restante := d_mm - _robot.ultrason_avant_mm - d_voulue_mm      # mm, < 0 = trop pres
	if recul_autorise and restante < -seuil_recul_mm:
		en_recul = true
	if en_recul:
		if restante >= 0.0:
			en_recul = false
			arrete = true
		else:
			return -vitesse_recul_mm_s
	if restante <= ARRET_TOLERANCE_MM:
		arrete = true
	elif restante > HYSTERESIS_MM:
		arrete = false
	if arrete:
		return 0.0
	return minf(cible, maxf(_robot.vitesse_freinage(restante), _robot.vitesse_approche_min_mm_s))
