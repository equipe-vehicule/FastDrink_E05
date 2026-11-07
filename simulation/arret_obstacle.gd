## Arret a une distance predeterminee d'un obstacle (precision exigee +/- 30 mm).
## Profil de freinage a deceleration constante ; recule si l'on s'est arrete trop pres.
class_name ArretObstacle
extends RefCounted

const ARRET_TOLERANCE_MM := 1.0   # arret declenche a 1 mm de la cible
const HYSTERESIS_MM := 30.0       # l'obstacle doit s'eloigner de plus que ca pour repartir (= tolerance exigee)

var distance_arret_cm := 3.0      # distance d'arret si on ne contourne pas (HC-SR04 : fiable a partir de ~3 cm)
var decalage_capteur_mm := 36.0   # l'avant de la voiture depasse le capteur de 36 mm (modele 3D ; a confirmer)
var recul_autorise := true        # recule si on s'est arrete trop pres (depassement)
var seuil_recul_mm := 15.0        # recule si on est plus pres que cible - seuil (moitie de la tolerance)
var vitesse_recul := 10           # commande de recul (doit vaincre la zone morte du moteur)

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
	var restante := d_mm - decalage_capteur_mm - d_voulue_mm      # mm, < 0 = trop pres
	if recul_autorise and restante < -seuil_recul_mm:
		en_recul = true
	if en_recul:
		if restante >= 0.0:
			en_recul = false
			arrete = true
		else:
			return -float(vitesse_recul)
	if restante <= ARRET_TOLERANCE_MM:
		arrete = true
	elif restante > HYSTERESIS_MM:
		arrete = false
	if arrete:
		return 0.0
	return minf(cible, maxf(_robot.vitesse_freinage(restante), float(_robot.vitesse_approche_min)))
