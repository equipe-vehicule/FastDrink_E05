## Position estimee de l'essieu arriere pendant les manoeuvres (contournement, recul),
## a partir de la vitesse commandee et de l'angle de braquage (modele "bicyclette").
## Repere : origine = essieu arriere au debut de la manoeuvre, x = vers la droite, y = vers l'avant.
##
## Le servo ne braque pas instantanement : on suit l'angle qu'il a reellement atteint (servo_estime),
## en le faisant tourner vers la commande a ReglagesRobot.vitesse_servo_deg_s. Sans cela, l'odometrie
## croit les roues deja braquees et se trompe de plusieurs centimetres a chaque virage.
class_name Odometrie
extends RefCounted

var x := 0.0                      # mm
var y := 0.0                      # mm
var cap := 0.0                    # rad, > 0 = vers la droite
var parcours := 0.0               # distance parcourue en marche arriere (mm)
var servo_estime := 90.0          # angle reellement atteint par le servo (deg)
# Position globale : suivie en permanence depuis le depart, jamais remise a zero (memes conventions, mm / rad)
var x_global := 0.0
var y_global := 0.0
var cap_global := 0.0

var _robot: ReglagesRobot

func _init(robot: ReglagesRobot) -> void:
	_robot = robot

## Remet la position a zero (le servo, lui, garde son angle).
func reinitialiser() -> void:
	x = 0.0
	y = 0.0
	cap = 0.0
	parcours = 0.0

## A appeler a chaque pas avec la commande envoyee au servo, meme hors manoeuvre.
func suivre_servo(commande: float, dt: float) -> void:
	var v := _robot.vitesse_servo_deg_s
	servo_estime = commande if v <= 0.0 else move_toward(servo_estime, commande, v * dt)

## Temps (s) pour que le servo passe de son angle actuel a `cible`.
func duree_servo(cible: float) -> float:
	var v := _robot.vitesse_servo_deg_s
	return 0.0 if v <= 0.0 else absf(cible - servo_estime) / v

## A appeler a chaque pas, dans tous les etats : position globale.
func integrer_global(v_mm_s: float, dt: float) -> void:
	var delta := deg_to_rad(_robot.servo_vers_roues_deg(servo_estime - 90.0))
	cap_global += v_mm_s / _robot.empattement_mm * tan(delta) * dt
	x_global += v_mm_s * sin(cap_global) * dt
	y_global += v_mm_s * cos(cap_global) * dt

## Point situe a distance_mm devant l'essieu arriere, dans le repere global.
func point_devant_global(distance_mm: float) -> Vector2:
	return Vector2(x_global + distance_mm * sin(cap_global), y_global + distance_mm * cos(cap_global))

func integrer(v_mm_s: float, dt: float) -> void:
	var delta := deg_to_rad(_robot.servo_vers_roues_deg(servo_estime - 90.0))
	cap += v_mm_s / _robot.empattement_mm * tan(delta) * dt
	x += v_mm_s * sin(cap) * dt
	y += v_mm_s * cos(cap) * dt
	parcours -= v_mm_s * dt          # > 0 en marche arriere
