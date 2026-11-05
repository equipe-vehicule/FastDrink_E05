## Position estimee de l'essieu arriere pendant les manoeuvres (contournement, recul),
## a partir de la vitesse commandee et de l'angle de braquage (modele "bicyclette").
## Repere : origine = essieu arriere au debut de la manoeuvre, x = vers la droite, y = vers l'avant.
class_name Odometrie
extends RefCounted

var x := 0.0                      # mm
var y := 0.0                      # mm
var cap := 0.0                    # rad, > 0 = vers la droite
var parcours := 0.0               # distance parcourue en marche arriere (mm)

var _robot: ReglagesRobot

func _init(robot: ReglagesRobot) -> void:
	_robot = robot

func reinitialiser() -> void:
	x = 0.0
	y = 0.0
	cap = 0.0
	parcours = 0.0

func integrer(v_mm_s: float, angle_servo: float, dt: float) -> void:
	var delta := deg_to_rad(_robot.servo_vers_roues_deg(angle_servo - 90.0))
	cap += v_mm_s / _robot.empattement_mm * tan(delta) * dt
	x += v_mm_s * sin(cap) * dt
	y += v_mm_s * cos(cap) * dt
	parcours -= v_mm_s * dt          # > 0 en marche arriere
