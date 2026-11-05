## Suivi de la ligne en reculant sur de courtes distances (< 200 mm).
##
## Les sondes sont a l'avant, donc a l'arriere du mouvement. Garder les sondes sur la ligne serait
## instable (l'essieu, qui mene, s'ecarterait de plus en plus). On asservit plutot un point virtuel
## situe derriere l'essieu arriere, estime avec l'ecart lu par les sondes et le cap odometrique.
class_name ReculLigne
extends RefCounted

const RECUL_MAX_MM := 200.0       # le suivi en marche arriere est limite aux courtes distances

enum Fin { EN_COURS, TERMINE, LIGNE_PERDUE }

var vitesse_recul_ligne := 15     # commande de recul (valeur absolue)
var point_virtuel_mm := 100.0     # distance du point asservi derriere l'essieu arriere (plus grand = plus doux)
var gain_recul := 1.5             # 1/s : vitesse de convergence du point virtuel vers la ligne
var ligne_perdue_recul_s := 0.3   # arret si la ligne reste invisible plus longtemps
var reprendre_apres_recul := false # true = repart en marche avant a la fin du recul, false = reste arrete
# Test "recul" sans toucher au reste du jeu : apres l'arret devant l'obstacle, on recule de distance_auto_mm.
# 0 = desactive (comportement normal : arret ou contournement).
var distance_auto_mm := 0.0
var attente_auto_s := 1.0

var fin := Fin.EN_COURS

var _robot: ReglagesRobot
var _odo: Odometrie
var _suivi: SuiviLigne
var _distance_mm := 0.0
var _t_ligne_perdue := 0.0

func _init(robot: ReglagesRobot, odo: Odometrie, suivi: SuiviLigne) -> void:
	_robot = robot
	_odo = odo
	_suivi = suivi

## La voiture doit etre sur la ligne et a peu pres dans son axe : ce cap sert de reference (cap = 0).
func demarrer(distance_mm: float) -> void:
	_distance_mm = clampf(distance_mm, 0.0, RECUL_MAX_MM)
	_odo.reinitialiser()
	_t_ligne_perdue = 0.0
	fin = Fin.EN_COURS

## Retourne Vector2(angle servo, vitesse cible). Regarder `fin` ensuite.
func calculer(somme: float, n: int, dt: float, cmd: float) -> Vector2:
	var restante := _distance_mm - _odo.parcours
	if n > 0:
		_suivi.derniere_erreur = somme / n
		_t_ligne_perdue = 0.0
	else:
		_t_ligne_perdue += dt
	if restante <= ArretObstacle.ARRET_TOLERANCE_MM:
		fin = Fin.TERMINE
		return Vector2(90.0, 0.0)
	if _t_ligne_perdue > ligne_perdue_recul_s:
		fin = Fin.LIGNE_PERDUE               # ligne perdue : on ne recule pas a l'aveugle
		return Vector2(90.0, 0.0)
	var angle := _angle_recul(_suivi.derniere_erreur, cmd)
	# meme profil de freinage que devant un obstacle, pour s'arreter a la distance demandee
	var v_freinage := _robot.vitesse_freinage(restante)
	var cible := -minf(float(vitesse_recul_ligne), maxf(v_freinage, float(_robot.vitesse_approche_min)))
	return Vector2(angle, cible)

## On asservit un point P situe point_virtuel_mm (B) derriere l'essieu :
##   ecart de P a la ligne : e_p = e + (D + B) * sin(cap)        (D = distance_sondes_mm)
##   loi : delta = L / B * (cap + gain_recul * e_p / |v|)  =>  P converge vers la ligne
## et, contrairement aux sondes, garder P sur la ligne est stable en marche arriere.
func _angle_recul(erreur_mm: float, cmd: float) -> float:
	var e_p := erreur_mm + (_robot.distance_sondes_mm + point_virtuel_mm) * sin(_odo.cap)
	var v := maxf(absf(cmd) * _robot.mm_s_par_cmd, 20.0)        # evite la division par ~0 au demarrage
	var delta := _robot.empattement_mm / point_virtuel_mm * (_odo.cap + gain_recul * e_p / v)
	var servo := _robot.roues_vers_servo(rad_to_deg(delta))
	return 90.0 + clampf(servo, -45.0, 45.0)
