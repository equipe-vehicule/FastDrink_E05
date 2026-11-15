## Suivi de la ligne en reculant sur de courtes distances (< ReglagesEpreuve.RECUL_MAX_MM).
## Utilise devant chaque obstacle (Controleur, etat RECUL_OBSTACLE) et en fin de ligne (piste Recul).
##
## Les sondes sont a l'avant, donc a l'arriere du mouvement. Garder les sondes sur la ligne serait
## instable (l'essieu, qui mene, s'ecarterait de plus en plus). On asservit plutot un point virtuel
## situe derriere l'essieu arriere, estime avec l'ecart lu par les sondes et le cap odometrique.
class_name ReculLigne
extends RefCounted


enum Fin { EN_COURS, TERMINE, LIGNE_PERDUE }

## Reglages : les valeurs sont dans reglages_algorithmes.json (objet « recul »), lues au demarrage ; ici : type et justification seulement.
@export var vitesse_recul_ligne_mm_s: float # (= ancienne commande 15 a 4 mm/s par commande) vitesse de recul, en valeur absolue
@export var point_virtuel_mm: float # distance du point asservi derriere l'essieu arriere (plus grand = plus doux)
@export var gain_recul: float     # 1/s : vitesse de convergence du point virtuel vers la ligne
@export var ligne_perdue_recul_s: float # ligne invisible plus longtemps : perdue
@export var vitesse_min_loi_mm_s: float # plancher de |v| dans la loi : evite la division par ~0 au demarrage
## Ligne perdue en cours de recul : true = finir la distance tout droit (odometrie), false = s'arreter (Fin.LIGNE_PERDUE).
## Voiture arretee de travers en sortie de virage (jusqu'a 19 deg) : en reculant, les sondes quittent la ligne apres
## ~110 mm (banc du 12 nov. : reculs de 78 a 108 mm pour 150 demandes). La distance de recul est imposee, et la ligne
## droite avant l'obstacle garantit la place.
@export var finir_sans_ligne: bool
var ligne_perdue_en_route := false # diagnostic : la fin du recul s'est faite sans la ligne
@export var reprendre_apres_recul: bool # true = repart en marche avant a la fin du recul, false = reste arrete

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
## attente_ligne_s : duree toleree sans ligne au debut (recul lance apres la fin de la ligne, sondes au-dela du ruban).
func demarrer(distance_mm: float, attente_ligne_s := 0.0) -> void:
	_distance_mm = clampf(distance_mm, 0.0, ReglagesEpreuve.RECUL_MAX_MM)
	_odo.reinitialiser()
	_t_ligne_perdue = -attente_ligne_s
	ligne_perdue_en_route = false
	fin = Fin.EN_COURS

## Retourne Vector2(angle servo, vitesse cible). Regarder `fin` ensuite.
func calculer(somme: float, n: int, dt: float, v_mm_s: float) -> Vector2:
	var restante := _distance_mm - _odo.parcours
	if n > 0:
		_suivi.derniere_erreur = somme / n
		_t_ligne_perdue = 0.0
	else:
		_t_ligne_perdue += dt
	if restante <= ArretObstacle.ARRET_TOLERANCE_MM:
		fin = Fin.TERMINE
		return Vector2(90.0, 0.0)
	var angle := _angle_recul(_suivi.derniere_erreur, v_mm_s)
	if _t_ligne_perdue > ligne_perdue_recul_s:
		if not finir_sans_ligne:
			fin = Fin.LIGNE_PERDUE           # ligne perdue : on ne recule pas a l'aveugle
			return Vector2(90.0, 0.0)
		ligne_perdue_en_route = true
		angle = 90.0                         # fin du recul tout droit

	# meme profil de freinage que devant un obstacle, pour s'arreter a la distance demandee
	var v_freinage := _robot.vitesse_freinage(restante)
	var cible := -minf(vitesse_recul_ligne_mm_s, maxf(v_freinage, _robot.vitesse_approche_min_mm_s))
	return Vector2(angle, cible)

## On asservit un point P situe point_virtuel_mm (B) derriere l'essieu :
##   ecart de P a la ligne : e_p = e + (D + B) * sin(cap)        (D = distance_sondes_mm)
##   loi : delta = L / B * (cap + gain_recul * e_p / |v|)  =>  P converge vers la ligne
## et, contrairement aux sondes, garder P sur la ligne est stable en marche arriere.
## Limite (revue 04, 14 nov.) : le cap de depart sert de direction de la ligne. Apres un virage de 90 deg, la voiture
## s'arrete a ~6 deg de la ligne, essieu a ~29 mm, sondes presque dessus ; en 150 mm de recul, la loi garde les sondes
## sur la ligne et l'essieu finit a ~38 mm. Essaye et rejete : estimer la direction de la ligne (correction integrale)
## ramene un peu l'essieu mais les sondes perdent la ligne.
func _angle_recul(erreur_mm: float, v_mm_s: float) -> float:
	var cap := _odo.cap
	var e_p := erreur_mm + (_robot.distance_sondes_mm + point_virtuel_mm) * sin(cap)
	var v := maxf(absf(v_mm_s), vitesse_min_loi_mm_s)
	var delta := _robot.empattement_mm / point_virtuel_mm * (cap + gain_recul * e_p / v)
	var servo := _robot.roues_vers_servo(rad_to_deg(delta))
	return 90.0 + clampf(servo, -45.0, 45.0)
