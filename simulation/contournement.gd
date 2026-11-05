## Contournement d'un obstacle pose sur la ligne, en quittant la ligne-guide (cote choisi par cote_contournement).
## Phases :
##   SORTIE (on quitte la ligne en braquant) -> DECALAGE (tout droit jusqu'au decalage lateral voulu)
##   -> REDRESSEMENT (cap parallele a la ligne) -> PARALLELE (on longe l'obstacle) -> RETOUR_VIRAGE
##   -> RETOUR (on revient vers la ligne) -> CROISEMENT (ligne vue : on continue tout droit jusqu'au bon
##   point de braquage) -> ALIGNEMENT (virage qui finit sur la ligne, dans l'axe) -> termine.
## Les sondes sont devant l'essieu : si l'on braque des qu'elles voient la ligne, l'arriere de la voiture
## est encore loin de la ligne et les sondes la perdent en se redressant. On repere donc la position
## de la ligne au moment ou elle est vue, et l'on braque quand le virage nous amenera dessus.
## La position pendant la manoeuvre est estimee par odometrie.
class_name Contournement
extends RefCounted

enum Phase { SORTIE, DECALAGE, REDRESSEMENT, PARALLELE, RETOUR_VIRAGE, RETOUR, CROISEMENT, ALIGNEMENT }

var actif := true                 # false = on s'arrete devant l'obstacle (test "Arret devant obstacle")
var cote_contournement := 1       # 1 = par la droite, -1 = par la gauche
var vitesse_manoeuvre := 25
var distance_manoeuvre_cm := 20.0 # distance d'arret avant de contourner (laisse la place de braquer)
var pause_s := 0.5                # immobilisation avant la manoeuvre
var distance_securite_mm := 40.0  # arret d'urgence si un obstacle apparait devant pendant la manoeuvre
var braquage_manoeuvre := 40.0    # degres de servo par rapport a 90 pendant les virages (max 45)
var cap_evitement_deg := 40.0     # cap pris pour s'ecarter de la ligne
var cap_retour_deg := 30.0        # cap pris pour revenir vers la ligne
var gain_cap := 1.0               # degres de roue par degre d'erreur de cap
var gain_lateral := 0.005         # rad de cap par mm d'ecart lateral (phase PARALLELE)
var correction_laterale_max_deg := 10.0
# Dimensions (mm) : A MESURER sur la piste "Contournement" et sur la voiture
var largeur_obstacle_mm := 100.0
var longueur_obstacle_mm := 100.0 # dans le sens de la marche
var largeur_voiture_mm := 120.0
var marge_laterale_mm := 30.0
var marge_longitudinale_mm := 60.0

var phase := Phase.SORTIE
var termine := false

var _robot: ReglagesRobot
var _odo: Odometrie
var _y_fin_obstacle := 0.0
var _x_ligne := 0.0               # position x de la ligne, reperee quand les sondes la revoient

func _init(robot: ReglagesRobot, odo: Odometrie) -> void:
	_robot = robot
	_odo = odo

func demarrer(d0_mm: float) -> void:
	_odo.reinitialiser()
	# l'essieu arriere doit depasser l'arriere de l'obstacle (+ marge) avant de revenir vers la ligne
	_y_fin_obstacle = d0_mm + _robot.longueur_avant_mm + longueur_obstacle_mm + marge_longitudinale_mm
	phase = Phase.SORTIE
	termine = false

func nom_phase() -> String:
	return Phase.keys()[phase]

## Ecart de la ligne a la fin de la manoeuvre (mm, > 0 = a droite), pour le suivi si les sondes ne la voient pas.
func ecart_ligne_mm() -> float:
	return clampf(_x_ligne - _odo.x, -SuiviLigne.ERREUR_MAX_MM, SuiviLigne.ERREUR_MAX_MM)

## Retourne Vector2(angle servo, vitesse cible). Regarder `termine` ensuite.
func calculer(somme: float, n: int) -> Vector2:
	var angle := 90.0
	var cible := float(vitesse_manoeuvre)
	var cote := float(cote_contournement)
	var x_cible := largeur_obstacle_mm / 2.0 + largeur_voiture_mm / 2.0 + marge_laterale_mm

	match phase:
		Phase.SORTIE:
			angle = 90.0 + cote * braquage_manoeuvre
			if cote * _odo.cap >= deg_to_rad(cap_evitement_deg):
				phase = Phase.DECALAGE

		Phase.DECALAGE:
			angle = _angle_vers_cap(cote * deg_to_rad(cap_evitement_deg))
			# le redressement ajoute encore R * (1 - cos(cap)) de decalage : on l'anticipe
			var gain_redressement := _rayon_virage_mm() * (1.0 - cos(_odo.cap))
			if cote * _odo.x >= x_cible - gain_redressement:
				phase = Phase.REDRESSEMENT

		Phase.REDRESSEMENT:
			angle = 90.0 - cote * braquage_manoeuvre
			if cote * _odo.cap <= deg_to_rad(3.0):
				phase = Phase.PARALLELE

		Phase.PARALLELE:
			# cap parallele a la ligne + petite correction pour rester a x_cible de la ligne
			var corr := clampf((cote * x_cible - _odo.x) * gain_lateral,
					-deg_to_rad(correction_laterale_max_deg), deg_to_rad(correction_laterale_max_deg))
			angle = _angle_vers_cap(corr)
			if _odo.y >= _y_fin_obstacle:
				phase = Phase.RETOUR_VIRAGE

		Phase.RETOUR_VIRAGE:
			angle = 90.0 - cote * braquage_manoeuvre
			if n > 0:
				_reperer_ligne(somme / n)
				phase = Phase.CROISEMENT
			elif cote * _odo.cap <= -deg_to_rad(cap_retour_deg):
				phase = Phase.RETOUR

		Phase.RETOUR:
			# garde le cap de retour (-cote * cap_retour_deg) jusqu'a revoir la ligne
			angle = _angle_vers_cap(-cote * deg_to_rad(cap_retour_deg))
			if n > 0:
				_reperer_ligne(somme / n)
				phase = Phase.CROISEMENT

		Phase.CROISEMENT:
			# tout droit : le virage d'alignement decale encore l'essieu de R * (1 - cos(cap)) vers la ligne
			angle = 90.0
			if cote * (_odo.x - _x_ligne) <= _rayon_virage_mm() * (1.0 - cos(_odo.cap)):
				phase = Phase.ALIGNEMENT

		Phase.ALIGNEMENT:
			# braquage constant : on finit dans l'axe, l'essieu arriere sur la ligne
			angle = 90.0 + cote * braquage_manoeuvre
			if cote * _odo.cap >= -deg_to_rad(3.0):
				termine = true
	return Vector2(angle, cible)

## Position x de la ligne (repere odometrie) a partir de l'ecart lu par les sondes (mm, > 0 = a droite).
## La barre de sondes est a distance_sondes_mm devant l'essieu, perpendiculaire au cap.
func _reperer_ligne(erreur_mm: float) -> void:
	_x_ligne = _odo.x + _robot.distance_sondes_mm * sin(_odo.cap) + erreur_mm * cos(_odo.cap)

## Angle servo pour rejoindre le cap voulu (rad), braquage limite a braquage_manoeuvre.
func _angle_vers_cap(cap_voulu: float) -> float:
	var servo := _robot.roues_vers_servo(rad_to_deg(cap_voulu - _odo.cap) * gain_cap)
	return 90.0 + clampf(servo, -braquage_manoeuvre, braquage_manoeuvre)

## Rayon de braquage (mm) de l'essieu arriere avec braquage_manoeuvre.
func _rayon_virage_mm() -> float:
	var delta := deg_to_rad(_robot.servo_vers_roues_deg(braquage_manoeuvre))
	return _robot.empattement_mm / tan(delta)
