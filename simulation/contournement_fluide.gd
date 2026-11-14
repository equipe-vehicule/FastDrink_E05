## Contournement « fluide » depuis l'arret : changement de voie en S a courbure continue, longement de l'obstacle,
## puis retour en S sur la ligne. La trajectoire est fixee au depart et suivie par poursuite d'un point (pure pursuit)
## sur l'odometrie. Utilise apres l'arret et le recul devant l'obstacle (Controleur, etat RECUL_OBSTACLE).
##
## Profil lateral : polynome de degre 5, q(t) = 10 t^3 - 15 t^4 + 6 t^5 (vitesse et acceleration laterales nulles aux
## extremites : le braquage evolue progressivement, sans saut). Courbure maximale ~ 5,77 * ecart / longueur^2,
## donc pour un rayon minimal R : longueur = racine(5,77 * ecart * R).
## Repere : celui de la LIGNE (y le long de la ligne, x vers la droite, x = 0 sur la ligne, y = 0 a l'essieu arriere au
## depart). La voiture peut partir a cote de la ligne et de travers (arret en sortie de virage) : la sortie part de sa
## position laterale reelle (x0) au lieu de supposer x0 = 0 (graine 7920 : depart a 91 mm de la ligne, collision).
## Sources : LaValle, Planning Algorithms (chemins et courbure) ; MathWorks, Pure Pursuit Controller.
## Hypothese : la ligne continue tout droit sous l'obstacle (precision du professeur : obstacles sur des lignes droites).
##
## Choix (choisir) : pour chaque rayon de rayons_candidats_mm, du plus doux au plus serre, on calcule la trajectoire et
## on verifie que la voiture (rectangle) ne touche pas l'obstacle (rectangle elargi de la tolerance de centrage et de
## marge_obstacle_mm). Aucun rayon possible : le Controleur utilise le contournement par phases.
class_name ContournementFluide
extends RefCounted

# ----- Trajectoire
## Rayon minimal de la voiture : 138 mm (45 deg). En dessous de ~1,15 fois cette valeur, la poursuite decroche
## (banc du 12 nov. : retour de rayon ~140 mm, voiture perdue). Du plus doux au plus serre.
var rayons_candidats_mm := [300.0, 250.0, 220.0, 190.0, 160.0]
var marge_laterale_mm := 40.0      # entre le flanc de la voiture et l'obstacle (tolerance de centrage en plus)
var marge_obstacle_mm := 20.0      # marge de la verification geometrique : retard de suivi mesure (12 mm) + marge
var marge_arriere_mm := 40.0       # le retour commence quand l'arriere a depasse l'obstacle de ca
## Retour en quintique : rayon minimal ~ L^2 / (5,77 * ecart) ; a 450 mm et 170 mm d'ecart : ~210 mm.
var longueur_retour_mm := 450.0
## Distance du point vise devant l'essieu (pure pursuit). Depuis l'arret (graine 39596) : a 250 mm, la voiture coupait
## la courbe avec jusqu'a 34 mm de retard et touchait l'obstacle ; a 150 mm, 12 mm de retard au plus, bille a 0,71.
var anticipation_mm := 150.0
var reprise_avant_fin_mm := 225.0  # le PID reprend des que les sondes voient la ligne dans la 2e moitie du retour.
								   # 0 (attendre la fin) : sur une piste aleatoire, la ligne tournait avant la fin du retour
								   # et etait perdue ; 225 teste sans chute de bille (pistes 4 et 7)
## Vitesse : la plus grande qui garde l'acceleration laterale v^2 / R sous acceleration_laterale_max, plafonnee.
## 0,3 m/s^2 : sous la rampe d'acceleration utilisee en ligne droite (125 cmd/s, soit 0,5 m/s^2), car la bille subit
## aussi les a-coups du braquage.
var acceleration_laterale_max := 0.3   # m/s^2
var vitesse_max := 65                  # commande (65 sans chute de bille au banc du 12 nov.)

var cote := 1
var termine := false
var fin_sans_ligne := false        # diagnostic : fin du retour sans avoir revu la ligne (cote devine)
var e_reprise := 0.0
var vitesse := 65                  # commande choisie pour la manoeuvre en cours
var rayon_choisi_mm := 0.0         # journal / banc d'essai

var _robot: ReglagesRobot
var _odo: Odometrie
var _epreuve: ReglagesEpreuve
var _ecart := 0.0                  # decalage lateral de l'essieu pendant le longement (mm)
var _y1 := 0.0                     # fin de la sortie
var _y2 := 0.0                     # debut du retour
var _y3 := 0.0                     # fin du retour
var _y_face := 0.0                 # face de l'obstacle (repere de la manoeuvre)
var _x0 := 0.0                     # position laterale au depart, du cote du contournement (mm, > 0 = vers ce cote)
var _cap0 := 0.0                   # cap au depart par rapport a la ligne (rad, > 0 = vers la droite)

func _init(robot: ReglagesRobot, odo: Odometrie, epreuve: ReglagesEpreuve) -> void:
	_robot = robot
	_odo = odo
	_epreuve = epreuve

## Decalage lateral de l'essieu necessaire pour passer a cote de l'obstacle (mm).
func ecart_mm() -> float:
	return _epreuve.largeur_a_contourner_mm() / 2.0 + _epreuve.tolerance_centrage_mm + _robot.largeur_voiture_mm / 2.0 + marge_laterale_mm

## d_capteur_mm : distance de l'obstacle lue par l'ultrason, voiture arretee. cote_voulu : 1 = droite, -1 = gauche.
## x0_mm, cap0 : position laterale de l'essieu (> 0 = a droite) et cap (> 0 = vers la droite) par rapport a la ligne.
## Prepare la trajectoire la plus douce qui passe sans toucher l'obstacle. Retourne false si aucune ne passe.
func choisir(d_capteur_mm: float, cote_voulu: int, x0_mm := 0.0, cap0 := 0.0) -> bool:
	cote = cote_voulu
	_x0 = cote * x0_mm
	_cap0 = cap0
	for r in rayons_candidats_mm:
		_preparer(d_capteur_mm, r)
		if _trajectoire_libre():
			rayon_choisi_mm = r
			var r_min_m := minf(r, _rayon_retour_mm()) / 1000.0
			var v_cmd := sqrt(acceleration_laterale_max * r_min_m) * 1000.0 / _robot.mm_s_par_cmd
			vitesse = mini(vitesse_max, int(v_cmd))
			return true
	rayon_choisi_mm = 0.0
	return false

## A appeler juste apres un choisir() qui a retourne true (trajectoire preparee, depart dans la pose donnee).
func demarrer() -> void:
	_odo.reinitialiser()
	_odo.x = cote * _x0
	_odo.cap = _cap0
	termine = false
	fin_sans_ligne = false

func _preparer(d_capteur_mm: float, rayon_mm: float) -> void:
	_ecart = ecart_mm()
	# capteur et mesure dans l'axe de la voiture, qui peut etre de travers par rapport a la ligne
	_y_face = (d_capteur_mm + _robot.longueur_avant_mm) * cos(_cap0)
	_y1 = sqrt(5.77 * maxf(absf(_ecart - _x0), 1.0) * rayon_mm)
	_y2 = maxf(_y1, _y_face + _epreuve.profondeur_a_contourner_mm() + _robot.arriere_voiture_mm + marge_arriere_mm)
	_y3 = _y2 + longueur_retour_mm

func _rayon_retour_mm() -> float:
	return longueur_retour_mm * longueur_retour_mm / (5.77 * _ecart)

## Parcourt la trajectoire prevue (cote droit ; le cote gauche est symetrique) et verifie qu'aucun coin de la voiture
## n'entre dans l'obstacle elargi, et qu'aucun coin de l'obstacle n'entre dans la voiture.
func _trajectoire_libre() -> bool:
	var demi := _epreuve.largeur_a_contourner_mm() / 2.0 + _epreuve.tolerance_centrage_mm + marge_obstacle_mm
	var obs_min := Vector2(-demi, _y_face - marge_obstacle_mm)
	var obs_max := Vector2(demi, _y_face + _epreuve.profondeur_a_contourner_mm() + marge_obstacle_mm)
	var y := 0.0
	var pas := 5.0
	while y <= _y3:
		var x := _x_prevu(y)
		var cap := atan2(_x_prevu(y + 1.0) - x, 1.0)
		if GeometrieVoiture.touche(Vector2(x, y), cap, _robot.avant_voiture_mm(), _robot.arriere_voiture_mm,
				_robot.largeur_voiture_mm / 2.0, obs_min, obs_max):
			return false
		y += pas
	return true

## Trajectoire prevue du cote droit (cote = 1).
func _x_prevu(y: float) -> float:
	if y <= 0.0:
		return _x0
	if y < _y1:
		return _x0 + (_ecart - _x0) * _q(y / _y1)
	if y < _y2:
		return _ecart
	if y < _y3:
		return _ecart * (1.0 - _q((y - _y2) / (_y3 - _y2)))
	return 0.0

static func _q(t: float) -> float:
	t = clampf(t, 0.0, 1.0)
	return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)

## Position laterale voulue de l'essieu (mm) a l'abscisse y.
func x_chemin(y: float) -> float:
	return cote * _x_prevu(y)

func nom_phase() -> String:
	if _odo.y < _y1: return "SORTIE"
	if _odo.y < _y2: return "LONGER"
	return "RETOUR"

## Retourne Vector2(angle servo, vitesse cible). Regarder `termine` ensuite.
func calculer(somme: float, n: int) -> Vector2:
	var y := _odo.y
	if n > 0 and y >= _y3 - reprise_avant_fin_mm:
		e_reprise = somme / n
		termine = true
		return Vector2(90.0, float(vitesse))
	if y >= _y3 + 150.0:
		# Ligne non revue : elle est du cote oppose a la position laterale de la voiture (x = 0 sur la ligne).
		# Avant : toujours -cote, faux si la voiture a deja croise la ligne (elle braquait alors a fond du mauvais cote).
		var signe := -signf(_odo.x) if absf(_odo.x) > 10.0 else float(-cote)
		e_reprise = signe * SuiviLigne.ERREUR_MAX_MM
		fin_sans_ligne = true
		termine = true
		return Vector2(90.0, float(vitesse))
	# Pure pursuit : courbure = 2 * ecart lateral / distance^2 vers le point vise
	var yt := y + anticipation_mm
	var v := Vector2(x_chemin(yt), yt) - Vector2(_odo.x, _odo.y)
	var c := _odo.cap
	var lat := v.x * cos(c) - v.y * sin(c)
	var av := v.x * sin(c) + v.y * cos(c)
	var kappa := 2.0 * lat / maxf(lat * lat + av * av, 1.0)
	var delta := rad_to_deg(atan(_robot.empattement_mm * kappa))
	return Vector2(90.0 + clampf(_robot.roues_vers_servo(delta), -45.0, 45.0), float(vitesse))
