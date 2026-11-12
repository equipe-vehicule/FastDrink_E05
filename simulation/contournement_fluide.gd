## Contournement « fluide » : changement de voie en S a courbure continue, longement de l'obstacle, puis retour en S
## sur la ligne. La trajectoire est fixee au depart (decalage ecart_mm) et suivie par poursuite d'un point (pure pursuit)
## sur l'odometrie. Peut demarrer en roulant (sans arret devant l'obstacle).
##
## Profil lateral : polynome de degre 5, q(t) = 10 t^3 - 15 t^4 + 6 t^5 (vitesse et acceleration laterales nulles aux
## extremites : le braquage evolue progressivement, sans saut). Courbure maximale ~ 5,77 * ecart / longueur^2.
## Repere : celui d'Odometrie (origine = essieu arriere au depart, y vers l'avant, x vers la droite).
## Sources : LaValle, Planning Algorithms (chemins et courbure) ; MathWorks, Pure Pursuit Controller.
## Hypothese : la ligne continue tout droit sous l'obstacle (obstacle pose sur une ligne droite).
## Choix par le Controleur : obstacle confirme entre distance_min_mm et distance_declenchement_mm, voiture en ligne
## droite et sur la ligne. Sinon, contournement par phases (Contournement), avec arret devant l'obstacle.
## Essais du 12 nov. (piste Contournement) : par phases 13,0 s ; fluide 9,5 s, bille en place.
class_name ContournementFluide
extends RefCounted

# ----- Declenchement (pendant le suivi de ligne)
var distance_declenchement_mm := 550.0  # distance lue par l'ultrason a laquelle la manoeuvre commence
var confirmations := 3              # pas de controle consecutifs avec un echo dans cette distance
## En dessous, le S devrait tourner plus serre que le rayon minimal (138 mm) : contournement par phases a la place.
## Longueur de sortie minimale ~ racine(5,77 * ecart * 180 mm) = 470 mm, soit ~520 mm lus par l'ultrason.
## Banc d'essai du 12 nov. : 22 obstacles sur 38 vus plus pres (obstacle peu apres un virage).
var distance_min_mm := 520.0
var points_direction_ligne := 20    # 20 x 20 mm = 400 mm de ligne pour mesurer son cap (MemoireParcours.direction_ajustee)

# ----- Trajectoire

## Decalage de l'essieu : 100 (demi-obstacle de 200 mm) + 40 (obstacle decale de la ligne, decalage_obstacle_max)
## + 70,5 (demi-voiture) + 40 (marge). Banc du 12 nov. : a 210 (sans le decalage), 2 collisions sur 40 pistes.
var ecart_mm := 250.0
var avant_voiture_mm := 229.0      # essieu arriere -> avant de la voiture (capteur a 193 mm + 36 mm)
var arriere_voiture_mm := 45.0     # depassement arriere (LISEZMOI Blender)
var profondeur_obstacle_mm := 60.0 # obstacle.blend : 200 x 60 x 200 mm
var marge_avant_mm := 20.0         # la sortie est terminee quand l'avant arrive a ca de la face de l'obstacle
var marge_arriere_mm := 40.0       # le retour commence quand l'arriere a depasse l'obstacle de ca
## Retour en quintique : rayon minimal ~ L^2 / (5,77 * ecart), soit ~140 mm a 450 (limite de la voiture : 138).
## Banc du 12 nov. (40 pistes) : 650 mm (rayon ~290) finissait trop pres d'un 2e obstacle rapproche (1 collision) ;
## 450 mm : 0 collision une fois le cap de depart corrige (MemoireParcours.direction_ajustee). On garde 450.
var longueur_retour_mm := 450.0
var longueur_sortie_min_mm := 300.0
var anticipation_mm := 250.0       # distance du point vise devant l'essieu (pure pursuit). 150 : braquage brusque au depart
var reprise_avant_fin_mm := 225.0  # le PID reprend des que les sondes voient la ligne dans la 2e moitie du retour.
								   # 0 (attendre la fin) : sur une piste aleatoire, la ligne tournait avant la fin du retour
								   # et etait perdue ; 225 teste sans chute de bille (pistes 4 et 7)
var vitesse := 65                  # commande pendant la manoeuvre (75 teste sans chute de bille ; 65 garde une marge)

var cote := 1
var termine := false
var e_reprise := 0.0

var _robot: ReglagesRobot
var _odo: Odometrie
var _y1 := 0.0                     # fin de la sortie
var _y2 := 0.0                     # debut du retour
var _y3 := 0.0                     # fin du retour

func _init(robot: ReglagesRobot, odo: Odometrie) -> void:
	_robot = robot
	_odo = odo

## d_capteur_mm : distance de l'obstacle lue par l'ultrason au depart.
## x0_mm, cap0 : position laterale de l'essieu et cap par rapport a la LIGNE (repere de la ligne, x = 0 sur la ligne).
## La trajectoire est ainsi tracee le long de la ligne, meme si la voiture n'est pas encore dans son axe.
func demarrer(d_capteur_mm: float, cote_voulu: int, x0_mm := 0.0, cap0 := 0.0) -> void:
	_odo.reinitialiser()
	_odo.x = x0_mm
	_odo.cap = cap0
	cote = cote_voulu
	termine = false
	var y_face := d_capteur_mm + _robot.longueur_avant_mm
	_y1 = maxf(longueur_sortie_min_mm, y_face - avant_voiture_mm - marge_avant_mm)
	_y2 = maxf(_y1, y_face + profondeur_obstacle_mm + arriere_voiture_mm + marge_arriere_mm)
	_y3 = _y2 + longueur_retour_mm

static func _q(t: float) -> float:
	t = clampf(t, 0.0, 1.0)
	return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)

## Position laterale voulue de l'essieu (mm) a l'abscisse y.
func x_chemin(y: float) -> float:
	if y <= 0.0:
		return 0.0
	if y < _y1:
		return cote * ecart_mm * _q(y / _y1)
	if y < _y2:
		return cote * ecart_mm
	if y < _y3:
		return cote * ecart_mm * (1.0 - _q((y - _y2) / (_y3 - _y2)))
	return 0.0

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
