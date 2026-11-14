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
## on REJOUE sa poursuite (#76, _poursuite_libre) : modele bicyclette, meme loi que calculer(), commande du servo entiere
## et bornee, servo a vitesse_servo_deg_s, un pas par tour de boucle (frequence_controle_hz), depuis la pose et l'angle
## du servo au depart. On verifie que la voiture (rectangle) ne touche pas l'obstacle (rectangle elargi de la tolerance
## de centrage et de marge_obstacle_mm). Avant le 14 nov., la courbe IDEALE etait verifiee : la poursuite, en retard,
## coupait la sortie et perdait ~31 mm de marge. Aucun rayon possible : le Controleur utilise le contournement par phases.
class_name ContournementFluide
extends RefCounted

## Reglages : les valeurs sont dans reglages_algorithmes.json (objet « fluide »), lues au demarrage ; ici : type et justification seulement.
# ----- Trajectoire
## Rayon minimal de la voiture : 138 mm (45 deg). En dessous de ~1,15 fois cette valeur, la poursuite decroche
## (banc du 12 nov. : retour de rayon ~140 mm, voiture perdue). Du plus doux au plus serre.
@export var rayons_candidats_mm: Array
@export var marge_laterale_mm: float # entre le flanc de la voiture et l'obstacle (tolerance de centrage en plus)
## Marge de la verification (une seule valeur, aussi passee a Contournement.choisir_cap). Depuis le 14 nov. (#76), le
## retard de la poursuite est simule : la marge ne couvre plus que les ecarts non simules (odometrie 1 a 2 mm, position
## de la face +1,6 mm en moyenne, acceleration au depart) et la securite voulue. HYPOTHESE : valeur de 20 mm inchangee,
## son ancienne justification (retard de suivi de 12 mm) ne s'applique plus ; a revoir avec les essais sur le robot.
@export var marge_obstacle_mm: float
@export var marge_arriere_mm: float # le retour commence quand l'arriere a depasse l'obstacle de ca
## Retour en quintique : rayon minimal ~ L^2 / (5,77 * ecart) ; a 450 mm et 170 mm d'ecart : ~210 mm.
@export var longueur_retour_mm: float
## Distance du point vise devant l'essieu (pure pursuit). Depuis l'arret (graine 39596) : a 250 mm, la voiture coupait
## la courbe avec jusqu'a 34 mm de retard et touchait l'obstacle ; a 150 mm, 12 mm de retard au plus, bille a 0,71.
## Historique (14 nov., #76) : avant le rejeu de la poursuite (_poursuite_libre), la marge la plus faible tombait dans
## la SORTIE, coin avant contre l'obstacle ; cap reel 27,5 deg pour 34,8 deg prevus, ~31 mm de marge perdue (D_recul 50,
## graine 110867 : 42,9 mm prevus, 10,6 mm reels). Essai non retenu par l'equipe, anticipation a 100 mm : marges min 20,3 mm (D_recul 50) et 28,1 mm (40 pistes) au lieu de 10,6 et 20,0, mais bille plus
## haute en ideal (pire_cas_avant_virage 0,64 -> 0,86) et oscillation en droite 3,6 -> 8,6 deg.
@export var anticipation_mm: float
@export var reprise_avant_fin_mm: float # le PID reprend des que les sondes voient la ligne dans la 2e moitie du retour.
								   # 0 (attendre la fin) : sur une piste aleatoire, la ligne tournait avant la fin du retour
								   # et etait perdue ; 225 teste sans chute de bille (pistes 4 et 7)
## Vitesse : la plus grande qui garde l'acceleration laterale v^2 / R sous acceleration_laterale_max, plafonnee.
## 0,3 m/s^2 : sous la deceleration de freinage utilisee en ligne droite (500 mm/s^2, soit 0,5 m/s^2), car la bille subit
## aussi les a-coups du braquage.
@export var acceleration_laterale_max: float # m/s^2
@export var vitesse_max_mm_s: float    # (= ancienne commande 65 a 4 mm/s par commande ; sans chute de bille au banc du 12 nov.)

var cote := 1
var termine := false
var fin_sans_ligne := false        # diagnostic : fin du retour sans avoir revu la ligne (cote devine)
var e_reprise := 0.0
var vitesse_mm_s := 260.0          # vitesse choisie pour la manoeuvre en cours (mm/s)
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
		var v := _vitesse_pour(r)
		if _poursuite_libre(v):
			rayon_choisi_mm = r
			vitesse_mm_s = v
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

## Vitesse de la manoeuvre (mm/s) pour le rayon de sortie rayon_mm (trajectoire deja preparee).
func _vitesse_pour(rayon_mm: float) -> float:
	var r_min_m := minf(rayon_mm, _rayon_retour_mm()) / 1000.0
	var v_voulue := sqrt(acceleration_laterale_max * r_min_m) * 1000.0   # mm/s
	# Vitesse atteignable par une commande ENTIERE (la commande du robot est un entier) : arrondie vers le bas
	var cmd_entiere := floorf(_robot.commande_pour_vitesse(v_voulue))
	return minf(vitesse_max_mm_s, _robot.vitesse_mm_s(cmd_entiere))

## Rejoue la poursuite de la trajectoire preparee (repere de la manoeuvre, cote reel ; l'obstacle est centre sur la
## ligne) et verifie a chaque pas qu'aucun coin de la voiture n'entre dans l'obstacle elargi, et qu'aucun coin de
## l'obstacle n'entre dans la voiture. Memes lois que calculer() et que le Controleur (_appliquer : commande du servo
## arrondie a l'entier, servo vers la commande a vitesse_servo_deg_s, odometrie bicyclette).
## Hypotheses : vitesse v des le depart (prudent : la vraie voiture accelere, donc parcourt moins de chemin pendant que
## le servo braque) ; poursuite rejouee sur une odometrie exacte (ecart reel 1 a 2 mm : couvert par marge_obstacle_mm).
## Fin de la verification quand l'arriere de la voiture a depasse l'obstacle elargi (plus de contact possible en
## avancant) ou a la fin du retour. Echec si la voiture se met en travers (cap de 90 deg ou plus : elle n'avance plus).
func _poursuite_libre(v_mm_s: float) -> bool:
	if v_mm_s <= 0.0:
		return false                                # la voiture n'avancerait pas : boucle sans fin
	var demi := _epreuve.largeur_a_contourner_mm() / 2.0 + _epreuve.tolerance_centrage_mm + marge_obstacle_mm
	var obs_min := Vector2(-demi, _y_face - marge_obstacle_mm)
	var obs_max := Vector2(demi, _y_face + _epreuve.profondeur_a_contourner_mm() + marge_obstacle_mm)
	var y_fin := minf(_y3, obs_max.y + _robot.avant_voiture_mm() + _robot.arriere_voiture_mm)
	var dt := 1.0 / _robot.frequence_controle_hz   # pas = periode de la boucle du robot (doit etre > 0)
	var p := Vector2(cote * _x0, 0.0)               # pose de depart, comme demarrer()
	var cap := _cap0
	var servo := _odo.servo_estime
	while p.y < y_fin:
		var angle := roundf(_angle_poursuite(p, cap))
		servo = angle if _robot.vitesse_servo_deg_s <= 0.0 else move_toward(servo, angle, _robot.vitesse_servo_deg_s * dt)
		var delta := deg_to_rad(_robot.servo_vers_roues_deg(servo - 90.0))
		cap += v_mm_s / _robot.empattement_mm * tan(delta) * dt
		p += Vector2(sin(cap), cos(cap)) * v_mm_s * dt
		if absf(cap) >= PI / 2.0:
			return false
		if GeometrieVoiture.touche(p, cap, _robot.avant_voiture_mm(), _robot.arriere_voiture_mm,
				_robot.largeur_voiture_mm / 2.0, obs_min, obs_max):
			return false
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
		return Vector2(90.0, vitesse_mm_s)
	if y >= _y3 + 150.0:
		# Ligne non revue : elle est du cote oppose a la position laterale de la voiture (x = 0 sur la ligne).
		# Avant : toujours -cote, faux si la voiture a deja croise la ligne (elle braquait alors a fond du mauvais cote).
		var signe := -signf(_odo.x) if absf(_odo.x) > 10.0 else float(-cote)
		e_reprise = signe * SuiviLigne.ERREUR_MAX_MM
		fin_sans_ligne = true
		termine = true
		return Vector2(90.0, vitesse_mm_s)
	return Vector2(_angle_poursuite(Vector2(_odo.x, _odo.y), _odo.cap), vitesse_mm_s)

## Loi de poursuite (pure pursuit) : angle du servo (deg, 90 = tout droit, borne a +-45) pour l'essieu en p au cap c.
## Utilisee par calculer() et rejouee par _poursuite_libre().
func _angle_poursuite(p: Vector2, c: float) -> float:
	# courbure = 2 * ecart lateral / distance^2 vers le point vise
	var yt := p.y + anticipation_mm
	var v := Vector2(x_chemin(yt), yt) - p
	var lat := v.x * cos(c) - v.y * sin(c)
	var av := v.x * sin(c) + v.y * cos(c)
	var kappa := 2.0 * lat / maxf(lat * lat + av * av, 1.0)
	var delta := rad_to_deg(atan(_robot.empattement_mm * kappa))
	return 90.0 + clampf(_robot.roues_vers_servo(delta), -45.0, 45.0)
