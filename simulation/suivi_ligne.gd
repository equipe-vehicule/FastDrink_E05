## Suivi de ligne PID a partir des 5 sondes (0/1, de gauche a droite), avec une detection robuste de la ligne.
##
## DETECTION : avant le PID, on decide si la lecture est fiable.
##   VUE           : des sondes voisines voient la ligne ; erreur = milieu des sondes allumees.
##   ENTRE_SONDES  : plus aucune sonde, mais la ligne n'etait pas au bord. Le ruban (18 mm) a pu passer entre
##                   deux sondes (espacees de 19 mm), ou une sonde a "clignote". On garde la commande precedente
##                   pendant delai_entre_sondes_s au lieu de braquer a fond.
##   PERDUE        : plus aucune sonde depuis plus longtemps, ou la ligne etait au bord : braquage maximal
##                   du dernier cote vu (comme avant).
##   4 ou 5 sondes voisines : la voiture coupe la ligne de face (apres un contournement, un demi-tour) : ligne VUE,
##   au milieu des sondes allumees. (Avant, ce motif etait ignore : la voiture se croyait perdue et continuait
##   de chercher alors qu'elle etait sur la ligne - journal Godot du 6 novembre.)
##   ARRIVEE : 5 sondes allumees pendant pas_arrivee mesures alors que la voiture suivait la ligne bien centree depuis
##   aligne_avant_arrivee_s : c'est la barre d'arrivee posee en travers de la piste (`arrivee` passe a true).
##   Motif incoherent (sondes non voisines : reflet) : lecture ignoree, on garde la commande precedente.
## Sans cette detection, une seule lecture "0 0 0 0 0" erronee declenchait un braquage maximal.
##
## COMMANDE : PID (erreur en mm -> angle servo), vitesse reduite quand on est decentre.
## Essais du 12 nov. (parcours fourni), abandonnes : loi de braquage par table (roues plus calmes mais 2 rattrapages par
## virage serre au lieu de 1) et ralentissement en entree de virage (aucun rattrapage evite, tour plus lent).
## Detail dans SUIVI_SIMULATION.md.
class_name SuiviLigne
extends RefCounted

enum Ligne { VUE, ENTRE_SONDES, PERDUE }

const POSITIONS_SONDES_MM := [-38.0, -19.0, 0.0, 19.0, 38.0]
const ERREUR_MAX_MM := 38.0       # ecart maximal mesurable avec la ligne visible
const INTEGRALE_MAX := 60.0       # anti-windup (mm*s)
const FILTRE_DERIVEE := 0.3       # 0..1 : plus petit = derivee plus lissee

# ----- Gains PID (a regler dans cet ordre : kp, puis kd, puis ki)
var kp := 1.80                    # degres par mm d'ecart
var ki := 0.5                     # degres par (mm*s). Essai a 0 (12 nov.) : plus de pertes de ligne dans les virages serres
var kd := 0.08                    # degres par (mm/s)

# ----- Vitesses (commande 0..100)
var vitesse_max := 55             # ligne droite, bien centre. Essais du 12 nov. (parcours fourni) : 40 -> 55,9 s,
								  # 55 -> 44,4 s sans chute de bille, 70 -> 37,3 s mais la bille tombe
var vitesse_virage := 15          # grand ecart ou ligne perdue

# ----- Detection
var delai_entre_sondes_s := 0.15  # duree pendant laquelle une disparition de la ligne est toleree
## Ligne perdue plus longtemps (fin du parcours) : arret (0 = ne jamais s'arreter).
## Pas moins de 8 s : en virage serre, les sondes (217 mm devant l'essieu) sortent de la ligne et la voiture
## la retrouve en tournant, ce qui peut prendre plusieurs secondes.
var abandon_s := 40.0
# ----- Ligne d'arrivee (barre noire en travers de la piste)
var aligne_avant_arrivee_s := 0.5 # la voiture doit suivre la ligne bien centree depuis au moins ca
var pas_arrivee := 3              # mesures consecutives avec les 5 sondes allumees
var arrivee := false

## Dernier ecart non nul (mm) : sert a savoir de quel cote chercher la ligne quand on la perd.
## Mis a jour aussi par le recul et par la fin du contournement.
var derniere_erreur := 0.0
var ligne := Ligne.PERDUE

var _erreur := 0.0                # derniere position valide (mm)
var _t_sans_ligne := 0.0
var _derniere_sortie := Vector2(90.0, 15.0)
var _erreur_prec := 0.0
var _integrale := 0.0
var _derivee := 0.0
var _ligne_visible_avant := false
var _t_aligne := 0.0              # temps depuis lequel la ligne est vue bien centree
var _n_cinq := 0                  # mesures consecutives avec les 5 sondes allumees

## Retourne Vector2(somme des positions des sondes allumees, nombre de sondes allumees).
static func lire(sondes: Array) -> Vector2:
	var somme := 0.0
	var n := 0
	for i in 5:
		if sondes[i] == 1:
			somme += POSITIONS_SONDES_MM[i]
			n += 1
	return Vector2(somme, n)

## Annule les termes I et D (arret devant un obstacle : pas d'accumulation).
func geler() -> void:
	_integrale = 0.0
	_derivee = 0.0

## Retour en suivi de ligne : tant qu'aucune sonde ne voit la ligne, on la cherche du cote de derniere_erreur.
func reinitialiser() -> void:
	geler()
	_ligne_visible_avant = false
	ligne = Ligne.PERDUE
	_t_sans_ligne = 0.0               # sinon RETROUVER repartait juste apres un contournement (journal du 6 novembre)
	_t_aligne = 0.0
	_n_cinq = 0

## Derniere position de la ligne vue avant de la perdre (mm, > 0 = a droite). Proche de 0 : la ligne a disparu
## sous les sondes centrales, donc devant la voiture (fin de la ligne), et non sur le cote (virage).
func position_avant_perte() -> float:
	return _erreur

## Temps (s) depuis lequel aucune sonde ne voit la ligne.
func temps_sans_ligne() -> float:
	return _t_sans_ligne

func nom_ligne() -> String:
	return Ligne.keys()[ligne]

## Retourne Vector2(angle servo, vitesse cible).
func calculer(sondes: Array, dt: float) -> Vector2:
	var e := _position(sondes)
	if is_inf(e):
		return _derniere_sortie                # motif incoherent : lecture ignoree
	# Barre d'arrivee : 5 sondes alors que la voiture suivait la ligne bien centree
	if sondes.count(1) == 5 and _t_aligne >= aligne_avant_arrivee_s:
		_n_cinq += 1
		if _n_cinq >= pas_arrivee:
			arrivee = true
		return _derniere_sortie                # on garde le cap en attendant la confirmation
	_n_cinq = 0
	if is_nan(e):
		_t_sans_ligne += dt
		if ligne != Ligne.PERDUE and absf(_erreur) < ERREUR_MAX_MM and _t_sans_ligne <= delai_entre_sondes_s:
			ligne = Ligne.ENTRE_SONDES
			return _derniere_sortie
		ligne = Ligne.PERDUE
		if abandon_s > 0.0 and _t_sans_ligne > abandon_s:
			_derniere_sortie = Vector2(90.0, 0.0)   # on ne tourne pas indefiniment en rond a l'aveugle
			return _derniere_sortie
	else:
		_t_sans_ligne = 0.0
		_erreur = e
		ligne = Ligne.VUE
	_derniere_sortie = _pid(e, dt)
	_t_aligne = _t_aligne + dt if ligne == Ligne.VUE and absf(_erreur) <= 19.0 else 0.0
	return _derniere_sortie

## Position de la ligne (mm, > 0 = a droite). NAN = aucune sonde ; INF = motif incoherent (sondes non voisines).
## 4 ou 5 sondes voisines (ligne coupee de face) : position = milieu des sondes allumees.
func _position(sondes: Array) -> float:
	var premiere := -1
	var derniere := -1
	var n := 0
	for i in 5:
		if sondes[i] == 1:
			if premiere < 0: premiere = i
			derniere = i
			n += 1
	if n == 0:
		return NAN
	if derniere - premiere + 1 != n:
		return INF
	return (POSITIONS_SONDES_MM[premiere] + POSITIONS_SONDES_MM[derniere]) / 2.0

## PID. erreur = NAN quand la ligne est perdue. Retourne Vector2(angle, vitesse cible).
func _pid(erreur: float, dt: float) -> Vector2:
	var angle := 90.0
	var cible := float(vitesse_virage)
	if not is_nan(erreur):
		if erreur != 0.0:
			derniere_erreur = erreur

		# Terme I : remis a zero quand la ligne change de cote (fin de courbe)
		if _ligne_visible_avant and erreur * _erreur_prec < 0.0:
			_integrale = 0.0
		_integrale = clampf(_integrale + erreur * dt, -INTEGRALE_MAX, INTEGRALE_MAX)

		# Terme D (filtre, car les sondes 0/1 donnent des sauts d'erreur)
		if _ligne_visible_avant:
			var brute := (erreur - _erreur_prec) / dt
			_derivee = lerpf(_derivee, brute, FILTRE_DERIVEE)
		else:
			_derivee = 0.0
		_erreur_prec = erreur
		_ligne_visible_avant = true

		angle = 90.0 + kp * erreur + ki * _integrale + kd * _derivee

		# Vitesse adaptative : plus on est decentre, plus on ralentit
		var ratio := clampf(absf(erreur) / ERREUR_MAX_MM, 0.0, 1.0)
		cible = lerpf(float(vitesse_max), float(vitesse_virage), ratio)
	else:
		# Ligne perdue : braquage maximal du dernier cote vu, vitesse reduite
		geler()
		_ligne_visible_avant = false
		if derniere_erreur > 0.0:
			angle = ReglagesRobot.ANGLE_MAX
		elif derniere_erreur < 0.0:
			angle = ReglagesRobot.ANGLE_MIN
	return Vector2(angle, cible)
