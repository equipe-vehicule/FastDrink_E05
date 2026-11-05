## Suivi de ligne PID a partir des 5 sondes (0/1, de gauche a droite), avec une detection robuste de la ligne.
##
## DETECTION : avant le PID, on decide si la lecture est fiable.
##   VUE           : des sondes voisines voient la ligne ; erreur = milieu des sondes allumees.
##   ENTRE_SONDES  : plus aucune sonde, mais la ligne n'etait pas au bord. Le ruban (18 mm) a pu passer entre
##                   deux sondes (espacees de 19 mm), ou une sonde a "clignote". On garde la commande precedente
##                   pendant delai_entre_sondes_s au lieu de braquer a fond.
##   PERDUE        : plus aucune sonde depuis plus longtemps, ou la ligne etait au bord : braquage maximal
##                   du dernier cote vu (comme avant).
##   Motif incoherent (sondes non voisines, ou 4 sondes et plus : croisement, marque, reflet) : lecture ignoree,
##   on garde la commande precedente.
## Sans cette detection, une seule lecture "0 0 0 0 0" erronee declenchait un braquage maximal.
##
## COMMANDE : PID inchange (erreur en mm -> angle servo), vitesse reduite quand on est decentre.
class_name SuiviLigne
extends RefCounted

enum Ligne { VUE, ENTRE_SONDES, PERDUE }

const POSITIONS_SONDES_MM := [-38.0, -19.0, 0.0, 19.0, 38.0]
const ERREUR_MAX_MM := 38.0       # ecart maximal mesurable avec la ligne visible
const INTEGRALE_MAX := 60.0       # anti-windup (mm*s)
const FILTRE_DERIVEE := 0.3       # 0..1 : plus petit = derivee plus lissee

# ----- Gains PID (a regler dans cet ordre : kp, puis kd, puis ki)
var kp := 1.80                    # degres par mm d'ecart
var ki := 0.5                     # degres par (mm*s)
var kd := 0.08                    # degres par (mm/s)

# ----- Vitesses (commande 0..100)
var vitesse_max := 40             # ligne droite, bien centre
var vitesse_virage := 15          # grand ecart ou ligne perdue

# ----- Detection
var delai_entre_sondes_s := 0.15  # duree pendant laquelle une disparition de la ligne est toleree

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

func nom_ligne() -> String:
	return Ligne.keys()[ligne]

## Retourne Vector2(angle servo, vitesse cible).
func calculer(sondes: Array, dt: float) -> Vector2:
	var e := _position(sondes)
	if is_inf(e):
		return _derniere_sortie                # motif incoherent : lecture ignoree
	if is_nan(e):
		_t_sans_ligne += dt
		if ligne != Ligne.PERDUE and absf(_erreur) < ERREUR_MAX_MM and _t_sans_ligne <= delai_entre_sondes_s:
			ligne = Ligne.ENTRE_SONDES
			return _derniere_sortie
		ligne = Ligne.PERDUE
	else:
		_t_sans_ligne = 0.0
		_erreur = e
		ligne = Ligne.VUE
	_derniere_sortie = _pid(e, dt)
	return _derniere_sortie

## Position de la ligne (mm, > 0 = a droite). NAN = aucune sonde ; INF = motif incoherent.
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
	if n >= 4 or derniere - premiere + 1 != n:
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
