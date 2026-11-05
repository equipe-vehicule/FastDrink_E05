## Suivi de ligne PID a partir des 5 sondes (0/1, de gauche a droite).
## Erreur = position moyenne des sondes qui voient la ligne (mm, > 0 = ligne a droite).
class_name SuiviLigne
extends RefCounted

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

## Dernier ecart connu (mm) : sert a savoir de quel cote chercher la ligne quand on la perd.
## Mis a jour aussi par le recul et par la fin du contournement.
var derniere_erreur := 0.0

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

## Repart de zero (retour en suivi de ligne).
func reinitialiser_pid() -> void:
	geler()
	_ligne_visible_avant = false

## Retourne Vector2(angle servo, vitesse cible).
func calculer(somme: float, n: int, dt: float) -> Vector2:
	var angle := 90.0
	var cible := float(vitesse_virage)
	if n > 0:
		var erreur: float = somme / n          # mm, > 0 = ligne a droite
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
