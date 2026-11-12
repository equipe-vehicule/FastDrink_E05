## Rattrapage d'un virage serre en marche arriere (methode de line_follower.py, code de depart SunFounder).
## Le rayon de braquage (138 mm a 45 deg) ne permet pas aux sondes, 217 mm devant l'essieu, de rester sur une courbe de
## 120 a 170 mm : un point de la barre decrit au moins racine(138^2 + 217^2) = 257 mm. Quand la ligne est perdue en
## virage, la voiture s'arrete, braque du cote oppose, recule jusqu'a ce que la sonde centrale revoie la ligne, s'arrete
## de nouveau, puis reprend le suivi : chaque marche arriere la fait tourner vers la courbe (manoeuvre en plusieurs fois).
##
## Ce module surveille aussi la ligne pendant le suivi pour distinguer un virage rate d'une fin de ligne : ligne perdue
## alors qu'elle etait centree et suivie sans interruption depuis un moment = fin de ligne -> arret (pas de rattrapage).
class_name Rattrapage
extends RefCounted

enum Reaction { AUCUNE, RATTRAPER, ARRETER }

var actif := true
var apres_s := 0.6                   # ligne perdue depuis ca (apres l'avoir suivie) : on recule. Plus court, la voiture
									 # reculait dans les virages de 90 deg qu'elle retrouve seule en 0,35 s
var pause_s := 0.3                   # arret avant et apres la marche arriere (bille : pas d'aller-retour brusque)
var vitesse := 15                    # commande de recul (valeur absolue)
var distance_max_mm := 250.0         # recul maximal par tentative
var tentatives_max := 3              # tentatives sans retrouver la ligne durablement
var ligne_retrouvee_s := 1.0         # ligne suivie au moins ca : le compteur de tentatives repart a zero
var arret_si_introuvable := true     # apres tentatives_max (ou fin de ligne) : arret de securite (BLOQUE)
var fin_ligne_ecart_mm := 10.0       # ligne perdue alors qu'elle etait a moins de ca du centre...
var fin_ligne_vue_s := 0.5           # ... et suivie sans interruption au moins ca : fin de ligne

var termine := false                 # ligne revue et voiture arretee : le suivi peut reprendre

var _odo: Odometrie
var _suivi: SuiviLigne
var _tentatives := 0
var _cote := 1                       # cote ou la ligne a ete vue en dernier (1 = droite)
var _recule := false
var _fini := false                   # ligne revue : arret avant de repartir en avant
var _t_fin := 0.0
# Surveillance de la ligne pendant le suivi
var _ligne_suivie := false           # la ligne a ete vue depuis le dernier passage en suivi
var _t_ligne_vue := 0.0
var _duree_derniere_vue := 0.0       # duree de la derniere periode ou la ligne a ete vue sans interruption

func _init(odo: Odometrie, suivi: SuiviLigne) -> void:
	_odo = odo
	_suivi = suivi


# ----- Pendant le suivi

## A appeler a chaque reprise du suivi de ligne.
func nouveau_suivi() -> void:
	_ligne_suivie = false
	_t_ligne_vue = 0.0
	_duree_derniere_vue = 0.0

## A appeler a chaque pas du suivi.
func observer(dt: float) -> void:
	if _suivi.ligne == SuiviLigne.Ligne.VUE:
		_ligne_suivie = true
		_t_ligne_vue += dt
		_duree_derniere_vue = _t_ligne_vue
		if _t_ligne_vue >= ligne_retrouvee_s:
			_tentatives = 0
	else:
		_t_ligne_vue = 0.0

## Vrai si la ligne, suivie depuis la reprise du suivi, est maintenant perdue.
func ligne_perdue() -> bool:
	return _ligne_suivie and _suivi.ligne == SuiviLigne.Ligne.PERDUE

## Que faire de la ligne perdue : rien (pas encore), rattraper en marche arriere, ou s'arreter (fin de ligne).
func reaction() -> Reaction:
	if not (actif and ligne_perdue() and _suivi.temps_sans_ligne() >= apres_s):
		return Reaction.AUCUNE
	var fin_de_ligne := absf(_suivi.position_avant_perte()) <= fin_ligne_ecart_mm and _duree_derniere_vue >= fin_ligne_vue_s
	if not fin_de_ligne and _tentatives < tentatives_max:
		return Reaction.RATTRAPER
	if arret_si_introuvable:
		return Reaction.ARRETER
	return Reaction.AUCUNE


# ----- Manoeuvre

func demarrer(cote_ligne: int) -> void:
	_tentatives += 1
	_cote = cote_ligne
	_recule = false
	_fini = false
	termine = false

## cmd : commande de vitesse actuelle ; t_etat : temps depuis le debut de la manoeuvre.
## Retourne Vector2(angle servo, vitesse cible). Regarder `termine` ensuite.
func calculer(sondes: Array, somme: float, n: int, cmd: float, t_etat: float, dt: float) -> Vector2:
	if not _recule:
		# 1) arret complet, roues braquees du cote oppose, pendant au moins pause_s
		if absf(cmd) < 0.5 and t_etat >= pause_s:
			_recule = true
			_odo.reinitialiser()             # odo.parcours = distance reculee
		return Vector2(90.0 - 45.0 * _cote, 0.0)
	# 3) ligne revue : arret complet pendant pause_s, puis reprise du suivi en marche avant
	if _fini:
		_t_fin += 0.0 if absf(cmd) >= 0.5 else dt
		if _t_fin >= pause_s:
			termine = true
		return Vector2(90.0, 0.0)
	# 2) marche arriere jusqu'a ce que la sonde centrale revoie la ligne (lf.wait_tile_center() du code de depart)
	if n > 0:
		_suivi.derniere_erreur = somme / n if somme != 0.0 else _suivi.derniere_erreur
	if sondes[2] == 1 or _odo.parcours >= distance_max_mm:
		_fini = true
		_t_fin = 0.0
		return Vector2(90.0, 0.0)
	return Vector2(90.0 - 45.0 * _cote, -float(vitesse))
