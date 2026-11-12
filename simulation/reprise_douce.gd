## Reprise douce du suivi apres une manoeuvre (contournement, rattrapage, demi-tour, recherche de ligne).
## Banc d'essai du 12 nov. : au passage manoeuvre -> suivi, le braquage passait d'un coup de 130 a 45 deg et la voiture
## reaccelerait avant d'etre centree ; la bille, deja en mouvement, tombait (2 pistes sur 40, dont la piste 7).
## Pendant duree_s : vitesse et vitesse de rotation du braquage limitees.
class_name RepriseDouce
extends RefCounted

var duree_s := 1.0
var vitesse_max := 25
var braquage_max_deg_s := 120.0

var _t := 0.0
var _angle_prec := 90.0              # dernier angle commande (deg de servo)

func demarrer() -> void:
	_t = duree_s

## A appeler une fois par pas. Vrai (et decompte le temps) pendant la reprise, si la voiture est en suivi.
func en_cours(en_suivi: bool, dt: float) -> bool:
	if _t > 0.0 and en_suivi:
		_t -= dt
		return true
	return false

func limiter_vitesse(cible: float) -> float:
	return float(vitesse_max) if cible > vitesse_max else cible

func limiter_angle(angle: float, dt: float) -> float:
	return move_toward(_angle_prec, angle, braquage_max_deg_s * dt)

## A appeler a chaque pas avec l'angle finalement commande.
func memoriser_angle(angle: float) -> void:
	_angle_prec = angle
