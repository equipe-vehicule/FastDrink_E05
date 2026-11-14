## Demi-tour quand la voiture roule a l'envers sur la ligne : elle pivote en plusieurs manoeuvres (avant braquage
## maximal, arriere braquage oppose : elle tourne toujours du meme cote) jusqu'a environ 180 deg, puis le suivi reprend
## des que la ligne est vue. Choix du moment et de l'autorisation : Controleur et MemoireParcours.
class_name DemiTour
extends RefCounted

var duree_avant_s := 1.0             # duree de chaque marche avant (braquage maximal)
var duree_arriere_s := 1.0           # duree de chaque marche arriere (braquage oppose)
var vitesse_mm_s := 72.0    # (= ancienne commande 18 a 4 mm/s par commande)
var duree_max_s := 20.0

var termine := false

var _cap_debut := 0.0
var _cote := 1                       # 1 = la voiture pivote vers la droite

## cap : cap global au depart (rad) ; cote : cote ou la ligne a ete vue en dernier.
func demarrer(cap: float, cote: int) -> void:
	_cap_debut = cap
	_cote = cote
	termine = false

## Ecart a donner au suivi quand le demi-tour est fini (la ligne est cherchee de ce cote).
func ecart_reprise_mm() -> float:
	return SuiviLigne.ERREUR_MAX_MM * _cote

## cap : cap global actuel (rad). Retourne Vector2(angle servo, vitesse cible). Regarder `termine` ensuite.
func calculer(n: int, cap: float, t_etat: float) -> Vector2:
	var tourne := absf(cap - _cap_debut)
	if (tourne >= deg_to_rad(150.0) and n > 0) or tourne >= deg_to_rad(210.0) or t_etat > duree_max_s:
		termine = true
		return Vector2(90.0, 0.0)
	var periode := duree_avant_s + duree_arriere_s
	if fmod(t_etat, periode) < duree_avant_s:
		return Vector2(90.0 + 45.0 * _cote, vitesse_mm_s)
	return Vector2(90.0 - 45.0 * _cote, -vitesse_mm_s)
