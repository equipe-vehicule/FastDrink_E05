## Demi-tour quand la voiture roule a l'envers sur la ligne : elle pivote en plusieurs manoeuvres (avant braquage
## maximal, arriere braquage oppose : elle tourne toujours du meme cote) jusqu'a environ 180 deg, puis le suivi reprend
## des que la ligne est vue. Choix du moment et de l'autorisation : Controleur et MemoireParcours.
class_name DemiTour
extends RefCounted

## Reglages : les valeurs sont dans reglages_algorithmes.json (objet « demi_tour »), lues au demarrage ; ici : type et justification seulement.
@export var duree_avant_s: float     # duree de chaque marche avant (braquage maximal)
@export var duree_arriere_s: float   # duree de chaque marche arriere (braquage oppose)
@export var vitesse_mm_s: float # (= ancienne commande 18 a 4 mm/s par commande)
@export var duree_max_s: float
@export var angle_ligne_deg: float   # rotation a partir de laquelle une sonde allumee termine le demi-tour (HYPOTHESE : ancienne valeur en dur)
@export var angle_max_deg: float     # rotation maximale : arret quoi qu'il arrive (HYPOTHESE : ancienne valeur en dur)

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
	var periode := duree_avant_s + duree_arriere_s
	var en_avant := fmod(t_etat, periode) < duree_avant_s
	# Ligne revue : on finit pendant une marche avant seulement (revue 06) ; fini en marche arriere, le suivi aurait
	# inverse la marche tout de suite. Rotation maximale ou duree maximale : arret immediat.
	if (tourne >= deg_to_rad(angle_ligne_deg) and n > 0 and en_avant) or tourne >= deg_to_rad(angle_max_deg) or t_etat > duree_max_s:
		termine = true
		return Vector2(90.0, 0.0)
	if en_avant:
		return Vector2(90.0 + 45.0 * _cote, vitesse_mm_s)
	return Vector2(90.0 - 45.0 * _cote, -vitesse_mm_s)
