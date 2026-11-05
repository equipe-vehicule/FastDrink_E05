## Parametres de la piste aleatoire.
## Modifiables dans l'inspecteur du noeud Simulation, ou enregistres dans des fichiers .tres
## (clic droit dans le FileSystem > Nouvelle ressource > ParametresPiste) pour garder plusieurs reglages.
class_name ParametresPiste
extends Resource

@export var graine := 0 ## 0 = nouvelle piste a chaque chargement ; sinon toujours la meme piste

@export_group("Grille")
@export_range(4, 30) var colonnes := 12
@export_range(4, 30) var lignes := 12

@export_group("Ruban")
## Cahier des charges : courbes de 120 mm et plus. Une case de la grille mesure 2 x rayon.
@export_range(0.12, 0.5, 0.005, "suffix:m") var rayon_virage := 0.12
## Cahier des charges : ruban electrique noir de 18 mm.
@export_range(0.005, 0.05, 0.001, "suffix:m") var largeur_ruban := 0.018
@export_range(3, 300) var longueur_min := 15 ## en cases
@export_range(3, 300) var longueur_max := 30 ## en cases
@export_range(0.0, 1.0, 0.05) var proba_tout_droit := 0.5 ## 1 = le moins de virages possible

@export_group("Obstacles")
@export_range(0, 20) var nb_obstacles := 2
@export var taille_obstacle := Vector3(0.10, 0.15, 0.10) ## largeur, hauteur, profondeur (m)
@export_range(1, 20) var ecart_min_obstacles := 4 ## en cases
@export_range(1, 20) var cases_libres_depart := 3 ## pas d'obstacle sur les premieres cases

func taille_case() -> float:
	return 2.0 * rayon_virage
