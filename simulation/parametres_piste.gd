## Parametres de la piste aleatoire.
## Modifiables dans l'inspecteur du noeud Simulation, ou enregistres dans des fichiers .tres
## (clic droit dans le FileSystem > Nouvelle ressource > ParametresPiste) pour garder plusieurs reglages.
##
## Par defaut, tout ce que le cahier des charges n'interdit pas peut arriver (le but est de trouver
## les cas ou la voiture echoue avant de passer au vrai robot). Pour un mode "facile" : rayon_virage_max = 0,12,
## autoriser_cases_voisines = false, obstacles_en_virage = false, ecart_min_obstacles = 5,
## taille_obstacle_max = taille_obstacle, decalage_obstacle_max = 0.
class_name ParametresPiste
extends Resource

@export var graine := 0 ## 0 = nouvelle piste a chaque chargement ; sinon toujours la meme piste

@export_group("Grille")
## Reduite automatiquement si les cases sont grandes, pour tenir sur le sol de 8 m.
@export_range(4, 30) var colonnes := 14
@export_range(4, 30) var lignes := 14

@export_group("Ruban")
## Cahier des charges : courbes de 120 mm et plus. Rayon minimal des virages ; une case mesure 2 x rayon.
@export_range(0.12, 0.5, 0.005, "suffix:m") var rayon_virage := 0.12
## Rayon maximal : chaque piste tire son rayon au hasard entre rayon_virage et celui-ci (= rayon_virage : toujours le meme).
@export_range(0.12, 0.5, 0.005, "suffix:m") var rayon_virage_max := 0.30
## Cahier des charges : ruban electrique noir de 18 mm.
@export_range(0.005, 0.05, 0.001, "suffix:m") var largeur_ruban := 0.018
@export_range(3, 300) var longueur_min := 30 ## en cases
@export_range(3, 300) var longueur_max := 50 ## en cases
@export_range(0.0, 1.0, 0.05) var proba_tout_droit := 0.5 ## 1 = le moins de virages possible
## Apres un virage, on continue tout droit au moins ce nombre de cases (si possible). 1 = aucune contrainte.
@export_range(1, 12) var longueur_droite_min := 1
## Cases voisines autorisees : demi-tours serres (deux virages colles) et lignes paralleles a une case d'ecart.
@export var autoriser_cases_voisines := true

@export_group("Obstacles")
@export_range(0.0, 1.0, 0.01) var proba_obstacle := 0.15 ## probabilite qu'une case du ruban recoive un obstacle
@export_range(0, 100) var nb_obstacles := 100 ## maximum par piste
@export var taille_obstacle := Vector3(0.10, 0.15, 0.10) ## taille minimale : largeur, hauteur, profondeur (m)
@export var taille_obstacle_max := Vector3(0.20, 0.20, 0.20) ## taille maximale, tiree au hasard pour chaque obstacle
@export_range(0.0, 0.1, 0.005, "suffix:m") var decalage_obstacle_max := 0.04 ## decalage lateral maximal par rapport a la ligne
## Ecart minimal entre deux obstacles, en cases. 1 = obstacles possibles sur des cases voisines.
@export_range(1, 20) var ecart_min_obstacles := 1
@export_range(2, 20) var cases_libres_depart := 2 ## pas d'obstacle sous la voiture au depart
## true = obstacles n'importe ou, y compris en virage. false = seulement au milieu d'une ligne droite (reglages ci-dessous).
@export var obstacles_en_virage := true
## Si obstacles_en_virage = false : cases entierement droites exigees avant et apres l'obstacle.
## 2 + 2 : tous les contournements reussis en simulation (le contournement a besoin de ~400 mm avant, ~600 mm apres).
@export_range(0, 6) var cases_droites_avant := 2
@export_range(0, 6) var cases_droites_apres := 2
