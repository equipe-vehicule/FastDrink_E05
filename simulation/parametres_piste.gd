## Parametres de la piste aleatoire.
## Modifiables dans l'inspecteur du noeud Simulation, ou enregistres dans des fichiers .tres
## (clic droit dans le FileSystem > Nouvelle ressource > ParametresPiste) pour garder plusieurs reglages.
##
## Ruban : tout ce que le cahier des charges n'interdit pas peut arriver (rayon >= 120 mm, virages de 90 deg, enchaines
## en S ou en demi-tour). Obstacles : conformes aux precisions du professeur (groupe Obstacles).
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
## Precisions du professeur : obstacles de 100 x 150 x 50 mm, droits et centres sur la ligne, seulement sur des lignes
## droites (juste avant ou apres un virage possible), toujours avec la place pour l'arret, le recul et le contournement.
@export_range(0.0, 1.0, 0.01) var proba_obstacle := 0.15 ## probabilite qu'une case droite recoive un obstacle
@export_range(0, 100) var nb_obstacles := 100 ## maximum par piste
@export var taille_obstacle := Vector3(0.10, 0.15, 0.05) ## largeur, hauteur, profondeur (m) : obstacle reel
@export var taille_obstacle_max := Vector3(0.10, 0.15, 0.05) ## taille maximale, tiree au hasard (= taille : toujours la meme)
## Precision du 13 nov. : le bloc peut etre pose sur n'importe quelle face. Vrai : une des 6 poses (permutation de
## largeur, hauteur, profondeur) tiree au hasard pour chaque obstacle. Faux : toujours debout (pistes d'avant le 13 nov.).
@export var faces_aleatoires := true
## Obstacle pose a la main : ecart maximal de son centre a la ligne (ReglagesEpreuve.tolerance_centrage_mm).
@export_range(0.0, 0.05, 0.005, "suffix:m") var decalage_obstacle_max := 0.01
@export_range(2, 20) var cases_libres_depart := 2 ## pas d'obstacle sous la voiture au depart
## Ligne droite avant la face de l'obstacle = place de la sequence arret + recul (calculee par le Controleur avec
## ReglagesEpreuve, voir Controleur.place_avant_obstacle_mm) + cette marge : la voiture qui sort d'un virage doit
## rouler un peu sur la droite pour s'y aligner. HYPOTHESE : ~1 empattement (138 mm), a confirmer avec le professeur.
@export_range(0.0, 1.0, 0.01, "suffix:m") var marge_droite_avant := 0.14
## Ligne droite exigee apres l'arriere de l'obstacle : retour du contournement fluide sur la ligne
## (arriere de la voiture degage de 135 mm puis retour de 450 mm, ligne revue par les sondes) = ~0,58 m.
@export_range(0.0, 2.0, 0.01, "suffix:m") var droite_apres_min := 0.60

@export_group("Generateur libre")
## grille = virages de 90 deg seulement (GenerateurPiste) ; libre = virages de angle_min_deg a angle_max_deg, sans grille
## (GenerateurLibre).
@export_enum("grille", "libre") var generateur: String = "grille"
## Le guide n'impose aucun angle, seulement le rayon (#73). HYPOTHESE : bornes choisies par l'equipe.
@export_range(1.0, 180.0, 1.0, "suffix:deg") var angle_min_deg := 20.0
@export_range(1.0, 180.0, 1.0, "suffix:deg") var angle_max_deg := 180.0
## Ecart minimal entre deux parties du ruban non voisines. HYPOTHESE : comme deux lignes paralleles a une case d'ecart sur
## la grille de 120 mm.
@export_range(0.01, 1.0, 0.005, "suffix:m") var ecart_lignes_min := 0.24
## Longueur d'une droite apres un virage. HYPOTHESE : bornes choisies par l'equipe. Maximum 2,5 m : un obstacle demande
## ~1,3 m de droite (place avant l'obstacle + obstacle + droite_apres_min) ; a 1,2 m, aucune piste n'avait d'obstacle
## (banc du 13 nov.) ; a 2,5 m, 33 pistes sur 40 en ont au moins un.
@export_range(0.0, 3.0, 0.01, "suffix:m") var droite_libre_min := 0.15
@export_range(0.0, 3.0, 0.01, "suffix:m") var droite_libre_max := 2.5
