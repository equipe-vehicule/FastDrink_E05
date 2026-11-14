## Ce qu'impose l'epreuve (guide du cours et precisions du professeur), partage par tous les algorithmes.
## Un seul endroit a modifier quand les valeurs officielles seront donnees. Ne depend pas de Godot.
##
## Guide (section 1) : arret a une distance predeterminee (+/- 30 mm), ruban de 18 mm, courbes de rayon >= 120 mm,
## contournement en quittant la ligne, recul en suivant la ligne sur moins de 200 mm.
## Precisions orales du professeur (12 nov.) : devant chaque obstacle, la voiture s'arrete a distance_arret_mm,
## recule de distance_recul_mm en suivant la ligne, puis contourne. Obstacles de 100 x 150 x 50 mm, droits et centres
## sur la ligne, seulement sur des lignes droites (juste avant ou apres un virage possible), toujours avec la place
## pour l'arret, le recul et le contournement.
class_name ReglagesEpreuve
extends RefCounted

const RECUL_MAX_MM := 200.0              # guide : recul sur moins de 200 mm
const TOLERANCE_ARRET_MM := 30.0         # guide : precision de l'arret

# ----- Sequence devant un obstacle (VALEURS DE TEST : les valeurs officielles seront donnees plus tard)
var distance_arret_mm := 100.0           # de l'avant de la voiture a la face de l'obstacle
var distance_recul_mm := 150.0           # recul en suivant la ligne, < RECUL_MAX_MM
var pause_avant_recul_s := 0.3           # immobile apres l'arret (bille : pas d'aller-retour brusque)
var pause_avant_contournement_s := 0.3   # immobile apres le recul

# ----- Obstacle (precisions du professeur)
var largeur_obstacle_mm := 100.0         # en travers de la ligne
var hauteur_obstacle_mm := 150.0
var profondeur_obstacle_mm := 50.0       # dans le sens de la marche
var tolerance_centrage_mm := 10.0        # obstacle pose a la main : ecart possible de son centre a la ligne

## Precision du 13 nov. : le bloc peut etre pose sur n'importe quelle face (debout, tourne, couche), toujours centre.
## L'ultrason ne donne qu'une distance : le robot ne sait pas sur quelle face il est pose. Il contourne donc la plus
## grande emprise au sol possible : la plus grande des 3 dimensions, en travers comme dans le sens de la marche
## (100 x 150 x 50 -> 150 x 150). Calcule ici pour ne pas ecrire 150 a un second endroit.
func largeur_a_contourner_mm() -> float:
	return maxf(largeur_obstacle_mm, maxf(hauteur_obstacle_mm, profondeur_obstacle_mm))

func profondeur_a_contourner_mm() -> float:
	return maxf(largeur_obstacle_mm, maxf(hauteur_obstacle_mm, profondeur_obstacle_mm))
