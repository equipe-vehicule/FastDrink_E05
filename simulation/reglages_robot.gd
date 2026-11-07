## Caracteristiques du robot partagees par tous les algorithmes (geometrie, moteur, servo).
## Ce fichier ne depend pas de Godot : la meme logique peut tourner sur le vrai robot.
class_name ReglagesRobot
extends RefCounted

const ANGLE_MIN := 45.0           # servo, braquage maximal a gauche
const ANGLE_MAX := 135.0          # servo, braquage maximal a droite

# ----- Geometrie (mm) : A MESURER sur la voiture
var empattement_mm := 138.0       # mesure : Vehicule.EMPATTEMENT (en m) x 1000
var longueur_avant_mm := 193.0    # de l'essieu arriere au capteur ultrason (modele 3D ; a confirmer a la regle)
var distance_sondes_mm := 217.0   # de l'essieu arriere a la barre de sondes de ligne (modele 3D ; a confirmer a la regle)
var angle_roues_max_deg := 45.0   # angle reel des roues quand le servo est a 45 ou 135 (voir "Roues" au HUD)
var vitesse_servo_deg_s := 300.0  # A MESURER : vitesse du servo de direction (deg de servo par s ; 0 = instantane)
var demi_angle_ultrason_deg := 7.5  # demi-ouverture du cone de l'ultrason (HC-SR04 : environ 15 deg au total)

# ----- Moteur et freinage
var mm_s_par_cmd := 4.0           # vitesse reelle par unite de commande (0,40 m/s a 100) : A RECALCULER
var rampe_cmd_par_s := 125.0      # limitation de l'acceleration (bille)
var marge_freinage := 0.6         # 0..1 : fraction de la rampe utilisee pour freiner
var vitesse_approche_min := 8     # vitesse minimale d'approche (evite de caler avant la cible)

## Commande permettant encore de s'arreter sur restante_mm (deceleration constante :
## d = K * c^2 / (2 * rampe * marge)).
func vitesse_freinage(restante_mm: float) -> float:
	return sqrt(2.0 * rampe_cmd_par_s * marge_freinage * restante_mm / mm_s_par_cmd)

## Ecart du servo par rapport a 90 -> angle reel des roues (deg).
func servo_vers_roues_deg(ecart_servo: float) -> float:
	return ecart_servo / 45.0 * angle_roues_max_deg

## Angle reel des roues (deg) -> ecart du servo par rapport a 90.
func roues_vers_servo(angle_roues_deg: float) -> float:
	return angle_roues_deg * 45.0 / angle_roues_max_deg
