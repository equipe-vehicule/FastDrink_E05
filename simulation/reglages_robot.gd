## Caracteristiques du robot partagees par tous les algorithmes (geometrie, moteur, servo).
## Ce fichier ne depend pas de Godot : la meme logique peut tourner sur le vrai robot.
## Les valeurs physiques viennent du fichier unique reglages_robot.json (racine du depot), par appliquer() :
## aucune valeur du robot n'est ecrite ici. Les reglages des algorithmes (rampe, freinage) restent ici.
class_name ReglagesRobot
extends RefCounted

const ANGLE_MIN := 45.0           # servo, braquage maximal a gauche (commande)
const ANGLE_MAX := 135.0          # servo, braquage maximal a droite (commande)

## Champ de cette classe <- cle de reglages_robot.json (seul endroit ou les noms se correspondent).
const CLES_JSON := {
	"empattement_mm": "empattement_mm",
	"longueur_avant_mm": "essieu_ultrason_mm",
	"distance_sondes_mm": "essieu_sondes_mm",
	"ultrason_avant_mm": "ultrason_avant_mm",
	"largeur_voiture_mm": "largeur_mm",
	"arriere_voiture_mm": "arriere_voiture_mm",
	"angle_roues_max_deg": "angle_roues_max_deg",
	"vitesse_servo_deg_s": "vitesse_servo_deg_s",
	"demi_angle_ultrason_deg": "demi_angle_ultrason_deg",
	"mm_s_par_cmd": "vitesse_mm_s_par_cmd",
	"frequence_controle_hz": "frequence_controle_hz",
}

# ----- Robot (reglages_robot.json)
var empattement_mm := 0.0         # essieu avant -> essieu arriere
var longueur_avant_mm := 0.0      # de l'essieu arriere au capteur ultrason
var distance_sondes_mm := 0.0     # de l'essieu arriere a la barre de sondes de ligne
var ultrason_avant_mm := 0.0      # de la face de l'ultrason a l'avant de la voiture
var largeur_voiture_mm := 0.0     # largeur hors tout
var arriere_voiture_mm := 0.0     # depassement derriere l'essieu arriere
var angle_roues_max_deg := 0.0    # angle reel des roues quand le servo est a 45 ou 135
var vitesse_servo_deg_s := 0.0    # vitesse du servo de direction (deg de servo par s ; 0 = instantane)
var demi_angle_ultrason_deg := 0.0  # demi-ouverture du cone de l'ultrason
var mm_s_par_cmd := 0.0           # vitesse reelle par unite de commande
var frequence_controle_hz := 0.0  # frequence de la boucle du programme du robot (Hz ; 0 = a chaque pas physique)

# ----- Algorithmes : moteur et freinage
var rampe_freinage_cmd_par_s := 125.0   # limitation de la commande au freinage (bille) ; vitesse_freinage() en depend
## Etape 5 (#67, 13 nov.) : rampe de la commande a l'acceleration seulement (cmd/s ; 0 = rampe_freinage_cmd_par_s).
## Le freinage garde rampe_freinage_cmd_par_s (vitesse_freinage en depend : arret devant l'obstacle).
## Cause visee : fin de la reprise douce apres un rattrapage, la commande passe de 25 a 45 en 0,16 s pendant que la
## bille oscille encore (parcours fourni 0,96 ; demi_tour 0,99).
## Avant le 13 nov. : 0 (125 cmd/s). Avec 60 : bille max 0,99 -> 0,82 (17 scenarios), 0,84 -> 0,80 (banc 40),
## rattrapages 14 -> 10, temps +3 %. 40 et 25 essayes : voir LISEZMOI_SIMULATION.md (etape 5). Provisoire : depend de
## la frequence propre de la bille (a mesurer sur le robot).
var rampe_montee_cmd_par_s := 60.0
var marge_freinage := 0.6         # 0..1 : fraction de la rampe utilisee pour freiner
var vitesse_approche_min := 8     # vitesse minimale d'approche (evite de caler avant la cible)

## Remplit les valeurs du robot depuis {cle JSON: valeur} (FichierReglages.lire). Signale toute cle absente.
func appliquer(valeurs: Dictionary) -> void:
	for champ in CLES_JSON:
		var cle: String = CLES_JSON[champ]
		if valeurs.has(cle):
			set(champ, float(valeurs[cle]))
		else:
			push_error("ReglagesRobot : « %s » absent de reglages_robot.json." % cle)

## De l'essieu arriere a l'avant de la voiture (capteur + depassement du capteur).
func avant_voiture_mm() -> float:
	return longueur_avant_mm + ultrason_avant_mm

## Commande permettant encore de s'arreter sur restante_mm (deceleration constante :
## d = K * c^2 / (2 * rampe * marge)).
func vitesse_freinage(restante_mm: float) -> float:
	return sqrt(2.0 * rampe_freinage_cmd_par_s * marge_freinage * restante_mm / mm_s_par_cmd)

## Ecart du servo par rapport a 90 -> angle reel des roues (deg).
func servo_vers_roues_deg(ecart_servo: float) -> float:
	return ecart_servo / 45.0 * angle_roues_max_deg

## Angle reel des roues (deg) -> ecart du servo par rapport a 90.
func roues_vers_servo(angle_roues_deg: float) -> float:
	return angle_roues_deg * 45.0 / angle_roues_max_deg
