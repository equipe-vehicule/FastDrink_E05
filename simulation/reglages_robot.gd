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
	"frequence_controle_hz": "frequence_controle_hz",
	"erreur_vitesse_odometrie": "erreur_vitesse_odometrie",
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
var erreur_vitesse_odometrie := 0.0  # erreur relative de l'odometrie sur la vitesse (0 = etalonnage parfait)
## Table de vitesse [[commande, mm/s], ...] (commandes croissantes, marche avant ; l'arriere est symetrique : hypothese).
## Vide = modele lineaire (mode ideal) : vitesse = commande x mm_s_par_cmd. Cle : table_vitesse_mm_s.
var table_vitesse_mm_s: Array = []
var mm_s_par_cmd := 0.0           # modele lineaire seulement (table vide) : mm/s par unite de commande (cle vitesse_mm_s_par_cmd)
## Choix du filtre de l'ultrason : "mediane3" ou "mediane_kalman" (voir FiltreUltrason). Cle : filtre_ultrason.
var filtre_ultrason := "mediane3"
var frequence_controle_hz := 0.0  # frequence de la boucle du programme du robot (Hz ; 0 = a chaque pas physique)

# ----- Algorithmes : vitesses et freinage (unites physiques ; la table convertit en commande a la sortie du controleur)
## Deceleration maximale de la vitesse de consigne au freinage (bille) ; vitesse_freinage() en depend.
## (= ancienne rampe_freinage_cmd_par_s 125 commandes/s a 4 mm/s par commande.)
var deceleration_freinage_mm_s2 := 500.0
## Etape 5 (#67, 13 nov.) : acceleration de la vitesse de consigne, a l'acceleration seulement (mm/s^2 ; 0 = deceleration_freinage_mm_s2).
## (= ancienne rampe_montee_cmd_par_s 60 commandes/s a 4 mm/s par commande.)
## Le freinage garde deceleration_freinage_mm_s2 (vitesse_freinage en depend : arret devant l'obstacle).
## Cause visee : fin de la reprise douce apres un rattrapage, la vitesse passe de 100 a 180 mm/s en 0,16 s pendant que la
## bille oscille encore (parcours fourni 0,96 ; demi_tour 0,99).
## Avant le 13 nov. : 0 (500 mm/s^2). Avec 240 : bille max 0,99 -> 0,82 (17 scenarios), 0,84 -> 0,80 (banc 40),
## rattrapages 14 -> 10, temps +3 %. 160 et 100 essayes (ex-commandes 40 et 25) : voir LISEZMOI_SIMULATION.md (etape 5).
## Provisoire : depend de la frequence propre de la bille (a mesurer sur le robot).
var acceleration_montee_mm_s2 := 240.0
var marge_freinage := 0.6         # 0..1 : fraction de la deceleration utilisee pour freiner
## Vitesse minimale d'approche (evite de caler avant la cible). (= ancienne commande 8 a 4 mm/s par commande.)
var vitesse_approche_min_mm_s := 32.0

## Remplit les valeurs du robot depuis {cle JSON: valeur} (FichierReglages.lire). Signale toute cle absente.
func appliquer(valeurs: Dictionary) -> void:
	for champ in CLES_JSON:
		var cle: String = CLES_JSON[champ]
		if valeurs.has(cle):
			set(champ, float(valeurs[cle]))
		else:
			push_error("ReglagesRobot : « %s » absent de reglages_robot.json." % cle)
	table_vitesse_mm_s = []
	for point in valeurs.get("table_vitesse_mm_s", []):
		table_vitesse_mm_s.append([float(point[0]), float(point[1])])
	if valeurs.has("vitesse_mm_s_par_cmd"):
		mm_s_par_cmd = float(valeurs["vitesse_mm_s_par_cmd"])
	elif table_vitesse_mm_s.size() < 2:
		push_error("ReglagesRobot : ni table_vitesse_mm_s (2 points au moins) ni vitesse_mm_s_par_cmd dans reglages_robot.json.")
	filtre_ultrason = str(valeurs.get("filtre_ultrason", "mediane3"))

## De l'essieu arriere a l'avant de la voiture (capteur + depassement du capteur).
func avant_voiture_mm() -> float:
	return longueur_avant_mm + ultrason_avant_mm

## Vrai si la vitesse est proportionnelle a la commande (table vide : modele ideal).
func est_lineaire() -> bool:
	return table_vitesse_mm_s.size() < 2

## Vitesse reelle (mm/s, signee) pour une commande de -100 a 100 : table_vitesse_mm_s (interpolation lineaire,
## symetrique en marche arriere), ou commande x mm_s_par_cmd si la table est vide (calcul ideal, inchange).
func vitesse_mm_s(cmd: float) -> float:
	if est_lineaire():
		return cmd * mm_s_par_cmd
	var c := absf(cmd)
	var v := _vitesse_table(c)
	return v if cmd >= 0.0 else -v

## Inverse de vitesse_mm_s, sur la partie strictement croissante de la table (au-dela de la zone morte) : commande
## (signee) qui donne mm_s. 0 pour une vitesse nulle ; bornee a la derniere commande de la table au-dela de sa vitesse.
func commande_pour_vitesse(mm_s: float) -> float:
	if est_lineaire():
		return mm_s / mm_s_par_cmd
	var v := absf(mm_s)
	var c := 0.0
	if v > 0.0:
		c = table_vitesse_mm_s[table_vitesse_mm_s.size() - 1][0]
		for i in range(1, table_vitesse_mm_s.size()):
			var a: Array = table_vitesse_mm_s[i - 1]
			var b: Array = table_vitesse_mm_s[i]
			if b[1] > a[1] and v <= b[1]:
				c = a[0] + (v - a[1]) / (b[1] - a[1]) * (b[0] - a[0])
				break
	return c if mm_s >= 0.0 else -c

## Vitesse (mm/s) que l'odometrie croit : la meme table (etalonnage suppose parfait) x (1 + erreur_vitesse_odometrie).
func vitesse_odometrie_mm_s(cmd: float) -> float:
	return vitesse_mm_s(cmd) * (1.0 + erreur_vitesse_odometrie)

## Table : vitesse (mm/s, >= 0) pour une commande c >= 0 ; au-dela du dernier point : derniere vitesse.
func _vitesse_table(c: float) -> float:
	var n := table_vitesse_mm_s.size()
	if c <= table_vitesse_mm_s[0][0]:
		return table_vitesse_mm_s[0][1]
	for i in range(1, n):
		var a: Array = table_vitesse_mm_s[i - 1]
		var b: Array = table_vitesse_mm_s[i]
		if c <= b[0]:
			return a[1] + (c - a[0]) / (b[0] - a[0]) * (b[1] - a[1])
	return table_vitesse_mm_s[n - 1][1]

## Vitesse (mm/s, >= 0) permettant encore de s'arreter sur restante_mm en freinant a deceleration_freinage_mm_s2 x
## marge_freinage : v = racine(2 x a x marge x d). Ne depend pas du moteur (l'ancienne formule en commandes en dependait).
func vitesse_freinage(restante_mm: float) -> float:
	return sqrt(2.0 * deceleration_freinage_mm_s2 * marge_freinage * restante_mm)

## Ecart du servo par rapport a 90 -> angle reel des roues (deg).
func servo_vers_roues_deg(ecart_servo: float) -> float:
	return ecart_servo / 45.0 * angle_roues_max_deg

## Angle reel des roues (deg) -> ecart du servo par rapport a 90.
func roues_vers_servo(angle_roues_deg: float) -> float:
	return angle_roues_deg * 45.0 / angle_roues_max_deg
