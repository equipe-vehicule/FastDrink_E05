"""Réglages matériels du robot n° 05, regroupés ici pour ne les changer qu'à un seul endroit.

Numérotation des broches : BCM. Sur le Robot HAT, le repère « Bxx » donne directement ce numéro.
"""

# Capteur ultrason (module à 4 broches modifié ; câblage vérifié au diagnostic du 5 nov. 2026)
ULTRASON_TRIG = 12             # B12, fil bleu : envoie l'impulsion de déclenchement
ULTRASON_ECHO = 16             # B16, fil jaune : reçoit l'écho
ULTRASON_TIMEOUT_S = 0.05      # attente maximale d'un front, valeur du pilote d'origine SunFounder
ULTRASON_PERIODE_MIN_S = 0.06  # au moins 60 ms entre deux mesures (manuel PiCar-S v2.0)

VITESSE_SON_M_S = 343.0        # dans l'air à environ 20 °C
CAPTEUR_AVANT_MM = 35.3        # distance face du capteur -> avant du véhicule (moyenne de 1000 mesures, 9 nov.)
# Alias gardé pour ne pas casser ultrasonic_stop_test.py et essai_ultrason.py (Sarah, 12 nov.) : même grandeur.
ULTRASONIC_OFFSET = CAPTEUR_AVANT_MM

# Géométrie du train avant (ajout de Sarah, 12 nov., venu de main)
FW_WHEEL_OFFSET = 17           # Front Wheels offset to the right.

# Filtre de Kalman (distance + vitesse). Valeurs PROVISOIRES, à remplacer par les mesures :
# sigma_z = écart type du capteur robot immobile, sigma_a = variation de vitesse possible.
KALMAN_SIGMA_Z_MM = 2.4        # bruit du capteur (mm) — écart type mesuré robot immobile, 9 nov.
KALMAN_SIGMA_A_MM_S2 = 200.0   # Kalman seul : accélération non modélisée (mm/s²) — réglé hors ligne, 9 nov.
# médiane 3 + Kalman (mm/s²). Valeur précédente : 400 (réglé hors ligne, 9 nov.).
# HYPOTHESE : 1000, réglée en simulation le 14 nov. (erreur d'arrêt max sur 40 pistes 11,7 -> 9,3 mm ;
# source : simulation/filtre_ultrason.gd, sigma_a_mm_s2) ; à vérifier sur le robot.
MEDIANE_KALMAN_SIGMA_A_MM_S2 = 1000.0
KALMAN_SEUIL_REJET = 9.0       # écart normalisé max (9 = 3 écarts types)
KALMAN_REJETS_AVANT_RESET = 5  # rejets consécutifs avant de repartir de la mesure
DISTANCE_MIN_MM = 20.0         # plage acceptée ; en dehors = mesure ignorée
DISTANCE_MAX_MM = 2000.0

# Vitesse maximale du robot (mm/s) : au-delà, la vitesse estimée par le Kalman est impossible (obstacle fixe).
# HYPOTHESE, point 100 de table_vitesse_mm_s de reglages_robot.json, #18 ; à remplacer quand le contrôleur lira le JSON.
VITESSE_MAX_MM_S = 614.0
