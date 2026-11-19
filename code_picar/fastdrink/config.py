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

# Filtre A (médiane de 3 + exponentiel) : utilisé robot IMMOBILE dans le filtre final
FILTRE_A_TAU_S = 0.17          # constante de temps de l'exponentiel (s) : alpha = 0,30 à 62 ms

# Filtre B (Kalman distance + vitesse) : utilisé robot EN MOUVEMENT dans le filtre final
# sigma_z = écart type du capteur robot immobile, sigma_a = variation de vitesse possible.
KALMAN_SIGMA_Z_MM = 2.4        # bruit du capteur (mm) — écart type mesuré robot immobile, 9 nov.
KALMAN_SIGMA_A_MM_S2 = 200.0   # filtre B : accélération non modélisée (mm/s²) — réglé sur le bruit réel, 10 nov.
MEDIANE_KALMAN_SIGMA_A_MM_S2 = 400.0  # variante médiane 3 + Kalman (écartée, gardée pour la comparaison)
KALMAN_SEUIL_REJET = 9.0       # écart normalisé max (9 = 3 écarts types)
KALMAN_REJETS_AVANT_RESET = 5  # rejets consécutifs avant de repartir de la mesure
DISTANCE_MIN_MM = 20.0         # plage acceptée ; en dehors = mesure ignorée
DISTANCE_MAX_MM = 2000.0
