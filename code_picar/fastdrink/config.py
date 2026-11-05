"""Réglages matériels du robot n° 05, regroupés ici pour ne les changer qu'à un seul endroit.

Numérotation des broches : BCM. Sur le Robot HAT, le repère « Bxx » donne directement ce numéro.
"""

# Capteur ultrason (module à 4 broches modifié ; câblage vérifié au diagnostic du 5 nov. 2026)
ULTRASON_TRIG = 12             # B12, fil bleu : envoie l'impulsion de déclenchement
ULTRASON_ECHO = 16             # B16, fil jaune : reçoit l'écho
ULTRASON_TIMEOUT_S = 0.05      # attente maximale d'un front, valeur du pilote d'origine SunFounder
ULTRASON_PERIODE_MIN_S = 0.06  # au moins 60 ms entre deux mesures (manuel PiCar-S v2.0)

VITESSE_SON_M_S = 343.0        # dans l'air à environ 20 °C
