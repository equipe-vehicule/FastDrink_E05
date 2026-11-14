"""Reprise douce du suivi après une manœuvre (contournement, rattrapage, demi-tour, recherche de ligne).
Portage de simulation/reprise_douce.gd.

Banc d'essai du 12 nov. : au passage manœuvre -> suivi, le braquage passait d'un coup de 130 à 45 deg et la voiture
réaccélérait avant d'être centrée ; la bille, déjà en mouvement, tombait (2 pistes sur 40, dont la piste 7).
Pendant duree_s : vitesse et vitesse de rotation du braquage limitées.
"""
from . import godot as gd


class RepriseDouce:
    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « reprise »), lues au démarrage ; ici : type
    ## et justification seulement.
    REGLAGES = {
        "duree_s": float,
        "vitesse_max_mm_s": float,        # (= ancienne commande 25 à 4 mm/s par commande)
        "braquage_max_deg_s": float,
    }

    def __init__(self):
        self.duree_s = 0.0
        self.vitesse_max_mm_s = 0.0
        self.braquage_max_deg_s = 0.0

        self._t = 0.0
        self._angle_prec = 90.0              # dernier angle commandé (deg de servo)

    def demarrer(self):
        self._t = self.duree_s

    def en_cours(self, en_suivi, dt):
        """À appeler une fois par pas. Vrai (et décompte le temps) pendant la reprise, si la voiture est en suivi."""
        if self._t > 0.0 and en_suivi:
            self._t -= dt
            return True
        return False

    def limiter_vitesse(self, cible):
        return self.vitesse_max_mm_s if cible > self.vitesse_max_mm_s else cible

    def limiter_angle(self, angle, dt):
        return gd.move_toward(self._angle_prec, angle, self.braquage_max_deg_s * dt)

    def memoriser_angle(self, angle):
        """À appeler à chaque pas avec l'angle finalement commandé."""
        self._angle_prec = angle
