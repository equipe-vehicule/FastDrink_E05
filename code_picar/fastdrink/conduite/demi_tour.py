"""Demi-tour quand la voiture roule à l'envers sur la ligne : elle pivote en plusieurs manœuvres (avant braquage
maximal, arrière braquage opposé : elle tourne toujours du même côté) jusqu'à environ 180 deg, puis le suivi reprend
dès que la ligne est vue. Choix du moment et de l'autorisation : Contrôleur et MemoireParcours.

Portage de simulation/demi_tour.gd (lot C).
"""
from . import constantes
from . import godot as gd
from .godot import Vec2


class DemiTour:
    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « demi_tour »), lues au démarrage ; ici : type
    ## et justification seulement.
    REGLAGES = {
        "duree_avant_s": float,      # durée de chaque marche avant (braquage maximal)
        "duree_arriere_s": float,    # durée de chaque marche arrière (braquage opposé)
        "vitesse_mm_s": float,       # (= ancienne commande 18 à 4 mm/s par commande)
        "duree_max_s": float,
    }

    def __init__(self):
        self.duree_avant_s = 0.0
        self.duree_arriere_s = 0.0
        self.vitesse_mm_s = 0.0
        self.duree_max_s = 0.0

        self.termine = False

        self._cap_debut = 0.0
        self._cote = 1                 # 1 = la voiture pivote vers la droite

    def demarrer(self, cap, cote):
        """cap : cap global au départ (rad) ; cote : côté où la ligne a été vue en dernier."""
        self._cap_debut = cap
        self._cote = cote
        self.termine = False

    def ecart_reprise_mm(self):
        """Écart à donner au suivi quand le demi-tour est fini (la ligne est cherchée de ce côté)."""
        return constantes.ERREUR_MAX_MM * self._cote

    def calculer(self, n, cap, t_etat):
        """cap : cap global actuel (rad). Retourne Vec2(angle servo, vitesse cible). Regarder `termine` ensuite."""
        tourne = gd.absf(cap - self._cap_debut)
        if (tourne >= gd.deg_to_rad(150.0) and n > 0) or tourne >= gd.deg_to_rad(210.0) or t_etat > self.duree_max_s:
            self.termine = True
            return Vec2(90.0, 0.0)
        periode = self.duree_avant_s + self.duree_arriere_s
        if gd.fmod(t_etat, periode) < self.duree_avant_s:
            return Vec2(90.0 + 45.0 * self._cote, self.vitesse_mm_s)
        return Vec2(90.0 - 45.0 * self._cote, -self.vitesse_mm_s)
