"""Filtre A, version simplifiée à alpha fixe (médiane glissante puis exponentiel).

La version complète du filtre A est filtre_a.py (alpha calculé à partir de tau, règles d'absence
d'écho). Cette version sert seulement à analyse/analyse_filtres.py, qui produit la figure du robot
immobile du rapport (alpha = 0,30, équivalent à tau = 0,17 s à 62 ms).
"""
from collections import deque
from statistics import median

from fastdrink import config

MESURE_RATEE = -1.0


class MedianeExponentiel:
    def __init__(self, n=3, alpha=0.3, d_min=config.DISTANCE_MIN_MM, d_max=config.DISTANCE_MAX_MM):
        self.fenetre = deque(maxlen=n)
        self.alpha = alpha
        self.d_min, self.d_max = d_min, d_max
        self.y = None

    def mettre_a_jour(self, z, t=None):
        """Renvoie la distance filtrée (mm) ; une mesure invalide garde la dernière sortie."""
        if z == MESURE_RATEE or not self.d_min <= z <= self.d_max:
            return self.y
        self.fenetre.append(z)
        if len(self.fenetre) < self.fenetre.maxlen:   # attendre une fenêtre pleine
            return self.y
        m = median(self.fenetre)
        self.y = m if self.y is None else self.alpha * m + (1 - self.alpha) * self.y
        return self.y
