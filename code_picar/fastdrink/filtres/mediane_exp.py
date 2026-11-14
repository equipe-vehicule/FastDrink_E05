"""Médiane glissante puis exponentiel (version de référence pour la comparaison).

La version détaillée de ce filtre est la partie de l'autre membre ; celle-ci sert seulement
à comparer les deux filtres sur les mêmes fichiers.
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
