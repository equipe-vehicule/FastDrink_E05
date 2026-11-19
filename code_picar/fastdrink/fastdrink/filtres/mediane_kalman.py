"""Médiane de 3 puis filtre de Kalman : la combinaison retenue après les mesures du 9 nov.

Pourquoi : le bruit réel du capteur a des valeurs extrêmes rares (environ 1 % à plus de 10 mm).
Un Kalman seul assez réactif pour suivre les accélérations en accepte parfois une, ce qui fausse
sa vitesse. La médiane de 3 enlève ces valeurs isolées avant le Kalman.

Le retard de la médiane est compensé : sur une distance qui varie régulièrement, la médiane de
3 mesures vaut la mesure du milieu ; on la date donc à l'instant du milieu, et le Kalman
projette ensuite l'estimation jusqu'à l'instant présent avec la vitesse estimée.
"""
from collections import deque

from fastdrink import config
from .kalman import KalmanDistance, MESURE_RATEE


class MedianeKalman:
    def __init__(self, sigma_z=config.KALMAN_SIGMA_Z_MM, sigma_a=config.MEDIANE_KALMAN_SIGMA_A_MM_S2, **kwargs):
        self.kf = KalmanDistance(sigma_z=sigma_z, sigma_a=sigma_a, **kwargs)
        self.fenetre = deque(maxlen=3)   # (instant, mesure) des 3 dernières mesures valides
        self.t = None                    # instant de la dernière mesure reçue

    def mettre_a_jour(self, z, t):
        """Même interface que KalmanDistance : renvoie (d, v) à l'instant t."""
        self.t = t
        if z == MESURE_RATEE or not self.kf.d_min <= z <= self.kf.d_max:
            if self.kf.d is not None:
                self.kf.mettre_a_jour(MESURE_RATEE, t)      # prédiction seule
            return self.kf.d, self.kf.v
        self.fenetre.append((t, z))
        if self.kf.d is None:                                # démarrage sur la première mesure
            return self.kf.mettre_a_jour(z, t)
        if len(self.fenetre) == 3:
            mediane = sorted(m for _, m in self.fenetre)[1]
            t_milieu = self.fenetre[1][0]
            if t_milieu > self.kf.t:
                self.kf.mettre_a_jour(mediane, t_milieu)
        return self.distance(), self.kf.v

    def distance(self):
        """Distance estimée à l'instant de la dernière mesure."""
        return None if self.kf.d is None else self.kf.d + self.kf.v * (self.t - self.kf.t)

    def delai_avant(self, d_cible, horizon_s):
        """Temps (s) avant d'atteindre d_cible si c'est dans moins de horizon_s, sinon None."""
        d = self.distance()
        if d is None:
            return None
        if d <= d_cible:
            return 0.0
        if self.kf.v >= 0:
            return None
        delai = (d - d_cible) / -self.kf.v
        return delai if delai <= horizon_s else None
