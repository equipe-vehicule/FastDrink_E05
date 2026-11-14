"""Médiane de 3 puis filtre de Kalman : la combinaison retenue après les mesures du 9 nov.

Pourquoi : le bruit réel du capteur a des valeurs extrêmes rares (environ 1 % à plus de 10 mm).
Un Kalman seul assez réactif pour suivre les accélérations en accepte parfois une, ce qui fausse
sa vitesse. La médiane de 3 enlève ces valeurs isolées avant le Kalman.

Le retard de la médiane est compensé : sur une distance qui varie régulièrement, la médiane de
3 mesures vaut la mesure du milieu ; on la date donc à l'instant du milieu, et le Kalman
projette ensuite l'estimation jusqu'à l'instant présent avec la vitesse estimée.

Ecarts VOULUS avec la version du 9 nov. (reportés le 14 nov. depuis simulation/filtre_ultrason.gd, étape 10) :
  1. Echo manqué : pas de prédiction qui avance l'instant de l'état (la sortie est seulement projetée à
     l'instant présent). Cause : la médiane suivante, datée au milieu de sa fenêtre (~62 ms plus tôt), échouait
     au test t_milieu > t_état et n'était jamais prise ; graine 300923 : sortie 559 -> 0 mm en 0,35 s.
  2. Redémarrage du Kalman sur la mesure si |v estimée| > VITESSE_MAX_MM_S (voir kalman.py ; HYPOTHESE #18).
  3. MEDIANE_KALMAN_SIGMA_A_MM_S2 : 400 -> 1000 (config.py ; HYPOTHESE réglée en simulation, à vérifier sur robot).

Ecarts connus RESTANTS avec la simulation (en-tête de simulation/filtre_ultrason.gd), non changés ici :
  - portée : la simulation ignore les mesures > 1500 mm (portee_max_mm) ; ici DISTANCE_MAX_MM = 2000 ;
  - remise à zéro après une absence d'écho (0,15 s ; 1 s si l'obstacle était proche) : n'existe pas ici ;
  - horodatage : la simulation date les mesures au tour du contrôleur (<= 20 ms) ; ici, l'instant de lecture t.
"""
from collections import deque

from fastdrink import config
from .kalman import KalmanDistance, MESURE_RATEE


class MedianeKalman:
    def __init__(self, sigma_z=config.KALMAN_SIGMA_Z_MM, sigma_a=config.MEDIANE_KALMAN_SIGMA_A_MM_S2,
                 vitesse_max_mm_s=config.VITESSE_MAX_MM_S, **kwargs):
        self.kf = KalmanDistance(sigma_z=sigma_z, sigma_a=sigma_a, vitesse_max_mm_s=vitesse_max_mm_s, **kwargs)
        self.fenetre = deque(maxlen=3)   # (instant, mesure) des 3 dernières mesures valides
        self.t = None                    # instant de la dernière mesure reçue

    def mettre_a_jour(self, z, t):
        """Même interface que KalmanDistance : renvoie (d, v) à l'instant t."""
        self.t = t
        if z == MESURE_RATEE or not self.kf.d_min <= z <= self.kf.d_max:
            # Echo manqué : l'état n'est pas touché (écart n° 1, 14 nov.) ; distance() projette à l'instant t.
            # Ancien code (9 nov.) : self.kf.mettre_a_jour(MESURE_RATEE, t)  # prédiction seule
            return self.distance(), self.kf.v
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
