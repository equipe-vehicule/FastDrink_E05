"""Filtre B : filtre de Kalman à vitesse constante pour le capteur ultrason.

Utilisé par le filtre final (filtre_final.py) quand le robot roule : pas de retard, rejet des
valeurs extrêmes au-delà de 3 écarts types, et arrêt programmé avec la vitesse estimée.

État x = [d, v] : distance (mm) et sa dérivée (mm/s, négative quand le robot s'approche).
Écrit sans numpy (matrices 2x2 à la main) pour tourner tel quel sur le Raspberry Pi.

Exemple :
    kf = KalmanDistance()
    d, v = kf.mettre_a_jour(z_mm, t_s)   # z_mm = -1 si l'écho manque
"""
from fastdrink import config

MESURE_RATEE = -1.0


class KalmanDistance:
    def __init__(self, sigma_z=config.KALMAN_SIGMA_Z_MM, sigma_a=config.KALMAN_SIGMA_A_MM_S2,
                 seuil_rejet=config.KALMAN_SEUIL_REJET,
                 rejets_avant_reset=config.KALMAN_REJETS_AVANT_RESET,
                 d_min=config.DISTANCE_MIN_MM, d_max=config.DISTANCE_MAX_MM):
        self.R = sigma_z ** 2
        self.sigma_a = sigma_a
        self.seuil_rejet = seuil_rejet
        self.rejets_avant_reset = rejets_avant_reset
        self.d_min, self.d_max = d_min, d_max
        self.reinitialiser()

    def reinitialiser(self):
        self.d = None            # pas encore de mesure valide
        self.v = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]
        self.t = None
        self.rejets = 0          # rejets consécutifs
        self.derniere_mesure_acceptee = False

    def _demarrer(self, z, t):
        self.d, self.v, self.t = z, 0.0, t
        self.P = [[self.R, 0.0], [0.0, 1000.0 ** 2]]  # vitesse inconnue au départ
        self.rejets = 0

    def _predire(self, T):
        (p11, p12), (p21, p22) = self.P
        self.d += self.v * T
        q = self.sigma_a ** 2
        # P = F P F^T + Q, avec F = [[1, T], [0, 1]] et Q = sigma_a² [[T⁴/4, T³/2], [T³/2, T²]]
        n11 = p11 + T * (p12 + p21) + T * T * p22 + q * T ** 4 / 4
        n12 = p12 + T * p22 + q * T ** 3 / 2
        n22 = p22 + q * T * T
        self.P = [[n11, n12], [n12, n22]]

    def mettre_a_jour(self, z, t):
        """Traite une mesure brute z (mm, ou -1) prise à l'instant t (s). Renvoie (d, v).
        Avant la première mesure valide, renvoie (None, 0.0)."""
        self.derniere_mesure_acceptee = False
        valide = z != MESURE_RATEE and self.d_min <= z <= self.d_max
        if self.d is None:
            if valide:
                self._demarrer(z, t)
                self.derniere_mesure_acceptee = True
            return self.d, self.v

        self._predire(max(t - self.t, 0.0))
        self.t = t
        if not valide:                      # écho manqué : prédiction seule
            return self.d, self.v

        y = z - self.d                      # écart entre mesure et prédiction
        S = self.P[0][0] + self.R
        if y * y / S > self.seuil_rejet:    # mesure trop surprenante : ignorée
            self.rejets += 1
            if self.rejets >= self.rejets_avant_reset:
                self._demarrer(z, t)        # on s'était trompé : on repart de la mesure
                self.derniere_mesure_acceptee = True
            return self.d, self.v

        self.rejets = 0
        (p11, p12), (_, p22) = self.P
        k1, k2 = p11 / S, p12 / S          # gain K = P H^T / S
        self.d += k1 * y
        self.v += k2 * y
        self.P = [[(1 - k1) * p11, (1 - k1) * p12],
                  [(1 - k1) * p12, p22 - k2 * p12]]
        self.derniere_mesure_acceptee = True
        return self.d, self.v

    def distance_anticipee(self, delai_s):
        """Distance prévue dans delai_s secondes."""
        return None if self.d is None else self.d + self.v * delai_s

    def delai_avant(self, d_cible, horizon_s):
        """Temps (s) avant d'atteindre d_cible si on y arrive dans moins de horizon_s, sinon None.

        Sert à programmer l'arrêt ENTRE deux mesures : à 300 mm/s le robot parcourt 18 mm
        pendant les 60 ms qui séparent deux mesures, presque la moitié de la tolérance.
        """
        if self.d is None:
            return None
        if self.d <= d_cible:
            return 0.0
        if self.v >= 0:
            return None                      # le robot ne se rapproche pas
        delai = (self.d - d_cible) / -self.v
        return delai if delai <= horizon_s else None
