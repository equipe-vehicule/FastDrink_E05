"""Filtre final de l'équipe E05 : filtre B (Kalman) quand le robot roule, filtre A (médiane + exponentiel) à l'arrêt.

    robot en mouvement : Kalman à vitesse constante (filtre B) sur les mesures brutes, avec rejet des
                         valeurs extrêmes et arrêt programmé à partir de la vitesse estimée
    robot immobile     : médiane des 3 dernières mesures puis lissage exponentiel (filtre A)

Le Kalman tourne en permanence (sa vitesse reste à jour pour le prochain départ) ; seule la
sortie change de filtre. Au passage à l'arrêt, l'exponentiel part de la distance du Kalman,
pour que la sortie ne saute pas.

Choix justifié par les mesures réelles du 9 nov. (rapport « Filtre tes connaissances ? ») :
le filtre A est le plus stable à l'arrêt (0,9 mm d'écart type), le filtre B le plus précis en mouvement
(arrêt à 6 mm près au pire). Placer la médiane DEVANT le Kalman n'apporte rien sur le bruit réel
et rend le filtre plus fragile aux échos parasites.

Exemple (sur le Pi) :
    f = FiltreFinal()
    d = f.mettre_a_jour(z_mm, time.perf_counter(), en_mouvement=(commande != 0))
    delai = f.delai_avant(d_arret_mm, 0.1)   # robot en mouvement : arrêt dans `delai` s, ou None
"""
import math

from fastdrink import config
from .kalman import KalmanDistance

MESURE_RATEE = -1.0


class FiltreFinal:
    def __init__(self, tau_repos_s=config.FILTRE_A_TAU_S, sigma_z=config.KALMAN_SIGMA_Z_MM,
                 sigma_a=config.KALMAN_SIGMA_A_MM_S2):
        self.tau = tau_repos_s
        self.filtre_b = KalmanDistance(sigma_z=sigma_z, sigma_a=sigma_a)
        self.fenetre = []            # 3 dernières mesures valides (médiane du filtre A)
        self.y_repos = None          # sortie de l'exponentiel du filtre A
        self.t_prec = None
        self.en_mouvement = True
        self.sortie = None

    def mettre_a_jour(self, z, t, en_mouvement):
        """z : mesure brute (mm, ou -1) ; t : instant (s) ; en_mouvement : commande moteur non nulle.
        Renvoie la distance filtrée (mm), ou None avant la première mesure valide."""
        d_kalman, _ = self.filtre_b.mettre_a_jour(z, t)
        periode = 0.0 if self.t_prec is None else t - self.t_prec
        self.t_prec = t
        valide = z != MESURE_RATEE and self.filtre_b.d_min <= z <= self.filtre_b.d_max
        if valide:
            self.fenetre = (self.fenetre + [z])[-3:]

        if en_mouvement:
            self.en_mouvement = True
            self.sortie = d_kalman
            return self.sortie

        if self.en_mouvement or self.y_repos is None:     # on vient de s'arrêter
            self.y_repos = d_kalman                       # continuité de la sortie
            self.en_mouvement = False
        elif valide and periode > 0:
            m = sorted(self.fenetre)[len(self.fenetre) // 2]
            alpha = 1.0 - math.exp(-periode / self.tau)
            self.y_repos = alpha * m + (1.0 - alpha) * self.y_repos
        self.sortie = self.y_repos
        return self.sortie

    def vitesse(self):
        """Vitesse estimée (mm/s, négative quand le robot s'approche)."""
        return self.filtre_b.v

    def delai_avant(self, d_cible, horizon_s):
        """Arrêt programmé (robot en mouvement) : délai avant d_cible si moins de horizon_s, sinon None."""
        return self.filtre_b.delai_avant(d_cible, horizon_s)
