"""Filtrage de l'ultrason : une mesure -1 isolée (pas d'écho) ne fait pas repartir la voiture.
Portage de simulation/filtre_ultrason.gd.

Deux filtres, au choix (ReglagesRobot.filtre_ultrason, clé filtre_ultrason de reglages_robot.json) :
  "mediane3"       : médiane des 3 dernières mesures (filtre d'origine de la simulation ; mode idéal). Il est
                     appelé à chaque tour du programme et ignore mesure_nouvelle (comportement inchangé).
  "mediane_kalman" : filtre retenu par l'équipe (#65, rapport « Filtre tes connaissances » du 10 nov.) : médiane de 3
                     puis Kalman à vitesse constante. Reproduit 09_Code-PiCar/fastdrink/filtres/mediane_kalman.py et
                     kalman.py (paramètres : config.py). Le Kalman n'est mis à jour qu'à l'arrivée d'une nouvelle
                     mesure (mesure_nouvelle) ; entre deux, la sortie est projetée avec la vitesse estimée.
Écarts restants avec le Python (mediane_kalman.py) : portee_max_mm = 1500 (Python : D_MAX 2000) ; remise à zéro
après absence_s (0,15 s ; 1 s si l'obstacle était proche), qui n'existe pas en Python ; horodatage au tour du
contrôleur (<= 20 ms) au lieu de l'instant de lecture.
Étape 10 (14 nov.) : trois écarts VOULUS avec le Python, retenus par l'équipe, à reporter dans mediane_kalman.py et
config.py (étape 11) : prediction_sur_echo_manque et redemarrer_si_vitesse_impossible (graine 300923 des pistes
libres, bille tombée), sigma_a_mm_s2 = 1000 (erreur d'arrêt).
Les constantes viennent de config.py de 09_Code-PiCar (dépendance simulation <-> Pi à garder synchronisée) ;
SIGMA_A_CONFIG_PY = 400 est la valeur MEDIANE_KALMAN_SIGMA_A_MM_S2 (et non KALMAN_SIGMA_A_MM_S2 = 200, Kalman seul) ;
elle n'est plus utilisée (sigma_a_mm_s2 = 1000 ci-dessous) et reste pour mémoire.
Aucune dépendance à Godot (RefCounted dans le .gd) : se porte tel quel en Python.
"""
from . import godot as gd


class FiltreUltrason:
    TAILLE_FILTRE = 3          # médiane sur les 3 dernières mesures

    # ----- Paramètres de "mediane_kalman" (config.py de 09_Code-PiCar/fastdrink ; réglés hors ligne le 9 nov.)
    SIGMA_Z_MM = 2.4           # bruit de mesure (mm) : config.py KALMAN_SIGMA_Z_MM (écart type, robot immobile, 9 nov.)
    SIGMA_A_CONFIG_PY = 400.0  # pour mémoire (non utilisée) : config.py MEDIANE_KALMAN_SIGMA_A_MM_S2
    SEUIL_REJET = 9.0          # rejet si nu^2 / S > 9 (3 écarts types) : config.py KALMAN_SEUIL_REJET
    REJETS_AVANT_RESET = 5     # rejets consécutifs avant de repartir de la mesure : config.py KALMAN_REJETS_AVANT_RESET
    D_MIN_MM = 20.0            # plage acceptée par le Kalman : config.py DISTANCE_MIN_MM
    D_MAX_MM = 2000.0          # config.py DISTANCE_MAX_MM (portee_max_mm ci-dessous, plus stricte, s'applique avant)
    V0_ECART_TYPE_MM_S = 1000.0  # P0 = diag(R, 1000^2), v0 = 0 : kalman.py _demarrer

    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « ultrason »), lues au démarrage ; ici : type
    ## et justification seulement.
    REGLAGES = {
        "portee_max_mm": float,   # au-delà, la mesure est ignorée
        "absence_s": float,       # durée sans écho avant de considérer qu'il n'y a plus d'obstacle
        "zone_proche_mm": float,  # sous cette distance, le capteur peut « perdre » un obstacle bien présent
        "absence_proche_s": float,  # durée sans écho exigée quand l'obstacle était très proche
        ## Étape 10 : False (défaut, écart voulu avec le Python) : un écho manqué ne touche pas l'état du Kalman ; True :
        ## prédiction seule, comme mediane_kalman.py ; la sortie est déjà projetée à l'instant présent
        ## (_kd + _kv x (t - _kt)). Pourquoi : la prédiction avance _kt à l'instant du manque ; la médiane suivante, datée
        ## au milieu de sa fenêtre (~62 ms plus tôt), échoue alors au test t_milieu > _kt et n'est jamais prise. Échos
        ## alternés avec des manques (virage) : plus aucune mise à jour, la sortie dérive (graine 300923 : 559 -> 0 mm
        ## en 0,35 s alors que le capteur lisait 547 à 559 mm).
        "prediction_sur_echo_manque": bool,
        ## Étape 10 : redémarrer le Kalman sur la mesure si sa vitesse estimée dépasse la vitesse maximale du robot
        ## (ReglagesRobot.vitesse_max_mm_s(), 614 mm/s : hypothèse, #18). Les obstacles sont fixes
        ## (précision du professeur) : la distance ne peut pas varier plus vite que la voiture. Graine 300923 : première
        ## mise à jour sur un bord d'obstacle balayé par le cône en virage (660 -> 572 mm en 62 ms) : vitesse estimée
        ## -1400 mm/s.
        "redemarrer_si_vitesse_impossible": bool,
        ## Étape 10 (14 nov., retenu par l'équipe) : accélération non modélisée du Kalman (mm/s^2), 1000 au lieu de
        ## SIGMA_A_CONFIG_PY (400, config.py) : à 400, le filtre garde la vitesse d'approche pendant le freinage et
        ## l'arrêt part trop tôt (erreur max 40 pistes 11,7 -> 9,3 mm). HYPOTHÈSE réglée en simulation ; à reporter dans
        ## config.py et vérifier.
        "sigma_a_mm_s2": float,
        ## Option d'essai (étape 10, erreur d'arrêt ; non retenue par l'équipe, défaut faux) : prédiction du Kalman et
        ## projection de la sortie avec l'avance mesurée par l'odométrie (obstacle fixe, droit devant) au lieu de la
        ## vitesse estimée. Cause visée (graine 308842, journal) : au freinage, le modèle à vitesse constante garde
        ## ~-200 mm/s alors que la voiture ralentit ; la distance filtrée passe 30 mm sous la vraie et la voiture
        ## s'arrête 12 mm trop loin.
        "projection_odometrie": bool,
    }

    def __init__(self, robot=None):
        self.portee_max_mm = 0.0
        self.absence_s = 0.0
        self.zone_proche_mm = 0.0
        self.absence_proche_s = 0.0
        self.prediction_sur_echo_manque = False
        self.redemarrer_si_vitesse_impossible = False
        self.sigma_a_mm_s2 = 0.0
        self.projection_odometrie = False
        ## Avance cumulée de la voiture selon l'odométrie (mm, > 0 en marche avant), donnée par le Contrôleur à chaque tour.
        self.avance_mm = 0.0

        self._robot = robot
        self._mesures = []
        self._d_filtre = -1.0
        self._t_absence = 0.0
        # État de "mediane_kalman"
        self._t = 0.0                     # temps écoulé (somme des dt)
        self._fenetre = []                # [[instant, mesure mm]] des 3 dernières mesures valides
        self._k_demarre = False
        self._kd = 0.0                    # distance estimée (mm) à l'instant _kt
        self._kv = 0.0                    # vitesse estimée (mm/s, négative si on s'approche)
        self._kt = 0.0
        self._p11 = 0.0
        self._p12 = 0.0
        self._p22 = 0.0
        self._rejets = 0
        self._k_avance = 0.0              # avance_mm à l'instant _kt
        self._avance_maj = 0.0            # avance_mm à l'instant de la mesure donnée à _k_mettre_a_jour

    def reinitialiser(self):
        del self._mesures[:]
        self._d_filtre = -1.0
        self._t_absence = 0.0
        del self._fenetre[:]
        self._k_demarre = False
        self._kv = 0.0
        self._rejets = 0

    def filtrer(self, distance_cm, dt, mesure_nouvelle=True):
        """Distance filtrée en mm, -1 s'il n'y a pas d'obstacle. distance_cm : dernière mesure reçue (-1 = pas d'écho) ;
        mesure_nouvelle : vrai si elle est arrivée depuis le tour précédent (ignoré par "mediane3").
        Un « pas d'écho » n'est accepté qu'après absence_s
        (absence_proche_s si l'obstacle était dans la zone où le capteur devient aveugle)."""
        if self._robot is not None and self._robot.filtre_ultrason == "mediane_kalman":
            return self._filtrer_kalman(distance_cm, dt, mesure_nouvelle)
        mm = distance_cm * 10.0
        if distance_cm >= 0.0 and mm <= self.portee_max_mm:
            self._mesures.append(mm)
            if len(self._mesures) > FiltreUltrason.TAILLE_FILTRE:
                self._mesures.pop(0)
            self._t_absence = 0.0
            tri = list(self._mesures)
            tri.sort()
            # 1 ou 2 mesures : on garde la plus proche (prudent) ; 3 : la médiane
            self._d_filtre = tri[1] if len(tri) >= 3 else tri[0]
        else:
            self._t_absence += dt
            delai = self.absence_proche_s if self._d_filtre >= 0.0 and self._d_filtre < self.zone_proche_mm else self.absence_s
            if self._t_absence >= delai:
                del self._mesures[:]
                self._d_filtre = -1.0
        return self._d_filtre

    def _filtrer_kalman(self, distance_cm, dt, nouvelle):
        """Médiane de 3 + Kalman (mediane_kalman.py) : la médiane des 3 dernières mesures valides est datée au milieu de
        la fenêtre (la médiane de 3 mesures d'une distance régulière vaut celle du milieu), le Kalman la reçoit à cet
        instant, puis la sortie est projetée à l'instant présent : d = d_est + v_est x (t - t_kalman)."""
        self._t += dt
        mm = distance_cm * 10.0
        # Comme en Python : une mesure hors de [D_MIN, D_MAX] est rejetée avant la fenêtre (traitée comme un écho manqué)
        if (distance_cm >= 0.0 and mm <= self.portee_max_mm and mm >= FiltreUltrason.D_MIN_MM
                and mm <= FiltreUltrason.D_MAX_MM):
            self._t_absence = 0.0
            if nouvelle:
                self._fenetre.append([self._t, mm, self.avance_mm])
                if len(self._fenetre) > FiltreUltrason.TAILLE_FILTRE:
                    self._fenetre.pop(0)
                if not self._k_demarre:
                    self._avance_maj = self.avance_mm
                    self._k_mettre_a_jour(mm, self._t)   # démarrage sur la première mesure valide
                elif len(self._fenetre) == FiltreUltrason.TAILLE_FILTRE:
                    tri = [self._fenetre[0][1], self._fenetre[1][1], self._fenetre[2][1]]
                    tri.sort()
                    t_milieu = self._fenetre[1][0]
                    if t_milieu > self._kt:
                        self._avance_maj = self._fenetre[1][2]
                        self._k_mettre_a_jour(tri[1], t_milieu)
        else:
            self._t_absence += dt
            if nouvelle and self._k_demarre and self.prediction_sur_echo_manque:
                self._avance_maj = self.avance_mm
                self._k_mettre_a_jour(-1.0, self._t)     # écho manqué : prédiction seule (comme le Python)
            delai = self.absence_proche_s if self._d_filtre >= 0.0 and self._d_filtre < self.zone_proche_mm else self.absence_s
            if self._t_absence >= delai:
                del self._fenetre[:]
                self._k_demarre = False
                self._kv = 0.0
                self._rejets = 0
                self._d_filtre = -1.0
                return self._d_filtre
        if not self._k_demarre:
            return -1.0
        if self.projection_odometrie:
            self._d_filtre = gd.maxf(self._kd - (self.avance_mm - self._k_avance), 0.0)
        else:
            self._d_filtre = gd.maxf(self._kd + self._kv * (self._t - self._kt), 0.0)
        return self._d_filtre

    def _k_mettre_a_jour(self, z, t):
        """kalman.py KalmanDistance.mettre_a_jour : mesure z (mm, ou -1) prise à l'instant t."""
        valide = z != -1.0 and z >= FiltreUltrason.D_MIN_MM and z <= FiltreUltrason.D_MAX_MM
        r = FiltreUltrason.SIGMA_Z_MM * FiltreUltrason.SIGMA_Z_MM
        if not self._k_demarre:
            if valide:
                self._k_demarrer(z, t)
            return
        self._k_predire(gd.maxf(t - self._kt, 0.0))
        self._kt = t
        self._k_avance = self._avance_maj
        if not valide:
            return
        y = z - self._kd
        s = self._p11 + r
        if y * y / s > FiltreUltrason.SEUIL_REJET:
            self._rejets += 1
            if self._rejets >= FiltreUltrason.REJETS_AVANT_RESET:
                self._k_demarrer(z, t)
            return
        self._rejets = 0
        k1 = self._p11 / s
        k2 = self._p12 / s
        self._kd += k1 * y
        self._kv += k2 * y
        if (self.redemarrer_si_vitesse_impossible and self._robot is not None
                and gd.absf(self._kv) > self._robot.vitesse_max_mm_s()):
            self._k_demarrer(z, t)                 # vitesse physiquement impossible : on repart de la mesure (v = 0)
            return
        p11 = (1.0 - k1) * self._p11
        p12 = (1.0 - k1) * self._p12
        p22 = self._p22 - k2 * self._p12
        self._p11 = p11
        self._p12 = p12
        self._p22 = p22

    def _k_demarrer(self, z, t):
        self._k_demarre = True
        self._kd = z
        self._kv = 0.0
        self._kt = t
        self._k_avance = self._avance_maj
        self._p11 = FiltreUltrason.SIGMA_Z_MM * FiltreUltrason.SIGMA_Z_MM
        self._p12 = 0.0
        self._p22 = FiltreUltrason.V0_ECART_TYPE_MM_S * FiltreUltrason.V0_ECART_TYPE_MM_S
        self._rejets = 0

    def _k_predire(self, tt):
        """Prédiction sur T secondes : P = F P F^T + Q, F = [[1, T], [0, 1]], Q = sigma_a^2 [[T^4/4, T^3/2], [T^3/2, T^2]]."""
        if self.projection_odometrie:
            self._kd -= self._avance_maj - self._k_avance   # avance de la voiture entre _kt et l'instant de la mesure (obstacle fixe)
        else:
            self._kd += self._kv * tt
        q = self.sigma_a_mm_s2 * self.sigma_a_mm_s2
        n11 = self._p11 + tt * (self._p12 + self._p12) + tt * tt * self._p22 + q * gd.pow_(tt, 4.0) / 4.0
        n12 = self._p12 + tt * self._p22 + q * gd.pow_(tt, 3.0) / 2.0
        n22 = self._p22 + q * tt * tt
        self._p11 = n11
        self._p12 = n12
        self._p22 = n22
