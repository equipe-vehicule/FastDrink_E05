"""Ce dont la voiture se souvient de son trajet, dans le repère global de l'odométrie (mm, voir Odometrie) :
  - la ligne vue par les sondes sur le dernier mètre (direction de la ligne, ligne entrante d'une manœuvre) ;
  - le chemin déjà parcouru sur la ligne (4 derniers mètres) : détection du contresens ;
  - les obstacles déjà contournés et les endroits des derniers demi-tours.
Sens de marche : la piste ne se croisant jamais, une voiture qui roule à l'envers finit devant un obstacle qu'elle a
déjà contourné, vu de dos, ou remonte un morceau de ligne déjà parcouru (journal Godot du 6 novembre).

Portage de simulation/memoire_parcours.gd (lot C). Vec2 (float32) partout où le .gd utilise un Vector2.
"""
from . import godot as gd
from .godot import Vec2


class MemoireParcours:
    PAS_MM = 20.0                 # un point mémorisé tous les PAS_MM
    POINTS_LIGNE = 50             # 50 x 20 mm = 1 m de ligne entrante
    POINTS_CHEMIN = 200           # 4 m de chemin parcouru
    POINTS_CHEMIN_RECENTS = 15    # les derniers points (300 mm) ne comptent pas : la voiture vient d'y passer
    DEMI_TOURS_MEMORISES = 6

    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « memoire »), lues au démarrage ; ici : type
    ## et justification seulement.
    REGLAGES = {
        "rayon_chemin_mm": float,               # ligne vue à moins de ça d'un point déjà parcouru : c'est ce morceau
        "rayon_meme_ligne_mm": float,           # ligne vue à moins de ça de la ligne mémorisée : c'est la ligne entrante
        "rayon_meme_obstacle_mm": float,        # écho à moins de ça d'un obstacle déjà contourné : c'est lui, vu de dos
        "obstacles_memorises": int,
        "rayon_meme_demi_tour_mm": float,
        "demi_tours_max_au_meme_endroit": int,  # ensuite on continue : tourner sur place ne résoudrait rien
        "points_direction_max": int,            # 20 x 20 mm = 400 mm au plus (voir derniere_droite)
        "tolerance_droite_mm": float,           # dispersion des points (sondes espacées de 19 mm : +/- 9,5 mm) + marge
    }

    def __init__(self, robot, odo):
        self.rayon_chemin_mm = 0.0
        self.rayon_meme_ligne_mm = 0.0
        self.rayon_meme_obstacle_mm = 0.0
        self.obstacles_memorises = 0
        self.rayon_meme_demi_tour_mm = 0.0
        self.demi_tours_max_au_meme_endroit = 0
        self.points_direction_max = 0
        self.tolerance_droite_mm = 0.0

        self.ligne = []                                  # Vec2 : points de la ligne vus par les sondes (dernier mètre)
        self.obstacles = []                              # Vec2 : centres des obstacles contournés
        # dernier obstacle évité alors que la voiture n'était pas sur la ligne
        self.obstacle_hors_ligne = Vec2(gd.INF, gd.INF)

        self._chemin = []                                # Vec2
        self._points_passage = 0                         # points ajoutés à _chemin depuis la dernière reprise du suivi
        self._demi_tours = []                            # Vec2
        self._robot = robot
        self._odo = odo

    # ----- Ligne et chemin parcouru

    def point_sous_sondes(self, e_mm):
        """Point de la ligne sous les sondes (repère global) : la barre est à distance_sondes_mm devant l'essieu,
        e_mm à droite."""
        d = self._robot.distance_sondes_mm
        c = self._odo.cap_global
        return Vec2(self._odo.x_global + d * gd.sin(c) + e_mm * gd.cos(c),
                    self._odo.y_global + d * gd.cos(c) - e_mm * gd.sin(c))

    def memoriser(self, e_mm):
        """Garde le point de la ligne vu par les sondes, un tous les PAS_MM."""
        p = self.point_sous_sondes(e_mm)
        if not self.ligne or p.distance_to(self.ligne[-1]) > self.PAS_MM:
            self.ligne.append(p)
            if len(self.ligne) > self.POINTS_LIGNE:
                self.ligne.pop(0)
            self._chemin.append(p)
            self._points_passage += 1
            if len(self._chemin) > self.POINTS_CHEMIN:
                self._chemin.pop(0)

    def nouveau_passage(self):
        """Nouveau passage en suivi : les points suivants pourront être annulés (annuler_passage)."""
        self._points_passage = 0

    def annuler_passage(self):
        """Oublie le passage en cours : il a été enregistré à contresens et retournerait la voiture sans fin."""
        del self._chemin[gd.maxi(0, len(self._chemin) - self._points_passage):]
        self._points_passage = 0

    def sens_sur_chemin(self, e_mm):
        """Accord entre le cap et le sens dans lequel ce morceau de ligne a déjà été parcouru (1 = même sens,
        -1 = inverse). NAN si la ligne sous les sondes n'est pas sur le chemin déjà parcouru."""
        q = self.point_sous_sondes(e_mm)
        avant = Vec2(gd.sin(self._odo.cap_global), gd.cos(self._odo.cap_global))
        for i in range(len(self._chemin) - self.POINTS_CHEMIN_RECENTS - 1, 2, -1):
            if q.distance_squared_to(self._chemin[i]) < self.rayon_chemin_mm * self.rayon_chemin_mm:
                sens = self._chemin[i] - self._chemin[i - 3]
                return sens.normalized().dot(avant) if sens.length() > 1.0 else gd.NAN
        return gd.NAN

    def remonte_le_chemin(self, e_mm):
        """Vrai si la voiture remonte un morceau de ligne déjà parcouru, en sens inverse."""
        accord = self.sens_sur_chemin(e_mm)
        return not gd.is_nan(accord) and accord < -0.5

    def en_avant_sur_chemin(self, somme, n):
        """Vrai si la voiture roule dans le bon sens sur un morceau déjà parcouru. Devant un obstacle déjà contourné,
        elle doit alors le recontourner : un demi-tour la renverrait à contresens, puis la surveillance la
        retournerait, sans fin."""
        if n == 0:
            return False
        accord = self.sens_sur_chemin(somme / n)
        return not gd.is_nan(accord) and accord > 0.3

    def sur_ligne_a_contresens(self, e_mm):
        """Vrai si la ligne vue est la ligne mémorisée et que la voiture y roule à contresens."""
        q = self.point_sous_sondes(e_mm)
        cap = self._odo.cap_global
        for i in range(len(self.ligne) - 1, 0, -1):
            if q.distance_to(self.ligne[i]) < self.rayon_meme_ligne_mm:
                sens = self.ligne[i] - self.ligne[gd.maxi(0, i - 5)]
                return sens.length() > 1.0 and sens.normalized().dot(Vec2(gd.sin(cap), gd.cos(cap))) < -0.3
        return False

    def sens_ligne(self):
        """Sens de marche sur la ligne, d'après les derniers points (environ 100 mm). Pour viser la ligne, pas pour
        la mesurer."""
        if len(self.ligne) >= 2:
            sens = self.ligne[-1] - self.ligne[gd.maxi(0, len(self.ligne) - 6)]
            if sens.length() > 1.0:
                return sens.normalized()
        return Vec2(gd.sin(self._odo.cap_global), gd.cos(self._odo.cap_global))

    def derniere_droite(self):
        """Direction de la dernière partie DROITE de la ligne, orientée dans le sens de marche : moindres carrés
        (axe principal) sur les derniers points, en remontant tant que tous restent à moins de tolerance_droite_mm de
        la droite ajustée (au-delà, la ligne tournait : ces points sont dans le virage précédent).
        sens_ligne() (2 points à 100 mm, sondes à +/- 9,5 mm près) se trompe jusqu'à ~10 deg. Obstacle juste après un
        virage (banc du 12 nov., graine 7920) : un ajustement sur 400 mm mélangeait virage et droite.
        Retourne [centre, direction] : un point de la droite (centre des points retenus) et sa direction unitaire."""
        retenus = self.ligne[gd.maxi(0, len(self.ligne) - 2):]
        meilleure = self.sens_ligne()
        for k in range(5, gd.mini(self.points_direction_max, len(self.ligne)) + 1):
            pts = self.ligne[len(self.ligne) - k:]
            d = self._axe_principal(pts)
            if self._ecart_max(pts, d) > self.tolerance_droite_mm:
                break
            meilleure = d
            retenus = pts
        return [self._centre(retenus), meilleure]

    @staticmethod
    def _centre(pts):
        m = Vec2.ZERO
        for p in pts:
            m = m + p
        return m / float(gd.maxi(1, len(pts)))

    @staticmethod
    def _axe_principal(pts):
        """Axe principal (moindres carrés) des points, orienté du premier vers le dernier."""
        m = MemoireParcours._centre(pts)
        sxx = 0.0
        syy = 0.0
        sxy = 0.0
        for p in pts:
            q = p - m
            sxx += q.x * q.x
            syy += q.y * q.y
            sxy += q.x * q.y
        theta = 0.5 * gd.atan2(2.0 * sxy, sxx - syy)
        dir = Vec2(gd.cos(theta), gd.sin(theta))
        return -dir if dir.dot(pts[-1] - pts[0]) < 0.0 else dir

    @staticmethod
    def _ecart_max(pts, d):
        """Plus grande distance d'un point à la droite de direction d passant par le centre des points (mm)."""
        m = MemoireParcours._centre(pts)
        normale = Vec2(-d.y, d.x)
        e = 0.0
        for p in pts:
            e = gd.maxf(e, gd.absf((p - m).dot(normale)))
        return e

    def vers_repere_manoeuvre(self):
        """La ligne mémorisée, exprimée dans le repère de la manœuvre qui commence (origine : essieu arrière, y vers
        l'avant, x vers la droite) : c'est la « ligne entrante » du contournement."""
        c = self._odo.cap_global
        avant = Vec2(gd.sin(c), gd.cos(c))
        droite = Vec2(gd.cos(c), -gd.sin(c))
        origine = Vec2(self._odo.x_global, self._odo.y_global)
        res = []
        for p in self.ligne:
            res.append(Vec2((p - origine).dot(droite), (p - origine).dot(avant)))
        return res

    # ----- Obstacles contournés et demi-tours

    def noter_obstacle(self, centre):
        self.obstacles.append(centre)
        if len(self.obstacles) > self.obstacles_memorises:
            self.obstacles.pop(0)

    def obstacle_deja_contourne(self, d_mm):
        """Vrai si l'écho à d_mm devant tombe sur un obstacle déjà contourné : la voiture le revoit de dos, elle roule
        à l'envers."""
        echo = self._odo.point_devant_global(self._robot.longueur_avant_mm + d_mm)
        for o in self.obstacles:
            if echo.distance_to(o) < self.rayon_meme_obstacle_mm:
                return True
        return False

    def noter_demi_tour(self):
        self._demi_tours.append(Vec2(self._odo.x_global, self._odo.y_global))
        if len(self._demi_tours) > self.DEMI_TOURS_MEMORISES:
            self._demi_tours.pop(0)

    def demi_tour_autorise(self):
        """Faux après plusieurs demi-tours au même endroit : la voiture tournerait sur place indéfiniment."""
        ici = Vec2(self._odo.x_global, self._odo.y_global)
        deja = 0
        for p in self._demi_tours:
            if ici.distance_to(p) < self.rayon_meme_demi_tour_mm:
                deja += 1
        return deja < self.demi_tours_max_au_meme_endroit
