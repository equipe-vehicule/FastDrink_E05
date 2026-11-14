"""Contournement « fluide » depuis l'arrêt (portage de simulation/contournement_fluide.gd) : changement de voie en S à
courbure continue, longement de l'obstacle, puis retour en S sur la ligne. La trajectoire est fixée au départ et suivie
par poursuite d'un point (pure pursuit) sur l'odométrie. Utilisé après l'arrêt et le recul devant l'obstacle
(Controleur, état RECUL_OBSTACLE).

Profil latéral : polynôme de degré 5, q(t) = 10 t^3 - 15 t^4 + 6 t^5 (vitesse et accélération latérales nulles aux
extrémités : le braquage évolue progressivement, sans saut). Courbure maximale ~ 5,77 * écart / longueur^2, donc pour un
rayon minimal R : longueur = racine(5,77 * écart * R).
Repère : celui de la LIGNE (y le long de la ligne, x vers la droite, x = 0 sur la ligne, y = 0 à l'essieu arrière au
départ). La voiture peut partir à côté de la ligne et de travers (arrêt en sortie de virage) : la sortie part de sa
position latérale réelle (x0) au lieu de supposer x0 = 0 (graine 7920 : départ à 91 mm de la ligne, collision).
Sources : LaValle, Planning Algorithms (chemins et courbure) ; MathWorks, Pure Pursuit Controller.
Hypothèse : la ligne continue tout droit sous l'obstacle (précision du professeur : obstacles sur des lignes droites).

Choix (choisir) : pour chaque rayon de rayons_candidats_mm, du plus doux au plus serré, on calcule la trajectoire et
on REJOUE sa poursuite (#76, _poursuite_libre) : modèle bicyclette, même loi que calculer(), commande du servo entière
et bornée, servo à vitesse_servo_deg_s, un pas par tour de boucle (frequence_controle_hz), depuis la pose et l'angle
du servo au départ. On vérifie que la voiture (rectangle) ne touche pas l'obstacle (rectangle élargi de la tolérance
de centrage et de marge_obstacle_mm). Avant le 14 nov., la courbe IDÉALE était vérifiée : la poursuite, en retard,
coupait la sortie et perdait ~31 mm de marge. Aucun rayon possible : le Controleur utilise le contournement par phases.
"""
from . import constantes
from . import godot as gd
from .geometrie_voiture import GeometrieVoiture
from .godot import Vec2


class ContournementFluide:
    ## Réglages : les valeurs sont dans reglages_algorithmes.json (objet « fluide »), lues au démarrage ; ici : type et
    ## justification seulement.
    REGLAGES = {
        "rayons_candidats_mm": list,
        "marge_laterale_mm": float,
        "marge_obstacle_mm": float,
        "marge_arriere_mm": float,
        "longueur_retour_mm": float,
        "anticipation_mm": float,
        "reprise_avant_fin_mm": float,
        "acceleration_laterale_max": float,
        "vitesse_max_mm_s": float,
    }

    def __init__(self, robot, odo, epreuve):
        # ----- Trajectoire
        ## Rayon minimal de la voiture : 138 mm (45 deg). En dessous de ~1,15 fois cette valeur, la poursuite décroche
        ## (banc du 12 nov. : retour de rayon ~140 mm, voiture perdue). Du plus doux au plus serré.
        self.rayons_candidats_mm = []
        self.marge_laterale_mm = 0.0   # entre le flanc de la voiture et l'obstacle (tolérance de centrage en plus)
        ## Marge de la vérification (une seule valeur, aussi passée à Contournement.choisir_cap). Depuis le 14 nov. (#76),
        ## le retard de la poursuite est simulé : la marge ne couvre plus que les écarts non simulés (odométrie 1 à 2 mm,
        ## position de la face +1,6 mm en moyenne, accélération au départ) et la sécurité voulue. HYPOTHÈSE : valeur de
        ## 20 mm inchangée, son ancienne justification (retard de suivi de 12 mm) ne s'applique plus ; à revoir avec les
        ## essais sur le robot.
        self.marge_obstacle_mm = 0.0
        self.marge_arriere_mm = 0.0    # le retour commence quand l'arrière a dépassé l'obstacle de ça
        ## Retour en quintique : rayon minimal ~ L^2 / (5,77 * écart) ; à 450 mm et 170 mm d'écart : ~210 mm.
        self.longueur_retour_mm = 0.0
        ## Distance du point visé devant l'essieu (pure pursuit). Depuis l'arrêt (graine 39596) : à 250 mm, la voiture
        ## coupait la courbe avec jusqu'à 34 mm de retard et touchait l'obstacle ; à 150 mm, 12 mm de retard au plus,
        ## bille à 0,71.
        ## Historique (14 nov., #76) : avant le rejeu de la poursuite (_poursuite_libre), la marge la plus faible
        ## tombait dans la SORTIE, coin avant contre l'obstacle ; cap réel 27,5 deg pour 34,8 deg prévus, ~31 mm de marge
        ## perdue (D_recul 50, graine 110867 : 42,9 mm prévus, 10,6 mm réels). Essai non retenu par l'équipe,
        ## anticipation à 100 mm : marges min 20,3 mm (D_recul 50) et 28,1 mm (40 pistes) au lieu de 10,6 et 20,0, mais
        ## bille plus haute en idéal (pire_cas_avant_virage 0,64 -> 0,86) et oscillation en droite 3,6 -> 8,6 deg.
        self.anticipation_mm = 0.0
        ## Le PID reprend dès que les sondes voient la ligne dans la 2e moitié du retour.
        ## 0 (attendre la fin) : sur une piste aléatoire, la ligne tournait avant la fin du retour et était perdue ;
        ## 225 testé sans chute de bille (pistes 4 et 7)
        self.reprise_avant_fin_mm = 0.0
        ## Vitesse : la plus grande qui garde l'accélération latérale v^2 / R sous acceleration_laterale_max, plafonnée.
        ## 0,3 m/s^2 : sous la décélération de freinage utilisée en ligne droite (500 mm/s^2, soit 0,5 m/s^2), car la
        ## bille subit aussi les à-coups du braquage.
        self.acceleration_laterale_max = 0.0   # m/s^2
        ## (= ancienne commande 65 à 4 mm/s par commande ; sans chute de bille au banc du 12 nov.)
        self.vitesse_max_mm_s = 0.0

        self.cote = 1
        self.termine = False
        self.fin_sans_ligne = False        # diagnostic : fin du retour sans avoir revu la ligne (côté deviné)
        self.e_reprise = 0.0
        self.vitesse_mm_s = 260.0          # vitesse choisie pour la manœuvre en cours (mm/s)
        self.rayon_choisi_mm = 0.0         # journal / banc d'essai

        self._robot = robot
        self._odo = odo
        self._epreuve = epreuve
        self._ecart = 0.0                  # décalage latéral de l'essieu pendant le longement (mm)
        self._y1 = 0.0                     # fin de la sortie
        self._y2 = 0.0                     # début du retour
        self._y3 = 0.0                     # fin du retour
        self._y_face = 0.0                 # face de l'obstacle (repère de la manœuvre)
        self._x0 = 0.0                     # position latérale au départ, du côté du contournement (mm, > 0 = vers ce côté)
        self._cap0 = 0.0                   # cap au départ par rapport à la ligne (rad, > 0 = vers la droite)

    def ecart_mm(self):
        """Décalage latéral de l'essieu nécessaire pour passer à côté de l'obstacle (mm)."""
        return self._epreuve.largeur_a_contourner_mm() / 2.0 + self._epreuve.tolerance_centrage_mm \
            + self._robot.largeur_voiture_mm / 2.0 + self.marge_laterale_mm

    def choisir(self, d_capteur_mm, cote_voulu, x0_mm=0.0, cap0=0.0):
        """d_capteur_mm : distance de l'obstacle lue par l'ultrason, voiture arrêtée. cote_voulu : 1 = droite, -1 =
        gauche. x0_mm, cap0 : position latérale de l'essieu (> 0 = à droite) et cap (> 0 = vers la droite) par rapport à
        la ligne. Prépare la trajectoire la plus douce qui passe sans toucher l'obstacle. Retourne False si aucune ne
        passe."""
        self.cote = cote_voulu
        self._x0 = self.cote * x0_mm
        self._cap0 = cap0
        for r in self.rayons_candidats_mm:
            self._preparer(d_capteur_mm, r)
            v = self._vitesse_pour(r)
            if self._poursuite_libre(v):
                self.rayon_choisi_mm = r
                self.vitesse_mm_s = v
                return True
        self.rayon_choisi_mm = 0.0
        return False

    def demarrer(self):
        """À appeler juste après un choisir() qui a retourné True (trajectoire préparée, départ dans la pose donnée)."""
        self._odo.reinitialiser()
        self._odo.x = self.cote * self._x0
        self._odo.cap = self._cap0
        self.termine = False
        self.fin_sans_ligne = False

    def _preparer(self, d_capteur_mm, rayon_mm):
        self._ecart = self.ecart_mm()
        # capteur et mesure dans l'axe de la voiture, qui peut être de travers par rapport à la ligne
        self._y_face = (d_capteur_mm + self._robot.longueur_avant_mm) * gd.cos(self._cap0)
        self._y1 = gd.sqrt(5.77 * gd.maxf(gd.absf(self._ecart - self._x0), 1.0) * rayon_mm)
        self._y2 = gd.maxf(self._y1, self._y_face + self._epreuve.profondeur_a_contourner_mm()
                           + self._robot.arriere_voiture_mm + self.marge_arriere_mm)
        self._y3 = self._y2 + self.longueur_retour_mm

    def _rayon_retour_mm(self):
        return gd.div_reel(self.longueur_retour_mm * self.longueur_retour_mm, 5.77 * self._ecart)

    def _vitesse_pour(self, rayon_mm):
        """Vitesse de la manœuvre (mm/s) pour le rayon de sortie rayon_mm (trajectoire déjà préparée)."""
        r_min_m = gd.minf(rayon_mm, self._rayon_retour_mm()) / 1000.0
        v_voulue = gd.sqrt(self.acceleration_laterale_max * r_min_m) * 1000.0   # mm/s
        # Vitesse atteignable par une commande ENTIÈRE (la commande du robot est un entier) : arrondie vers le bas
        cmd_entiere = gd.floorf(self._robot.commande_pour_vitesse(v_voulue))
        return gd.minf(self.vitesse_max_mm_s, self._robot.vitesse_mm_s(cmd_entiere))

    def _poursuite_libre(self, v_mm_s):
        """Rejoue la poursuite de la trajectoire préparée (repère de la manœuvre, côté réel ; l'obstacle est centré sur
        la ligne) et vérifie à chaque pas qu'aucun coin de la voiture n'entre dans l'obstacle élargi, et qu'aucun coin de
        l'obstacle n'entre dans la voiture. Mêmes lois que calculer() et que le Controleur (_appliquer : commande du
        servo arrondie à l'entier, servo vers la commande à vitesse_servo_deg_s, odométrie bicyclette).
        Hypothèses : vitesse v dès le départ (prudent : la vraie voiture accélère, donc parcourt moins de chemin pendant
        que le servo braque) ; poursuite rejouée sur une odométrie exacte (écart réel 1 à 2 mm : couvert par
        marge_obstacle_mm).
        Fin de la vérification quand l'arrière de la voiture a dépassé l'obstacle élargi (plus de contact possible en
        avançant) ou à la fin du retour. Échec si la voiture se met en travers (cap de 90 deg ou plus : elle n'avance
        plus)."""
        if v_mm_s <= 0.0:
            return False                                # la voiture n'avancerait pas : boucle sans fin
        demi = self._epreuve.largeur_a_contourner_mm() / 2.0 + self._epreuve.tolerance_centrage_mm \
            + self.marge_obstacle_mm
        obs_min = Vec2(-demi, self._y_face - self.marge_obstacle_mm)
        obs_max = Vec2(demi, self._y_face + self._epreuve.profondeur_a_contourner_mm() + self.marge_obstacle_mm)
        y_fin = gd.minf(self._y3, obs_max.y + self._robot.avant_voiture_mm() + self._robot.arriere_voiture_mm)
        dt = gd.div_reel(1.0, self._robot.frequence_controle_hz)   # pas = période de la boucle du robot (doit être > 0)
        p = Vec2(self.cote * self._x0, 0.0)             # pose de départ, comme demarrer()
        cap = self._cap0
        servo = self._odo.servo_estime
        while p.y < y_fin:
            angle = gd.roundf(self._angle_poursuite(p, cap))
            servo = angle if self._robot.vitesse_servo_deg_s <= 0.0 \
                else gd.move_toward(servo, angle, self._robot.vitesse_servo_deg_s * dt)
            delta = gd.deg_to_rad(self._robot.servo_vers_roues_deg(servo - 90.0))
            cap += v_mm_s / self._robot.empattement_mm * gd.tan(delta) * dt
            p = p + Vec2(gd.sin(cap), gd.cos(cap)) * v_mm_s * dt
            if gd.absf(cap) >= gd.PI / 2.0:
                return False
            if GeometrieVoiture.touche(p, cap, self._robot.avant_voiture_mm(), self._robot.arriere_voiture_mm,
                                       self._robot.largeur_voiture_mm / 2.0, obs_min, obs_max):
                return False
        return True

    def _x_prevu(self, y):
        """Trajectoire prévue du côté droit (cote = 1)."""
        if y <= 0.0:
            return self._x0
        if y < self._y1:
            return self._x0 + (self._ecart - self._x0) * self._q(gd.div_reel(y, self._y1))
        if y < self._y2:
            return self._ecart
        if y < self._y3:
            return self._ecart * (1.0 - self._q(gd.div_reel(y - self._y2, self._y3 - self._y2)))
        return 0.0

    @staticmethod
    def _q(t):
        t = gd.clampf(t, 0.0, 1.0)
        return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)

    def x_chemin(self, y):
        """Position latérale voulue de l'essieu (mm) à l'abscisse y."""
        return self.cote * self._x_prevu(y)

    def nom_phase(self):
        if self._odo.y < self._y1:
            return "SORTIE"
        if self._odo.y < self._y2:
            return "LONGER"
        return "RETOUR"

    def calculer(self, somme, n):
        """Retourne Vec2(angle servo, vitesse cible). Regarder `termine` ensuite."""
        y = self._odo.y
        if n > 0 and y >= self._y3 - self.reprise_avant_fin_mm:
            self.e_reprise = somme / n
            self.termine = True
            return Vec2(90.0, self.vitesse_mm_s)
        if y >= self._y3 + 150.0:
            # Ligne non revue : elle est du côté opposé à la position latérale de la voiture (x = 0 sur la ligne).
            # Avant : toujours -cote, faux si la voiture a déjà croisé la ligne (elle braquait alors à fond du mauvais
            # côté).
            signe = -gd.signf(self._odo.x) if gd.absf(self._odo.x) > 10.0 else float(-self.cote)
            self.e_reprise = signe * constantes.ERREUR_MAX_MM
            self.fin_sans_ligne = True
            self.termine = True
            return Vec2(90.0, self.vitesse_mm_s)
        return Vec2(self._angle_poursuite(Vec2(self._odo.x, self._odo.y), self._odo.cap), self.vitesse_mm_s)

    def _angle_poursuite(self, p, c):
        """Loi de poursuite (pure pursuit) : angle du servo (deg, 90 = tout droit, borné à +-45) pour l'essieu en p au
        cap c. Utilisée par calculer() et rejouée par _poursuite_libre()."""
        # courbure = 2 * écart latéral / distance^2 vers le point visé
        yt = p.y + self.anticipation_mm
        v = Vec2(self.x_chemin(yt), yt) - p
        lat = v.x * gd.cos(c) - v.y * gd.sin(c)
        av = v.x * gd.sin(c) + v.y * gd.cos(c)
        kappa = 2.0 * lat / gd.maxf(lat * lat + av * av, 1.0)
        delta = gd.rad_to_deg(gd.atan(self._robot.empattement_mm * kappa))
        return 90.0 + gd.clampf(self._robot.roues_vers_servo(delta), -45.0, 45.0)
