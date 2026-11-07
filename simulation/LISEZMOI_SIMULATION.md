
Corrections du 26 octobre 2026
controleur.gd : en cas de perte de ligne, le robot braque du dernier côté non nul où la ligne a été vue. Avant, une dernière lecture centrée le faisait partir tout droit (demi-tour de la piste Courbes 120 raté).
simulation.gd : la voiture reste immobile tant que la bille n'est pas posée et que son modèle n'a pas mesuré la plaquette à l'arrêt. Avant, un démarrage brutal pouvait échapper au modèle de la bille.
simulation.gd : l'accélération affichée est moyennée sur 50 ms (les commandes entières créaient des pics d'un seul pas) ; l'accélération latérale (v² · tan(δ) / L) est aussi affichée.
Guide pas à pas : document « Guide Godot – Comprendre la simulation pas à pas » (Claude Docs).
Modifications du 5 novembre 2026
Découpage de la simulation en modules
simulation.gd faisait tout à la fois. Il ne fait plus qu'assembler les modules et décider qui conduit la voiture (personne tant que la bille n'est pas prête, sinon le contrôleur ou le clavier). Le comportement est inchangé.

simulation.gd : chef d'orchestre, crée les modules, relie les signaux, choisit qui conduit.
couches.gd : couches de collision nommées (Couches.RUBAN, OBSTACLE, SOL, VEHICULE, BILLE).
monde.gd : ciel, soleil, sol 8 × 8 m. Le sol est maintenant un enfant du nœud Monde.
pistes.gd : liste des pistes, chargement (Blender ou aléatoire), réglage des collisions, point de départ.
bille_cuvette.gd : modèle de la bille dans la cuvette, signal chute quand elle tombe. Elle est toujours mise à jour depuis simulation.gd pour garder l'ordre commande → mesure → bille.
mesure_acceleration.gd : accélération moyenne (50 ms) et maximale.
journal_csv.gd : journal CSV des essais (touche L).
entrees.gd : clavier, un signal par touche d'action, commandes de conduite manuelle.
camera_suivi.gd : caméra poursuite ou vue de dessus (touche C).
hud.gd : panneau d'informations.
Les @export (masse et amortissement de la bille, piste initiale, mode) restent sur le nœud Simulation.
Piste aléatoire
Nouvelle piste n° 7, générée à partir d'une matrice. Elle a la même structure qu'une piste Blender (Ruban, Obstacle_n, Depart), donc le reste de la simulation l'utilise sans modification.

Fichiers :

parametres_piste.gd : ressource ParametresPiste, tous les réglages de la piste.
generateur_piste.gd : matrice et chemin aléatoire (sans 3D), affiche la matrice dans la console.
construction_piste.gd : transforme le chemin en ruban 3D (droites et quarts de cercle) et en obstacles.
pistes.gd (modifié) : entrée ["Piste aleatoire", ""], un chemin vide déclenche la génération.
Respect du cahier des charges :

ruban noir de 18 mm (largeur_ruban) ;
courbes de 120 mm et plus : une case mesure 2 × rayon_virage, et ce réglage ne descend pas sous 0,12 m.
Règles de génération :

deux portions du ruban ne se touchent jamais (les sondes ne doivent pas voir une ligne voisine) ;
les 2 premières cases sont droites (départ dans l'axe, cap −Z) ;
un obstacle est toujours posé au milieu d'une ligne droite (2 cases droites avant, 2 après par défaut), avec une case libre de chaque côté pour le contournement ;
même graine = même piste.
Réglages (ParametresPiste) :

graine : 0 = nouvelle piste à chaque fois, sinon toujours la même.
colonnes, lignes : taille de la zone (12 × 12 cases ≈ 2,9 m).
rayon_virage : rayon des courbes (min. 0,12 m).
largeur_ruban : 0,018 m.
longueur_min, longueur_max : longueur du parcours, en cases.
proba_tout_droit : proche de 1 = peu de virages, proche de 0 = sinueux.
nb_obstacles, taille_obstacle : nombre et dimensions des obstacles (10 × 15 × 10 cm par défaut, à mesurer).
ecart_min_obstacles, cases_libres_depart : espacement des obstacles et zone sans obstacle au départ.
cases_droites_avant, cases_droites_apres : longueur de ligne droite exigée autour d'un obstacle (2 et 2).
Utilisation :

inspecteur : propriété Parametres Piste du nœud Simulation, ou fichiers .tres (Nouvelle ressource > ParametresPiste) pour garder plusieurs réglages ;
touches : 7 charge la piste aléatoire, G en génère une nouvelle ; la graine est affichée dans le HUD ;
ligne de commande : godot --path . -- --piste=6 --graine=1234 pour rejouer exactement une piste.
Testé : les pistes générées sont bien différentes à chaque génération.

Découpage du contrôleur en classes
Le contrôleur est réparti en 8 classes, une par exigence. Les calculs sont repris à l'identique, la classe s'appelle toujours Controleur et ses fonctions publiques ne changent pas (calculer(), demarrer_recul(), nom_etat(), reprendre(), reinitialiser()). Aucune dépendance à Godot : la logique reste transposable sur le vrai robot.

Fichiers :

controleur.gd : machine à états (SUIVI, PAUSE, BLOQUE, ARRET, CONTOURNEMENT, RECUL_LIGNE).
suivi_ligne.gd : lecture des 5 sondes et PID du suivi de ligne.
arret_obstacle.gd : arrêt à distance de l'obstacle (± 30 mm), recul si trop près.
contournement.gd : les 8 phases du contournement (SORTIE → … → ALIGNEMENT).
recul_ligne.gd : suivi de ligne en marche arrière (< 200 mm, point virtuel).
filtre_ultrason.gd : médiane des mesures, gestion des « pas d'écho ».
odometrie.gd : position estimée pendant les manœuvres.
reglages_robot.gd : dimensions, moteur et servo partagés.
Le timeout de 10 s s'applique toujours à chaque phase du contournement et au recul. nom_etat() renvoie la phase du contournement en cours (DECALAGE, RETOUR…).

Les réglages sont maintenant rangés par module :

controleur.kp, ki, kd, vitesse_max → controleur.suivi.kp…
controleur.contourner_obstacles → controleur.contournement.actif
controleur.cote_contournement, distance_manoeuvre_cm… → controleur.contournement.…
controleur.distance_arret_cm, decalage_capteur_mm → controleur.arret.…
controleur.recul_auto_mm, attente_avant_recul_s → controleur.recul.distance_auto_mm, attente_auto_s
controleur.mm_s_par_cmd, empattement_mm, rampe_cmd_par_s… → controleur.robot.…
À vérifier : ce découpage n'a pas encore été testé dans Godot. À comparer avec l'ancienne version sur une piste connue (par exemple « Contournement ») avant de la remplacer définitivement.

Suivi de ligne : détection robuste
suivi_ligne.gd : le PID est inchangé, mais une couche de détection décide d'abord si la lecture des sondes est fiable.

VUE : des sondes voisines voient la ligne, l'erreur est le milieu des sondes allumées.
ENTRE_SONDES : plus aucune sonde alors que la ligne n'était pas au bord (ruban de 18 mm passé entre deux sondes espacées de 19 mm, ou lecture fausse). On garde la commande précédente pendant 0,15 s (delai_entre_sondes_s) au lieu de braquer à fond.
PERDUE : braquage maximal du dernier côté vu, comme avant.
Motif incohérent (sondes non voisines, 4 sondes et plus) : lecture ignorée.
controleur.gd passe directement les sondes au suivi ; nom_etat() affiche aussi l'état de la ligne.
Avant : une seule lecture 0 0 0 0 0 erronée faisait braquer au maximum. En simulation Python (virages de 120 mm, 10 pistes, 3 % de lectures fausses par sonde), l'ancien PID termine 1 piste sur 10, la nouvelle version 10 sur 10. Coût : environ 5 % de temps en plus sur une piste propre. Une première idée (loi géométrique « pure pursuit ») a été essayée puis abandonnée : plus lente, sans gain de fiabilité.

Obstacles de la piste aléatoire : correction
Problème observé : après avoir contourné un obstacle, le robot ne retrouvait pas la ligne.

Cause : le générateur vérifiait seulement que la case de l'obstacle était droite. Les cases voisines pouvaient déjà contenir un virage, donc la ligne n'était droite que sur ±120 mm autour de l'obstacle. Or le contournement a besoin d'environ 410 mm de ligne droite avant le centre de l'obstacle (arrêt dans l'axe) et 590 mm après (retour sur la ligne). Le robot revenait là où la ligne avait déjà tourné.

Correction : generateur_piste.gd exige maintenant cases_droites_avant cases entièrement droites avant l'obstacle et cases_droites_apres après. Un avertissement s'affiche si tous les obstacles demandés n'ont pas pu être placés.

Résultats en simulation Python (12 pistes, 2 obstacles demandés par piste) :

règle d'origine : 1 piste terminée sur 12, 8 contournements réussis sur 24 (surtout des collisions et des sorties) ;
1 case droite avant et 1 après : 6 pistes sur 12, 13 contournements sur 18 ;
2 avant et 2 après (valeur retenue) : 12 pistes sur 12, 7 contournements sur 7 ;
2 avant et 3 après : 12 pistes sur 12, 3 contournements sur 3.
Plus la règle est stricte, moins il y a d'obstacles placés : augmenter proba_tout_droit ou la longueur de la piste pour en avoir davantage.

Outils : outils/banc_suivi.py (suivi de ligne) et outils/banc_contournement.py (contournement sur pistes aléatoires). C'est un modèle cinématique simplifié, pas Godot : à confirmer dans la simulation.

Braquage : 45° maximum et servo non instantané
Braquage maximal de 45° : déjà respecté (servo de 45 à 135 = roues à ±45°, angle_roues_max_deg). Le contournement braque à 40° (braquage_manoeuvre), soit un rayon de 167 mm pour l'essieu arrière.
Servo non instantané : nouveau réglage vitesse_servo_deg_s dans reglages_robot.gd (300 °/s par défaut, À MESURER ; 0 = instantané).
odometrie.gd suit l'angle réellement atteint par le servo (servo_estime, suivre_servo() appelé à chaque pas par le contrôleur) au lieu de l'angle commandé.
contournement.gd anticipe chaque changement de phase : il change de braquage un peu avant le cap visé (cap encore pris pendant le retour des roues) et avant le point de virage (distance parcourue pendant que le servo braque).
Avant, l'odométrie croyait les roues braquées dès la commande. Mesures en simulation Python (contournement sur ligne droite, erreur de position à la fin de la manœuvre) :

vitesse de manœuvre 25 : erreur de 30 mm (servo 300 °/s), 46 mm (150 °/s), 67 mm (60 °/s) ; avec la correction, moins de 1 mm ;
vitesse de manœuvre 50 : erreur de 77 mm (300 °/s), 164 mm (150 °/s), échec à 60 °/s ; avec la correction, moins de 1 mm et réussite ;
la marge laissée à l'obstacle redevient celle prévue (environ 33 mm pour marge_laterale_mm = 30).
Outil : outils/mesure_servo.py.

Simulation plus proche du vrai robot
vehicule.gd reproduit maintenant les limites du vrai matériel :

servo à vitesse finie (vitesse_servo_deg_s, 300 °/s) : angle_servo_reel rejoint la commande progressivement, et c'est lui qui fait tourner la voiture ;
zone morte du moteur (zone_morte_moteur) : les commandes plus petites n'ont pas d'effet ;
ultrason en cône (demi_angle_ultrason_deg, ±7,5°) et à cadence limitée (frequence_ultrason_hz, 20 Hz) : entre deux mesures, la distance garde sa valeur ;
lectures fausses des sondes (proba_erreur_sonde) ;
geometrie_mm() donne les distances mesurées sur le modèle 3D (essieu → sondes, essieu → ultrason).
simulation.gd :

frequence_controle_hz (50 Hz) : le contrôleur n'est appelé qu'à la fréquence de la boucle du vrai robot, la commande reste la même entre deux tours ;
_verifier_coherence() : au démarrage, un avertissement signale tout écart entre ReglagesRobot (contrôleur) et Vehicule (modèle) : empattement, distance des sondes, position de l'ultrason, angle des roues, vitesse du servo, vitesse par commande.
reglages_robot.gd : empattement_mm passe de 140 à 138 mm, la valeur mesurée du modèle.

Vérifié en simulation Python : contrôleur appelé à 50 Hz puis à 30 Hz avec un servo à 300 °/s, 10 pistes sur 10 terminées et 10 contournements sur 10 dans les deux cas.

Fiche de mesures sur le vrai robot
Chaque valeur ci-dessous est à mesurer, puis à reporter aux deux endroits indiqués (le contrôleur et le modèle doivent rester identiques).

Distance essieu arrière → barre de sondes : à la règle. ReglagesRobot.distance_sondes_mm ; vérifier que le modèle 3D donne la même valeur (avertissement au démarrage sinon).
Distance essieu arrière → ultrason : à la règle. ReglagesRobot.longueur_avant_mm.
Distance ultrason → avant de la voiture : à la règle. ArretObstacle.decalage_capteur_mm.
Angle réel des roues à 45 et 135 : faire un cercle braquage maximal, mesurer le rayon R de l'essieu arrière, angle = atan(empattement / R). ReglagesRobot.angle_roues_max_deg et Vehicule.angle_roues_max_deg.
Vitesse du servo : filmer le passage de 45 à 135 (téléphone au ralenti), vitesse = 90 / durée. ReglagesRobot.vitesse_servo_deg_s et Vehicule.vitesse_servo_deg_s.
Vitesse réelle : distance parcourue en 2 s aux commandes 25, 50, 75, 100. ReglagesRobot.mm_s_par_cmd et Vehicule.vitesse_max (si la vitesse n'est pas proportionnelle à la commande, le noter : le modèle devra être adapté).
Zone morte du moteur : commande minimale qui fait avancer la voiture posée au sol. Vehicule.zone_morte_moteur.
Temps de réponse du moteur : filmer un départ arrêté → commande 50, temps pour atteindre environ 63 % de la vitesse finale. Vehicule.constante_temps_moteur.
Fréquence de la boucle du programme : mesurer le temps d'un tour (lecture des capteurs, calcul, commande) sur le Pi. Simulation.frequence_controle_hz.
Bruit de l'ultrason : 50 mesures face à un obstacle fixe, écart-type. Vehicule.bruit_ultrason_cm.
Erreurs des sondes : sur ruban et hors ruban, compter les lectures fausses sur 200 mesures. Vehicule.proba_erreur_sonde.
Validation : faire rouler le vrai robot et la simulation sur la même piste, enregistrer les deux journaux CSV (touche L) et comparer vitesse, servo et pertes de ligne. Tant que les courbes ne se ressemblent pas, la simulation n'est pas fidèle.

Plus d'obstacles sur la piste aléatoire
Problème : 0 à 2 obstacles par piste (nb_obstacles valait 2, et les pistes sinueuses offraient rarement 5 cases droites d'affilée).

Changements dans parametres_piste.gd et generateur_piste.gd :

longueur_droite_min (6) : après un virage, le chemin continue tout droit au moins 6 cases si c'est possible. Il faut 6 déplacements alignés pour obtenir une zone d'obstacle (2 cases droites + obstacle + 2 cases droites) ;
pistes plus longues (longueur_min 30, longueur_max 50) sur une grille de 14 × 14 (3,4 m) ;
nb_obstacles = 100 : plus de limite, on place autant d'obstacles que les lignes droites le permettent ;
ecart_min_obstacles = 1 : des obstacles peuvent être sur des cases voisines, pour tester volontairement les cas difficiles.
Mesuré sur 40 pistes (avec un écart minimal de 4) : 3 obstacles par piste en moyenne, jusqu'à 5, et plus aucune piste sans obstacle (avant : 0,5 en moyenne, 21 pistes sur 40 sans obstacle). Avec un écart d'au moins 5 cases : 44 contournements réussis sur 44 (15 pistes, servo 300 °/s, boucle à 50 Hz).

Comportement avec deux obstacles rapprochés (simulation Python, ligne droite, outils/obstacles_proches.py) :

1 case d'écart (240 mm entre centres) : collision. La voiture longe le premier obstacle, puis revient vers la ligne et heurte le second par le côté ;
2 cases (480 mm) : collision. Elle revient sur la ligne juste devant le second obstacle ; l'arrêt d'urgence se déclenche trop tard ;
3 cases (720 mm) et plus : les deux obstacles sont contournés l'un après l'autre.
Cause : le contournement prévoit un seul obstacle ; pendant le retour vers la ligne, il ne vérifie pas si un autre obstacle se trouve sur son chemin. Dans Godot, une collision ne bloque pas la voiture : elle traverse l'obstacle et le HUD affiche « COLLISION ! ».

Piste aléatoire sans contrainte (tous les cas)
But : produire tous les cas que le cahier des charges n'interdit pas, pour trouver les échecs avant le vrai robot. Contraintes gardées : ruban de 18 mm, virages de 120 mm et plus, pas de croisement, 2 cases libres au départ (la voiture est posée dessus).

Nouveaux réglages (parametres_piste.gd), valeurs par défaut :

rayon_virage_max (0,30 m) : chaque piste tire son rayon entre 120 et 300 mm ; la grille est réduite automatiquement pour tenir sur le sol ;
autoriser_cases_voisines (oui) : demi-tours serrés, lignes parallèles à une case d'écart ;
obstacles_en_virage (oui) : obstacles n'importe où sur le ruban, posés au milieu de l'arc en virage et orientés dans le sens de la marche ;
proba_obstacle (0,15) : probabilité qu'une case reçoive un obstacle ; nb_obstacles 100, ecart_min_obstacles 1 ;
taille_obstacle (10 × 15 × 10 cm) à taille_obstacle_max (20 × 20 × 20 cm), tirée pour chaque obstacle ;
decalage_obstacle_max (40 mm) : obstacle décalé par rapport à la ligne ;
longueur_droite_min revient à 1 (aucune contrainte). Le haut de parametres_piste.gd indique les valeurs d'un mode « facile ».
Résultats (simulation Python, outils/banc_tous_cas.py, 30 pistes, boucle 50 Hz, servo 300 °/s, ultrason en cône à 20 Hz) : 0 piste terminée sur 30 (24 collisions, 6 sorties de ligne). Seul le premier échec de chaque piste est compté. Facteurs présents dans les échecs :

obstacle plus grand que 150 mm : 20 (le contournement suppose un obstacle de 100 mm) ;
obstacle en virage : 14 ;
obstacle décalé de plus de 20 mm : 13 ;
virage à moins de 2 cases de l'obstacle : 11 (dont 3 où la ligne n'est pas retrouvée après le contournement) ;
obstacle proche du précédent : 1 ;
obstacle isolé, petit, centré, loin des virages : 1 (à analyser) ;
suivi de ligne sans obstacle proche : 1 (à analyser). Observé aussi : au bout de la ligne, la voiture tourne en rond en cherchant la ligne.
Le suivi de ligne seul passe les demi-tours serrés et les rayons de 120 à 300 mm ; les faiblesses sont dans le contournement.

Refonte de la détection, de la décision et du contournement
Détection (controleur.gd) :

distance de détection adaptée au virage (_portee_utile_mm()) : en virage, l'ultrason vise hors de la piste ; un écho plus loin que la distance où la ligne sort du cône (calculée avec le rayon déduit du braquage moyen) est ignoré ;
nouveau réglage demi_angle_ultrason_deg (7,5°) dans reglages_robot.gd, vérifié contre le modèle au démarrage.
Décision (controleur.gd, état DECISION, ex-PAUSE) :

en virage (braquage moyen au-delà de seuil_virage_deg, 8°), contournement par l'intérieur, là où la ligne revient vers la voiture ; en ligne droite, cote_par_defaut (droite) ;
arrêt à 30 cm au lieu de 20 (distance_manoeuvre_cm) : à 20 cm, le coin avant touchait les obstacles larges en braquant ;
pause réduite à 0,3 s.
Contournement (contournement.gd, réécrit) : 6 phases au lieu de 8, en boucle fermée sur le cap.

ECART : cap d'évitement jusqu'à ce que l'ultrason ne voie plus l'obstacle ; le bord de l'obstacle est alors au plus sur le rayon extrême du cône, d'où sa position latérale ; on s'écarte jusqu'à ce bord + demi-largeur de voiture + marge. La manœuvre s'adapte ainsi à la taille et au décalage de l'obstacle ;
LONGER : parallèle à la ligne jusqu'à avoir dépassé l'obstacle le plus long prévu (longueur_obstacle_max_mm, 200) ; si les sondes voient la ligne (elle revient sous la voiture en virage), on la rejoint ;
RETOUR : cap de retour jusqu'à voir la ligne ; RECHERCHE si on dépasse sa position prévue (virage serré vers elle) ;
CROISEMENT, ALIGNEMENT : inchangés ;
un obstacle vu devant pendant LONGER ou RETOUR relance ECART (obstacles rapprochés ou décalés) ;
manœuvre impossible (trop loin sur le côté, ligne introuvable) : echec, la voiture s'arrête en BLOQUE au lieu de partir au hasard.
Suivi de ligne (suivi_ligne.gd) : abandon_s (3 s) : ligne perdue plus longtemps (fin du parcours), la voiture s'arrête au lieu de tourner en rond.

Résultats (simulation Python, outils/banc_tous_cas.py et outils/controleur_v2.py, 30 pistes sans contrainte, 178 obstacles, boucle 50 Hz, servo 300 °/s) :

ancien contrôleur : 12 obstacles franchis sur 178, 0 piste terminée, 25 collisions ;
nouveau contrôleur : 46 obstacles franchis sur 178, 1 piste terminée, 6 collisions ;
échecs restants : surtout la ligne non retrouvée après un contournement près d'un virage (14 cas), puis 6 collisions (dont un obstacle jamais vu par l'ultrason : en virage et décalé vers l'extérieur, il reste hors du cône ; seul un capteur plus large ou un second capteur peut le voir). Ces pistes cumulent volontairement les cas difficiles (obstacles de 20 cm, décalés, en virage, collés) : seul le premier échec de chaque piste est compté.
Points à surveiller
Les sondes sont espacées de 19 mm pour un ruban de 18 mm : si elles sont modélisées comme des points, le ruban peut passer entre deux sondes (lecture 0 0 0 0 0 alors que la voiture est sur la ligne). À observer sur les pistes aléatoires, surtout dans les virages.
Les gains PID ont été réglés sur les pistes de test ; les pistes aléatoires sinueuses (proba_tout_droit bas) permettront de vérifier qu'ils tiennent.
La taille réelle des obstacles reste à mesurer (taille_obstacle, et largeur_obstacle_mm / longueur_obstacle_mm dans Contournement).
