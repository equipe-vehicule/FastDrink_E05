
## Corrections du 26 octobre 2026

controleur.gd : en cas de perte de ligne, le robot braque du dernier côté non nul où la ligne a été vue. Avant, une dernière lecture centrée le faisait partir tout droit (demi-tour de la piste Courbes 120 raté).
simulation.gd : la voiture reste immobile tant que la bille n'est pas posée et que son modèle n'a pas mesuré la plaquette à l'arrêt. Avant, un démarrage brutal pouvait échapper au modèle de la bille.
simulation.gd : l'accélération affichée est moyennée sur 50 ms (les commandes entières créaient des pics d'un seul pas) ; l'accélération latérale (v² · tan(δ) / L) est aussi affichée.

Guide pas à pas : document « Guide Godot – Comprendre la simulation pas à pas » (Claude Docs).

## Modifications du 5 novembre 2026
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
un obstacle est toujours posé sur une ligne droite de 3 cases, avec une case libre de chaque côté pour le contournement ;
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

Points à surveiller
Les sondes sont espacées de 19 mm pour un ruban de 18 mm : si elles sont modélisées comme des points, le ruban peut passer entre deux sondes (lecture 0 0 0 0 0 alors que la voiture est sur la ligne). À observer sur les pistes aléatoires, surtout dans les virages.
Les gains PID ont été réglés sur les pistes de test ; les pistes aléatoires sinueuses (proba_tout_droit bas) permettront de vérifier qu'ils tiennent.
La taille réelle des obstacles reste à mesurer (taille_obstacle, et largeur_obstacle_mm / longueur_obstacle_mm dans Contournement).
