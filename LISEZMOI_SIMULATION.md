
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
controleur.mm_s_par_cmd (depuis l'étape 9 : mode idéal seulement, voir étape 9), empattement_mm, deceleration_freinage_mm_s2, acceleration_montee_mm_s2… → controleur.robot.…
À vérifier : ce découpage n'a pas encore été testé dans Godot. À comparer avec l'ancienne version sur une piste connue (par exemple « Contournement ») avant de la remplacer définitivement.

Points à surveiller
Les sondes sont espacées de 19 mm pour un ruban de 18 mm : si elles sont modélisées comme des points, le ruban peut passer entre deux sondes (lecture 0 0 0 0 0 alors que la voiture est sur la ligne). À observer sur les pistes aléatoires, surtout dans les virages.
Les gains PID ont été réglés sur les pistes de test ; les pistes aléatoires sinueuses (proba_tout_droit bas) permettront de vérifier qu'ils tiennent.
La taille réelle des obstacles reste à mesurer (taille_obstacle, et largeur_obstacle_mm / longueur_obstacle_mm dans Contournement).


## Modifications du 12 novembre 2026

Voir `SUIVI_SIMULATION.md` pour les mesures, les limites et la suite.

controleur.gd : état RATTRAPAGE (ligne perdue en virage serré : arrêt, marche arrière braquée à l'opposé jusqu'à revoir la ligne, arrêt, reprise) ; arrêt en fin de ligne (BLOQUE) ; recul en fin de ligne (`recul_fin_ligne_mm`) ; état CONTOURNEMENT_FLUIDE avec repli sur le contournement par phases.
contournement_fluide.gd (nouveau) : changement de voie en S à courbure continue (polynôme de degré 5), suivi par poursuite d'un point (pure pursuit), tracé dans le repère de la ligne, sans arrêt devant l'obstacle.
suivi_ligne.gd : vitesse maximale 55. Les options d'essai `loi_table` et `ralentir_virage` ont été retirées lors de la restructuration (résultats dans SUIVI_SIMULATION.md).
contournement.gd : vitesse de manœuvre 40, pause 0,15 s, profondeur d'obstacle 100 mm.
recul_ligne.gd : délai de grâce avant de voir la ligne (recul lancé après la fin de la ligne).
simulation.gd : mode par piste (« Arrêt devant obstacle » : sans contournement ; « Recul » : recul de 180 mm en fin de ligne), `journal_au_demarrage`, `reglages_essai`.
journal_csv.gd : colonne `collision`.
piste_demo.tres (nouveau) : piste aléatoire conforme au cahier des charges (pas d'épingle ni d'obstacle en virage, obstacles de 200 × 60 mm), graine 884064.

## Corrections du 12 novembre (après le banc d'essai)

- `controleur.gd` : reprise douce après toute manœuvre (1 s, vitesse ≤ 25, braquage ≤ 120 °/s) ; cap de départ du contournement fluide par moindres carrés (`MemoireParcours.direction_ajustee`) ; compteur `raisons_repli` (diagnostic seulement).
- `contournement_fluide.gd` : `ecart_mm` 210 → 250 (décalage de l'obstacle compté) ; ligne cherchée du côté réel si elle n'est pas revue.
- `banc_essai.gd` : colonne `raisons_repli`.
- Résultat sur 40 pistes aléatoires : 0 chute, 0 collision, 0 demi-tour. Détails et justifications : `SUIVI_SIMULATION.md`, section 7.

## Restructuration du code (12 novembre)

`controleur.gd` (759 lignes) a été découpé : il ne contient plus que la machine à états (quel état, quelle transition). Chaque manœuvre est dans son propre fichier, sans dépendance à Godot, pour être portée telle quelle en Python. Le comportement est identique : le banc d'essai (40 pistes) donne exactement les mêmes résultats avant et après.

| Fichier | Rôle |
| --- | --- |
| `controleur.gd` | Machine à états : choisit l'algorithme qui commande à chaque pas |
| `suivi_ligne.gd` | Lecture des 5 sondes, PID, détection de la ligne et de l'arrivée |
| `arret_obstacle.gd` | Arrêt à distance prédéterminée de l'obstacle |
| `reglages_epreuve.gd` | Ce qu'impose l'épreuve : distances d'arrêt et de recul, cotes de l'obstacle (un seul endroit à modifier) |
| `contournement_fluide.gd` | Contournement en S lisse, depuis l'arrêt après le recul |
| `contournement.gd` | Contournement par phases (ECART, LONGER, RETOUR…), une fonction par phase |
| `rattrapage.gd` | Virage serré raté : marche arrière ; détection de la fin de ligne |
| `retrouver_ligne.gd` | Ligne perdue longtemps : aller où elle doit être, puis spirale |
| `demi_tour.gd` | Pivot d'environ 180° quand la voiture roule à l'envers |
| `recul_ligne.gd` | Recul en suivant la ligne (< 200 mm) |
| `reprise_douce.gd` | Vitesse et braquage limités 1 s après une manœuvre (bille) |
| `memoire_parcours.gd` | Ligne vue, chemin parcouru, obstacles contournés, demi-tours (repère global) |
| `filtre_ultrason.gd`, `odometrie.gd`, `reglages_robot.gd` | Mesures et dimensions du robot |
| `banc_essai.gd` | Essais automatiques sur N pistes aléatoires (`journaux/banc_*.csv`) |

Réglages renommés (pour `reglages_essai` dans l'inspecteur du nœud Simulation, ou `controleur.<module>.<réglage>`) :

| Avant | Maintenant |
| --- | --- |
| `reprise_douce_s`, `vitesse_reprise`, `braquage_reprise_deg_s` | `reprise:duree_s`, `reprise:vitesse_max_mm_s`, `reprise:braquage_max_deg_s` |
| `rattrapage_actif`, `rattrapage_apres_s`, `vitesse_rattrapage`, `rattrapage_max_mm`, `rattrapages_max` | `rattrapage:actif`, `rattrapage:apres_s`, `rattrapage:vitesse_mm_s`, `rattrapage:distance_max_mm`, `rattrapage:tentatives_max` |
| `distance_fluide_mm`, `confirmations_fluide`, `distance_fluide_min_mm` | `fluide:distance_declenchement_mm`, `fluide:confirmations`, `fluide:distance_min_mm` |
| `retrouver_apres_s`, `vitesse_retrouver`… | `retrouver:apres_s`, `retrouver:vitesse_mm_s`… |
| `vitesse_demi_tour`, `demi_tour_avant_s`… | `demi_tour:vitesse_mm_s`, `demi_tour:duree_avant_s`… |
| `obstacles_memorises`, `rayon_meme_obstacle_mm`… | `memoire:obstacles_memorises`, `memoire:rayon_meme_obstacle_mm`… |

Retiré : `fluide_en_roulant` (variante du contournement fluide avec arrêt, jamais retenue), `loi_table` et `ralentir_virage` (suivi de ligne), et les 3 fichiers `simulation.tscn*.tmp` (temporaires de Godot, maintenant ignorés par `.gitignore`).

## Séquence arrêt → recul → contournement (12 novembre, précisions du professeur)

Devant chaque obstacle, la voiture s'arrête à D_arret, recule de D_recul en suivant la ligne, puis contourne. Les valeurs officielles seront données plus tard : on les change à un seul endroit, `simulation/reglages_epreuve.gd`.

| Réglage (`ReglagesEpreuve`) | Valeur de test | Rôle |
| --- | --- | --- |
| `distance_arret_mm` | 100 | Avant de la voiture → face de l'obstacle (guide : ± 30 mm) |
| `distance_recul_mm` | 150 | Recul en suivant la ligne (guide : < 200 mm, `RECUL_MAX_MM`) |
| `pause_avant_recul_s`, `pause_avant_contournement_s` | 0,3 | Immobile entre deux mouvements (bille) |
| `largeur_obstacle_mm`, `hauteur_obstacle_mm`, `profondeur_obstacle_mm` | 100, 150, 50 | Obstacle réel |
| `tolerance_centrage_mm` | 10 | Obstacle posé à la main : écart possible à la ligne |

Pour essayer d'autres valeurs sans toucher au code : `reglages_essai` du nœud Simulation, par exemple `{"epreuve:distance_arret_mm": 150, "epreuve:distance_recul_mm": 190}`.

Ce qui a changé :

- `controleur.gd` : nouvel état `RECUL_OBSTACLE` (arrêt → pause → recul → pause → contournement). Le contournement part de la position réelle de la voiture par rapport à la ligne, calculée avec la ligne mémorisée (`_pose_par_rapport_a_la_ligne`), même si la voiture s'est arrêtée de travers. `place_avant_obstacle_mm()` donne la ligne droite occupée par l'arrêt et le recul.
- `contournement_fluide.gd` : `choisir(d, côté, x0, cap0)` choisit le rayon le plus doux qui passe sans toucher l'obstacle (vérification géométrique voiture / obstacle), en partant de la position réelle. Sinon, repli sur le contournement par phases.
- `recul_ligne.gd` : si la ligne sort des sondes pendant le recul, la voiture finit la distance tout droit (`finir_sans_ligne`).
- `memoire_parcours.gd` : `derniere_droite()` ajuste une droite sur la dernière partie droite de la ligne vue (sans les points du virage).
- Pistes aléatoires (`generateur_piste.gd`, `parametres_piste.gd`) : obstacle de 100 × 150 × 50 mm, centré à ± 10 mm, seulement sur les lignes droites, juste avant ou après un virage possible, obstacles enchaînés possibles. Ligne droite avant l'obstacle = `place_avant_obstacle_mm()` (D_arret + 193 mm capteur → essieu + D_recul) + `marge_droite_avant` (140 mm, hypothèse : le temps de s'aligner en sortie de virage). Elle suit donc D_arret et D_recul.
- Pistes de test `test_01_arret` et `test_04_contournement` : obstacle remis aux vraies cotes (100 × 50 × 150 mm, avant 200 × 60 × 200) dans les .blend, réexportées.
- `banc_essai.gd` : nouvelles colonnes `arrets`, `erreur_arret_max_mm`, `recul_min_mm`, `recul_max_mm`, `pertes_ligne_apres_contournement`, `perte_ligne_apres_max_s`, `perte_ligne_ailleurs_max_s`, `fluide_fin_sans_ligne`.

Réglages retirés ou déplacés : `arret.distance_arret_cm` → `epreuve:distance_arret_mm` ; `recul.distance_auto_mm`, `recul.attente_auto_s` (retirés) ; `contournement.pause_s`, `largeur_obstacle_min_mm`, `longueur_obstacle_max_mm` → cotes de `ReglagesEpreuve` ; `parametres_piste` : `obstacles_en_virage`, `ecart_min_obstacles`, `cases_droites_avant/apres`, `droite_avant_min` → `marge_droite_avant`.

Résultats (banc d'essai, 40 pistes aléatoires, D_arret 100 / D_recul 150) : 40/40 arrivées, 0 collision, 0 chute de bille, arrêt à 6 mm près au plus, recul de 150 mm à chaque obstacle, 0 perte de ligne après un contournement. Détails : `SUIVI_SIMULATION.md`, section 9.

## Fichier unique des valeurs du robot (13 novembre, tâche #63, partie robot)

Toutes les valeurs physiques du robot sont maintenant dans **`reglages_robot.json`** (racine du dépôt). Le fichier est lu par la simulation, et le sera plus tard par le code Python du robot avec `json.load`. Chaque entrée porte une valeur, une unité, un statut (`provisoire`, `mesuré` ou `hypothèse`), une source et une tâche. Il y a 35 entrées (étape 9 : 28 provisoires, 3 mesurées, 4 hypothèses) : géométrie, moteur, direction, ultrason, sondes, bille et cuvette, fréquence de la boucle. La liste des mesures à faire est dans `MESURES_ROBOT.md`.

Doublons supprimés :
- vitesse : `ReglagesRobot.mm_s_par_cmd` et `Vehicule.vitesse_max` ; (depuis l'étape 9 : mode idéal seulement, voir étape 9)
- braquage, servo et cône de l'ultrason : `reglages_robot.gd` et `vehicule.gd` ;
- empattement et largeur de la voiture : `vehicule.gd`, `contournement.gd`, `contournement_fluide.gd` ;
- `ArretObstacle.decalage_capteur_mm` ;
- les exports `masse_bille`, `amortissement_bille` et `frequence_controle_hz` du nœud Simulation ;
- les constantes de `bille_cuvette.gd`.

Ce qui a changé :

- `simulation/fichier_reglages.gd` (nouveau, chargé par `preload`) : il lit le JSON, vérifie que chaque entrée a ses 5 champs et que le statut est connu, puis rend `{clé: valeur}`.
- `reglages_robot.gd` : il ne contient plus aucune valeur du robot. `appliquer(valeurs)` les remplit, et la table `CLES_JSON` fait la correspondance entre ses champs et les clés du fichier, à un seul endroit. Nouveaux champs : `ultrason_avant_mm`, `largeur_voiture_mm`, `arriere_voiture_mm`, et `avant_voiture_mm()` (calculé : 193 + 36). Les réglages des algorithmes (rampe, freinage) restent dans ce fichier.
- `vehicule.gd`, `bille_cuvette.gd` : `regler(valeurs)`, appelé avant `add_child`. Les constantes `EMPATTEMENT`, `RAYON_ROUE`, `ENCOMBREMENT`, `CENTRE_ENCOMBREMENT`, `RAYON`, `R_CUVETTE`, `R_BORD` deviennent des variables (`empattement`, `rayon_roue`, `encombrement`, `centre_encombrement`, `rayon`, `r_cuvette`, `r_bord`).
- `simulation.gd` :
  - il lit le fichier au lancement ;
  - `_verifier_coherence()` ne compare plus que le fichier aux positions des pièces du modèle 3D (sondes, ultrason) ;
  - une clé de `reglages_essai` qui ne correspond à rien est maintenant **signalée** (avant, `set_indexed` l'ignorait sans rien dire).

Essais sans toucher au fichier : `reglages_essai` accepte `"robot:<clé du JSON>"`. Exemple : `{"robot:vitesse_servo_deg_s": 200}` change la valeur pour le modèle **et** pour le contrôleur. Les anciennes clés `arret:decalage_capteur_mm` et `contournement_fluide:largeur_voiture_mm` deviennent `robot:ultrason_avant_mm` et `robot:largeur_mm`.

### Banc d'essai sans éditeur

```
Godot_v4.7-stable_win64.exe --headless --path <dépôt> --fixed-fps 240 res://simulation/simulation.tscn -- --banc=40 --graine-depart=1 --vitesse-temps=4 --quitter
```

- Il faut donner la scène `res://simulation/simulation.tscn` : la scène principale du projet (`main.tscn`, code de départ du cours) attend une connexion réseau et ne se termine jamais.
- Durée : environ 70 s pour 40 pistes. Plus besoin de modifier les exports du nœud Simulation, puis de les remettre.
- Le résultat est **déterministe** : deux lancements donnent le même CSV. Avec `--vitesse-temps=4`, il est identique au banc lancé dans l'éditeur le 12 nov. (`journaux/banc_2026-11-12T15-49-28.csv`).
- **À savoir** : `vitesse_temps` (`Engine.time_scale`) agrandit aussi le pas de la physique. À 4, la simulation avance par pas de 1/60 s au lieu de 1/240 s. Les bancs passés ont tous été faits ainsi. Avec `--vitesse-temps=1`, les résultats diffèrent un peu : sur les 3 premières pistes, arrêt à 3,6 mm au lieu de 5,5 mm et recul de 149,7 mm au lieu de 150,0 mm. **Choix fait à l'étape 3 : le pas de 1/240 s est la référence** (section suivante).

### Vérification : comportement identique

- 32 valeurs lues dans le fichier et converties, comparées aux anciens littéraux du code : 0 écart (égalité exacte, types compris).
- Banc de 40 pistes (graine 1, `--vitesse-temps=4`) avant et après : **CSV identique ligne pour ligne**. Résultat : 40/40 arrivées, 0 collision, 0 chute.
- Banc 100/50 de 15 pistes : identique au banc du 12 nov. (`journaux/banc_2026-11-12T15-49-28.csv` pour 100/150 ; `banc_2026-11-12T16-14-03.csv` pour 100/50). Ce banc fait passer le contournement par phases 6 fois, avec la collision connue sur la graine 71272 (#72). Fichier JSON relu aussi en Python (`json.load`) : 28 entrées.
- Essais ciblés :
  - `robot:vitesse_servo_deg_s` = 200 est bien lu par le modèle, le contrôleur et l'odométrie ;
  - une clé inconnue et une ancienne clé sont signalées ;
  - la piste « Courbes de 120 mm » roule normalement.

Écarts relevés (valeurs inchangées, à trancher par la mesure) :
- dépassement arrière : 45 mm (LISEZMOI Blender) contre 56 mm selon la boîte d'encombrement ;
- ultrason → avant du robot : 36 mm contre 35,3 dans `config.py` (branche `main`) ;
- cadence de l'ultrason : 20 mesures/s alors que le manuel conseille au plus 16,7 ;
- rayon de la bille : non donné par le plan du support.

## Essais rejouables, boucle à 50 Hz et mesures de stabilité (13 novembre, étape 3)

### Pas de référence : 1/240 s, boucle du programme à 50 Hz

- **Nouvelle référence** : la physique avance par pas de 1/240 s (`--vitesse-temps=1`, valeur par défaut). Les bancs du 12 et du 13 novembre faits dans l'éditeur avec `vitesse_temps` = 4 tournaient par pas de 1/60 s : leurs chiffres ne sont plus comparables directement.
- **Boucle du programme corrigée** (`simulation.gd`) :
  - Avant, le compteur était remis à 0 à chaque tour. La boucle « 50 Hz » tournait donc à 48 Hz au pas de 1/240 s, et à 30 Hz au pas de 1/60 s.
  - Il y a maintenant deux compteurs :
    - `_t_cadence` décide quand le contrôleur tourne. On lui retire la période, ce qui donne exactement 50 Hz en moyenne, en alternant 4 et 5 pas, comme la gigue d'un vrai programme.
    - `_t_ecoule` est le temps réellement écoulé : c'est lui que reçoit le contrôleur (odométrie, PID).
- **Chaque cas du banc démarre de la même façon.** Le premier cas est chargé comme les suivants, au début d'un pas physique, et les compteurs de la boucle sont remis à 0. Avant, le premier cas partait un pas plus tôt que les autres. Le même scénario donnait donc 3,4 mm d'erreur d'arrêt en première position et 3,7 mm ensuite.
- **Chiffres** (banc de 40 pistes, graine 1, D_arret 100 / D_recul 150) :

| Banc | Arrivées | Collisions | Bille max | Erreur d'arrêt max | Recul |
| --- | --- | --- | --- | --- | --- |
| Pas de 1/60 s, boucle à 30 Hz (référence du 13 nov. matin) | 40/40 | 0 | 0,88 | 5,8 mm | 150,0–150,1 mm |
| Pas de 1/240 s, boucle à 48 Hz | 40/40 | 0 | 0,78 | 3,8 mm | 149,7–149,8 mm |
| **Pas de 1/240 s, boucle à 50 Hz (nouvelle référence)** | 40/40 | 0 | 0,77 | 3,9 mm | 149,7–150,3 mm |

- La référence est gardée dans **`essais/reference_1-240.csv`** (dossier suivi par git ; `journaux/` ne l'est pas). Les scénarios de référence sont dans `essais/scenarios_reference.csv`.

Commandes (aucun export du nœud Simulation à modifier, puis à remettre) :

```
Godot_v4.7-stable_win64.exe --headless --path <dépôt> --fixed-fps 240 res://simulation/simulation.tscn -- --banc=40 --quitter
Godot_v4.7-stable_win64.exe --headless --path <dépôt> --fixed-fps 240 res://simulation/simulation.tscn -- --scenarios=tous --quitter
```

- `--banc=40` : environ 5 min ; `--scenarios=tous` : environ 2 min.
- `--scenarios=nom1,nom2` : seulement quelques cas.
- `--reglage=<chemin>=<valeur>` (option répétable) : même effet qu'une entrée de `reglages_essai`, ex. `--reglage=suivi:kp=5.4`.
- Le nom du CSV contient le numéro du processus : deux essais lancés en parallèle n'écrivent pas dans le même fichier.

### Scénarios courts et rejouables (`simulation/scenarios.gd`)

- **Format d'un scénario** :
  - un chemin en lettres, une par déplacement depuis la case de départ : T (tout droit), G (gauche), D (droite) ;
  - le rayon des virages ;
  - les obstacles, placés par tronçon droit. La face avant est placée au plus tôt (`"min"`) ou au plus tard (`"max"`) selon la même règle que le générateur, juste derrière l'obstacle précédent (`"apres_precedent"`), ou à une distance donnée en mm ;
  - des réglages du contrôleur propres au cas.
- **Construction** : `GenerateurPiste.depuis_scenario()`, sans hasard. Le calcul de position (`_ajouter_obstacle`) est partagé avec le générateur aléatoire, dont les résultats ne changent pas.
- **Règles** :
  - les clés `robot:` sont interdites dans un scénario, parce qu'elles ne sont lues qu'au lancement ;
  - un scénario invalide est signalé et noté ÉCHEC ;
  - une position hors de la place exigée est signalée.
- **Verdict OK** quand toutes ces conditions sont remplies :
  - arrivée ;
  - aucune collision ni chute de la bille ;
  - erreur d'arrêt ≤ 30 mm (`ReglagesEpreuve.TOLERANCE_ARRET_MM`, guide) ;
  - recul < 200 mm (`RECUL_MAX_MM`, guide) ;
  - autant d'arrêts et de reculs que d'obstacles (précision du 12 nov.).

  Le verdict est aussi ajouté au banc aléatoire. Une ligne par cas s'affiche dans la console, puis un bilan.
- **Rejouable** : deux lancements de `--scenarios=tous` donnent des CSV identiques. Deux bancs de 40 pistes donnent aussi des CSV identiques.

Résultats (`essais/scenarios_reference.csv`) : **11 cas OK sur 13**.

| Scénario | Verdict | Bille max | Erreur d'arrêt | Oscillation en ligne droite | Braquage à f_bille |
| --- | --- | --- | --- | --- | --- |
| reference_droite (= face_debout) | OK | 0,75 | 3,7 mm | 5,35 Hz, 3,1° | 1,21° |
| obstacle_apres_virage | OK | 0,71 | 3,5 mm | non mesurée (pas de droite de 4 s) | 2,21° |
| obstacle_avant_virage | OK | 0,75 | 3,7 mm | aucune | 4,00° |
| deux_obstacles | OK | 0,75 | 3,7 mm | non mesurée | non mesuré |
| face_debout_tourne, face_flanc, face_flanc_tourne | OK | 0,75 | 3,7 mm | identiques à face_debout | |
| face_couche, face_couche_tourne (50 mm de haut) | **ÉCHEC** | 0,64 | — | — | — |
| virage_serre | OK | 0,64 | — | 5,05 Hz, 9,0° | 0,83° |
| s_serre | OK | 0,78 | — | 4,75 Hz, 1,9° | 1,28° |
| demi_tour | OK | **0,99** | — | 5,10 Hz, 5,6° | 1,93° |

- **Cas « couché »** : l'ultrason passe au-dessus de l'obstacle de 50 mm. Il y a collision, sans arrêt ni recul. C'est attendu, et ce sera traité à l'étape 4 (hauteur de l'ultrason).
- **Faces de 150 mm de large ou de profondeur** : aucune collision, avec des chiffres identiques à ceux de l'obstacle debout. La taille construite a été vérifiée (boîtes de 150 × 100 × 50, etc.). Le contrôleur ne mesure ni la largeur ni la profondeur : sa trajectoire ne change pas, et elle passait déjà avec de la marge. **La marge latérale n'est pas encore mesurée**, ce sera l'étape 4.
- **demi_tour** : la bille monte à 0,99 du bord, avec deux virages de 120 mm collés. C'est à surveiller (#67).

### Mesures de stabilité (`simulation/mesures_stabilite.gd`, module pur)

Ce sont des **choix de méthode**, pas des exigences : le guide ne chiffre pas la stabilité. Colonnes ajoutées à la fin du CSV (les 21 premières ne changent pas) :

| Colonne | Définition |
| --- | --- |
| `braquage_activite_deg_m` | Σ \|variation de l'angle réel des roues\| / distance parcourue, sur tout l'essai |
| `braquage_vitesse_rms_deg_s` | moyenne quadratique de la vitesse de braquage des roues |
| `braquage_f_dom_hz`, `braquage_amp_f_dom_deg` | **oscillation du suivi** : pic du spectre de l'angle des roues pendant le suivi **en ligne droite** seulement, c'est-à-dire quand la barre de sondes et l'essieu arrière sont tous les deux sur une case droite de la piste. Vide = non mesurée (pas de segment de 4 s) ; amplitude < 0,05° = aucune oscillation |
| `f_bille_hz` | fréquence propre de la bille, f0 = (1/2π)·√((5/7)·g/(R_cuvette − r_bille)) = 1,157 Hz (`BilleCuvette.frequence_propre_hz()`) |
| `braquage_amp_f_bille_deg` | **excitation de la bille** : amplitude maximale du braquage dans la bande f0 ± 0,3 Hz, sur tout le suivi, virages compris (voulu : un virage excite vraiment la bille) |
| `bille_marge_moy` | moyenne de la position de la bille rapportée au bord |
| `segments_droits`, `segments_suivi` | nombre de segments analysés |

- **Méthode** :
  - chaque segment d'au moins 4 s est analysé à part ;
  - le signal est rééchantillonné à 50 Hz ;
  - on retire la moyenne glissante sur 2 s ;
  - on applique une fenêtre de Hann ;
  - bande de 0,25 à 10 Hz, par pas de 0,05 Hz.
- **Pourquoi deux mesures (écart avec le plan)** : le plan prévoyait un seul spectre sur tout le suivi. L'essai sur des signaux connus montre qu'un virage (impulsion de braquage d'environ 1 s) ressort à 0,35–1,2 Hz, avec 1,3 à 4,7° selon le filtre. Aucun réglage du filtre ne le sépare d'une oscillation. L'oscillation du suivi est donc mesurée en ligne droite seulement, à partir de la géométrie réelle de la piste (pas de seuil réglé à la main).
- **Contrôle sur des signaux connus** : une sinusoïde de 3° à 1,2 Hz est mesurée à 1,20 Hz et 2,62°. Le retrait de la moyenne glissante l'atténue d'environ 13 % ; au-dessus de 4 Hz, l'atténuation est de moins de 1 %. Une sinusoïde de 1° à 4 Hz est mesurée à 4,00 Hz et 0,99°.
- **Contrôle avec le gain du PID × 3** (`--reglage=suivi:kp=5.4`) :
  - le braquage à f_bille monte nettement, ce qui montre que la mesure réagit :
    - reference_droite : de 1,21 à 1,94° ;
    - virage_serre : de 0,83 à 1,80° ;
    - s_serre : de 1,28 à 3,42° ;
  - l'oscillation en ligne droite change de forme : reference_droite passe de 5,35 Hz et 3,1° à 0,70 Hz et 3,5°.

**Premier résultat, pour l'étape 5** (banc de référence, 40 pistes) :
- le suivi zigzague en ligne droite à **4,4 à 7,0 Hz (médiane 4,95 Hz)**, avec une amplitude médiane de **7,9° de roue** (de 2,3 à 10,8°) ;
- l'activité du braquage vaut 618 deg/m en médiane ;
- le braquage autour de la fréquence de la bille reste faible : 0,79° en médiane, 2,35° au maximum ;
- bille : marge maximale 0,77, moyenne 0,15.

Le zigzag est loin de la fréquence de la bille et ne la fait pas tomber en simulation idéale. Il use cependant le servo, et il risque d'être pire sur le robot, avec du retard et une zone morte.

## Obstacles posés sur différentes faces (13 novembre, étape 4)

Source : précision du professeur du 13 nov. Le bloc de 100 × 150 × 50 mm peut être posé sur n'importe quelle face (debout, tourné, couché), toujours centré sur la ligne.

### Ce qui a changé

1. **Marge latérale mesurée** (`banc_essai.gd`) : nouvelle colonne `marge_obstacle_min_mm`, aussi affichée sur la ligne de la console. C'est la plus petite distance, vue de dessus, entre la boîte d'encombrement de la voiture et chaque obstacle, sur tout l'essai ; 0 = contact. C'est une mesure de la simulation (vérité terrain), pas du robot. Les obstacles viennent de la métadonnée `obstacles` de `ConstructionPiste`.
2. **Hauteur de l'ultrason** :
   - nouvelle entrée `hauteur_ultrason_mm` = 60 dans `reglages_robot.json` (`07_Modelisation-Blender/LISEZMOI.md`, mesure à la règle). Au démarrage, elle est comparée à la position de `Capteur_Ultrason` dans `picar.glb` : écart de moins de 5 mm ;
   - le cône de l'ultrason (`vehicule.gd`) a maintenant **5 rayons** : l'axe, ± le demi-angle à gauche et à droite, **± le demi-angle en haut et en bas**.
   - **Hypothèse** : le cône est circulaire, avec le même demi-angle en hauteur. Le manuel (p. 34) donne un cône de 30 à 40° sans séparer les deux directions. Avec 7,5° (valeur provisoire), le rayon du bas voit un bloc couché de 50 mm de haut entre 456 et 76 mm du capteur.
   - **Non simulé (#53)** : l'écho du sol. Le rayon du bas touche le sol à environ 456 mm, et le sol n'est pas sur la couche des obstacles. À mesurer sur le robot.
   - **Biais de la mesure du banc** : pour un bloc couché, la distance est lue le long du rayon incliné. Elle est environ 0,9 % trop longue, soit à peu près 1 mm à l'arrêt.
3. **Contournement du pire cas** (`reglages_epreuve.gd`) :
   - L'ultrason ne donne qu'une distance : le robot ne sait pas sur quelle face le bloc est posé. Il contourne donc la plus grande emprise possible, soit la plus grande des 3 dimensions en travers comme dans le sens de la marche : **150 × 150 mm**.
   - Ces valeurs sont données par `largeur_a_contourner_mm()` et `profondeur_a_contourner_mm()`, calculées à partir des 3 dimensions et non écrites une seconde fois.
   - Elles sont utilisées par `contournement_fluide.gd`, `contournement.gd` et `controleur.gd` (mémoire des obstacles, portée utile de l'ultrason en virage).
   - Effet : l'écart latéral passe de 170,5 à 195,5 mm, et le retour sur la ligne commence 100 mm plus loin.
4. **Générateur** : `ParametresPiste.faces_aleatoires` (vrai par défaut) tire au hasard une des 6 poses pour chaque obstacle (`GenerateurPiste.poser()`). Le tirage est fait après la taille : avec `false`, la suite des tirages est la même qu'avant, donc les pistes aussi (contrôle : même nombre d'obstacles sur les 40 pistes de la référence). Nouvelle colonne `poses` dans le banc.
5. **Ligne de commande** : `--parametre-piste=<nom>=<valeur>`, ex. `--parametre-piste=faces_aleatoires=false`.
6. **2 nouveaux scénarios** (15 au total) :
   - `pire_cas_avant_virage` : bloc couché de 150 de profondeur, placé au plus tard avant un virage ;
   - `pire_cas_deux_obstacles` : deux blocs sur le flanc (150 de large), à l'espacement minimal.

### Résultats

Marge latérale minimale des scénarios, à chaque étape (obstacle centré) :

| Scénario | Avant | + cône vertical | + pire cas 150 × 150 |
| --- | --- | --- | --- |
| face_debout (100 de large) | 35 mm | 35 mm | 40 mm |
| face_debout_tourne, face_flanc_tourne (50 de large) | 57 mm | 57 mm | 59 mm |
| face_flanc (150 de large) | **10,5 mm** | 10,5 mm | **20,5 mm** |
| face_couche (100 de large, 50 de haut) | **collision** | 34 mm | 39 mm |
| face_couche_tourne (150 de large, 50 de haut) | **collision** | **10 mm** | **19,8 mm** |
| pire_cas_avant_virage / pire_cas_deux_obstacles | — | — | 39 / 20,5 mm |

- Scénarios : **15 OK sur 15**, toujours par le contournement fluide (aucun repli). Les blocs couchés s'arrêtent à 100 mm ± 3,7 mm, puis reculent de 150 mm.
- `obstacle_avant_virage` : 1 perte de ligne de 0,6 s après le contournement, et bille à 0,82 (0,75 avant). Le retour finit 100 mm plus loin, plus près du virage.

Bancs de 40 pistes (pas de 1/240 s) :

| Banc | OK | Collisions | Bille max | Marge min (médiane) | Remarque |
| --- | --- | --- | --- | --- | --- |
| Référence de l'étape 3 (debout, ancien contournement) | 40/40 | 0 | 0,77 | non mesurée | `essais/reference_1-240_etape3.csv` |
| Mêmes pistes, contournement du pire cas | 40/40 | 0 | 0,77 | 25,7 mm (43) | 44 contournements fluides ; temps total + 13 s sur 2 552 s |
| **Faces aléatoires (nouvelle référence)** | 40/40 | 0 | 0,78 | **14,0 mm** (42) | `essais/reference_1-240.csv` ; 42 obstacles, chacune des 6 poses tirée 5 à 8 fois |

- **Marge de 14 mm** (graine 308842), sous les 20 mm visés. Les 3 pires cas sont tous des blocs de 150 de large. Explication probable : le générateur décale le bloc jusqu'à 10 mm du côté du contournement (tolérance de centrage), alors que les scénarios sont centrés et donnent 20 mm. Sur 40 mm de marge latérale prévue (`marge_laterale_mm`), la poursuite et la rotation de la voiture en perdent environ 20 en simulation idéale.
- **D_recul 50 (#72)** : bancs de 15 pistes, contournement par phases presque toujours (11 fois sur 13 obstacles).
  - Debout : 15/15, mais marge minimale de 1,7 mm (graine 110867) et une perte de ligne de 9 s.
  - Faces aléatoires : 14/15, avec **1 collision** (graine 63353 : blocs de 50 puis 150 de large) et une autre marge de 0 mm (graine 95029).
  - Le contournement par phases ne vérifie pas sa trajectoire : c'est la tâche #72 de l'étape 6.

## Stabilité du suivi de ligne, #67 et #74 (13 novembre, étape 5)

### Défaut du banc corrigé : les cas n'étaient pas indépendants

Rejouée seule, la graine 221733 donnait une bille à 0,79 ; rejouée juste après la graine 213814, elle donnait 0,97. Trois restes du cas précédent passaient d'un essai à l'autre (`vehicule.gd`) :
- l'accélération, les sondes, la distance de l'ultrason et la collision n'étaient pas remises à zéro par `placer()` ;
- au pas de placement, les capteurs étaient lus alors que l'ancienne piste était encore dans le monde physique : la voiture voyait l'ancienne barre d'arrivée (`11111`). Cette lecture est supprimée ; la première lecture se fait au pas suivant ;
- le modèle relisait `global_transform`, qui repasse par le serveur physique (`sync_to_physics`) et gardait un reste (1e-7) de l'orientation précédente. La pose est maintenant portée par le modèle (`_pose`).

Vérification : la graine 221733 seule ou après 213814, et deux scénarios enchaînés, donnent des lignes `CAS` identiques. Toutes les références ont été refaites : les anciennes portent le suffixe `_etape4`.

Ajouts pour les essais : option `--journal` (journal détaillé `journaux/essai_*.csv` sans éditeur) et numéro de processus dans le nom du journal détaillé (deux essais lancés dans la même seconde n'écrivent plus dans le même fichier).

### Sources consultées

| Méthode | Source | Ce qu'on en retient |
| --- | --- | --- |
| Dérivée filtrée, anti-windup | MathWorks, « PID Controller » (Simulink) | Le filtre de la dérivée limite le bruit ; le cas d'une erreur quantifiée n'est pas traité |
| Pure pursuit | Snider, CMU-RI-TR-09-08, § 2.2–2.3 | Correcteur P de gain 2/d² ; anticipation courte → oscillation |
| Suiveurs à sondes numériques | Pololu, QTRSensors, `readLine()` | Ki « usually not necessary », Kd « usually much bigger » que Kp |
| Anticipation et vitesse | Taylor et al., IJRR 1999 | Qualitatif : plus de vitesse déstabilise |
| Limiteur de braquage, zone morte | **non vérifié** (résumés seulement) | Traités comme des hypothèses à tester |

### Le braquage (candidats C1 à C4) : aucun retenu

L'erreur mesurée ne prend que les valeurs 0, ±9,5, ±19, ±28,5 et ±38 mm (sondes à 19 mm, ruban de 18 mm). Avec `kp` = 1,8, le plus petit pas vaut 17° de servo. Les candidats sont des réglages de `suivi_ligne.gd`, **désactivés par défaut** (comportement inchangé, vérifié). Ils restent dans le code pour d'autres essais.

Tri sur 6 cas (`reference_droite`, `virage_serre`, `s_serre`, `demi_tour`, `courbes_120`, `parcours_fourni`) :

| Réglage | Verdicts | Zigzag moyen | Bille max | Rattrapages | Écarté parce que |
| --- | --- | --- | --- | --- | --- |
| C0 départ | 6/6 | 5,1° | 0,99 | 13 | — |
| C1 `kp` 0,8 (avec ou sans `kd` 0,2) | 5/6 | 4,0–4,6° | 0,84–0,98 | 10–12 | `demi_tour` bloqué |
| C1 `kp` 1,2 | 6/6 | 3,7° | 0,93 | 10 | validation (plus bas) |
| C2 limiteur 300 / 150°/s | 5/6 | 5,0 / 4,4° | 1,00 | 9 / 6 | bille tombée (retard du limiteur) |
| C3 zone neutre | 6/6 | 6,3° à 2,5 Hz | 0,89 | 10 | zigzag plus lent, plus près de la bille (excitation 5,9°) |
| C4 `kp_centre` 0,34 | 6/6 | 2,2° | 0,87 | 10 | validation (plus bas) |

Validation de C1 et C4 sur le banc corrigé :

| Réglage | Banc 40 : bille max | D_recul 50 : collisions |
| --- | --- | --- |
| C0 | 0,84 | 1 (graine 63353) |
| C1 `kp` 1,2 | 0,95 | 3 |
| C4 `kp_centre` 0,34 | 0,95 | 3 |

Pourquoi C4 échoue : le terme P vaut 3, 34, 51 et 68° pour 9,5, 19, 28,5 et 38 mm. Le saut de 31° entre 9,5 et 19 mm donne des à-coups lents (servo 63 → 111 → 61° en 0,45 s, environ une demi-période de la bille), que la bille amplifie. Le zigzag de C0 (environ 5 Hz) est 4 fois au-dessus de la fréquence propre de la bille (1,16 Hz), qui le filtre.

Leçon : le tri sur 6 cas ne voyait pas ce défaut. Deux barrières ont été ajoutées avant de retenir un réglage : (1) sur le banc de 40 pistes, une bille max pas pire qu'au départ ; (2) avec D_recul 50, pas plus de collisions que C0.

### #74 : pertes de ligne dans les virages de 120 mm

Elles sont **géométriquement inévitables** avec ces sondes, comme le décrit déjà `rattrapage.gd`. Les sondes sont à 217 mm devant l'essieu arrière, et le rayon minimal est de 138 mm à 45°. Une sonde décrit donc un cercle d'au moins √(138² + 217²) = 257 mm de rayon, alors qu'une sonde qui doit voir un ruban de 120 mm de rayon doit rester à moins de 120 + 38 + 9 = 167 mm du centre. Le rattrapage en marche arrière traite ces pertes. Sa fréquence ne pourrait baisser qu'en changeant la position des sondes, ce qui n'est pas fait.

### #67 : la bille pendant la réaccélération

Les deux pics ont la même cause (journaux détaillés) :
- `parcours_fourni` (0,96) : la bille oscille encore après l'arrêt du rattrapage, puis la reprise douce se termine (1 s après le rattrapage). La commande passe alors de 25 à 45 en 0,16 s (rampe de 125 cmd/s, environ 0,5 m/s²) ;
- `demi_tour` (0,99) : même déroulement, de 25 à 55.

Correction : rampe séparée pour l'accélération, `rampe_montee_cmd_par_s` (renommée à l'étape 9 : `acceleration_montee_mm_s2`) (`reglages_robot.gd`, avec `rampe_freinage_cmd_par_s` = 125, ex-`rampe_cmd_par_s`, dont dépend la distance d'arrêt devant l'obstacle). Réglage d'un essai : `--reglage=robot:rampe_montee_cmd_par_s=40` ou `reglages_essai` `{"robot:rampe_montee_cmd_par_s": 40}` (idem `robot:rampe_freinage_cmd_par_s` ; les clés `robot:` absentes de `reglages_robot.json` s'appliquent au contrôleur seulement).

Nettoyage du 13 nov. (sans changement de comportement) : candidats C2 à C4 retirés de `suivi_ligne.gd` (résultats ci-dessus conservés) ; fréquence de la boucle (`frequence_controle_hz`, `reglages_robot.json`) lue par `contournement.gd` (pas de la prévision) et fournie à `mesures_stabilite.gd` par le banc ; `choisir_cap` reçoit `fluide.marge_obstacle_mm` : **un seul réglage de marge** (un essai `fluide:marge_obstacle_mm` change maintenant aussi la vérification du contournement par phases) ; le banc contrôle la « fin de ligne atteinte » des scénarios BLOQUE (`distance_fin_ligne_max_mm` = 150 mm, hypothèse ; fin du ruban d'une piste Blender = sommet le plus éloigné du départ le long du maillage, approximation).

| Rampe d'accélération | 6 cas : bille max | Rattrapages | Temps |
| --- | --- | --- | --- |
| 125 (départ) | 0,99 | 13 | 132,8 s |
| **60** | **0,78** | **9** | 125,0 s |
| 40 | 0,78 | 12 | 142,2 s |
| 25 | 0,78 | 7 | 135,4 s |
| 125 + reprise douce de 2 s | 0,83 | 9 | 122,6 s |

Validation complète :

| Rampe | 17 scénarios : bille max / rattrapages | Banc 40 : bille max (médiane) / temps | D_recul 50 : collisions / bille max |
| --- | --- | --- | --- |
| 125 (départ, banc corrigé) | 0,99 / 14 | 0,84 (0,73) / 2 556 s | 1 / 0,92 |
| **60 (retenue)** | **0,82 / 10** | **0,80 (0,64)** / 2 644 s | 1 / **0,73** |
| 40 | 0,78 / 13 | 0,74 (0,67) / 2 719 s | 1 / 0,91 |

- **60 retenue** : meilleur pire cas sur les trois bancs (0,82), le moins de rattrapages, 3 % de temps en plus. Tous les verdicts OK sauf la collision connue de la graine 63353 (#72), déjà présente au départ.
- L'excitation du braquage à la fréquence de la bille monte (max 1,96 → 2,49°) : la rampe ne touche pas au braquage. La bille, elle, descend partout.
- Valeur **provisoire** : elle dépend de la fréquence propre de la bille, à mesurer sur le robot (`MESURES_ROBOT.md`).
- Le code reproduit l'essai à l'identique (17 scénarios, CSV identique).

Références (`essais/`) : `reference_1-240.csv`, `scenarios_reference.csv` et `reference_recul50.csv` avec la rampe de 60. Les fichiers `_c0` sont le départ sur le banc corrigé ; les fichiers `_etape4` sont l'ancien banc.

Détail par cas avec la rampe de 60 (avant → après) : `courbes_120` : voir la remarque ci-dessous ; `parcours_fourni` : bille 0,96 → 0,77, rattrapages 4 → 4, 45,6 → 47,2 s ; `demi_tour` : bille 0,99 → 0,78, rattrapages 3 → 3.

**Remarque sur `courbes_120` (fin attendue BLOQUE).** Avec la rampe de 60, la voiture s'arrête en fin de ruban, à (−0,17 ; −1,61) m, à 23,3 s. Au départ (C0), elle atteint le même point à 22 s, mais ne reconnaît pas la fin de ligne : elle fait un rattrapage, retrouve la ligne, la reprend **en sens inverse**, puis se bloque ailleurs à 35,1 s. Le verdict OK ne le voyait pas : le banc ne vérifie pas où un BLOQUE a lieu (à ajouter). Sur le vrai parcours, les deux font 2 rattrapages ; bille 0,81 → 0,70. Les chiffres de `courbes_120` dans les tableaux ci-dessus (6 rattrapages, 0,84, 35 s pour C0 ; 24 s pour C4) comptent donc ce retour en arrière.

**Robustesse de la rampe de 60** (6 cas ciblés, rampe de 125 contre 60) :

| Variante du modèle | Rampe 125 : verdicts / bille max | Rampe 60 : verdicts / bille max |
| --- | --- | --- |
| Servo deux fois plus lent (150°/s) | 6/6 / 0,93 | 6/6 / **0,84** |
| Cuvette de 98 mm de rayon (−30 %, bille plus rapide) | 6/6 / 0,71 | 6/6 / 0,72 |
| Cuvette de 182 mm de rayon (+30 %, bille plus lente, fond plus plat) | **3/6, bille tombée 3 fois** | 6/6 / **0,99** |

La rampe de 60 garde l'avantage. Avec une cuvette plus plate, la marge devient très mince (0,99) : le rayon réel de la cuvette (plan `support_bille_v2` : R140, unités non écrites) et la fréquence propre de la bille sont à mesurer en priorité (#47).

## Contournement par phases vérifié (#72) et virages de tout angle (#73) (13 novembre, étape 6)

### #72 : le contournement par phases vérifie sa trajectoire

**Cause de la collision** (graine 63353, D_recul 50, journal détaillé) : contact pendant la phase ECART, sur le 2e obstacle (bloc de 150 de large). La loi de cap proportionnelle n'avait décalé la voiture que de 43 mm après 142 mm d'avance : le coin avant touchait l'obstacle.

**Ce qui a changé :**
- `simulation/geometrie_voiture.gd` (nouveau, module pur) : test « rectangle de la voiture contre rectangle de l'obstacle » (coins de l'une dans l'autre, dans les deux sens). Sorti de `ContournementFluide._trajectoire_libre`, qui l'appelle maintenant (17 scénarios identiques à la référence).
- `contournement.gd` : `choisir_cap(d0_mm, marge_mm)` simule à l'avance ECART, LONGER et le début du RETOUR avec les mêmes lois que la manœuvre (servo à sa vitesse, vitesse de manœuvre), pour chaque cap de `caps_evitement_candidats_deg` (40 à 90°, du plus doux au plus serré), contre l'obstacle élargi de `marge_mm` (= `ContournementFluide.marge_obstacle_mm`, 20 mm). Le premier cap qui passe est gardé.
- `controleur.gd` : si aucun cap ne passe, arrêt sûr en **ARRET**, raison `phases_sans_trajectoire` (choix de Mehdi : s'arrêter plutôt que reculer davantage). ARRET et non BLOQUE : BLOQUE repart dès que la ligne est vue, et elle est sous la voiture.
- Scénarios ajoutés (D_recul 50) : `recul_court_debout`, `recul_court_flanc` (bloc de 150 décalé de 10 mm, cas de la graine 63353), `recul_court_deux_obstacles`.

**Résultats :**

| Banc | Avant | Après |
| --- | --- | --- |
| D_recul 50, 15 pistes | 14/15, 1 collision, marge min 0 mm | **15/15, 0 collision**, marge min 1,1 mm |
| 40 pistes (D_recul 150) | 40/40 | identique (aucun contournement par phases) |
| 17 scénarios d'origine | 17/17 | identiques |
| D_arret 30, D_recul 0 (voie de refus) | – | arrêt en ARRET, `phases_sans_trajectoire`, 0 collision |

Les deux marges les plus faibles avec D_recul 50 (1,1 mm graine 95029, 5,7 mm graine 15839) viennent du **contournement fluide**, pas des phases (vérifié en désactivant le fluide).

**Essai non appliqué (décision de Mehdi) :** `fluide:marge_obstacle_mm` 20 → 40 mm. Marges min : 25,7 mm (D_recul 50), 62 mm (40 pistes) ; mais le fluide n'est plus choisi, les phases font tout, et le temps monte de 13 % (40 pistes) à 29 % (scénarios).

Références : `essais/reference_recul50.csv` (après #72) ; ancienne : `reference_recul50_etape5.csv`.

### #73 : générateur de pistes libre

Le guide (section 1) impose un rayon de 120 mm et plus, **aucun angle**. Le générateur sur grille ne faisait que des virages de 90°.

- `simulation/generateur_libre.gd` (nouveau, `GenerateurLibre`) : tracé « tortue » de droites et d'arcs, angle de 20 à 180°, à gauche ou à droite ; rejet si le ruban sort du sol ou passe à moins de `ecart_lignes_min` (240 mm) d'une autre partie de lui-même ; obstacles sur les droites seulement, avec les mêmes règles de place que la grille.
- `construction_piste.gd` : `construire_depuis_ligne(...)`, commun aux deux générateurs (pistes de la grille identiques).
- `parametres_piste.gd` : `generateur` (`"grille"` par défaut, `"libre"`), `angle_min_deg` 20, `angle_max_deg` 180, `ecart_lignes_min` 0,24 m, `droite_libre_min` 0,15 m, `droite_libre_max` 2,5 m. **Toutes des hypothèses** (`MESURES_ROBOT.md`). À 1,2 m de droite au plus, aucune piste n'avait d'obstacle (il faut ~1,3 m de droite).
- Scénarios à angles fixes (clé `"trace"`) : `virage_45`, `virage_135`, `s_60`, `epingle_180`, `obstacle_apres_virage_30`.
- En ligne de commande : `--parametre-piste=generateur=\"libre\"`.

**Résultats :**

| Essai | Résultat |
| --- | --- |
| 25 scénarios | 25/25 OK (`essais/scenarios_reference.csv` ; 17 cas d'avant : `scenarios_reference_17cas.csv`) |
| `virage_135` / `epingle_180` | bille 0,80 / 0,76 ; 1 / 3 rattrapages |
| 40 pistes libres (`essais/reference_libre.csv`) | **39/40**, 0 collision, 0 chute, bille max 0,94 ; 74 obstacles sur 33 pistes, tous contournés en fluide ; marge min 7,9 mm |

**Défauts trouvés (non corrigés, proposés à Mehdi) :**
- **Graine 47515 (BLOQUE)** : l'obstacle est sur une droite qui finit juste avant une épingle de 165° de rayon 126 mm, **plus serrée que la voiture** (138 mm). Le contournement fluide finit près de l'entrée ; la voiture y arrive décalée, braquée à fond, perd la ligne, fait 3 rattrapages (`tentatives_max`) et s'arrête. Le même enchaînement de virages sans obstacle passe (essai temporaire, 3 tracés, bille max 0,79).
- Bille à 0,94 sur les pistes libres (0,80 sur la grille) : à analyser.
- Marge latérale min du fluide 7,9 mm sur les pistes libres (14,3 mm sur la grille, 20 mm visés).

## Modèle réaliste (13 novembre, étape 9)

### Ce qui change

Jusqu'à l'étape 8, la simulation était **idéale** : ultrason sans bruit ni retard, vitesse proportionnelle à la commande (4 mm/s par unité), pas de zone morte. L'étape 9 ajoute un modèle réaliste, avec les valeurs mesurées par l'équipe. Il est dans `reglages_robot.json` ; le modèle idéal reste disponible (voir plus bas).

- **Ultrason (`vehicule.gd`)** : mesure = `gain_ultrason` × distance réelle (depuis la face du capteur) + `biais_ultrason_mm`, plus un bruit gaussien (`bruit_ultrason_mm`), plus une valeur aberrante rare (`proba_aberrante_ultrason`, écart de signe aléatoire, amplitude uniforme dans `aberrante_ultrason_mm`), livrée après un **retard pur** `retard_ultrason_s` (une mesure prise à t est livrée à t + retard : file d'attente `_en_transit`). Cadence `frequence_ultrason_hz` = 16,13. Le véhicule expose `compteur_mesures` (nombre de mesures livrées) : `simulation.gd` le compare à la valeur du tour précédent et passe `mesure_nouvelle` au contrôleur (`Controleur.calculer(..., mesure_nouvelle)`).
- **Aléatoire reproductible** : le véhicule a ses propres `RandomNumberGenerator` (ultrason, sondes), réensemencés à chaque `placer()` avec `vehicule.graine_aleatoire` : graine de la piste au banc, hash du nom du scénario (`String.hash()`) en scénario, 0 ailleurs. Deux exécutions identiques donnent le même CSV.
- **Filtre (#65, `filtre_ultrason.gd`)** : choix par `filtre_ultrason` : `mediane3` (filtre d'origine) ou `mediane_kalman` (médiane de 3 + Kalman à vitesse constante, reproduction de `mediane_kalman.py`, `kalman.py` et `config.py` : σ_z 2,4 mm, σ_a 400 mm/s², rejet si ν²/S > 9, reset après 5 rejets, plage 20–2000 mm, P0 = diag(R, 1000²)). Le Kalman n'est mis à jour qu'à l'arrivée d'une mesure ; entre deux, la sortie est projetée avec la vitesse estimée. Les règles de sécurité (absence d'écho 0,15 s, zone proche, portée 1500 mm) sont gardées. Ni le contrôleur ni `arret_obstacle.gd` ne compensent un retard du filtre : rien n'a été adapté.
- **Moteurs** : `table_vitesse_mm_s` = [[0, 0], [13, 0], [20, 50], [30, 113], [50, 256], [100, 614]] (interpolation linéaire, même table en arrière : hypothèse). La zone morte (13) découle de la table ; `zone_morte_moteur_cmd` et `vitesse_mm_s_par_cmd` sont supprimées du fichier. `ReglagesRobot.vitesse_mm_s(cmd)` et `commande_pour_vitesse(mm_s)` remplacent partout `mm_s_par_cmd` / `vitesse_max` (`vehicule.gd`, `controleur.gd`, `contournement.gd`, `contournement_fluide.gd`, `recul_ligne.gd`, `vitesse_freinage`). `vitesse_freinage(d)` = sqrt(2 × décélération × marge × d).
- **Odométrie** : même table que le modèle, multipliée par (1 + `erreur_vitesse_odometrie`), clé à 0,0 (hypothèse, pour les essais de sensibilité). Appliquée dans `controleur.gd` (position, contresens, vitesse donnée au contournement) ; `odometrie.gd` ne change pas.

### Algorithmes en unités physiques

Pourquoi : les algorithmes raisonnaient en commandes (65, 45, 25, minimums 8 et 10, rampes de 60 et 125 commandes/s), calibrées pour 4 mm/s par commande. Avec la table réelle (≈ 7 mm/s par commande), le freinage atteignait ≈ 0,9 m/s² et la bille tombait (18 scénarios sur 25, 7 OK) ; les commandes 8 et 10 étaient sous la zone morte (13).

Ce qui change : toutes les vitesses des algorithmes sont en **mm/s**, avec le suffixe `_mm_s` (valeur = ancienne commande × 4, exactement). Le contrôleur garde une vitesse de consigne `_v_mm_s`, applique les accélérations en mm/s², puis convertit **à la sortie seulement** : `commande = commande_pour_vitesse(_v_mm_s)` (la table, zone morte comprise), arrondie à l'entier envoyé au moteur. L'odométrie utilise `vitesse_odometrie_mm_s` de la commande ainsi calculée, comme avant. `vitesse_freinage()` devient `racine(2 × deceleration_freinage_mm_s2 × marge_freinage × distance)`, indépendante du moteur. Le contournement fluide arrondit sa vitesse vers le bas à une commande entière (`floorf` de la commande, puis table) : même troncature qu'avant en mode idéal, et vitesse toujours atteignable en réaliste.

Noms des réglages (ancien → nouveau) : `rampe_montee_cmd_par_s` 60 → `acceleration_montee_mm_s2` 240 ; `rampe_freinage_cmd_par_s` 125 → `deceleration_freinage_mm_s2` 500 ; `vitesse_approche_min` 8 → `vitesse_approche_min_mm_s` 32 ; `suivi:vitesse_max` 55 → `vitesse_max_mm_s` 220 ; `suivi:vitesse_virage` 15 → `vitesse_virage_mm_s` 60 ; `arret:vitesse_recul` 10 → `vitesse_recul_mm_s` 40 ; `recul:vitesse_recul_ligne` 15 → `vitesse_recul_ligne_mm_s` 60 ; `rattrapage:vitesse` 15 → `vitesse_mm_s` 60 ; `retrouver:vitesse` et `demi_tour:vitesse` 18 → `vitesse_mm_s` 72 ; `reprise:vitesse_max` 25 → `vitesse_max_mm_s` 100 ; `contournement:vitesse_manoeuvre` 25 → `vitesse_manoeuvre_mm_s` 100 ; `fluide:vitesse_max` 65 → `vitesse_max_mm_s` 260. Essai sans toucher au code : `--reglage=robot:acceleration_montee_mm_s2=200` (ou `reglages_essai` `{"robot:deceleration_freinage_mm_s2": 400}`).

Contrôle : `--modele=ideal` redonne `essais/scenarios_reference.csv` ligne pour ligne (0 ligne différente) ; en réaliste, 25 scénarios OK sur 25.

### Clés de `reglages_robot.json`

| Clé | Valeur | Statut | Source |
| --- | --- | --- | --- |
| `frequence_ultrason_hz` | 16,13 | mesuré | TEX « Filtre tes connaissances » (10 nov.) section 1 ; `config.py` |
| `bruit_ultrason_mm` | 2,5 | mesuré | TEX tableau 1 (remplace `bruit_ultrason_cm`) |
| `proba_aberrante_ultrason`, `aberrante_ultrason_mm` | 0,01 ; [10, 19] | mesuré | `Analyse_filtres_ultrason_E05.md` (9 nov.) |
| `gain_ultrason`, `biais_ultrason_mm` | 0,971 ; 8,3 | mesuré | TEX section 6.5 |
| `retard_ultrason_s` | 0,0 | hypothèse | 0,3 s observé avec l'ancien pilote, 0 avec `fastdrink` : à tester en sensibilité |
| `filtre_ultrason` | `mediane_kalman` | provisoire | choix de l'équipe (#65) |
| `table_vitesse_mm_s` | voir ci-dessus | hypothèse | essais en mouvement du 9 nov. (20, 30, 50) ; 13 et 100 : hypothèses |
| `erreur_vitesse_odometrie` | 0,0 | hypothèse | essais de sensibilité |

### Écarts du filtre avec le Python et cadence de l'ultrason

Écarts restants entre `filtre_ultrason.gd` (`mediane_kalman`) et `09_Code-PiCar/fastdrink/filtres/mediane_kalman.py` : portée de 1500 mm (Python : 2000 mm) ; remise à zéro après `absence_s` (0,15 s ; 1 s si l'obstacle était proche), qui n'existe pas en Python ; horodatage au tour du contrôleur (≤ 20 ms) au lieu de l'instant de lecture. Une mesure hors de [20 ; 2000] mm est rejetée avant la fenêtre, comme en Python. Les constantes viennent de `config.py` (σ_a = 400 = `MEDIANE_KALMAN_SIGMA_A_MM_S2`, pas `KALMAN_SIGMA_A_MM_S2` = 200) : à garder synchronisées.

Cadence de l'ultrason : la période réelle est un multiple du pas physique, soit 62,5 ms à 1/240 s (bancs) et 66,7 ms à 1/60 s (`vitesse_temps` 4). En modèle réaliste, le contrôleur arrondit la commande avant l'odométrie (même valeur entière que celle envoyée au véhicule) ; en modèle idéal (table linéaire) elle reste flottante pour l'odométrie, car l'arrondir change les résultats de `scenarios_reference.csv` (0 → 32 lignes différentes).

### Modèle idéal et contrôle

`--modele=ideal` en ligne de commande (propriété exportée `modele_ideal` du nœud Simulation, défaut faux ; `--modele=realiste` pour l'inverse) applique par-dessus le fichier la liste `FichierReglages.VALEURS_IDEALES` (`fichier_reglages.gd`, seul endroit) : fréquence 20 Hz, bruit 0, pas d'aberrante, gain 1, biais 0, retard 0, filtre `mediane3`, table vide (vitesse = 4 mm/s × commande, calcul linéaire d'avant), constante de temps 0, erreur d'odométrie 0. Les clés `robot:` de `reglages_essai` s'appliquent après, y compris en idéal : `--modele=ideal --reglage=robot:retard_ultrason_s=0.3` isole un seul effet.

**Contrôle fait le 13 nov.** : 25 scénarios en `--modele=ideal` : CSV identique ligne pour ligne à `essais/scenarios_reference.csv` (0 ligne différente). Le banc de 40 pistes n'a pas été relancé dans cette étape.

### Résultats

**Depuis l'étape 9, le banc tourne par défaut en modèle réaliste.** Les références idéales d'avant restent valables avec `--modele=ideal` : 40/40, 15/15 (D_recul 50), 39/40 (libres), 25/25, tous identiques aux références à la ligne près.

| Banc (modèle réaliste) | Résultat | Idéal (rappel) |
| --- | --- | --- |
| 40 pistes (`essais/reference_realiste_1-240.csv`) | **40/40**, 0 collision, bille max 0,81, erreur d'arrêt max 15,2 mm, recul 150,0 mm, marge min 17,5 mm | 40/40, 0,80, 3,9 mm, 150 mm, 14,3 mm |
| 25 scénarios (`essais/scenarios_reference_realiste.csv`) | **25/25**, bille max 0,85, erreur d'arrêt max 15,1 mm | 25/25, 0,82 |
| D_recul 50, 15 pistes (`essais/reference_realiste_recul50.csv`) | **15/15**, 0 collision, marge min 4,6 mm | 15/15, 1,1 mm |
| 40 pistes libres (`essais/reference_realiste_libre.csv`) | **38/40**, 0 collision, bille max 1,00, marge min 8,5 mm | 39/40, 0,94 |

Avant le passage en unités physiques, les scénarios réalistes donnaient 7/25 (bille tombée 14 fois : freinage à ~0,9 m/s² avec la table) ; le bruit, le biais, la cadence et le filtre de Kalman seuls passaient.

**Sensibilité** (40 pistes, modèle réaliste) :

| Variante | Résultat | Conclusion |
| --- | --- | --- |
| Retard de l'ultrason 0,3 s (valeur observée avec l'ancien pilote) | 39/40 : erreur d'arrêt 30,2 mm (> ± 30 mm, graine 110867) | Le retard réel du pilote `fastdrink` est à mesurer en priorité ; s'il existe, le compenser |
| Odométrie fausse de +10 % (vitesse par commande mal étalonnée) | 39/40 : **1 collision** (graine 308842), recul 136 mm au lieu de 150 | La table de vitesse doit être mesurée avec soin (#18) ; le contournement dépend de l'odométrie |

**Défauts trouvés (non corrigés) :**
- Graine 300923 (pistes libres, réaliste seulement) : bille tombée à 6,2 s, avant tout obstacle, sans rattrapage. Cause à chercher (#67).
- Graine 47515 : toujours en échec (#75).
- L'erreur d'arrêt passe de 4 à 15 mm (biais du capteur non compensé : la loi 0,971 d + 8,3 n'est pas inversée dans le contrôleur ; le rapport la prévoit). Dans la tolérance, à corriger avec les mesures.
- Recul : 156 mm au premier essai, 150,0 mm après correction (l'odométrie utilisait la commande non arrondie alors que le moteur reçoit la commande entière).

