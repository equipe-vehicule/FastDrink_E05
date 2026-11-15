# Mesures du robot à faire – FastDrink E05

Mis à jour le 14 novembre 2026. Liste de toutes les valeurs **provisoires** de la simulation : à remplacer par des mesures sur le vrai robot.

**Toutes les valeurs du robot sont dans un seul fichier : `reglages_robot.json` (racine du dépôt).** Il est lu par la simulation et sera lu par le code Python du robot. Chaque entrée a une valeur, une unité, un statut (`provisoire`, `mesuré` ou `hypothèse`), une source et une tâche.

Quand une mesure est faite :
1. noter le résultat en commentaire de la tâche GitHub ;
2. dans `reglages_robot.json`, changer la valeur, passer le statut à `mesuré` et écrire la source (date, méthode) ;
3. passer la ligne ci-dessous à « Mesuré » ;
4. relancer les essais de la simulation.

Pour les valeurs de `reglages_robot.json`, il n'y a rien à changer dans le code. Les valeurs des algorithmes (tableaux « Algorithmes » et « Épreuve », et les mentions « hors fichier ») sont dans **`reglages_algorithmes.json`** (racine du dépôt, un objet par module : `suivi`, `ultrason`, `fluide`, `rattrapage`, `epreuve`, `robot`…) : c'est la seule source de ces valeurs, le code `.gd` ne porte que leur type et leur justification. Pour les changer : modifier ce fichier, sans toucher au code.

Tant que ces valeurs sont provisoires, la simulation ne prédit pas le robot : elle valide seulement la logique des algorithmes.

## Robot (moteurs, direction)

| Valeur | Actuelle (provisoire) | Clé de `reglages_robot.json` | Tâche | Statut |
| --- | --- | --- | --- | --- |
| Vitesse réelle par commande, avant et arrière | Table `[commande, mm/s]` = [[0, 0], [13, 0], [20, 50], [30, 113], [50, 256], [100, 614]], interpolation linéaire, même table en arrière (**hypothèse**). 20, 30 et 50 : vitesses moyennes de l'essai, démarrage et arrêt compris (48–52, 112–115, 256 mm/s ; Analyse_filtres_ultrason_E05.md, 9 nov.) ; [13, 0] et [100, 614] : **hypothèses** (614 extrapolé, cohérent avec 180 tr/min × roue de 65 mm, manuel p. 61). Modèle idéal : 4 mm/s par commande (`VALEURS_IDEALES`) | `table_vitesse_mm_s` (le modèle, les freinages et l'odométrie la lisent par `ReglagesRobot.vitesse_mm_s` et `commande_pour_vitesse`). Remplace `vitesse_mm_s_par_cmd`, supprimée du fichier | #18 | Hypothèse (3 points issus d'essais en mouvement) ; à mesurer en régime établi |
| Vitesse maximale du robot (commande 100), utilisée par le filtre de l'ultrason pour rejeter une vitesse impossible | 614 mm/s (**hypothèse** : point haut de la table ci-dessus, extrapolé) ; `reglages_robot.gd` `vitesse_max_mm_s()`, lue par `filtre_ultrason.gd` (`ultrason:redemarrer_si_vitesse_impossible` de `reglages_algorithmes.json`) | `table_vitesse_mm_s` (point [100, 614]) | #18 | Hypothèse, à mesurer |
| Zone morte des moteurs (avant et arrière) | 13 (**hypothèse** : ≈ 13 noté dans `SUIVI_SIMULATION.md`, sans mesure). Elle découle de la table (point [13, 0]) : la clé `zone_morte_moteur_cmd` est supprimée (une valeur, un endroit). Vitesses minimales des algorithmes (dans `reglages_algorithmes.json`), maintenant en **mm/s** (ex-commandes × 4) : la zone morte est gérée par la table, qui convertit en commande au-dessus de 13 : `robot:vitesse_approche_min_mm_s` (32), `arret:vitesse_recul_mm_s` (40), `recul:vitesse_recul_ligne_mm_s` (60), `rattrapage:vitesse_mm_s` (60) | `table_vitesse_mm_s` | #51 | Hypothèse, à mesurer |
| Temps de réponse du moteur | 0 s (instantané) | `constante_temps_moteur_s` | #51 | À mesurer |
| Vitesse du servo de direction | 300 ° de servo par seconde (manuel p. 61, à vide : 0,12 à 0,14 s/60°) | `vitesse_servo_deg_s` | #51 | À mesurer |
| Angle réel des roues (servo à 45 et 135) | 45° (rayon minimal 138 mm) ; mesuré (Mehdi, confirmé le 14 nov., revue de l'étape 2) | `angle_roues_max_deg` | #19 | Mesuré |
| Dérive en ligne droite (servo à 90) | aucune (dérive à droite à basse vitesse observée) | non modélisée | #18, #51 | À mesurer |

## Géométrie

| Valeur | Actuelle | Clé de `reglages_robot.json` | Tâche | Statut |
| --- | --- | --- | --- | --- |
| Essieu arrière → face de l'ultrason | 193 mm | `essieu_ultrason_mm` | #49 | Mesuré (modèle Blender) |
| Essieu arrière → barre de sondes | 217 mm | `essieu_sondes_mm` | #49 | Mesuré (modèle Blender) |
| Écart entre sondes voisines, hauteur du dessous de la barre | 19 mm ; 15 mm (lus par aucun code pour l'instant, prévus pour le Python) | `ecart_sondes_mm`, `hauteur_sondes_mm` | #52 | Mesuré (modèle Blender) |
| Face de l'ultrason → avant du robot | 36 mm. **Écart** : `config.py` (branche `main`) `ULTRASONIC_OFFSET = 35.3` | `ultrason_avant_mm` | #20 | Mesuré (modèle Blender) |
| Dépassement derrière l'essieu arrière | 56 mm : arrière des moteurs dans `picar.glb` (la plaque du bas s'arrête à 45 mm). Choix de Mehdi du 14 nov. | `arriere_voiture_mm` | #49 | Mesuré (modèle Blender) |
| Hauteur de l'ultrason au-dessus du sol | 60 mm (LISEZMOI Blender, mesure à la règle) ; cône supposé circulaire (même demi-angle en hauteur), rempli de rayons tous les 2,5° (`PAS_CONE_ULTRASON_DEG`, revue 03) : un bloc couché de 50 mm est vu (HYPOTHÈSE, à mesurer). **Écho du sol non simulé** | `hauteur_ultrason_mm` (vérifiée contre `Capteur_Ultrason` du modèle 3D) | #49, #53 | Mesuré (modèle Blender) |
| Empattement, roues, encombrement | 138 mm ; roue de 65 mm (205 mm par tour) ; 285 × 141 × 80 mm | `empattement_mm`, `rayon_roue_mm`, `longueur_mm`, `largeur_mm`, `hauteur_mm`, `centre_encombrement_y_mm`, `centre_encombrement_z_mm` | #49 | Mesuré (modèle Blender) |

Les positions des sondes et de l'ultrason sont aussi dessinées dans le modèle 3D (`picar.glb`). Au démarrage, la simulation signale tout écart de plus de 5 mm entre le fichier et le modèle : après une mesure, mettre aussi le modèle Blender à jour (#55).

## Capteurs

| Valeur | Actuelle (provisoire) | Clé de `reglages_robot.json` | Tâche | Statut |
| --- | --- | --- | --- | --- |
| Cône de l'ultrason | 30° au total, choix de Mehdi du 14 nov. (manuel : environ 30°, p. 60 ; 30 à 40°, p. 34 ; avant : 15°) | `demi_angle_ultrason_deg` (15) | #53 | À mesurer |
| Bruit de l'ultrason | 2,5 mm (écart type ; TEX « Filtre tes connaissances », 10 nov., tableau 1 : 2,4 / 2,5 / 2,7 mm à 100 / 300 / 600 mm). Modèle idéal : 0 | `bruit_ultrason_mm` (remplace `bruit_ultrason_cm`) | #53 | Mesuré (9 nov.) |
| Valeurs aberrantes de l'ultrason | 1 % des mesures, écart de 10 à 19 mm de signe aléatoire (amplitude uniforme ; Analyse_filtres_ultrason_E05.md, 9 nov.). Modèle idéal : aucune | `proba_aberrante_ultrason` (0,01), `aberrante_ultrason_mm` ([10, 19]) | #53 | Mesuré (9 nov.) |
| Biais de l'ultrason | mesure = 0,971 × distance réelle (depuis la face du capteur) + 8,3 mm (TEX section 6.5 : 0,971 d + 7,3 mm à l'avant ; décalage capteur–avant 35,3 mm). Modèle idéal : gain 1, biais 0. Depuis l'étape 10 (14 nov.), le contrôleur inverse cette loi : d = (z − biais) / gain (`ReglagesRobot.corriger_ultrason_cm`) ; le robot réel utilisera la droite du 19 nov. | `gain_ultrason` (0,971), `biais_ultrason_mm` (8,3) | #23 | Provisoire (étalonnage du 19 nov. à venir) |
| Cadence de l'ultrason | 16,13 mesures/s (période nominale 62 ms ; TEX section 1 ; `config.py` `ULTRASON_PERIODE_MIN_S = 0.06`). Modèle idéal : 20 mesures/s | `frequence_ultrason_hz` | #53 | Hypothèse (période nominale, gigue ± 2 ms non modélisée) |
| Retard de l'ultrason | 0 s : **hypothèse**. 0,3 s observé en mouvement avec l'ancien pilote (moyenne de 5 mesures de 60 ms, cause supposée, TEX section 2) ; 0 avec le pilote `fastdrink` : à tester en sensibilité. Retard pur dans `vehicule.gd` | `retard_ultrason_s` | #53 | Hypothèse, à tester |
| Filtre de l'ultrason | médiane de 3 + Kalman à vitesse constante (σ_z 2,4 mm, rejet à 9, reset après 5 rejets, plage 20–2000 mm ; `config.py`). **Étape 10 (écarts voulus avec `config.py` et `mediane_kalman.py`, à reporter à l'étape 11) :** σ_a **1000 mm/s²** au lieu de 400 (**hypothèse** réglée en simulation : erreur d'arrêt max 11,7 → 9,3 mm sur 40 pistes ; `reglages_algorithmes.json` `ultrason:sigma_a_mm_s2`) ; redémarrage du Kalman si |v| > vitesse max ; écho manqué sans prédiction. Modèle idéal : médiane de 3 seule | `filtre_ultrason` (`mediane_kalman` ou `mediane3`) ; constantes du Kalman dans `filtre_ultrason.gd`, réglages dans `reglages_algorithmes.json` (objet `ultrason`) | #65 | Retenu par l'équipe ; σ_a = 1000 : hypothèse, à vérifier sur le robot |
| Erreur de l'odométrie sur la vitesse | 0 (étalonnage supposé parfait : même table que le modèle, × (1 + valeur)) | `erreur_vitesse_odometrie` | #18 | Hypothèse (essais de sensibilité) |
| Précision de la distance de recul (revue 04, 14 nov.) | Aucun capteur de roue : la distance reculée est la vitesse estimée par la table **en marche arrière** (hypothèse « même table en arrière »), intégrée. En simulation elle est exacte par construction (150,0 à 150,6 mm sur les bancs). Sensibilité (essais de simulation de la revue 04, `reference_droite`, réaliste ; pas une mesure du robot) : odométrie qui surestime la vitesse de 10 % → 136,3 mm reculés ; qui la sous-estime de 10 % → 166,2 mm. La limite du guide (< 200 mm) tient jusqu'à 25 % de sous-estimation. À mesurer : vitesse réelle en arrière vers 60 mm/s (`recul:vitesse_recul_ligne_mm_s`, commande ≈ 22 par interpolation de la table hypothétique entre [20, 50] et [30, 113]) | `table_vitesse_mm_s`, `erreur_vitesse_odometrie` | #18 | À mesurer |
| Portée de l'ultrason | 3,0 m (manuel : non précisée) | `portee_ultrason_m` | #53 | À mesurer |
| Zone vue par une sonde | rayon 3 mm | `rayon_tache_sonde_mm` | #52 | À mesurer |
| Lectures fausses des sondes | 0 | `proba_erreur_sonde` | #52 | À mesurer |

## Bille et programme

| Valeur | Actuelle (provisoire) | Clé de `reglages_robot.json` | Tâche | Statut |
| --- | --- | --- | --- | --- |
| Masse de la bille | 10 g | `masse_bille_kg` | #50 | À mesurer |
| Rayon de la bille | 7,5 mm (Ø 15 mesuré à la règle, LISEZMOI Blender) | `rayon_bille_mm` | #50 | Mesuré |
| Pertes de la bille (roulement) | 1 /s | `amortissement_bille_1_s` | #47 | À comparer au vrai robot |
| Cuvette | rayon de courbure 140 mm, bord à 20 mm (plan `support_bille_v2` : R140, Ø40, unités non écrites) ; fréquence propre calculée ≈ 1,2 Hz | `rayon_cuvette_mm`, `rayon_bord_cuvette_mm` | #47 | À vérifier sur la pièce |
| Fréquence de la boucle du programme sur le Pi | 50 Hz | `frequence_controle_hz` | #64 | À mesurer |
| Rampe d'accélération (bille) | 240 mm/s², soit 0,24 m/s² (choisie le 13 nov. sur le banc : bille max 0,99 → 0,82 ; ex-60 commandes/s à 4 mm/s par commande). Freinage : 500 mm/s² (ex-125 commandes/s). Ce sont des mm/s² : la table du moteur convertit en commande à la sortie | `reglages_algorithmes.json` `robot:acceleration_montee_mm_s2` et `robot:deceleration_freinage_mm_s2` | #67 | À revoir avec la fréquence propre réelle de la bille et la vitesse réelle. Sensible au rayon de la cuvette : à 182 mm (+30 %), bille à 0,99 avec 60, chute avec 125 |

## Algorithmes (valeurs réglées en simulation, étape 10)

Hors `reglages_robot.json`, dans **`reglages_algorithmes.json`** : valeurs choisies en simulation, **hypothèses**, à revoir avec le robot réel.

| Valeur | Actuelle | Où dans le code | Tâche | Statut |
| --- | --- | --- | --- | --- |
| Accélération non modélisée du Kalman (σ_a) | 1000 mm/s² (`config.py` : 400 ; 40 pistes : erreur d'arrêt max 11,7 → 9,3 mm). À reporter dans `config.py` (étape 11) | `reglages_algorithmes.json` `ultrason:sigma_a_mm_s2` | #65 | Hypothèse, à vérifier |
| Tentatives de rattrapage et côté gardé | `tentatives_max` 4 (plus petite valeur qui passe, graine 47515) ; `garder_cote` vrai. Épingles de rayon ≈ 125 à 140 mm, sous le rayon minimal de la voiture (138 mm) | `reglages_algorithmes.json` `rattrapage:tentatives_max`, `rattrapage:garder_cote` | #75 | Hypothèse |
| Durée maximale de l'arrêt d'urgence en manœuvre (écho filtré sous `distance_securite_mm` = 40 mm) | 3,0 s (**HYPOTHESE**, 14 nov., Mehdi) : doit dépasser `absence_proche_s` (1,0 s), plus 2 s pour un obstacle passager (main), sans mesure ; au-delà, la manœuvre se termine comme au timeout de 10 s (RETROUVER ou BLOQUE) | `reglages_algorithmes.json` `controleur:arret_urgence_max_s` | #64 | À vérifier sur le robot |
| Plancher de vitesse de la loi de recul | 20 mm/s (**HYPOTHESE**, sans source ; était codé en dur dans `recul_ligne.gd`, déplacé par la revue 04) : évite la division par une vitesse presque nulle au démarrage du recul | `reglages_algorithmes.json` `recul:vitesse_min_loi_mm_s` | aucune | Hypothèse |
| Marge de la vérification du contournement fluide | 20 mm, inchangée ; son ancienne justification (retard de suivi de 12 mm) ne tient plus depuis que la vérification rejoue la poursuite (marge prévue ≈ réelle à 3 à 6 mm près) | `reglages_algorithmes.json` `fluide:marge_obstacle_mm` | #76 | Hypothèse, à revoir avec les essais sur le robot |
| Reprise de la ligne en fin de contournement fluide | 260 mm avant la fin du retour (**HYPOTHESE**, revue 05, 14 nov., Mehdi ; était 225) : à 225, la graine libre 269247 (réaliste) croisait la ligne avant l'ouverture de la reprise ; à 300, la bille de la graine 7920 montait de 0,61 à 0,87. Essayée sur ces 2 graines seulement ; la bille y est sensible (effet de seuil entre 260 et 280) | `reglages_algorithmes.json` `fluide:reprise_avant_fin_mm` | aucune | Hypothèse, à revoir avec les essais sur le robot |
| Retour stable sur la ligne (mesure du banc seulement) | 0,5 s de SUIVI avec les sondes sur la ligne (**HYPOTHESE** de mesure, revue 05) ; sans effet sur la conduite | `banc_essai.gd` `RETOUR_STABLE_S` | aucune | Hypothèse de mesure |
| Limite de rotation du braquage pendant la reprise douce | 60 °/s (**HYPOTHESE**, revue 06, 15 nov., Mehdi ; était 120) : bille max 1,00 → 0,80 sur 5 graines, puis banc complet (0,94 → 0,80 idéal, 1,00 → 0,83 réaliste sur les pistes libres) ; quelques graines montent (15839 : 0,66 → 0,82 à 0,89). Effet de seuil | `reglages_algorithmes.json` `reprise:braquage_max_deg_s` | aucune | Hypothèse, à revoir avec les essais sur le robot |
| Rotation du demi-tour (ligne revue / maximale) | 150° / 210° (**HYPOTHESE**, sans source ; valeurs qui étaient codées en dur dans `demi_tour.gd`, déplacées par la revue 06, même valeur) | `reglages_algorithmes.json` `demi_tour:angle_ligne_deg`, `demi_tour:angle_max_deg` | aucune | Hypothèse ; aucun banc n'entre en demi-tour |
| Arrêt en bout de ruban sans barre d'arrivée | ARRET vaut arrivée (**HYPOTHESE**, revue 08, 15 nov. ; avant : BLOQUE accepté par le banc) ; BLOQUE = échec | `rattrapage.gd` `Reaction.FIN_LIGNE`, `scenarios.gd` `courbes_120` | aucune | Hypothèse, à confirmer avec le professeur |
| Borne de l'intégrale et filtre de la dérivée du suivi | `INTEGRALE_MAX` 60 mm·s (au plus 30° de servo) ; `FILTRE_DERIVEE` 0,3 **par tour de boucle** (≈ 56 ms à 50 Hz ; si la fréquence mesurée sur le Pi diffère, garder ≈ 56 ms : 1 − exp(−dt / 0,056)). Réglages par essais, sans source écrite. Encore en `const` : passage dans `reglages_algorithmes.json` reporté jusqu'à la reprise du Python (le chargeur Python refuse une clé inconnue ; revue de l'étape 2, P2) | `simulation/suivi_ligne.gd` (`const`) ; `code_picar/fastdrink/conduite/suivi_ligne.py` | #64 | Hypothèse, à vérifier sur le robot |

## Épreuve (à demander au professeur)

Ce ne sont pas des valeurs du robot : elles restent dans les fichiers de la simulation.

| Valeur | Actuelle (valeur de test) | Où dans le code | Tâche |
| --- | --- | --- | --- |
| Distance d'arrêt devant l'obstacle | 100 mm | `reglages_algorithmes.json` `epreuve:distance_arret_mm` | #12 |
| Distance de recul | 150 mm | `reglages_algorithmes.json` `epreuve:distance_recul_mm` | #12 |
| Ligne droite pour s'aligner avant un obstacle (hypothèse) | 140 mm | `parametres_piste.gd` `marge_droite_avant` | #56 |
| Pas de ruban dans les cases latérales de la droite d'un obstacle (grille) | règle (HYPOTHÈSE : « toujours la place pour la manœuvre » non chiffré) | `generateur_piste.gd` `_case_droite` | #56 |
| Positions possibles de l'obstacle (debout, tourné, couché) | 6 poses tirées au hasard ; contournement de la plus grande emprise (150 × 150) | `parametres_piste.gd` `faces_aleatoires`, `reglages_epreuve.gd` `largeur_a_contourner_mm()` | à créer |
| Largeur du ruban | 18 mm (guide) ; 20 mm dans le parcours fourni | pistes | #54 |
| Angles des virages (hypothèse ; le guide impose seulement le rayon) | 20 à 180° sur les pistes libres | `parametres_piste.gd` `angle_min_deg`, `angle_max_deg` | #73 |
| Écart minimal entre deux parties du ruban (hypothèse) | 240 mm (deux lignes à une case d'écart sur la grille) | `parametres_piste.gd` `ecart_lignes_min` | #73 |
| Longueur des droites entre virages (hypothèse) | 0,15 à 2,5 m (un obstacle demande ~1,3 m) | `parametres_piste.gd` `droite_libre_min`, `droite_libre_max` | #73 |
