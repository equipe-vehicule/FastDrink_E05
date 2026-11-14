# Mesures du robot à faire – FastDrink E05

Mis à jour le 13 novembre 2026. Liste de toutes les valeurs **provisoires** de la simulation : à remplacer par des mesures sur le vrai robot.

**Toutes les valeurs du robot sont dans un seul fichier : `reglages_robot.json` (racine du dépôt).** Il est lu par la simulation et sera lu par le code Python du robot. Chaque entrée a une valeur, une unité, un statut (`provisoire`, `mesuré` ou `hypothèse`), une source et une tâche.

Quand une mesure est faite :
1. noter le résultat en commentaire de la tâche GitHub ;
2. dans `reglages_robot.json`, changer la valeur, passer le statut à `mesuré` et écrire la source (date, méthode) ;
3. passer la ligne ci-dessous à « Mesuré » ;
4. relancer les essais de la simulation.

Pour les valeurs de `reglages_robot.json`, il n'y a rien à changer dans le code. Les valeurs marquées « hors fichier » dans les tableaux, elles, sont dans le code (fichier indiqué).

Tant que ces valeurs sont provisoires, la simulation ne prédit pas le robot : elle valide seulement la logique des algorithmes.

## Robot (moteurs, direction)

| Valeur | Actuelle (provisoire) | Clé de `reglages_robot.json` | Tâche | Statut |
| --- | --- | --- | --- | --- |
| Vitesse réelle par commande, avant et arrière | Table `[commande, mm/s]` = [[0, 0], [13, 0], [20, 50], [30, 113], [50, 256], [100, 614]], interpolation linéaire, même table en arrière (**hypothèse**). 20, 30 et 50 : vitesses moyennes de l'essai, démarrage et arrêt compris (48–52, 112–115, 256 mm/s ; Analyse_filtres_ultrason_E05.md, 9 nov.) ; [13, 0] et [100, 614] : **hypothèses** (614 extrapolé, cohérent avec 180 tr/min × roue de 65 mm, manuel p. 61). Modèle idéal : 4 mm/s par commande (`VALEURS_IDEALES`) | `table_vitesse_mm_s` (le modèle, les freinages et l'odométrie la lisent par `ReglagesRobot.vitesse_mm_s` et `commande_pour_vitesse`). Remplace `vitesse_mm_s_par_cmd`, supprimée du fichier | #18 | Hypothèse (3 points issus d'essais en mouvement) ; à mesurer en régime établi |
| Zone morte des moteurs (avant et arrière) | 13 (**hypothèse** : ≈ 13 noté dans `SUIVI_SIMULATION.md`, sans mesure). Elle découle de la table (point [13, 0]) : la clé `zone_morte_moteur_cmd` est supprimée (une valeur, un endroit). Vitesses minimales des algorithmes (hors fichier), maintenant en **mm/s** (ex-commandes × 4) : la zone morte est gérée par la table, qui convertit en commande au-dessus de 13 : `reglages_robot.gd` `vitesse_approche_min_mm_s` (32), `arret_obstacle.gd` `vitesse_recul_mm_s` (40), `recul_ligne.gd` `vitesse_recul_ligne_mm_s` (60), `rattrapage.gd` `vitesse_mm_s` (60) | `table_vitesse_mm_s` | #51 | Hypothèse, à mesurer |
| Temps de réponse du moteur | 0 s (instantané) | `constante_temps_moteur_s` | #51 | À mesurer |
| Vitesse du servo de direction | 300 ° de servo par seconde (manuel p. 61, à vide : 0,12 à 0,14 s/60°) | `vitesse_servo_deg_s` | #51 | À mesurer |
| Angle réel des roues (servo à 45 et 135) | 45° (rayon minimal 138 mm) | `angle_roues_max_deg` | #19 | À mesurer |
| Dérive en ligne droite (servo à 90) | aucune (dérive à droite à basse vitesse observée) | non modélisée | #18, #51 | À mesurer |

## Géométrie

| Valeur | Actuelle (provisoire) | Clé de `reglages_robot.json` | Tâche | Statut |
| --- | --- | --- | --- | --- |
| Essieu arrière → face de l'ultrason | 193 mm (modèle 3D) | `essieu_ultrason_mm` | #49 | À mesurer |
| Essieu arrière → barre de sondes | 217 mm (modèle 3D) | `essieu_sondes_mm` | #49 | À mesurer |
| Face de l'ultrason → avant du robot | 36 mm (modèle 3D). **Écart** : `config.py` (branche `main`) `ULTRASONIC_OFFSET = 35.3` | `ultrason_avant_mm` | #20 | À mesurer |
| Dépassement derrière l'essieu arrière | 45 mm (LISEZMOI Blender). **Écart** : la boîte d'encombrement donne 56 mm (285 − 229) | `arriere_voiture_mm` | #49 | À mesurer |
| Hauteur de l'ultrason au-dessus du sol | 60 mm (LISEZMOI Blender, mesure à la règle) ; cône supposé circulaire (même demi-angle en hauteur) : un bloc couché de 50 mm est vu. **Écho du sol non simulé** | `hauteur_ultrason_mm` (vérifiée contre `Capteur_Ultrason` du modèle 3D) | #49, #53 | À mesurer (hauteur, cône vertical, écho du sol) |
| Empattement, roues, encombrement | 138 mm ; roue de 65 mm (205 mm par tour) ; 285 × 141 × 80 mm | `empattement_mm`, `rayon_roue_mm`, `longueur_mm`, `largeur_mm`, `hauteur_mm`, `centre_encombrement_y_mm`, `centre_encombrement_z_mm` | #49 | À vérifier |

Les positions des sondes et de l'ultrason sont aussi dessinées dans le modèle 3D (`picar.glb`). Au démarrage, la simulation signale tout écart de plus de 5 mm entre le fichier et le modèle : après une mesure, mettre aussi le modèle Blender à jour (#55).

## Capteurs

| Valeur | Actuelle (provisoire) | Clé de `reglages_robot.json` | Tâche | Statut |
| --- | --- | --- | --- | --- |
| Cône de l'ultrason | 15° au total (manuel : 30 à 40°, p. 34 ; environ 30°, p. 60) | `demi_angle_ultrason_deg` (7,5) | #53 | À mesurer |
| Bruit de l'ultrason | 2,5 mm (écart type ; TEX « Filtre tes connaissances », 10 nov., tableau 1 : 2,4 / 2,5 / 2,7 mm à 100 / 300 / 600 mm). Modèle idéal : 0 | `bruit_ultrason_mm` (remplace `bruit_ultrason_cm`) | #53 | Mesuré (9 nov.) |
| Valeurs aberrantes de l'ultrason | 1 % des mesures, écart de 10 à 19 mm de signe aléatoire (amplitude uniforme ; Analyse_filtres_ultrason_E05.md, 9 nov.). Modèle idéal : aucune | `proba_aberrante_ultrason` (0,01), `aberrante_ultrason_mm` ([10, 19]) | #53 | Mesuré (9 nov.) |
| Biais de l'ultrason | mesure = 0,971 × distance réelle (depuis la face du capteur) + 8,3 mm (TEX section 6.5 : 0,971 d + 7,3 mm à l'avant ; décalage capteur–avant 35,3 mm). Modèle idéal : gain 1, biais 0. Aucune compensation dans le contrôleur | `gain_ultrason` (0,971), `biais_ultrason_mm` (8,3) | #23 | Provisoire (étalonnage du 19 nov. à venir) |
| Cadence de l'ultrason | 16,13 mesures/s (période nominale 62 ms ; TEX section 1 ; `config.py` `ULTRASON_PERIODE_MIN_S = 0.06`). Modèle idéal : 20 mesures/s | `frequence_ultrason_hz` | #53 | Hypothèse (période nominale, gigue ± 2 ms non modélisée) |
| Retard de l'ultrason | 0 s : **hypothèse**. 0,3 s observé en mouvement avec l'ancien pilote (moyenne de 5 mesures de 60 ms, cause supposée, TEX section 2) ; 0 avec le pilote `fastdrink` : à tester en sensibilité. Retard pur dans `vehicule.gd` | `retard_ultrason_s` | #53 | Hypothèse, à tester |
| Filtre de l'ultrason | médiane de 3 + Kalman à vitesse constante (σ_z 2,4 mm, σ_a 400 mm/s², rejet à 9, reset après 5 rejets, plage 20–2000 mm ; `config.py`). Modèle idéal : médiane de 3 seule | `filtre_ultrason` (`mediane_kalman` ou `mediane3`) ; paramètres dans `filtre_ultrason.gd` | #65 | Retenu par l'équipe |
| Erreur de l'odométrie sur la vitesse | 0 (étalonnage supposé parfait : même table que le modèle, × (1 + valeur)) | `erreur_vitesse_odometrie` | #18 | Hypothèse (essais de sensibilité) |
| Portée de l'ultrason | 3,0 m (manuel : non précisée) | `portee_ultrason_m` | #53 | À mesurer |
| Zone vue par une sonde | rayon 3 mm | `rayon_tache_sonde_mm` | #52 | À mesurer |
| Lectures fausses des sondes | 0 | `proba_erreur_sonde` | #52 | À mesurer |

## Bille et programme

| Valeur | Actuelle (provisoire) | Clé de `reglages_robot.json` | Tâche | Statut |
| --- | --- | --- | --- | --- |
| Masse de la bille | 10 g | `masse_bille_kg` | #50 | À mesurer |
| Rayon de la bille | 7,5 mm. **Le plan `support_bille_v2` ne le donne pas** (bille fournie dans la boîte) | `rayon_bille_mm` | #50 | À mesurer |
| Pertes de la bille (roulement) | 1 /s | `amortissement_bille_1_s` | #47 | À comparer au vrai robot |
| Cuvette | rayon de courbure 140 mm, bord à 20 mm (plan `support_bille_v2` : R140, Ø40, unités non écrites) ; fréquence propre calculée ≈ 1,2 Hz | `rayon_cuvette_mm`, `rayon_bord_cuvette_mm` | #47 | À vérifier sur la pièce |
| Fréquence de la boucle du programme sur le Pi | 50 Hz | `frequence_controle_hz` | #64 | À mesurer |
| Rampe d'accélération (bille) | 240 mm/s², soit 0,24 m/s² (choisie le 13 nov. sur le banc : bille max 0,99 → 0,82 ; ex-60 commandes/s à 4 mm/s par commande). Freinage : 500 mm/s² (ex-125 commandes/s). Ce sont des mm/s² : la table du moteur convertit en commande à la sortie | hors fichier : `reglages_robot.gd` `acceleration_montee_mm_s2` et `deceleration_freinage_mm_s2` | #67 | À revoir avec la fréquence propre réelle de la bille et la vitesse réelle. Sensible au rayon de la cuvette : à 182 mm (+30 %), bille à 0,99 avec 60, chute avec 125 |

## Épreuve (à demander au professeur)

Ce ne sont pas des valeurs du robot : elles restent dans les fichiers de la simulation.

| Valeur | Actuelle (valeur de test) | Où dans le code | Tâche |
| --- | --- | --- | --- |
| Distance d'arrêt devant l'obstacle | 100 mm | `reglages_epreuve.gd` `distance_arret_mm` | #12 |
| Distance de recul | 150 mm | `reglages_epreuve.gd` `distance_recul_mm` | #12 |
| Ligne droite pour s'aligner avant un obstacle (hypothèse) | 140 mm | `parametres_piste.gd` `marge_droite_avant` | #56 |
| Positions possibles de l'obstacle (debout, tourné, couché) | 6 poses tirées au hasard ; contournement de la plus grande emprise (150 × 150) | `parametres_piste.gd` `faces_aleatoires`, `reglages_epreuve.gd` `largeur_a_contourner_mm()` | à créer |
| Largeur du ruban | 18 mm (guide) ; 20 mm dans le parcours fourni | pistes | #54 |
| Angles des virages (hypothèse ; le guide impose seulement le rayon) | 20 à 180° sur les pistes libres | `parametres_piste.gd` `angle_min_deg`, `angle_max_deg` | #73 |
| Écart minimal entre deux parties du ruban (hypothèse) | 240 mm (deux lignes à une case d'écart sur la grille) | `parametres_piste.gd` `ecart_lignes_min` | #73 |
| Longueur des droites entre virages (hypothèse) | 0,15 à 2,5 m (un obstacle demande ~1,3 m) | `parametres_piste.gd` `droite_libre_min`, `droite_libre_max` | #73 |
