# Mesures du robot à faire – FastDrink E05

Mis à jour le 13 novembre 2026. Liste de toutes les valeurs **provisoires** de la simulation : à remplacer par des mesures sur le vrai robot.

**Toutes les valeurs du robot sont dans un seul fichier : `reglages_robot.json` (racine du dépôt).** Il est lu par la simulation et sera lu par le code Python du robot. Chaque entrée a une valeur, une unité, un statut (`provisoire` ou `mesuré`), une source et une tâche.

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
| Vitesse réelle par commande, avant et arrière | proportionnelle : 4 mm/s par unité (0,40 m/s à 100). Premières valeurs (rapport des filtres, démarrage compris) : 48–52 mm/s à 20, 112–115 à 30, 256 à 50 | `vitesse_mm_s_par_cmd` (le modèle et l'odométrie en dérivent) | #18 | À mesurer |
| Zone morte des moteurs (avant et arrière) | 0 (≈ 13 noté dans `SUIVI_SIMULATION.md`, sans mesure documentée) | `zone_morte_moteur_cmd`. Commandes minimales des algorithmes (hors fichier) : `reglages_robot.gd` `vitesse_approche_min` (8), `arret_obstacle.gd` `vitesse_recul` (10), `recul_ligne.gd` `vitesse_recul_ligne` (15), `rattrapage.gd` `vitesse` (15) | #51 | À mesurer |
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
| Bruit de l'ultrason | 0 (mesuré à l'arrêt : écart type ≈ 2,4 mm) | `bruit_ultrason_cm` | #53 | Mesure existante à reporter |
| Biais de l'ultrason | aucun (mesuré : ≈ 0,971 × d + 7,3 mm) | non modélisé | #23 | Mesure existante à reporter |
| Retard et cadence de l'ultrason | 20 mesures/s, sans retard (essais en mouvement : ≈ 0,3 s de retard, à confirmer). **Écart** : le manuel (p. 32) conseille un cycle de plus de 60 ms, soit au plus 16,7 mesures/s ; `config.py` (main) `ULTRASON_PERIODE_MIN_S = 0.06` | `frequence_ultrason_hz` ; filtre : `filtre_ultrason.gd` | #53 | À mesurer |
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
| Rampe d'accélération (bille) | 60 commandes/s, soit 0,24 m/s² avec 4 mm/s par commande (choisie le 13 nov. sur le banc : bille max 0,99 → 0,82). Freinage : 125 commandes/s | hors fichier : `controleur.gd` `rampe_montee_cmd_par_s`, `reglages_robot.gd` `rampe_cmd_par_s` | #67 | À revoir avec la fréquence propre réelle de la bille et la vitesse réelle. Sensible au rayon de la cuvette : à 182 mm (+30 %), bille à 0,99 avec 60, chute avec 125 |

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
