# Suivi de la simulation – FastDrink E05

Mis à jour le 13 novembre 2026 (contournement par phases vérifié, #72 ; générateur de pistes libre et virages de tout angle, #73 ; stabilité du suivi, rampe d'accélération pour la bille, banc aux cas indépendants ; obstacles posés sur différentes faces ; essais rejouables, boucle à 50 Hz, mesures de stabilité ; fichier unique des valeurs du robot ; avant : séquence arrêt → recul → contournement, 12 nov.). Ce fichier garde l'état de la simulation, ses limites, les directives pour le portage sur le robot et les tâches restantes. Le détail des modifications est dans `LISEZMOI_SIMULATION.md`.

## 0. Où en est le projet et ce qu'il reste à faire (13 novembre)

### Ce qui est fait (simulation Godot)

| Exigence (guide + précisions du professeur) | État en simulation |
| --- | --- |
| Suivre un ruban de 18 mm, courbes de 120 mm et plus | Fait : PID + rattrapage en marche arrière dans les virages serrés |
| S'arrêter à une distance prédéterminée de l'obstacle (± 30 mm) | Fait : erreur max 3,9 mm sur le banc de référence (D_arret de test = 100 mm) |
| Reculer en suivant la ligne (< 200 mm) | Fait : 150 mm (valeur de test) à chaque obstacle ; finit tout droit si la ligne sort des sondes |
| Devant chaque obstacle : arrêt, recul, puis contournement | Fait : état `RECUL_OBSTACLE`, distances dans `reglages_epreuve.gd` |
| Contourner l'obstacle (100 × 150 × 50 mm, posé sur n'importe quelle face, centré, en ligne droite) | Fait : contournement fluide de la plus grande emprise possible (150 × 150), repli par phases ; bloc couché de 50 mm vu par le cône vertical de l'ultrason (hypothèse : cône circulaire). Marge latérale min 14 mm sur 40 pistes à faces aléatoires (20 mm visés) |
| Transporter la bille sans la faire tomber | Fait en simulation : marge max 0,80 sur 40 pistes et 0,82 sur les 17 scénarios avec la rampe d'accélération de 60 cmd/s (13 nov., #67 ; 0,84 et 0,99 avant) |
| Pistes de test conformes | Pistes aléatoires conformes (obstacles en ligne droite, place pour la manœuvre) ; `test_01` et `test_04` remises aux vraies cotes |

Banc d'essai de référence depuis le 13 nov. (étape 5) : pas de 1/240 s, boucle à 50 Hz, faces de l'obstacle tirées au hasard, cas indépendants (rejouables un par un), rampe d'accélération de 60 cmd/s, `essais/reference_1-240.csv` ; 40/40 verdicts OK, 0 collision. Scénarios ciblés : 25/25 OK (`essais/scenarios_reference.csv`, détail dans `LISEZMOI_SIMULATION.md`). Avec D_recul 50 : 15/15, 0 collision depuis la vérification du contournement par phases (#72, étape 6 ; `essais/reference_recul50.csv`). Pistes libres, virages de 20 à 180° (#73) : 39/40, 0 collision, bille max 0,94 (`essais/reference_libre.csv`) ; échec de la graine 47515, voir plus bas. Stabilité : le suivi zigzague en ligne droite à environ 5 Hz, 8° de roue en médiane ; la bille (1,16 Hz) le filtre. Les réglages essayés pour le réduire (gains, limiteur, zone neutre, gain au centre) font monter la bille ou ajoutent des collisions : non retenus (étape 5). Bancs plus anciens (pas de 1/60 s) : 40/40 arrivées, 0 collision, 0 chute, 0 perte de ligne après un contournement. Autres valeurs (50/100, 150/190) : 0 collision ; 100/50 : 1 collision (voir plus bas). Détail : section 9.

**Depuis l'étape 9 (13 nov.), modèle réaliste par défaut** (bruit, biais, aberrantes, cadence de 16 Hz de l'ultrason, filtre médiane 3 + Kalman, table de vitesse des moteurs ; algorithmes en mm/s et mm/s²) : 40/40, 25/25, 15/15 (D_recul 50), 38/40 (pistes libres) ; sensibles au retard de l'ultrason et à l'étalonnage de la vitesse (détail : `LISEZMOI_SIMULATION.md`, étape 9). Références idéales : `--modele=ideal`.

**Ce que la simulation ne garantit pas encore :** les valeurs du modèle réaliste viennent du rapport des filtres (3 vitesses moyennes, ultrason à l'arrêt) ; zone morte, temps de réponse, servo et dimensions ne sont pas mesurés. Avant l'étape 9, elle était idéale. Pas de bruit ni de retard sur l'ultrason, pas de zone morte des moteurs, vitesse proportionnelle à la commande, odométrie parfaite, dimensions du robot prises sur le modèle 3D. Tant que ces écarts ne sont pas mesurés et ajoutés, un résultat de simulation ne prédit pas le robot.

### Ce qu'il reste à faire pour une simulation fidèle (par priorité)

**1. Mesurer le robot réel (prérequis à tout le reste)**
- [ ] Vitesse réelle par commande (table commande → mm/s, avant et arrière) et zone morte des moteurs. Premières mesures : 50 / 113 / 256 mm/s aux commandes 20 / 30 / 50, zone morte ≈ 13 (section 3).
- [ ] Angle de braquage maximal réel et vitesse du servo (simulation : 45° de roue, 300 °/s provisoire).
- [ ] Positions réelles de l'ultrason (193 mm) et des sondes (217 mm) par rapport à l'essieu arrière : « à confirmer à la règle » (`reglages_robot.gd`).
- [ ] Ultrason : bruit, retard de mesure (≈ 0,3 s observé), cône, calibration (z = 0,971 d + 7,3 du rapport).
- [ ] Fréquence réelle de la boucle sur le Raspberry Pi (simulation : 50 Hz).
- [ ] Masse de la bille et du robot, comportement de la bille dans la cuvette (amortissement provisoire).

**2. Mettre ces mesures dans le modèle (`vehicule.gd`, `odometrie.gd`, `filtre_ultrason.gd`)**
- [x] Table de vitesse (3 points mesurés, zone morte 13 et point à 100 = hypothèses) dans le modèle et l'odométrie (étape 9).
- [x] Bruit, biais, aberrantes et cadence de l'ultrason ; filtre médiane 3 + Kalman (`config.py`) (étape 9). Retard : 0 par défaut (hypothèse), 0,3 s testé en sensibilité : erreur d'arrêt jusqu'à 35,7 mm.
- [ ] Compenser le biais de l'ultrason dans le contrôleur (erreur d'arrêt 15 mm au lieu de 4).
- [ ] Retard du moteur et du servo.
- [ ] Vitesses minimales au-dessus de la zone morte : les algorithmes sont maintenant en mm/s (`vitesse_approche_min_mm_s` 32, `vitesse_recul_mm_s` 40) et la table convertit en commande ; reste à vérifier les valeurs sur le robot.
- [x] Banc repassé avec le modèle réaliste (étape 9) : faiblesses = retard de l'ultrason et étalonnage de la vitesse (odométrie +10 % : 1 collision ; retard 0,3 s : erreur d'arrêt 30,2 mm) ; bille tombée sur la graine 300923 (pistes libres).

**3. Algorithmes encore fragiles**
- [x] Contournement par phases : trajectoire vérifiée avant de partir (cap de 40 à 90°), arrêt sûr en ARRET si rien ne passe (#72, étape 6). D_recul 50 : 15/15, 0 collision.
- [ ] Contournement fluide : marges min 1,1 mm (D_recul 50, graine 95029) et 7,9 mm (pistes libres). Essai `marge_obstacle_mm` 40 : marges ≥ 25,7 mm mais le fluide n'est plus choisi, temps +13 à +29 % ; décision à prendre.
- [ ] Obstacle juste avant un virage plus serré que la voiture (graine 47515, pistes libres : épingle de 165° de rayon 126 mm) : la voiture sort du contournement décalée, perd la ligne, BLOQUE après 3 rattrapages.
- [ ] Bille à 0,94 sur les pistes libres (0,80 sur la grille) : à analyser.
- [ ] Marge latérale du contournement fluide : 14 mm au pire (bloc de 150 de large décalé de 10 mm) pour 40 mm prévus ; `marge_laterale_mm` à revoir après les mesures du robot.
- [ ] Ligne droite après l'obstacle (`droite_apres_min` = 0,60 m, hypothèse) : le retour du contournement du pire cas finit environ 0,68 m après l'arrière d'un bloc de 50 de profondeur (perte de ligne de 0,6 s dans `obstacle_avant_virage`) ; à confirmer avec le professeur (#56).
- [ ] Écho du sol sur l'ultrason (cône vertical, capteur à 60 mm) : non simulé, à mesurer (#53).
- [x] Pertes de ligne dans les virages de 120 mm (#74) : inévitables avec les sondes à 217 mm devant l'essieu (rayon minimal décrit par la barre : 257 mm), rattrapées en marche arrière. Nombre de rattrapages inchangé avec la rampe de 60 (2 dans `courbes_120`, 4 sur le parcours fourni).
- [x] Bille après un rattrapage (#67) : rampe d'accélération séparée (`reglages_robot.gd` `acceleration_montee_mm_s2` = 240, ex-`rampe_montee_cmd_par_s` = 60, provisoire) ; bille max 0,99 → 0,82 sur les scénarios.
- [ ] Fin de ligne non reconnue (`courbes_120` sans la rampe de 60) : la voiture reprend la ligne en sens inverse ; contrôle « fin de ligne atteinte » ajouté au banc le 13 nov. (raison `bloque_loin_fin_ligne`, seuil 150 mm = hypothèse).
- [ ] Rampe d'accélération : à revoir avec la fréquence propre réelle de la bille et la vitesse réelle par commande.
- [ ] Retour sur la ligne après le contournement fluide : arrivée avec un angle jusqu'à ~29° (piste 4), sans perte de ligne mais à adoucir.

**4. Pistes et exigences à confirmer avec le professeur**
- [ ] Valeurs officielles de D_arret et D_recul et leurs tolérances (à mettre dans `reglages_epreuve.gd`, puis repasser le banc).
- [ ] Espacement minimal virage → obstacle et entre deux obstacles (simulation : D_arret + 193 + D_recul + 140 mm, la marge de 140 mm est une hypothèse).
- [x] Angles des virages : générateur libre (`generateur = "libre"`, angles de 20 à 180°, hypothèses) et scénarios de 45, 135, 180° et S de 60° (#73, étape 6). Confirmer avec le professeur les angles et l'écart minimal entre deux parties du ruban (240 mm supposés).
- [ ] Côté du contournement imposé ou libre ; largeur du ruban (18 ou 20 mm).

**5. Code et portage**
- [x] Valeurs du robot regroupées dans `reglages_robot.json` (13 nov., #63 partie robot) : lu par la simulation, à relire par le code Python. Banc inchangé (CSV identique).
- [ ] Gains des algorithmes dans le même fichier (#63, suite) ; tests automatiques des modules (sans Godot).
- [x] Banc sans éditeur en ligne de commande (`--banc=40 --quitter`, environ 5 min, déterministe). Référence au pas de 1/240 s (13 nov.) ; `vitesse_temps` = 4 fait tourner la physique par pas de 1/60 s (bancs plus anciens).
- [x] Scénarios courts et rejouables (`--scenarios=tous`, 25 cas depuis l'étape 6, verdict par cas) et mesures de stabilité dans le banc (13 nov., étape 3).
- [x] Boucle du programme à 50 Hz exacts (elle tournait à 48 Hz, ou à 30 Hz au pas de 1/60 s).
- [x] Cas du banc indépendants (13 nov., étape 5) : un cas rejoué seul donne le même résultat que dans la série ; option `--journal` (journal détaillé sans éditeur). `--reglage` accepte aussi un réglage du contrôleur lui-même, sans module, ex. `--reglage=robot:acceleration_montee_mm_s2=160`.
- [ ] Portage Python module par module (section 4), validation sur le robot avant le 3 décembre, tableau simulation / réel pour le rapport QR (9 décembre).

## 1. État au 12 novembre, avant la séquence arrêt → recul (historique ; état actuel : section 0)

| Piste (touche) | Résultat | Exigence client |
| --- | --- | --- |
Revérifié le 12 novembre avec le code final (après les corrections de la section 7). « Bille max » = position de la bille rapportée au bord de la cuvette (1 = tombée).

| Piste (touche) | Résultat | Bille max | Exigence client |
| --- | --- | --- | --- |
| 1 Ligne droite | Suivie jusqu'au bout, arrêt à 9,3 s | 0,65 | Ruban de 18 mm |
| 2 Courbes 120 | Franchie, 3 rattrapages, arrêt à 23,8 s | **0,94** | Courbes de 120 mm et plus |
| 3 Arrêt devant obstacle | Immobile à ≈ 24 mm (ultrason 60,4 − 36) pour 30 visés | 0,65 | Arrêt à ± 30 mm |
| 4 Contournement | Fluide, 9,5 s, sans collision | 0,81 | Contournement |
| 5 Recul | 178,9 mm reculés en suivant la ligne (cible 180) | 0,65 | Recul < 200 mm |
| 6 Parcours fourni | Arrivée en 46,0 s | 0,85 | Synthèse |
| 7 Aléatoire (graine 884064) | Arrivée en 87,3 s, 2 obstacles (1 par phases, 1 fluide) | 0,81 | Synthèse |

Banc d'essai (40 pistes aléatoires, 38 obstacles) : 40 arrivées, 0 bille tombée, 0 collision, 0 demi-tour, bille max 0,88, temps moyen 64,4 s.

## 2. Choix retenus et pourquoi (mesures du 12 novembre)

| Sujet | Choix | Mesure qui le justifie |
| --- | --- | --- |
| Suivi de ligne | PID existant (kp 1,8, ki 0,5, kd 0,08) | Loi par table (retirée) : roues plus calmes mais 2 rattrapages par virage au lieu de 1, tour plus lent |
| Vitesse en suivi | 55 (au lieu de 40) | Parcours fourni : 55,9 s → 44,0 s. À 62 et 70, la bille tombe |
| Virage serré | Rattrapage en marche arrière (méthode de `line_follower.py` SunFounder) | Rayon minimal 138 mm (45° mesurés) ; sondes à 217 mm : un point de la barre décrit au moins √(138² + 217²) = 257 mm. Aucun virage < 217 mm ne se suit d'un trait |
| Ralentissement avant virage | Retiré | N'évite aucun rattrapage, tour plus lent (45,5 s contre 44,0) |
| Fin de ligne | Arrêt (BLOQUE) si la ligne disparaît centrée après 0,5 s de suivi, ou après 3 rattrapages ratés | Avant : errance jusqu'à 90 s |
| Contournement | Fluide en roulant (`contournement_fluide.gd`), vitesse 65, écart 250 mm, retour 450 mm, cap de départ par moindres carrés sur 400 mm de ligne ; repli sur le contournement par phases si l'obstacle est vu à moins de 520 mm ou si la voiture n'est pas dans l'axe | Piste 4 : 13,0 s → 9,5 s. Écart 250 = 100 (demi-obstacle) + 40 (décalage de l'obstacle) + 70,5 (demi-voiture) + 40 (marge) ; à 210 : 2 collisions sur 40 pistes |
| Reprise après manœuvre | Reprise douce : 1 s à vitesse ≤ 25 et braquage ≤ 120 °/s après contournement, rattrapage, demi-tour ou recherche | Sans elle, la bille tombait sur 2 pistes sur 40 (dont la piste 7) |
| Contournement par phases | Vitesse 25, pause 0,3 s (valeurs d'origine) | À 40 / 0,15 s, la bille tombait sur 4 pistes sur 6 |
| Recul | Piste Recul : arrêt en fin de ligne puis 180 mm | Module existant, jamais appelé avant |

## 3. Limites connues

1. Simulation idéale : bruit ultrason, biais, zone morte (≈ 13), retard moteur, retard de mesure (≈ 0,3 s) et vitesse non linéaire (50 / 113 / 256 mm/s aux commandes 20 / 30 / 50) ne sont pas modélisés.
2. Le contournement fluide et le rattrapage s'appuient sur l'odométrie calculée à partir de la commande (4 mm/s par unité). Sur le robot, ce rapport est faux et non linéaire : à recalibrer avant tout essai.
3. Le contournement fluide suppose un obstacle d'au plus 200 mm de large, décalé d'au plus 40 mm, vu à au moins 520 mm sur une ligne droite. Sur le banc, il n'est choisi que pour 16 obstacles sur 38 : les 22 autres sont vus trop près (obstacle peu après un virage) et passent par le contournement par phases, plus saccadé mais sans chute ni collision.
4. Validé sur le banc de 40 pistes aléatoires (graines 1 + i × 7919) et les 7 pistes de démonstration : toujours pas une garantie pour la piste mystère. Piste 2 : marge de la bille 0,94 (suivi à 55 dans les courbes serrées ; à 50 : 0,85 mais 7 rattrapages au lieu de 3).
5. Vitesses minimales (en mm/s depuis le 13 nov. : `vitesse_approche_min_mm_s` 32, `vitesse_recul_mm_s` 40, rattrapage 60) : la table les convertit en commandes au-dessus de la zone morte ; à confirmer sur le robot (fin du recul à −3 à −5 commandes à l'origine).
6. Vitesses 55 (suivi) et 65 (manœuvre) choisies en simulation ; la marge de la bille sur le vrai robot est inconnue (rampe exprimée en unités de commande, pas en m/s²).
7. Le filtre retenu dans le rapport (Kalman en mouvement, médiane + exponentiel à l'arrêt) n'est pas dans la simulation (médiane de 3 seulement). **Fait à l'étape 9** (`mediane_kalman`, `filtre_ultrason.gd`) ; reste : le filtre de l'arrêt (médiane + exponentiel).
8. La distance d'arrêt prédéterminée (30 mm avant l'obstacle) n'a pas de justification écrite.
9. Valeurs encore provisoires : vitesse du servo (300 °/s), fréquence de boucle (50 Hz), masse et amortissement de la bille, tache des sondes.

## 4. Directives pour le portage sur le robot (Python, Raspberry Pi)

1. Mesurer d'abord : vitesse réelle par commande (table), zone morte, vitesse du servo, fréquence réelle de la boucle, retard de la mesure ultrason.
2. Remplacer `mm_s_par_cmd` (linéaire) par une table commande → vitesse, utilisée par l'odométrie ET par le modèle de simulation. **Fait à l'étape 9** (`table_vitesse_mm_s`).
3. Remonter toutes les commandes minimales au-dessus de la zone morte mesurée.
4. Exprimer la rampe en m/s² (limite de la bille), puis la convertir en unités de commande avec la table.
5. Porter module par module, avec les mêmes noms qu'en GDScript (`suivi_ligne`, `arret_obstacle`, `contournement_fluide`, `recul_ligne`, `odometrie`, `filtre_ultrason`, `controleur`) ; garder la même interface `calculer(sondes, distance_cm, dt) -> (vitesse, angle)`.
6. Brancher le filtre du rapport (code `09_Code-PiCar/fastdrink/filtres/`) à la place de la médiane de 3, et appliquer la calibration de l'ultrason (z = 0,971 d + 7,3).
7. Valider dans l'ordre : ligne droite → arrêt → virages → rattrapage → recul → contournement par phases → contournement fluide (en dernier, car il dépend le plus de l'odométrie).
8. Réinjecter ensuite les mesures dans la simulation (bruit, zone morte, table de vitesse) et refaire les essais : c'est le tableau simulation / réel demandé dans le rapport QR.

## 5. Qualité du code (lisibilité, modularité)

Fait le 12 novembre : 3 avertissements GDScript corrigés ; journal CSV avec colonne `collision` ; réglages d'essai sans toucher au code (`reglages_essai` dans l'inspecteur du nœud Simulation) ; journal automatique (`journal_au_demarrage`).

À faire :

- [x] `controleur.gd` découpé (12 novembre) : machine à états seule (≈ 500 lignes avec la documentation) + `rattrapage.gd`, `retrouver_ligne.gd`, `demi_tour.gd`, `reprise_douce.gd`, `memoire_parcours.gd` ; `contournement.gd` : une fonction par phase. Banc de 40 pistes identique avant/après.
- [ ] Regrouper les réglages dans une ressource unique (`ReglagesControleur.tres`) au lieu de variables dispersées ; même fichier relu par le code Python (JSON).
- [x] Options abandonnées retirées (`loi_table`, `ralentir_virage`, `fluide_en_roulant`) ; résultats gardés dans ce fichier.
- [ ] Désactiver puis retirer DEMI_TOUR, ORBITE et RETROUVER s'ils ne servent pas sur le robot (complexité élevée, jamais nécessaires sur nos pistes conformes).
- [ ] Tests automatiques des modules purs (sans Godot) : une piste, une graine, un résultat attendu (temps, marge, bille), lancés en ligne de commande.
- [x] Fichiers `simulation.tscn*.tmp` déplacés hors du projet (`_A-supprimer/simulation_tmp`) ; `*.tmp` ajouté au `.gitignore`.
- [ ] Repasser le banc d'essai (`banc_essai_pistes` = 40 dans l'inspecteur du nœud Simulation, `vitesse_temps` = 4) après chaque modification du contrôleur : 0 chute, 0 collision, 0 demi-tour attendus.
- [ ] Ne jamais pousser l'activation du plugin MCP (`project.godot` : autoload `MCPRuntime` et `addons/godot_mcp`).

## 6. Tâches restantes (historique ; liste à jour : section 0)

- [ ] Fixer et justifier la distance d'arrêt prédéterminée (équipe).
- [ ] Mesures robot de la section 4.1.
- [ ] Ajouter bruit, biais, zone morte, retard et table de vitesse au modèle `vehicule.gd`.
- [ ] Ajouter le filtre du rapport dans `filtre_ultrason.gd`.
- [x] Tester le contournement fluide sur au moins 10 graines avec obstacles (banc de 40 pistes, 12 novembre).
- [ ] Rendre le contournement par phases moins saccadé (lisser ses changements de braquage) puis repasser le banc.
- [ ] Porter les algorithmes en Python (section 4) et les valider sur le robot avant le 3 décembre.
- [ ] Préparer le tableau simulation / réel pour le rapport QR (9 décembre).
- [ ] Fusionner la branche de simulation dans `main` après revue de l'équipe ; archiver `adm`.

## 7. Banc d'essai et correction de la piste 7 (12 novembre)

**Problème signalé :** piste 7 (graine 884064), la bille tombait au premier obstacle.

**Méthode :** banc d'essai automatique (`banc_essai.gd`) : 40 pistes aléatoires, résultat par piste dans `journaux/banc_<date>.csv` (résultat, temps, marge de la bille, collision, nombre de chaque manœuvre, raison du repli vers le contournement par phases). Chaque cas anormal rejoué seul avec le journal complet.

**Trois défauts trouvés, corrigés et vérifiés :**

| Défaut | Cause | Correction | Avant → après (40 pistes) |
| --- | --- | --- | --- |
| Bille tombée (piste 7 et graine 31677) | À la fin d'une manœuvre, le braquage passe d'un coup de 130 à 45 et la voiture réaccélère à 47 : la bille, déjà en mouvement, sort | Reprise douce (`reprise_douce.gd` : `duree_s`, `vitesse_max`, `braquage_max_deg_s`), pour toutes les manœuvres | 2 chutes → 0 |
| Collision pendant le contournement fluide (graines 142543, 150462) | Écart de 210 mm sans compter le décalage latéral de l'obstacle (jusqu'à 40 mm) : marge nulle | `ecart_mm` 250 | 2 → 0 |
| Voiture perdue après le contournement fluide (graine 150462 : 3 demi-tours, 136 s) | Cap de départ calculé sur 2 points à 100 mm (jusqu'à ~10° d'erreur) ; 4° suffisent pour finir à 110 mm de la ligne. Puis la ligne était cherchée du mauvais côté | Cap par moindres carrés sur 400 mm (`MemoireParcours.direction_ajustee`) ; côté de recherche selon la position réelle | 6 demi-tours → 0 ; graine 150462 : 68 s |

**Essai rejeté :** retour du contournement fluide de 650 mm (rayon plus large) : 1 collision sur deux obstacles rapprochés. On garde 450 mm.

**Valeurs remises à l'origine :** contournement par phases à vitesse 25 et pause 0,3 s (40 et 0,15 faisaient tomber la bille).

**Ce qui peut encore arriver :** la marge de la bille reste haute dans les courbes serrées à vitesse 55 (0,94 sur la piste 2) et le modèle est idéal (pas de bruit, pas de retard). Sur le robot, la reprise douce et le contournement fluide dépendent de l'odométrie : à recalibrer d'abord (section 4).

## 8. Nouvelles exigences (12 novembre) : plan, avant toute modification du code

**Précisions du professeur (orales, complètent le guide) :**
- Devant chaque obstacle : arrêt à une distance D_arret, puis recul sur une distance D_recul, puis contournement. D_arret et D_recul seront donnés plus tard : ce sont des paramètres, pas des constantes.
- Pas d'obstacle dans un virage, mais possiblement juste à l'entrée ou à la sortie d'un virage.
- Obstacles possiblement enchaînés, avec toujours assez d'espace pour la voiture et la manœuvre.
- Obstacle réel : 100 mm de large, 150 mm de haut, 50 mm de profondeur. Toujours droit et centré sur la ligne (pas d'orientation bizarre), seulement sur des lignes droites, juste avant ou après un virage possible, avec toujours la place pour l'arrêt, le recul et le contournement. Notre simulation utilisait 200 × 60 mm décalé jusqu'à 40 mm : à corriger.
- Virages : le guide donne seulement « ligne-guide courbe de rayon de 120 mm et plus » (section 1). C'est un rayon minimal, pas un angle : l'angle des virages n'est pas précisé. Le générateur testera donc des rayons de 120 mm et plus et des angles de 45° à 180°.
- Recul : il doit suivre la ligne (confirmé), sur moins de 200 mm (guide).
- Valeurs de test en attendant les valeurs officielles : D_arret = 100 mm et D_recul = 150 mm par défaut (250 mm de place, de quoi faire le S avec un rayon d'environ 300 mm) ; balayage au banc d'essai : D_arret ∈ {50, 100, 150} mm, D_recul ∈ {50, 100, 150, 190} mm.

**Écarts avec le code actuel :**
- Le contournement fluide « en roulant » ne respecte plus la règle (pas d'arrêt ni de recul) : à remplacer par un contournement qui part à l'arrêt, après le recul.
- Le contournement par phases s'arrête bien, mais ne recule pas avant de contourner.
- Le recul existe (`recul_ligne.gd`, < 200 mm, suit la ligne) mais n'est utilisé qu'en fin de ligne (piste 5).
- Le contournement fluide suppose ~0,75 m de ligne droite après l'obstacle : faux si l'obstacle est à l'entrée d'un virage.

**Séquence visée pour chaque obstacle :** SUIVI → approche ralentie → ARRÊT à D_arret (± 30 mm) → pause → RECUL sur la ligne de D_recul → pause → CONTOURNEMENT depuis l'arrêt → retour sur la ligne → reprise douce → SUIVI.

**Calcul (S de deux arcs, voiture 285 × 141 mm, obstacle 100 × 50 mm, tolérance de 10 mm sur le centrage, marge 20 mm) :** écart latéral nécessaire ≈ 170 mm (50 + 10 + 70,5 + 40), au lieu de 250. Distance minimale entre l'avant de la voiture et l'obstacle pour réussir le S depuis l'arrêt : 75 mm (rayon 138), 90 mm (160), 95 mm (180), 115 mm (220), 150 mm (300) ; le S avance de 255 à 420 mm. À 150 mm d'écart, la marge disparaît. Après arrêt + recul, cette distance vaut D_arret + D_recul : le recul donne la place de faire un S serré, sans revenir au contournement saccadé. À confirmer en simulation (servo, odométrie).

**Plan de travail (dans l'ordre) :**
1. Paramètres D_arret et D_recul (et pauses) dans un seul fichier de réglages (#63), valeurs provisoires en attendant le professeur.
2. Nouvelle séquence dans la machine à états : ARRÊT → RECUL_AVANT_CONTOURNEMENT → CONTOURNEMENT, quel que soit l'algorithme de contournement.
3. Approche de l'obstacle : profil de vitesse limité par la bille et compensation du retard de l'ultrason, pour tenir ± 30 mm même quand l'obstacle apparaît à la sortie d'un virage.
4. Recul avant contournement : tester le suivi de ligne en marche arrière quand l'obstacle est juste après un virage (le recul repart dans la courbe).
5. Contournement depuis l'arrêt : S de deux arcs dont le rayon est choisi selon la place disponible (D_arret + D_recul) ; repli sur le contournement par phases si la place manque.
6. Retour sur la ligne robuste à un virage juste après l'obstacle (recherche autour de l'obstacle, comme l'orbite actuelle).
7. Obstacles enchaînés : après la reprise, la séquence complète repart pour l'obstacle suivant.
8. Générateur de pistes et banc d'essai : obstacles seulement en ligne droite, placés aussi juste avant ou juste après un virage, obstacles enchaînés avec espacement minimal ; balayage de D_arret et D_recul ; nouvelles mesures : erreur d'arrêt, distance reculée, collisions, marge de la bille, temps.

**Questions pour le professeur :** valeurs officielles de D_arret et D_recul et leurs tolérances ; angles des virages (plage) ; espacement minimal entre deux obstacles et entre un obstacle et un virage ; côté du contournement imposé ou libre ; largeur du ruban (18 ou 20 mm).

## 9. Séquence arrêt → recul → contournement : code et résultats (12 novembre)

> Résultats du 12 novembre (historique). Résultats à jour (13 nov., étapes 3 à 6) : section 0 et `LISEZMOI_SIMULATION.md`.

Le plan de la section 8 est appliqué ; le détail des fichiers est dans `LISEZMOI_SIMULATION.md`. Les valeurs D_arret = 100 mm et D_recul = 150 mm sont des valeurs de test, à modifier dans `reglages_epreuve.gd`.

**Défauts trouvés au banc et corrigés :**

| Défaut | Cause | Correction | Avant → après (40 pistes) |
| --- | --- | --- | --- |
| Collision pendant le contournement fluide (graine 39596) | Point visé à 250 mm : la voiture coupait la courbe (34 mm de retard) | `anticipation_mm` 250 → 150 (12 mm de retard) | 7 → 2 collisions |
| Bille tombée après un obstacle en sortie de virage (graine 7920) | La direction de la ligne était calculée avec des points du virage (voiture 17° de travers) | `MemoireParcours.derniere_droite()` : ajustement sur la seule partie droite | 9 chutes → 0 |
| Recul trop court (78 à 108 mm pour 150) | La ligne sort des sondes en reculant quand la voiture est de travers : le recul s'arrêtait | `ReculLigne.finir_sans_ligne` : la distance est finie tout droit | Recul toujours 150 mm |
| Collisions (graines 7920, 71272…) | 1) Pistes générées avec seulement 300 mm de ligne droite avant l'obstacle : on avait oublié les 193 mm capteur → essieu (il en faut D_arret + 193 + D_recul = 443 mm). La voiture reculait dans le virage. 2) Le contournement supposait la voiture sur la ligne au départ (elle était à 91 mm) | 1) Ligne droite avant l'obstacle calculée depuis `ReglagesEpreuve` (+ 140 mm pour s'aligner, hypothèse). 2) Trajectoire qui part de la position réelle (ligne mémorisée + odométrie) | 11 → 0 collision |
| Collision et ligne perdue sur la piste de test « Contournement » | Obstacle Blender de 200 mm de large, trajectoire calculée pour 100 mm | `test_01` et `test_04` remis à 100 × 50 × 150 mm | Collision → aucune |

**Banc final (40 pistes, D_arret 100, D_recul 150)** : 40/40 arrivées, 0 collision, 0 chute (marge bille max 0,88), erreur d'arrêt max 5,8 mm (guide : ± 30), recul 150 mm à chaque obstacle, 44 contournements fluides, 0 perte de ligne de plus de 0,1 s (après un contournement ou ailleurs).

**Autres valeurs de D_arret / D_recul (15 pistes chacune) :**

| D_arret / D_recul (mm) | Arrivées | Collisions | Bille max | Erreur d'arrêt max | Contournements fluides / par phases |
| --- | --- | --- | --- | --- | --- |
| 100 / 150 (40 pistes) | 40/40 | 0 | 0,88 | 5,8 mm | 44 / 0 |
| 50 / 100 | 15/15 | 0 | 0,82 | 5,6 mm | 8 / 5 |
| 150 / 190 | 15/15 | 0 | 0,75 | 5,7 mm | 9 / 0 |
| 100 / 50 | 15/15 | 1 (graine 71272) | 0,93 | 5,7 mm | 7 / 6 |

**Limites connues :**
- Quand D_arret + D_recul est petit (~150 mm), le contournement fluide n'a pas la place et le code passe au contournement par phases. Celui-ci prend un cap de 40° sans vérifier la trajectoire du coin de la voiture : 1 collision (graine 71272, 100 / 50). Deux solutions possibles, à choisir quand les valeurs officielles seront connues : rendre le contournement par phases géométrique (même vérification que le fluide, rayon jusqu'à 138 mm), ou, si le professeur l'autorise, reculer un peu plus avant de contourner (sans dépasser 200 mm).
- Marge de 140 mm de ligne droite pour s'aligner en sortie de virage : hypothèse à confirmer (espacement minimal virage → obstacle, question pour le professeur).
- Virages du générateur : quarts de cercle (90°) seulement.
- « Parcours fourni » (sans obstacle) : 4 pertes de ligne d'environ 0,6 s dans les virages serrés, rattrapées. Suivi de ligne, pas lié aux obstacles : à traiter à part.
- Modèle idéal (pas de bruit sur l'ultrason ni l'odométrie) : à recalibrer sur le robot.
