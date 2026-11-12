# Suivi de la simulation – FastDrink E05

Mis à jour le 12 novembre 2026 (après le banc d'essai). Ce fichier garde l'état de la simulation, ses limites, les directives pour le portage sur le robot et les tâches restantes. Le détail des modifications est dans `LISEZMOI_SIMULATION.md`.

## 1. État actuel (simulation idéale : pas de bruit, pas de zone morte, vitesse linéaire)

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
5. Commandes sous la zone morte mesurée : `vitesse_approche_min` 8, `vitesse_recul` 10, fin du recul (−3 à −5), rattrapage à 15 (limite).
6. Vitesses 55 (suivi) et 65 (manœuvre) choisies en simulation ; la marge de la bille sur le vrai robot est inconnue (rampe exprimée en unités de commande, pas en m/s²).
7. Le filtre retenu dans le rapport (Kalman en mouvement, médiane + exponentiel à l'arrêt) n'est pas dans la simulation (médiane de 3 seulement).
8. La distance d'arrêt prédéterminée (30 mm avant l'obstacle) n'a pas de justification écrite.
9. Valeurs encore provisoires : vitesse du servo (300 °/s), fréquence de boucle (50 Hz), masse et amortissement de la bille, tache des sondes.

## 4. Directives pour le portage sur le robot (Python, Raspberry Pi)

1. Mesurer d'abord : vitesse réelle par commande (table), zone morte, vitesse du servo, fréquence réelle de la boucle, retard de la mesure ultrason.
2. Remplacer `mm_s_par_cmd` (linéaire) par une table commande → vitesse, utilisée par l'odométrie ET par le modèle de simulation.
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

## 6. Tâches restantes (projet)

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
