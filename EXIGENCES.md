# Exigences du projet – FastDrink E05

Établi le 13 novembre 2026. Chaque ligne cite sa source exacte. Quand un document ne dit rien, c'est écrit « non précisé ». Les valeurs de la simulation viennent du code (dépôt `FastDrink_E05`, dossier `simulation/`) ; les résultats d'essai viennent de `SUIVI_SIMULATION.md` (section 0, bancs du 13 nov. : 40 pistes sur grille, 15 pistes avec D_recul 50, 40 pistes libres, 25 scénarios).

## Documents lus

| Document | Pages | Lecture |
| --- | --- | --- |
| `01_Consignes-du-cours/guide_e_tudiants_gif502_a26.pdf` (guide, version 3, automne 2026) | 14 + 1 vide | Entier, texte et images des pages (formules p. 12 comprises) |
| `03_Materiel-PiCar/picar-s_v2_0_user_manual_59_pages-4.pdf` (manuel SunFounder PiCar-S v2.0) | 62 + 1 vide | Entier, en texte (pdftotext). Pages surtout faites de figures, non lues comme images : p. 11-21 (montage, câblage), 33, 47-50 (ordinogrammes), 53. Formule de distance de l'ultrason (p. 32) illisible en texte. Numéros = pages du PDF (page imprimée + 4). |
| `03_Materiel-PiCar/schema_rpi_godot_maudeb_francais_2024_10.png-3.pdf` (schéma Raspberry Pi ↔ Godot) | 1 | Entier (image) |
| `03_Materiel-PiCar/support_bille_v2_1-2.pdf` (plan du support de bille) | 1 | Entier (image + cotes en texte) |
| `03_Materiel-PiCar/coffredistribue.pdf` (contenu de la boîte) | 1 (photo) | Entier ; étiquette du couvercle lue sur la photo, petite et floue : **lecture à confirmer** |

Les précisions orales du professeur (12 et 13 nov.) ne sont dans aucun document : leur source est la note de l'équipe dans `CLAUDE.md`. À faire confirmer par écrit (tâche #56).

## 1. Réponses aux questions

| Question | Réponse | Source |
| --- | --- | --- |
| Angles des virages imposés ? | **Non précisé dans le guide.** Le guide ne fixe qu'un rayon minimal : « courbe de rayon de 120 mm et plus ». Aucun angle n'est donné dans les autres documents. | Guide p. 4, section 1 |
| Rayon minimal | **120 mm** (exigence du client). Le manuel recommande, lui, des courbes de plus de 138 mm : à 45° de braquage, le rayon de virage égale l'empattement. **Une courbe de 120 mm est donc plus serrée que ce que le manuel juge faisable.** Ceci explique le rattrapage en marche arrière de la simulation. | Guide p. 4 ; manuel p. 47 « semidiameter of curves: more than 138mm » |
| Largeur du ruban | **18 mm**, « ruban électrique noir ». Le manuel recommande 18 à 30 mm, « nearly the distance between two probes ». Le parcours fourni (`parcours.glb`) a un ruban de 20 mm (source : `MESURES_ROBOT.md`). | Guide p. 4 ; manuel p. 47 |
| Bille | Une bille dans une soucoupe attachée au véhicule représente les breuvages ; si elle tombe, « votre prototype est inutilisable ». Un démarrage rapide et un freinage brusque risquent de renverser les breuvages. Séance 4.4 : calculer l'accélération maximale qui évite la chute (conservation de l'énergie mécanique). À la compétition, un jalon vaut 100 points avec la bille, 50 sans. Le classement au temps ne compte que si le parcours est complet avec la bille. | Guide p. 4, p. 11 (4.4), p. 12 (4.10) |
| Stabilité | Non précisé comme exigence chiffrée. Seule mention : « un véhicule qui démarre rapidement et freine brusquement risque de faire renverser les breuvages ». | Guide p. 4 |
| Obstacles | Guide : « contourner des obstacles en quittant la ligne-guide ». Ni dimensions, ni nombre, ni position, ni côté de contournement. Le professeur (oral) : obstacle de 100 × 150 × 50 mm (l × h × p), centré sur la ligne, sur des lignes droites (juste avant ou après un virage possible), obstacles enchaînés possibles, place toujours suffisante. Depuis le 13 nov., il peut être posé sur différentes faces. | Guide p. 4 ; `CLAUDE.md` (oral, 12 et 13 nov.) |
| Arrêt | « Arrêter à une distance prédéterminée d'un obstacle (précision de l'arrêt ± 30 mm) ». Valeur de la distance : non précisée dans le guide ; donnée plus tard (professeur). | Guide p. 4 |
| Recul | « Suivre la ligne-guide en reculant sur de courtes distances, c-à-d., moins de 200 mm ». Le guide ne relie pas le recul à l'obstacle. Le professeur (oral) : devant chaque obstacle, arrêt à D_arret, recul de D_recul, puis contournement. | Guide p. 4 ; `CLAUDE.md` (oral, 12 nov.) |
| Contournement | « Contourner des obstacles en quittant la ligne-guide au sol ». Côté imposé : non précisé. | Guide p. 4 |
| Communication Godot ↔ PiCar | Le guide n'en parle pas. Schéma : Godot sur PC (ou Raspberry Pi sans écran), PiCar sur Raspberry Pi, liaison « Réseau (Websocket) » des deux côtés, messages « JSON », « Wifi ou localhost ». Côté Godot : algorithme de contrôle, simulation des capteurs, physique de la bille (« utiliser physique de base de Godot »). Côté PiCar : capteur de distance et suiveur de ligne → « Filtrage et contrôle » → contrôle des moteurs. Non précisé : ports, rôle client ou serveur, champs JSON, fréquence. | Schéma, p. 1 |
| Critères de la compétition | Traverser « le plus rapidement possible » un parcours à obstacles, de façon autonome, en utilisant « seulement et strictement les informations provenant des capteurs ». Parcours remis quelques jours avant ; piste mystère dévoilée le matin même (contre le « hardcoding »). Points avec le meilleur essai par piste : voir section 3. | Guide p. 5, p. 12 |

## 2. Exigences du client et état en simulation

« Testé » : **oui** = mesuré au banc ou sur une piste de test ; **partiellement** = une partie seulement, ou modèle idéal ; **non** = pas simulé.

| # | Exigence | Source | Où dans le code | Testé en simulation |
| --- | --- | --- | --- | --- |
| E1 | Arrêter à une distance prédéterminée d'un obstacle, précision ± 30 mm | Guide p. 4 | `reglages_epreuve.gd` (`TOLERANCE_ARRET_MM`, `distance_arret_mm` = 100, valeur de test) ; `arret_obstacle.gd` ; état `DECISION`/`ARRET` de `controleur.gd` | **Oui** : erreur max 3,9 mm (banc de référence de 40 pistes, 13 nov.). Ultrason idéal (sans bruit ni retard) |
| E2 | Suivre un ruban électrique noir de 18 mm | Guide p. 4 | `parametres_piste.gd` `largeur_ruban` = 0,018 m ; `suivi_ligne.gd` (PID) ; `vehicule.gd` (5 sondes) | **Oui** sur les pistes aléatoires (18 mm) ; parcours fourni à 20 mm |
| E3 | Suivre une courbe de rayon de 120 mm et plus | Guide p. 4 | `parametres_piste.gd` (rayon des virages) ; `generateur_piste.gd`, `generateur_libre.gd` ; `rattrapage.gd` ; piste `test_03_courbes_r120.glb` | **Partiellement** : virages de 20 à 180° depuis le 13 nov. (générateur libre et scénarios, #73) ; 39/40 pistes libres, échec quand un obstacle précède une épingle plus serrée que la voiture (graine 47515) ; pertes de ligne inévitables dans les virages de 120 mm, rattrapées en marche arrière (#74) ; bille max 0,82 sur les scénarios avec la rampe de 60 (#67), 0,94 sur les pistes libres |
| E4 | Contourner les obstacles en quittant la ligne | Guide p. 4 | `contournement_fluide.gd`, `contournement.gd` (repli), `rattrapage.gd`, `retrouver_ligne.gd` | **Oui** pour les 6 poses (13 nov., étape 4) : 40/40 pistes à faces aléatoires sans collision, 15/15 scénarios ; marge latérale min 14 mm (7,9 mm sur les pistes libres). Avec 100/50 : 15/15, 0 collision depuis la vérification du contournement par phases (#72, 13 nov.) ; 25/25 scénarios |
| E5 | Suivre la ligne en reculant, moins de 200 mm | Guide p. 4 | `reglages_epreuve.gd` (`RECUL_MAX_MM`, `distance_recul_mm` = 150, valeur de test) ; `recul_ligne.gd` ; état `RECUL_OBSTACLE` | **Oui** : 150 mm à chaque obstacle |
| E6 | La bille ne doit pas tomber | Guide p. 4 | `bille_cuvette.gd` (chute = contact au-delà du bord) ; `banc_essai.gd` (`bille_marge_max`) | **Oui** en simulation : 0 chute, marge max 0,80 (40 pistes), 0,82 (25 scénarios), 0,94 (40 pistes libres). Modèle de bille à vérifier (#47) |
| E7 | Pas de démarrage rapide ni de freinage brusque | Guide p. 4 | `reglages_robot.gd` `rampe_cmd_par_s` = 125 (freinage), `controleur.gd` `rampe_montee_cmd_par_s` = 60 (accélération, provisoire), `marge_freinage` ; `reprise_douce.gd` ; pauses de 0,3 s (`reglages_epreuve.gd`) | **Partiellement** : suivi par la marge de la bille, sans lien avec l'accélération maximale calculée en 4.4 (livrable non trouvé dans `06_Livrables`) |
| E8 | Le modèle simulé doit représenter le plus possible le comportement réel | Guide p. 4 (encadré) | `vehicule.gd`, `reglages_robot.gd`, `MESURES_ROBOT.md` | **Partiellement** : modèle idéal, toutes les valeurs du robot sont provisoires |
| E9 | Mêmes algorithmes de décision et de contrôle dans la simulation et le robot | Guide p. 4 (encadré) | Modules `RefCounted` sans dépendance à Godot (`controleur.gd` et modules) | **Non** : portage Python à faire (#64) |
| E10 | N'utiliser « seulement et strictement » que les informations des capteurs | Guide p. 5 | Entrées du contrôleur : 5 sondes, ultrason, dt. Le contrôleur utilise aussi l'odométrie calculée à partir des commandes (`odometrie.gd`) et la mémoire de la ligne vue (`memoire_parcours.gd`) | **Partiellement** : aucune donnée de la piste n'est lue. **À clarifier avec le professeur** : l'odométrie tirée des commandes (sans encodeur) est-elle permise ? |
| E11 | Piste mystère : pas de « hardcoding » | Guide p. 5 (encadré) | `generateur_piste.gd` (pistes aléatoires) ; `banc_essai.gd` | **Partiellement** : 40/40 pistes sur grille et 39/40 pistes libres (virages de 20 à 180°, #73) |
| E12 | Quatre parcours de test, à réussir plusieurs fois (Conduite à l'épreuve) | Guide p. 5, p. 12 (4.9) | Pistes `test_01_arret`, `test_02_ligne_droite`, `test_03_courbes_r120`, `test_04_contournement`, `test_05_recul` (`pistes.gd`) | **Partiellement** : les vrais parcours ne sont pas connus |
| E13 | Simulation dans Godot et Blender ; démontrer que **toutes** les contraintes sont respectées (GoDot Grand prix) | Guide p. 4, p. 11 (4.6) | Tout le dossier `simulation/` ; modèles `07_Modelisation-Blender` | **Partiellement** : voir E3, E4 (#72), E8 |
| E14 | QR : tableau comparant simulation et robot ; AR : tableau de conformité aux spécifications | Guide p. 10 (4.3) | Ce fichier (base du tableau de conformité) ; `SUIVI_SIMULATION.md` | Sans objet (livrables du 9 et du 11 déc., #36, #41) |

### Précisions orales du professeur (source : `CLAUDE.md`, à faire confirmer par écrit)

| # | Précision | Où dans le code | Testé en simulation |
| --- | --- | --- | --- |
| P1 | Devant **chaque** obstacle : arrêt à D_arret, recul de D_recul en suivant la ligne, puis contournement (12 nov.) | `controleur.gd` état `RECUL_OBSTACLE` | **Oui** (40 pistes, 44 obstacles) |
| P2 | D_arret et D_recul donnés plus tard ; valeurs de test 100 et 150 mm (12 nov.) | `reglages_epreuve.gd` (un seul endroit) | **Oui** pour 100/150, 50/100, 150/190 et 100/50 (0 collision depuis #72) ; si aucune trajectoire ne passe, arrêt sûr en ARRET |
| P3 | Obstacle de 100 × 150 × 50 mm (largeur × hauteur × profondeur), centré sur la ligne (12 nov.) | `reglages_epreuve.gd` (cotes, `tolerance_centrage_mm` = 10, hypothèse) | **Oui** (6 poses, voir P6) |
| P4 | Obstacles seulement sur des lignes droites, possiblement juste avant ou après un virage ; place toujours suffisante (12 nov.) | `generateur_piste.gd` ; `parametres_piste.gd` `marge_droite_avant` = 140 mm (hypothèse) | **Oui** (pistes aléatoires) ; scénarios `obstacle_apres_virage`, `obstacle_avant_virage`, `obstacle_apres_virage_30` |
| P5 | Obstacles enchaînés possibles (12 nov.) | `generateur_piste.gd` | **Oui** : scénarios `deux_obstacles`, `pire_cas_deux_obstacles`, `recul_court_deux_obstacles` |
| P6 | Obstacle posé sur différentes faces (debout, tourné, couché), toujours centré (13 nov.) | `faces_aleatoires` (générateur), scénarios `face_*` et `pire_cas_*`, contournement de 150 × 150, cône vertical de l'ultrason (hauteur 60 mm, hypothèse : cône circulaire ; écho du sol non simulé) | **Oui en simulation** (étape 4) ; à confirmer sur le robot (#53) |

## 3. Évaluations et compétition (guide)

| Élément | Contenu | Source |
| --- | --- | --- |
| Conduite à l'épreuve (3 déc., 80 points C1) | Quatre courts trajets, chacun valide une contrainte ; 20 minutes pour les quatre. Points = Σ nb_reussi(i) / nb_essai(i), avec nb_essai(i) ≥ 5 et nb_trajet = 5. **Incohérence du guide** : « quatre courts trajets » mais nb_trajet = 5. | Guide p. 12 (4.9), p. 14 |
| Compétition « Que le meilleur serve ! » (10 déc., 150 points C1) | Meilleur essai par piste. point_jalon(i) = 100 / nb_jalon si jalon franchi avec la bille, 50 / nb_jalon sans la bille, 0 sinon ; un jalon peut être « un tournant, un obstacle, et arrêt ». point_rang(j) = 50 × [meilleur_temps(j) / ton_temps(j)] si parcours complet avec la bille, 0 sinon. point_course = Σ point_jalon + Σ point_rang. Le sens exact des crochets [ ] n'est pas expliqué. | Guide p. 12 (4.10), p. 14 |
| Conséquence pour la conception | La bille compte double sur chaque jalon et conditionne tous les points de temps : **la stabilité passe avant la vitesse**. Le temps rapporte au plus 50 points par piste. | Déduit des formules ci-dessus (calcul, pas une exigence) |
| Parcours | Remis quelques jours avant la compétition, essais pendant la séance Essais routiers (9 déc.) ; piste mystère le matin de la compétition | Guide p. 5 |
| Accélération maximale (Breuvage, 17 sept.) | Calculer l'accélération maximale pour que la bille ne tombe pas, par conservation de l'énergie mécanique | Guide p. 11 (4.4) |
| Filtres (5 nov.), distances (19 nov.), confiance (26 nov.) | Deux filtres de l'ultrason ; régression mesures/réelles (Matlab) ; intervalles de confiance sur au moins deux distances | Guide p. 11-12 |
| QR (9 déc.) / AR (11 déc.) | Voir E14 | Guide p. 10 |
| Règles générales | Fichiers PDF nommés selon la nomenclature (pénalité 10 % ; 20 % si le format n'est pas PDF) ; retard : 10 % par jour ; IAG non permise « pour la résolution de la problématique », permise pour l'apprentissage et l'aide à la rédaction à condition d'être citée | Guide p. 7-8 |

## 4. Matériel : informations utiles à la simulation

Ce ne sont pas des exigences du client, mais des données du fabricant ou du cours. Les écarts avec la simulation sont en gras.

| Sujet | Information | Source | Simulation | Écart |
| --- | --- | --- | --- | --- |
| Capteur de ligne | 5 sondes TCRT5000, sortie numérique 1 = noir / 0 = blanc, bus I2C 0x11 ; référence = moyenne blanc/noir ; largeur du module 125 mm ; espacement des sondes et hauteur au sol non précisés | Manuel p. 21, 44-48, 52 | 5 sondes (`vehicule.gd`), tache de 3 mm, aucune fausse lecture | Espacement pris du modèle 3D (#52) |
| Ultrason | Module 25 kHz, une seule broche (BCM 17), `UltraSonic_Avoidance(17)`, distance en cm ; cycle de mesure conseillé **> 60 ms** ; cône « about 30~40 degrees » (p. 34), « about 30 degrees » (p. 60) ; lectures erronées possibles, Pi non temps réel ; portée non précisée | Manuel p. 32-36, 60 | `frequence_ultrason_hz` = 20 (50 ms) ; cône 15° ; sans bruit | **Cadence plus rapide que le cycle conseillé (≤ 16,7 mesures/s)** ; cône plus étroit (#53) |
| Servo de direction | Neutre à 90 ; commande limitée par logiciel (pas de butée) ; `turning_offset` conseillé entre -30 et 30 ; vitesse à vide 0,14 s/60° (4,8 V), 0,12 s/60° (6 V) ; PWM 60 Hz ; angle des roues en degrés non précisé | Manuel p. 28-29, 36-37, 61 | `vitesse_servo_deg_s` = 300 °/s ; 45° de roue au servo 45/135 | Vitesse à vide du fabricant ≈ 430 à 500 °/s (calcul : 60° / 0,14 s et 60° / 0,12 s) ; en charge : non précisé (#51, #19) |
| Sens du servo | p. 37 : réduire l'angle pour tourner à gauche ; p. 51 : gauche = 90 + step. **Le manuel se contredit** | Manuel p. 37, 51 | — | À vérifier sur le robot |
| Moteurs arrière | `set_speed` de 0 à 100 ; moteur F130SA, 4,5-6 V, 200 tr/min à vide (180 en essai), réduction 1:48 ; vitesse en mm/s non précisée ; diamètre de roue non précisé | Manuel p. 30, 37, 52, 61 | 4 mm/s par unité (0,40 m/s à 100) ; roue de 65 mm (modèle 3D) | Calcul indicatif avec la roue de 65 mm du modèle : 180 tr/min ≈ 0,61 m/s à vide. À mesurer (#18) |
| Alimentation | 2 piles 18650, entrée 7,4-8,4 V | Manuel p. 8, 57 | Non modélisée | Effet de la charge des piles sur la vitesse : non précisé |
| Support de bille | Plaque 50 × 50, cuvette Ø 40, rayon de courbure R 140, épaisseur 1,50 ; **unités non écrites** (probablement mm) ; profondeur, bord et bille non cotés | Plan, p. 1 | `reglages_robot.json` : `rayon_cuvette_mm` 140, `rayon_bord_cuvette_mm` 20 (= Ø 40 / 2), `rayon_bille_mm` 7,5 | **Rayon de la bille non donné par le plan** : à mesurer (#50) |
| Contenu de la boîte | Étiquette (lecture à confirmer) : 1 Sunfounder Car + Raspberry Pi + carte micro-SD, 2 batteries Li-ion Samsung INR18650-25R, 1 chargeur Nitecore D2, 1 câble 120 VAC, 1 clavier, 1 souris, 1 câble USB, 1 câble HDMI, **1 bille dans un pot**, 1 piédestal 3D. Aucun obstacle ni ruban | Photo `coffredistribue.pdf` | — | Bille et support réels à peser et mesurer (#47, #50) |
| Communication | WebSocket + JSON, Wifi ou localhost (voir section 1) | Schéma p. 1 | **Aucune** (ni WebSocket ni JSON dans `simulation/`) | Tâche #30 |

## 5. Points à clarifier (questions pour le professeur, tâche #56)

1. Valeurs officielles de D_arret et D_recul, et leurs tolérances.
2. Odométrie calculée à partir des commandes (sans encodeur) : est-ce une « information provenant des capteurs » (guide p. 5) ?
3. Côté du contournement : imposé ou libre ?
4. Ruban de 18 mm (guide) ou de 20 mm (parcours fourni) ?
5. Conduite à l'épreuve : quatre trajets ou cinq (nb_trajet = 5) ?
6. Formule point_rang : sens des crochets [ ] (arrondi ou simple parenthèse) ?
7. Espacement minimal virage → obstacle et entre deux obstacles.
8. Le rôle de la communication Godot ↔ PiCar (schéma) est-il évalué, et à quelle séance ?
