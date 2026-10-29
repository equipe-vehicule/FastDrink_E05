# Simulation du PiCar-S dans Godot (équipe 05)

Godot 4.4.1. Scène à lancer : `simulation/simulation.tscn` (ouvrir la scène, puis F6 « Lancer la scène actuelle »).

## Commandes au clavier

| Touche | Action |
| --- | --- |
| 1 à 6 | Choisir la piste : ligne droite, courbes de 120 mm, arrêt, contournement, recul, parcours fourni |
| R | Remettre la voiture au départ (et la bille dans la cuvette) |
| M | Passer du mode manuel au mode automatique (algorithmes de `controleur.gd`), et inversement |
| Flèches haut et bas | Mode manuel : commande de vitesse ±10 (de −100 à 100) |
| Flèches gauche et droite | Mode manuel : braquer au maximum (servo à 45 ou 135) |
| Espace | Mode manuel : vitesse à 0 |
| C | Vue de derrière ou vue de dessus |
| L | Démarrer ou arrêter l'enregistrement d'un journal CSV dans `journaux/` (temps, commandes, vitesse, accélération, sondes, ultrason, bille) |

L'affichage en haut à gauche montre les commandes, la vitesse, l'accélération (et son maximum), les 5 sondes, la distance de l'ultrason, l'état de la bille et les collisions.

## Organisation

| Fichier | Rôle |
| --- | --- |
| `controleur.gd` | **Les algorithmes de l'équipe.** Entrées : 5 sondes (0/1), distance de l'ultrason (cm, −1 si rien), dt. Sorties : commande de vitesse (−100 à 100) et angle du servo (45 à 135). Ce sont les mêmes valeurs que `bw.speed` et `fw.turn` sur le vrai robot. Il contient un exemple de départ : suivi de ligne proportionnel, reprise quand la ligne est perdue, rampe d'accélération. |
| `vehicule.gd` | Modèle du robot : géométrie mesurée, mouvement, capteurs simulés. |
| `simulation.gd` | Scène : sol, pistes, bille, caméra, affichage, clavier, journal. |

## Modèle du véhicule

- **Mouvement :** modèle cinématique « bicyclette » de référence, calculé au milieu de l'essieu arrière, avec un empattement de 138 mm.
- **Sondes :** chaque sonde voit une tache de 3 mm de rayon au sol, et seulement le ruban (couche de collision 2). Avec des rayons infiniment fins, une ligne de 18 mm passant entre deux sondes espacées de 19 mm serait invisible.
- **Ultrason :** rayon vers l'avant depuis le centre du capteur, portée de 3 m, −1 si rien n'est détecté (comme la bibliothèque SunFounder). Le bruit est réglable.
- **Collision :** signalée quand la boîte d'encombrement du véhicule (141 × 80 × 285 mm) touche un obstacle.

### Paramètres provisoires, à mesurer sur le vrai robot

Ces paramètres se règlent dans l'inspecteur de Godot ou dans `vehicule.gd` et `simulation.gd`.

| Paramètre | Valeur provisoire | Comment le mesurer |
| --- | --- | --- |
| `vitesse_max` | 0,40 m/s à la commande 100 | Chronométrer le robot sur 1 m pour plusieurs commandes |
| `angle_roues_max_deg` | 45° pour un servo à 45 ou 135 | Roues braquées à fond : diamètre du cercle décrit ; angle = atan(138 / rayon) |
| `constante_temps_moteur` | 0 (réponse instantanée) | Filmer un démarrage |
| `bruit_ultrason_cm` | 0 | Écart-type mesuré pour le rapport « Test ta confiance » |
| `rayon_tache_sonde` | 3 mm | Déplacer le ruban sous une sonde et noter quand elle change d'état |
| `masse_bille` | 10 g | Balance |
| `amortissement_bille` | 1 /s | Comparer avec la bille sur le vrai robot |

## Physique de la bille

La bille est simulée par le modèle exact d'une sphère qui roule sans glisser dans la cuvette (rayon de courbure de 140 mm, bord à 20 mm du centre, plan `support_bille_v2`). Le calcul se fait dans le repère de la voiture, à partir de l'accélération réelle de la cuvette : démarrages, freinages et virages. Quand le point de contact dépasse le bord, la bille est déclarée tombée et repasse en physique libre.

Pourquoi ce choix : avec le moteur physique générique, une bille de 15 mm posée sur une plateforme mobile restait collée à la voiture même à 3 m/s². Ce n'est pas réaliste.

**Validation** (accélération maintenue depuis l'arrêt, sans amortissement) :

- théorie : limite = g × 0,020 / (2 × 0,140) = 0,70 m/s² ;
- simulation : la bille tient à 0,70 m/s² et tombe à 0,71 m/s².

C'est cohérent avec le calcul de la séance 2 (0,736 m/s², par conservation de l'énergie).

## Choix techniques

- **Moteur physique Jolt** (réglage du projet). Le moteur par défaut de Godot ne détectait pas le ruban de 1 mm d'épaisseur dans les courbes.
- **240 pas de calcul par seconde**, pour la précision de la bille et des sondes.
- **Les obstacles reçoivent une forme de collision convexe pleine.** L'import crée une surface creuse, que le moteur ne détecte pas quand la voiture est dedans.

## Limite géométrique importante (courbes de 120 mm)

Les sondes sont à 217 mm devant l'essieu arrière (79 + 138 mm). Le centre de rotation d'une voiture étant toujours sur l'axe de l'essieu arrière, les sondes décrivent au mieux un cercle de 217 mm de rayon. **Le robot ne peut donc pas garder ses sondes sur une courbe de 120 mm en continu** : il perd la ligne un instant dans chaque virage serré, en simulation comme en vrai. L'algorithme doit le prévoir (mémoriser le dernier côté vu, braquer, réduire la vitesse). L'exemple de `controleur.gd` le fait déjà : il termine la piste des courbes en perdant la ligne 4 fois et en la retrouvant à chaque fois.

## Résultats de l'exemple d'algorithme (au 26 octobre 2026)

| Piste | Résultat |
| --- | --- |
| Ligne droite | Suivie jusqu'au bout, bille en place |
| Courbes de 120 mm | Suivie jusqu'au bout (4 pertes de ligne retrouvées), bille en place |
| Parcours fourni | Plus de 5 m suivis, bille en place |
| Arrêt, contournement, recul | **Pas encore programmés** : la voiture percute l'obstacle (collision signalée) |
