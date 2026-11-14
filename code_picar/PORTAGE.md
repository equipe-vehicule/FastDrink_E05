# Portage du contrôleur Godot en Python (étape 11)

But : le Raspberry Pi exécute **la même logique** que la simulation validée sur les bancs (`simulation/controleur.gd` et ses modules), avec la même interface : `calculer(sondes, distance_cm, dt, mesure_nouvelle=True) -> (vitesse, angle)`. Critère « porté » : `tests/test_rejeu.py` rejoue les 7 traces de référence sans écart.

## Socle (fait)

| Fichier | Rôle |
| --- | --- |
| `fastdrink/conduite/godot.py` | Fonctions GDScript à l'identique (`roundf`, `move_toward`, `lerpf`, `clampf`, `wrapf`, `fposmod`, `signf`, `minf`/`maxf`, `deg_to_rad`, `pow_`, `sqrt`, division et `%` entiers…) et `Vec2` (Vector2 en float32). Vérifiés contre des valeurs calculées par Godot (`tests/test_godot.py`). |
| `fastdrink/conduite/reglages.py` | `ReglagesRobot`, `ReglagesEpreuve`, lecture de `reglages_algorithmes.json` et `reglages_robot.json` (racine du dépôt par défaut, autre chemin en paramètre), `regler_controleur()`, table `MODULES`. |
| `fastdrink/conduite/odometrie.py` | `Odometrie` (exemple de style ; identique au bit près sur les 6 traces). |
| `fastdrink/conduite/constantes.py` | Constantes et énumérations utilisées par plusieurs lots (`ERREUR_MAX_MM`, `Ligne`, `Fin`, `Reaction`, `Phase`) : les classes les reprennent (`SuiviLigne.Ligne = constantes.Ligne`). |
| `fastdrink/conduite/rejeu.py` | Lecture des traces, réglage d'un contrôleur depuis l'en-tête, rejeu et comparaison tour par tour. |
| `tests/traces/*.jsonl.gz` | Traces de référence Godot (option `--trace`). |

## Correspondance des fichiers

| GDScript (`simulation/`) | Python (`fastdrink/conduite/`) | Classe | Lot |
| --- | --- | --- | --- |
| `reglages_robot.gd`, `reglages_epreuve.gd`, `reglages_algorithmes.gd`, `fichier_reglages.gd` | `reglages.py` | `ReglagesRobot`, `ReglagesEpreuve` | socle |
| `odometrie.gd` | `odometrie.py` | `Odometrie` | socle |
| `suivi_ligne.gd` | `suivi_ligne.py` | `SuiviLigne` | A |
| `filtre_ultrason.gd` | `filtre_ultrason.py` | `FiltreUltrason` | A |
| `arret_obstacle.gd` | `arret_obstacle.py` | `ArretObstacle` | A |
| `recul_ligne.gd` | `recul_ligne.py` | `ReculLigne` | A |
| `rattrapage.gd` | `rattrapage.py` | `Rattrapage` | A |
| `reprise_douce.gd` | `reprise_douce.py` | `RepriseDouce` | A |
| `contournement.gd` | `contournement.py` | `Contournement` | B |
| `contournement_fluide.gd` | `contournement_fluide.py` | `ContournementFluide` | B |
| `geometrie_voiture.gd` | `geometrie_voiture.py` | `GeometrieVoiture` (`touche`) | B (appelée par les deux contournements, ajoutée au portage) |
| `memoire_parcours.gd` | `memoire_parcours.py` | `MemoireParcours` | C |
| `retrouver_ligne.gd` | `retrouver_ligne.py` | `RetrouverLigne` | C |
| `demi_tour.gd` | `demi_tour.py` | `DemiTour` | C |
| `controleur.gd` | `controleur.py` | `Controleur` | C (intégration) |

Non portés (simulation seulement) : `simulation.gd`, `vehicule.gd`, `bille_cuvette.gd`, `banc_essai.gd`, pistes, journal, HUD, `trace_controleur.gd`.

**Filtre de l'ultrason.** Le contrôleur Python utilise `conduite/filtre_ultrason.py`, port exact de `filtre_ultrason.gd` (version validée par les bancs, rejouée par les traces réalistes). `fastdrink/filtres/` (dont `mediane_kalman.py`) reste l'outil d'analyse et du rapport (réglage hors ligne sur les mesures) ; le contrôleur ne l'importe pas. Raison : deux codes du même filtre divergent (écarts déjà documentés : portée, remise à zéro après absence, horodatage) ; un seul est vérifié contre la simulation, c'est celui que le robot doit exécuter. Tout changement du filtre se fait d'abord dans `filtre_ultrason.gd`, puis dans le port, puis on régénère les traces.

## Conventions

- Mêmes noms qu'en GDScript : classes, méthodes, attributs (y compris les « privés » `_v_mm_s`…), énumérations (`enum.IntEnum`, mêmes membres dans le même ordre). Une fonction GDScript = une fonction Python, dans le même ordre dans le fichier.
- snake_case, commentaires et docstrings en français ; reprendre les commentaires `##` du .gd.
- Aucune API Godot ; Python 3.9+ (pas de `match`, pas de `X | Y` dans les annotations, pas de numpy).
- Constructeurs : mêmes dépendances, mêmes objets partagés (le Contrôleur crée `robot`, `epreuve`, `odo`, `suivi`… et les passe aux modules, comme `controleur.gd`).
- Réglages : chaque classe déclare `REGLAGES = {nom: type}` avec **exactement** ses `@export` (vérifié par `tests/test_reglages.py`) et les initialise à `0.0`/`False`/`0`/`[]` comme Godot ; les valeurs viennent seulement des JSON. Les autres variables (état) sont initialisées comme dans le .gd.
- Garder l'expression telle quelle : même ordre des opérandes et des parenthèses (`a / b * c`, `2.0 * x * y` : l'arrondi dépend de l'ordre).

## Pièges trouvés (et la règle à suivre)

1. **Vector2 est en float32** (Godot 4.7 standard, `OS.has_feature("double")` = false). Partout où le .gd construit, stocke ou rend un `Vector2`, le Python utilise `Vec2`, jamais un tuple de floats. En particulier, les modules rendent `Vector2(angle, vitesse)` : l'angle et la consigne sont arrondis en float32 avant `_appliquer`. `SuiviLigne.lire()` rend `Vec2(somme, n)` ; `point_devant_global()`, `memoire.ligne`, `obstacles`, `_cible`, `_centre`… sont des `Vec2`. `Vec2` est immuable : `v.x = a` s'écrit `v = Vec2(a, v.y)`. `Vec2 * s` convertit `s` en float32 d'abord (comme Godot). Les scalaires (`var x := 0.0`) restent des float (double).
2. **Arrondis** : `round()`/`roundf()` de GDScript arrondissent .5 en s'éloignant de zéro (et `roundf(0.49999999999999994)` = 1) ; le `round()` de Python arrondit au pair : toujours `gd.roundf`. `int(x)` tronque dans les deux langages.
3. **min, max, clamp, sign** : `gd.minf`, `gd.maxf`, `gd.clampf`, `gd.signf` (définis par comparaison ; `signf(-0.0)` = 0). Jamais `min()`/`max()` de Python sur des flottants.
4. **`move_toward`, `lerpf`, `wrapf`, `fposmod`** : formules exactes de Godot dans `godot.py` (`wrapf` rend le minimum si le résultat est « presque » le maximum).
5. **Exceptions Python absentes de Godot** : division réelle par zéro (Godot : ±INF ou NaN) → `gd.div_reel` là où le diviseur peut être nul ; `math.sqrt` d'un négatif → `gd.sqrt` ; `math.pow` → `gd.pow_` (de plus, `pow(x, 3)` et `pow(x, 4)` de Godot sont des multiplications successives : reproduit au bit près).
6. **Entiers** : `/` entre deux `int` GDScript tronque vers zéro (`gd.div_entier`), `%` garde le signe du dividende (`gd.mod_entier`). Aucune division entière trouvée dans les modules (les `somme / n` ont `somme` réel), mais vérifier à chaque `int`.
7. **NaN et INF** : `NAN`, `INF`, `is_nan`, `is_inf` → `gd.NAN`, `gd.INF`, `gd.is_nan`… ; dans les traces, NaN est écrit `null` et le signe de `-0.0` est perdu (JSON de Godot).
8. **Tableaux** : `pop_front()` → `pop(0)` ; `slice(a)` → `[a:]` ; `resize(n)` (réduction) → `del l[n:]` ; `duplicate()` → `list(l)` ; `for i in 5` → `range(5)` ; les tableaux GDScript sont des références (comme les listes Python) : garder les mêmes partages.
9. **`match`** (`contournement.gd`, `controleur.gd`) → `if/elif` ; `Etat.keys()[e]` → `e.name`.
10. **Arguments par défaut** : `calculer(..., mesure_nouvelle := true)`, `demarrer(distance, attente := 0.0)` : mêmes valeurs par défaut.
11. **Bibliothèque mathématique** : `sin`, `cos`, `tan` de Godot (Windows) diffèrent de 1 ulp de CPython pour ~5 % des valeurs d'un échantillon large ; sur les traces de référence, aucun écart observé (odométrie identique au bit près). Le rejeu garde une petite tolérance (`rejeu.py`) ; sur le Pi (glibc), même ordre de grandeur, sans effet sur la conduite.
12. **Sur le robot** : `mesure_nouvelle` doit venir du pilote ultrason (vrai si la mesure est arrivée depuis le tour précédent) ; `dt` = temps réellement écoulé depuis le tour précédent ; `distance_cm` = -1 sans écho. Le modèle idéal de la simulation n'existe pas en Python. Contrat d'appel (relecture du 14 nov.) : créer le contrôleur puis appeler `reglages.regler_controleur(c)` avant le premier `calculer` (sinon division par zéro) ; `sondes` = liste de 5 valeurs 0/1 de gauche à droite, 1 = ligne (seuiller les valeurs analogiques avant) ; `dt` fini et borné par le pilote (un `dt` très grand est intégré tel quel par l'odométrie ; borne à mesurer, #64) ; `distance_cm` NaN à remplacer par -1 ; `frequence_controle_hz` > 0 dans le JSON (boucles de prévision des contournements). Ces garde-fous relèvent du pilote du robot (#31) : le port reste identique au .gd.

## Rejeu

- Une trace = une ligne d'en-tête (tous les réglages effectifs du contrôleur Godot et les valeurs physiques du robot), une ligne par tour (entrées de `calculer()`, sorties, puis état, consigne, distance filtrée, odométries), une ligne de fin.
- Comparaison : états, phases, vitesse et angle exactement ; réels à 1e-9 + 1e-12 × |valeur| près. Le premier tour divergent est rendu avec les entrées et les 3 tours précédents.
- `py -3 -m pytest -q tests/test_rejeu.py -s` (un cas : `-k recul_court`).
- Régénérer les traces (après un changement de la simulation ou d'un JSON de réglages) : depuis la racine du dépôt, `G --headless --path . --fixed-fps 240 res://simulation/simulation.tscn -- --scenarios=virage_serre,obstacle_avant_virage,recul_court_debout --modele=ideal --trace --quitter`, puis `-- --scenarios=reference_droite,parcours_fourni --trace --quitter`, puis `-- --banc=1 --graine-depart=79191 --reglage=epreuve:distance_recul_mm=50.0 --trace --quitter`, puis `-- --scenarios=epingle_180 --reglage=rattrapage:actif=false --reglage=retrouver:apres_s=1.0 --trace --quitter` ; copier chaque `journaux/trace_*.jsonl` imprimé (« TRACE ») en `tests/traces/<modele>_<cas>.jsonl.gz` (gzip). Vérifié : ces commandes redonnent les traces actuelles octet pour octet (trace déterministe, cas indépendants).

| Trace | Tours | Couvre |
| --- | --- | --- |
| `ideal_virage_serre` | 514 | suivi, arrivée |
| `ideal_obstacle_avant_virage` | 1018 | arrêt, recul, contournement fluide, rattrapage |
| `ideal_recul_court_debout` | 1055 | contournement par phases (ÉCART, LONGER, RETOUR, CROISEMENT, ALIGNEMENT) |
| `realiste_reference_droite` | 830 | filtre médiane + Kalman, table de vitesse, arrêt, recul, fluide |
| `realiste_parcours_fourni` | 2305 | suivi et rattrapages sur le parcours du cours |
| `realiste_graine_79191_recul_50` | 3000 | D_recul 50 : contournements par phases enchaînés |
| `realiste_epingle_180_retrouver` | 1322 | RETROUVER : rattrapage désactivé par réglages d'essai (enregistrés dans l'en-tête) |

Non couverts par les traces (aucun banc ne les atteint) : DEMI_TOUR (tests unitaires seulement), BLOQUE et ARRET en cours de route, RECUL_LIGNE de la piste « Recul », évitements hors ligne, surveillance du contresens. ORBITE, SORTIE_ORBITE, RECHERCHE, TRAVERS et CROISEMENT sont couverts par `test_lot_b` (entrées pseudo-aléatoires comparées aux valeurs calculées par Godot) ; ALIGNEMENT par les traces.

## Trois lots indépendants (fichiers disjoints) – portés le 14 nov., rejeu des 7 traces identique

| Lot | Fichiers à écrire | Lignes .gd | Dépend du socle | Test avant intégration |
| --- | --- | --- | --- | --- |
| **A – ligne et capteur** | `suivi_ligne.py`, `filtre_ultrason.py`, `arret_obstacle.py`, `recul_ligne.py`, `rattrapage.py`, `reprise_douce.py` | 691 | `godot`, `reglages` (ReglagesRobot, ReglagesEpreuve.RECUL_MAX_MM), `odometrie`, `constantes` | `FiltreUltrason` seul contre la colonne `d_filtre_mm` des traces (entrée : `robot.corriger_ultrason_cm(d_cm)`, `dt` borné à 0,001, `nouvelle` ; `avance_mm` inutile, `projection_odometrie` faux) |
| **B – contournements** | `contournement.py`, `contournement_fluide.py` | 807 | `godot` (Vec2), `reglages`, `odometrie`, `constantes.ERREUR_MAX_MM` | tests unitaires ciblés (`ContournementFluide.choisir`, `_q`, `x_chemin`, `Contournement.choisir_cap`) ; rejeu à l'intégration |
| **C – mémoire, recherche, contrôleur** | `memoire_parcours.py`, `retrouver_ligne.py`, `demi_tour.py`, puis `controleur.py` | 303 + 562 | `godot`, `reglages`, `odometrie`, `constantes` (énumérations), `rejeu` (interface) ; `controleur.py` importe aussi les classes des lots A et B (`SuiviLigne.lire`, constructeurs) : il s'écrit en parallèle mais ne se teste qu'après eux | `MemoireParcours` sur des points connus |

Règles : un lot n'écrit que ses fichiers ; les énumérations viennent de `constantes.py` (ne pas les redéfinir) ; une constante partagée manquante s'ajoute à `constantes.py` (signaler aux autres lots) ; chaque classe déclare `REGLAGES` ; `tests/test_reglages.py` doit passer pour chaque classe portée.

**Ordre d'intégration** (suit la section 4.7 de `SUIVI_SIMULATION.md`) :
1. A, B et C en parallèle ; `controleur.py` complet (lot C) avec ses importations.
2. `ideal_virage_serre` (suivi seul : SuiviLigne, RepriseDouce, FiltreUltrason « mediane3 », mémoire).
3. `realiste_reference_droite` (Kalman, table, ArretObstacle, ReculLigne, ContournementFluide).
4. `ideal_obstacle_avant_virage` (rattrapage), puis `realiste_parcours_fourni`.
5. `ideal_recul_court_debout`, puis `realiste_graine_79191_recul_50` (Contournement par phases).
6. Mesurer le temps d'un tour sur le Pi (`Vec2` en float32 coûte ; `godot.regler_precision_godot(False)` calcule en double si nécessaire, au prix du rejeu exact).
