# Code PiCar – FastDrink E05

Code Python du robot n° 05 : pilote du capteur à ultrasons, filtres de la distance et outils d'analyse.

## Structure

```
fastdrink/
  config.py              broches, constantes et réglages des filtres (un seul endroit à modifier)
  capteurs/
    ultrason.py          classe Ultrason
  filtres/
    kalman.py            filtre de Kalman distance + vitesse (tourne sur le Pi, sans numpy)
    mediane_exp.py       médiane + exponentiel, version de référence pour la comparaison
    mediane_kalman.py    médiane de 3 + Kalman (filtre retenu ; écarts avec la simulation en en-tête)
essai_ultrason.py        essai du capteur sur le robot
ultrasonic_stop_test.py  essai d'arrêt devant un obstacle (Sarah, 12 nov.)
reset.py                 remise à zéro (venu de main)
mesures_*mm.txt          mesures brutes du capteur (venues de main)
enregistrer_mesures.py   enregistre les mesures brutes en CSV (sur le robot)
analyse/
  analyse_filtres.py     réglage du Kalman et comparaison des filtres (sur PC, numpy + matplotlib)
  simuler_donnees.py     fichiers CSV SIMULÉS pour préparer l'analyse avant les vraies mesures
  approches_bruit_mesure.py, figures_capteur.py, xlsx_vers_csv.py   autres outils d'analyse
tests/                   tests sur PC, sans robot
```

## Branchement du capteur (robot n° 05)

Module ultrason à 4 fils modifié. Les numéros sont en BCM (sur le Robot HAT, « Bxx » = BCM xx).

| Fil | Broche |
| --- | --- |
| Rouge | 5 V |
| Noir | GND |
| Bleu (TRIG) | B12 |
| Jaune (ECHO, via résistances) | B16 |

Si le câblage change, modifier seulement `fastdrink/config.py`.

Dans `config.py`, `ULTRASONIC_OFFSET` est un alias de `CAPTEUR_AVANT_MM` (35,3 mm, même grandeur) et `FW_WHEEL_OFFSET` vient de Sarah (12 nov.).

## Utilisation

Essai sur le robot (depuis ce dossier) :

```
python3 essai_ultrason.py -n 200 -d 300
```

`-n` : nombre de mesures, `-d` : distance réelle en mm (facultatif, sert à calculer l'erreur moyenne).

Dans un autre programme :

```python
from fastdrink.capteurs import Ultrason, MESURE_RATEE

with Ultrason() as capteur:
    d = capteur.mesurer_mm()      # float en mm, ou MESURE_RATEE (-1.0)
    m = capteur.moyenne_mm(5)     # moyenne en ignorant les mesures ratées
```

Les méthodes `distance()`, `get_distance()` et `less_than()` du code de départ existent aussi (en cm), pour rester compatible.

Tests sur PC (pas besoin du robot) :

```
python3 -m pip install pytest
python3 -m pytest tests
```

## Mesures et analyse des filtres

Sur le robot (depuis ce dossier), un fichier par essai dans `donnees/` :

```
python3 enregistrer_mesures.py -o donnees/statique_300.csv -n 300 -d 300 --note "statique 30 cm"
python3 enregistrer_mesures.py -o donnees/approche_c30_1.csv --duree 8 -c 30 --note "approche cmd 30"
```

Le script ne pilote pas les moteurs : pour une approche, lancer le robot tout droit avec votre code
habituel pendant l'enregistrement, et noter la commande avec `-c`.

Sur le PC (copier le dossier `donnees/`), puis regarder les figures et `resultats.csv` :

```
python3 -m pip install numpy matplotlib
python3 analyse/analyse_filtres.py --statique donnees/statique_*.csv --approche donnees/approche_*.csv --d-arret 150 -o resultats
```

Pour essayer sans mesures : `python3 analyse/simuler_donnees.py -o donnees_simulees` puis la même
commande sur `donnees_simulees/`. Ces fichiers sont marqués SIMULE et ne vont pas dans le rapport.

Dans un programme sur le robot :

```python
from fastdrink.filtres import KalmanDistance
kf = KalmanDistance()                    # réglages dans config.py
d, v = kf.mettre_a_jour(z_mm, t_s)       # distance (mm) et vitesse (mm/s, < 0 en approche)
delai = kf.delai_avant(150, 0.1)         # temps avant 150 mm si moins de 0,1 s, sinon None
```

## Différences avec le code de départ

Le fichier `Ultrasonic_Avoidance.py` du code de départ n'est pas modifié. Ce pilote le remplace parce que :
`distance()` y est vide, le délai d'attente (0,5 ms) est trop court pour une mesure, il utilise une seule broche alors que notre capteur en a deux, et `test()` appelle des méthodes qui n'existent pas.
Ici, la mesure attend l'écho au plus 50 ms, respecte 60 ms entre deux mesures, renvoie -1 en cas d'échec et la moyenne ignore les échecs.

## Ajouter un capteur

Créer `fastdrink/capteurs/<capteur>.py`, mettre ses constantes dans `config.py` et l'exporter dans `fastdrink/capteurs/__init__.py`. Ajouter un test dans `tests/`.

Compatible Python 3.9 (version du Pi).
