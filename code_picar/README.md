# Code PiCar – FastDrink E05

Code Python qui tourne sur le Raspberry Pi du robot n° 05. Pour l'instant : le pilote du capteur à ultrasons.

## Structure

```
fastdrink/
  config.py              broches et constantes (un seul endroit à modifier)
  capteurs/
    ultrason.py          classe Ultrason
essai_ultrason.py        essai du capteur sur le robot
tests/test_ultrason.py   tests sur PC, sans robot (faux GPIO)
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

## Différences avec le code de départ

Le fichier `Ultrasonic_Avoidance.py` du code de départ n'est pas modifié. Ce pilote le remplace parce que :
`distance()` y est vide, le délai d'attente (0,5 ms) est trop court pour une mesure, il utilise une seule broche alors que notre capteur en a deux, et `test()` appelle des méthodes qui n'existent pas.
Ici, la mesure attend l'écho au plus 50 ms, respecte 60 ms entre deux mesures, renvoie -1 en cas d'échec et la moyenne ignore les échecs.

## Ajouter un capteur

Créer `fastdrink/capteurs/<capteur>.py`, mettre ses constantes dans `config.py` et l'exporter dans `fastdrink/capteurs/__init__.py`. Ajouter un test dans `tests/`.

Compatible Python 3.9 (version du Pi).
