
## Corrections du 26 octobre 2026

- `controleur.gd` : en cas de perte de ligne, le robot braque du dernier côté non nul où la ligne a été vue. Avant, une dernière lecture centrée le faisait partir tout droit (demi-tour de la piste Courbes 120 raté).
- `simulation.gd` : la voiture reste immobile tant que la bille n'est pas posée et que son modèle n'a pas mesuré la plaquette à l'arrêt. Avant, un démarrage brutal pouvait échapper au modèle de la bille.
- `simulation.gd` : l'accélération affichée est moyennée sur 50 ms (les commandes entières créaient des pics d'un seul pas) ; l'accélération latérale (v² · tan(δ) / L) est aussi affichée.
- Guide pas à pas : document « Guide Godot – Comprendre la simulation pas à pas » (Claude Docs).
