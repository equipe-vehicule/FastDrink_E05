"""Vérification géométrique vue de dessus (portage de simulation/geometrie_voiture.gd) : la voiture (rectangle autour de
l'essieu arrière) touche-t-elle un obstacle (rectangle aligné sur les axes du repère) ? Pure logique, partagée par
ContournementFluide et Contournement.

Fichier ajouté par le lot B : geometrie_voiture.gd n'était pas dans la table de PORTAGE.md mais les deux contournements en
dépendent (GeometrieVoiture.touche).

Repère des manœuvres : x vers la droite, y vers l'avant ; cap > 0 = vers la droite.
On teste les coins de la voiture dans l'obstacle ET les coins de l'obstacle dans la voiture (un coin de l'obstacle peut
entrer dans le flanc de la voiture sans qu'aucun coin de la voiture n'entre dans l'obstacle).
"""
from . import godot as gd
from .godot import Vec2


class GeometrieVoiture:
    @staticmethod
    def touche(essieu, cap, avant_mm, arriere_mm, demi_largeur_mm, obs_min, obs_max):
        """essieu : milieu de l'essieu arrière (mm, Vec2) ; avant_mm / arriere_mm : de l'essieu à l'avant / à l'arrière ;
        demi_largeur_mm : demi-largeur de la voiture ; obs_min / obs_max : coins de l'obstacle (Vec2, déjà élargi des
        marges voulues)."""
        avant = Vec2(gd.sin(cap), gd.cos(cap))
        droite = Vec2(gd.cos(cap), -gd.sin(cap))
        for a in [avant_mm, -arriere_mm]:
            for b in [demi_largeur_mm, -demi_largeur_mm]:
                c = essieu + avant * a + droite * b
                if c.x >= obs_min.x and c.x <= obs_max.x and c.y >= obs_min.y and c.y <= obs_max.y:
                    return True
        for o in [obs_min, Vec2(obs_max.x, obs_min.y), obs_max, Vec2(obs_min.x, obs_max.y)]:
            rel = o - essieu
            long = rel.dot(avant)
            lat = rel.dot(droite)
            if long >= -arriere_mm and long <= avant_mm and gd.absf(lat) <= demi_largeur_mm:
                return True
        return False
