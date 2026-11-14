## Verification geometrique vue de dessus : la voiture (rectangle autour de l'essieu arriere) touche-t-elle un obstacle
## (rectangle aligne sur les axes du repere) ? Pure logique, partagee par ContournementFluide et Contournement.
## Repere des manoeuvres : x vers la droite, y vers l'avant ; cap > 0 = vers la droite.
## On teste les coins de la voiture dans l'obstacle ET les coins de l'obstacle dans la voiture (un coin de l'obstacle
## peut entrer dans le flanc de la voiture sans qu'aucun coin de la voiture n'entre dans l'obstacle).
class_name GeometrieVoiture
extends RefCounted

## essieu : milieu de l'essieu arriere (mm) ; avant_mm / arriere_mm : de l'essieu a l'avant / a l'arriere ;
## obs_min / obs_max : coins de l'obstacle (deja elargi des marges voulues).
static func touche(essieu: Vector2, cap: float, avant_mm: float, arriere_mm: float, demi_largeur_mm: float,
		obs_min: Vector2, obs_max: Vector2) -> bool:
	var avant := Vector2(sin(cap), cos(cap))
	var droite := Vector2(cos(cap), -sin(cap))
	for a in [avant_mm, -arriere_mm]:
		for b in [demi_largeur_mm, -demi_largeur_mm]:
			var c: Vector2 = essieu + avant * a + droite * b
			if c.x >= obs_min.x and c.x <= obs_max.x and c.y >= obs_min.y and c.y <= obs_max.y:
				return true
	for o in [obs_min, Vector2(obs_max.x, obs_min.y), obs_max, Vector2(obs_min.x, obs_max.y)]:
		var rel: Vector2 = o - essieu
		var long := rel.dot(avant)
		var lat := rel.dot(droite)
		if long >= -arriere_mm and long <= avant_mm and absf(lat) <= demi_largeur_mm:
			return true
	return false
