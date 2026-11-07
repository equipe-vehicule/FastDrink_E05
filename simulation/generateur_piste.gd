## Matrice de la piste aleatoire : un chemin sans croisement, puis des obstacles sur ce chemin.
## Pure logique (aucun noeud 3D) : ConstructionPiste transforme le resultat en piste.
##
## Le chemin part de la derniere ligne et monte (Vector2i.UP = cap -Z dans le monde, cap du vehicule au depart).
## Contraintes gardees (cahier des charges et physique) :
##   - ruban de largeur_ruban, virages de rayon >= rayon_virage (120 mm), pas de croisement de lignes ;
##   - les 2 premieres cases sont droites et sans obstacle (la voiture est posee dessus au depart).
## Tout le reste est libre par defaut (reglable dans ParametresPiste) :
##   - rayon des virages tire au hasard pour chaque piste entre rayon_virage et rayon_virage_max ;
##   - cases voisines autorisees : demi-tours serres, lignes paralleles a une case d'ecart ;
##   - obstacles n'importe ou sur le ruban, y compris en virage, colles les uns aux autres,
##     de taille variable et decales par rapport a la ligne.
class_name GenerateurPiste
extends RefCounted

enum { VIDE, RUBAN, OBSTACLE }
const DIRECTIONS := [Vector2i.UP, Vector2i.DOWN, Vector2i.LEFT, Vector2i.RIGHT]
const ESSAIS_MAX := 20000
const TAILLE_SOL_UTILE := 7.6     # m : la grille doit tenir sur le sol de 8 m

var p: ParametresPiste
var graine := 0
var rayon := 0.12                 # rayon des virages de cette piste (m)
var colonnes := 0                 # taille reelle de la grille (reduite si les cases sont grandes)
var lignes := 0
var matrice: Array = []           # matrice[y][x]
var chemin: Array[Vector2i] = []  # cases dans l'ordre de parcours
var obstacles: Array[int] = []    # indices dans chemin
var tailles_obstacles: Array[Vector3] = []   # meme ordre que obstacles
var decalages_obstacles: Array[float] = []   # m, decalage lateral par rapport a la ligne (> 0 = a droite)

var _rng := RandomNumberGenerator.new()

func _init(parametres: ParametresPiste) -> void:
	p = parametres

func generer(graine_choisie: int) -> void:
	graine = graine_choisie
	_rng.seed = graine
	rayon = _rng.randf_range(p.rayon_virage, maxf(p.rayon_virage, p.rayon_virage_max))
	var max_cases := int(TAILLE_SOL_UTILE / taille_case())
	colonnes = mini(p.colonnes, max_cases)
	lignes = mini(p.lignes, max_cases)
	_vider_matrice()
	_tracer_chemin()
	_placer_obstacles()

func taille_case() -> float:
	return 2.0 * rayon

## Matrice lisible dans la console : D depart, # ruban, X obstacle, . vide
func en_texte() -> String:
	var texte := PackedStringArray(["Piste aleatoire, graine %d, rayon %d mm, %d cases, %d obstacle(s)" % [
		graine, roundi(rayon * 1000.0), chemin.size(), obstacles.size()]])
	for y in lignes:
		var ligne := ""
		for x in colonnes:
			if chemin.size() > 0 and Vector2i(x, y) == chemin[0]: ligne += "D "
			else: ligne += [". ", "# ", "X "][matrice[y][x]]
		texte.append(ligne)
	return "\n".join(texte)

# ---------------------------------------------------------------- chemin
## Parcours en profondeur avec retour en arriere : on avance au hasard,
## et quand on est coince on recule d'une case pour essayer une autre direction.
func _tracer_chemin() -> void:
	var cible := _rng.randi_range(p.longueur_min, maxi(p.longueur_min, p.longueur_max))
	var depart := Vector2i(_rng.randi_range(1, colonnes - 2), lignes - 1)
	chemin = [depart]
	_marquer(depart, RUBAN)
	var choix: Array = [_candidats(depart, Vector2i.UP)]   # choix[i] = cases encore a essayer depuis chemin[i]
	var meilleur := chemin.duplicate()
	var essais := 0
	while chemin.size() < cible and essais < ESSAIS_MAX:
		essais += 1
		var options: Array = choix[-1]
		if options.is_empty():
			if chemin.size() == 1: break
			choix.pop_back()
			_marquer(chemin.pop_back(), VIDE)
			continue
		var suivante: Vector2i = options.pop_front()
		chemin.append(suivante)
		_marquer(suivante, RUBAN)
		choix.append(_candidats(suivante, suivante - chemin[-2]))
		if chemin.size() > meilleur.size(): meilleur = chemin.duplicate()
	# Cible inatteignable (grille trop petite) : on garde le plus long chemin trouve
	if chemin.size() < meilleur.size():
		chemin = meilleur
		_vider_matrice()
		for c in chemin: _marquer(c, RUBAN)
	if chemin.size() < p.longueur_min:
		push_warning("Piste aleatoire : seulement %d cases (minimum demande %d). Agrandir la grille ?" % [chemin.size(), p.longueur_min])

## Cases possibles apres `case` quand on y arrive dans la direction `dir`, deja melangees.
func _candidats(case: Vector2i, dir: Vector2i) -> Array:
	var tout_droit := case + dir
	if chemin.size() <= 2:
		return [tout_droit] if _libre(tout_droit, case) else []
	var options := []
	for d in DIRECTIONS:
		if d != -dir and _libre(case + d, case):
			options.append(case + d)
	_melanger(options)
	if tout_droit in options and _rng.randf() < p.proba_tout_droit:
		options.erase(tout_droit)
		options.push_front(tout_droit)
	# Ligne droite minimale : tant que la ligne droite en cours est trop courte, on continue tout droit si possible.
	if tout_droit in options and _longueur_droite(dir) < p.longueur_droite_min:
		options.erase(tout_droit)
		options.push_front(tout_droit)
	return options

## Nombre de deplacements consecutifs dans la direction dir a la fin du chemin.
func _longueur_droite(dir: Vector2i) -> int:
	var n := 0
	while n + 1 < chemin.size() and chemin[-1 - n] - chemin[-2 - n] == dir:
		n += 1
	return n

## Vrai si `v` est dans la grille et vide. Si les cases voisines sont interdites,
## `v` ne doit en plus toucher aucune case du chemin a part `depuis`.
func _libre(v: Vector2i, depuis: Vector2i) -> bool:
	if not _dans_grille(v) or matrice[v.y][v.x] != VIDE: return false
	if p.autoriser_cases_voisines: return true
	for d in DIRECTIONS:
		var voisin: Vector2i = v + d
		if voisin != depuis and _dans_grille(voisin) and matrice[voisin.y][voisin.x] != VIDE:
			return false
	return true

# ---------------------------------------------------------------- obstacles
## Chaque case possible recoit un obstacle avec la probabilite proba_obstacle.
func _placer_obstacles() -> void:
	obstacles.clear()
	tailles_obstacles.clear()
	decalages_obstacles.clear()
	var dernier := -1000
	for i in range(maxi(p.cases_libres_depart, 2), chemin.size()):
		if obstacles.size() >= p.nb_obstacles: break
		if i - dernier < p.ecart_min_obstacles: continue
		if not p.obstacles_en_virage and not _droite_avec_place(i): continue
		if _rng.randf() >= p.proba_obstacle: continue
		obstacles.append(i)
		dernier = i
		_marquer(chemin[i], OBSTACLE)
		var t_min := p.taille_obstacle
		var t_max := p.taille_obstacle_max
		tailles_obstacles.append(Vector3(_rng.randf_range(t_min.x, maxf(t_min.x, t_max.x)),
				_rng.randf_range(t_min.y, maxf(t_min.y, t_max.y)), _rng.randf_range(t_min.z, maxf(t_min.z, t_max.z))))
		decalages_obstacles.append(_rng.randf_range(-p.decalage_obstacle_max, p.decalage_obstacle_max))

## Utilise seulement si obstacles_en_virage = false : case i entierement droite, ainsi que
## cases_droites_avant cases avant et cases_droites_apres cases apres (une case est droite quand la precedente,
## elle et la suivante sont alignees : sinon elle contient un virage), avec une case de chaque cote dans la grille.
func _droite_avec_place(i: int) -> bool:
	if i - 1 - p.cases_droites_avant < 0 or i + 1 + p.cases_droites_apres >= chemin.size(): return false
	var dir := chemin[i] - chemin[i - 1]
	for k in range(i - p.cases_droites_avant, i + p.cases_droites_apres + 1):
		if chemin[k] - chemin[k - 1] != dir or chemin[k + 1] - chemin[k] != dir: return false
	var cote := Vector2i(dir.y, dir.x)
	return _dans_grille(chemin[i] + cote) and _dans_grille(chemin[i] - cote)

# ---------------------------------------------------------------- outils
func _vider_matrice() -> void:
	matrice.clear()
	for y in lignes:
		var ligne := []
		ligne.resize(colonnes)
		ligne.fill(VIDE)
		matrice.append(ligne)

func _marquer(c: Vector2i, valeur: int) -> void:
	matrice[c.y][c.x] = valeur

func _dans_grille(c: Vector2i) -> bool:
	return c.x >= 0 and c.x < colonnes and c.y >= 0 and c.y < lignes

# Melange avec le generateur de la graine (shuffle() utiliserait le hasard global)
func _melanger(a: Array) -> void:
	for i in range(a.size() - 1, 0, -1):
		var j := _rng.randi_range(0, i)
		var tmp = a[i]; a[i] = a[j]; a[j] = tmp
