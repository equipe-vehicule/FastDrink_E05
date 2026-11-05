## Matrice de la piste aleatoire : un chemin sans croisement, puis des obstacles sur ce chemin.
## Pure logique (aucun noeud 3D) : ConstructionPiste transforme le resultat en piste.
##
## Le chemin part de la derniere ligne et monte (Vector2i.UP = cap -Z dans le monde, cap du vehicule au depart).
## Regles :
##   - une case du chemin ne touche aucune autre case du chemin, sauf la precedente et la suivante
##     (sinon les sondes verraient le ruban voisin) ;
##   - les 2 premieres cases sont en ligne droite (le vehicule demarre droit) ;
##   - un obstacle est pose au milieu d'une ligne droite (cases_droites_avant + 1 + cases_droites_apres cases
##     entierement droites), avec une case libre de chaque cote : le contournement revient sur la ligne
##     environ 600 mm apres l'obstacle, la ligne doit encore etre droite a cet endroit.
class_name GenerateurPiste
extends RefCounted

enum { VIDE, RUBAN, OBSTACLE }
const DIRECTIONS := [Vector2i.UP, Vector2i.DOWN, Vector2i.LEFT, Vector2i.RIGHT]
const ESSAIS_MAX := 20000

var p: ParametresPiste
var graine := 0
var matrice: Array = []           # matrice[y][x]
var chemin: Array[Vector2i] = []  # cases dans l'ordre de parcours
var obstacles: Array[int] = []    # indices dans chemin

var _rng := RandomNumberGenerator.new()

func _init(parametres: ParametresPiste) -> void:
	p = parametres

func generer(graine_choisie: int) -> void:
	graine = graine_choisie
	_rng.seed = graine
	_vider_matrice()
	_tracer_chemin()
	_placer_obstacles()

## Matrice lisible dans la console : D depart, # ruban, X obstacle, . vide
func en_texte() -> String:
	var lignes := PackedStringArray(["Piste aleatoire, graine %d, %d cases, %d obstacle(s)" % [graine, chemin.size(), obstacles.size()]])
	for y in p.lignes:
		var ligne := ""
		for x in p.colonnes:
			if chemin.size() > 0 and Vector2i(x, y) == chemin[0]: ligne += "D "
			else: ligne += [". ", "# ", "X "][matrice[y][x]]
		lignes.append(ligne)
	return "\n".join(lignes)

# ---------------------------------------------------------------- chemin
## Parcours en profondeur avec retour en arriere : on avance au hasard,
## et quand on est coince on recule d'une case pour essayer une autre direction.
func _tracer_chemin() -> void:
	var cible := _rng.randi_range(p.longueur_min, maxi(p.longueur_min, p.longueur_max))
	var depart := Vector2i(_rng.randi_range(1, p.colonnes - 2), p.lignes - 1)
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
	return options

## Vrai si `v` est dans la grille, vide, et ne touche aucune case du chemin a part `depuis`.
func _libre(v: Vector2i, depuis: Vector2i) -> bool:
	if not _dans_grille(v) or matrice[v.y][v.x] != VIDE: return false
	for d in DIRECTIONS:
		var voisin: Vector2i = v + d
		if voisin != depuis and _dans_grille(voisin) and matrice[voisin.y][voisin.x] != VIDE:
			return false
	return true

# ---------------------------------------------------------------- obstacles
func _placer_obstacles() -> void:
	obstacles.clear()
	var possibles := []
	for i in range(p.cases_libres_depart, chemin.size() - 1):
		if _droite_avec_place(i): possibles.append(i)
	_melanger(possibles)
	for i in possibles:
		if obstacles.size() >= p.nb_obstacles: break
		if obstacles.all(func(o): return absi(o - i) >= p.ecart_min_obstacles):
			obstacles.append(i)
			_marquer(chemin[i], OBSTACLE)
	obstacles.sort()
	if obstacles.size() < p.nb_obstacles:
		push_warning("Piste aleatoire : %d obstacle(s) place(s) sur %d (pas assez de lignes droites ; augmenter proba_tout_droit ou la longueur)." % [obstacles.size(), p.nb_obstacles])

## Case i entierement droite, ainsi que cases_droites_avant cases avant et cases_droites_apres cases apres
## (une case est droite quand la precedente, elle et la suivante sont alignees : sinon elle contient un virage),
## avec une case de chaque cote dans la grille.
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
	for y in p.lignes:
		var ligne := []
		ligne.resize(p.colonnes)
		ligne.fill(VIDE)
		matrice.append(ligne)

func _marquer(c: Vector2i, valeur: int) -> void:
	matrice[c.y][c.x] = valeur

func _dans_grille(c: Vector2i) -> bool:
	return c.x >= 0 and c.x < p.colonnes and c.y >= 0 and c.y < p.lignes

# Melange avec le generateur de la graine (shuffle() utiliserait le hasard global)
func _melanger(a: Array) -> void:
	for i in range(a.size() - 1, 0, -1):
		var j := _rng.randi_range(0, i)
		var tmp = a[i]; a[i] = a[j]; a[j] = tmp
