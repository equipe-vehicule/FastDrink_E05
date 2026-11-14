## Sonde du lot B : valeurs de reference de ContournementFluide et Contournement calculees par Godot lui-meme
## (choisir, _poursuite_libre via choisir, x_chemin, _q, choisir_cap, et des suites de calculer() sur des entrees
## pseudo-aleatoires enregistrees). tests/test_lot_b.py rejoue les memes entrees avec les ports Python et compare.
## Lancement (depuis la racine du depot ; code_picar/ est ignore par Godot grace a .gdignore, d'ou le chemin absolu) :
##   G --headless --path . --script <chemin absolu>/code_picar/tests/outils/sonde_lot_b.gd -- <fichier de sortie .json>
## puis compresser : py -3 -c "import gzip,shutil;shutil.copyfileobj(open('x.json','rb'),gzip.open('x.json.gz','wb'))"
## Ecrit un JSON en pleine precision (JSON.stringify full_precision : 17 chiffres, NAN -> null).
extends SceneTree

const ReglagesAlgorithmes := preload("res://simulation/reglages_algorithmes.gd")
const FichierReglages := preload("res://simulation/fichier_reglages.gd")

var _alea := RandomNumberGenerator.new()
var _phases_vues := {}


func _init() -> void:
	_alea.seed = 20261114
	var sortie := String(OS.get_cmdline_user_args()[0]) if OS.get_cmdline_user_args().size() > 0 else "lot_b_godot.json"
	var res := {
		"double": OS.has_feature("double"),
		"q": _q(),
		"fluide": _fluide(),
		"choisir_cap": _choisir_cap(),
		"contournement": _contournement(),
	}
	var f := FileAccess.open(sortie, FileAccess.WRITE)
	f.store_string(JSON.stringify(res, "", false, true))
	f.close()
	print("PHASES VUES ", _phases_vues)
	print("ECRIT ", sortie)
	quit()


## Controleur regle comme Simulation._controleur_regle (reglages_algorithmes.json + reglages_robot.json), puis variantes.
func _nouveau(ov: Dictionary) -> Controleur:
	var c := Controleur.new()
	ReglagesAlgorithmes.appliquer(c)
	c.robot.appliquer(FichierReglages.lire())
	for mod in ov:
		if mod == "servo_estime":
			c.odo.servo_estime = ov[mod]
			continue
		for cle in ov[mod]:
			c.get(mod).set(cle, ov[mod][cle])
	return c


func _alea_f(a: float, b: float) -> float:
	return _alea.randf_range(a, b)


## Variantes de reglages : modele ideal du robot, dimensions de l'obstacle, marges, servo.
func _variante() -> Dictionary:
	var ov := {"servo_estime": _alea_f(45.0, 135.0) if _alea.randf() < 0.5 else 90.0}
	var robot := {}
	var epreuve := {}
	var fluide := {}
	var cont := {}
	if _alea.randf() < 0.3:
		robot["table_vitesse_mm_s"] = []
		robot["mm_s_par_cmd"] = 4.0
	if _alea.randf() < 0.2:
		robot["vitesse_servo_deg_s"] = 0.0
	elif _alea.randf() < 0.3:
		robot["vitesse_servo_deg_s"] = _alea_f(150.0, 600.0)
	if _alea.randf() < 0.4:
		epreuve["largeur_obstacle_mm"] = [100.0, 150.0, 50.0][_alea.randi_range(0, 2)]
		epreuve["hauteur_obstacle_mm"] = [100.0, 150.0, 50.0][_alea.randi_range(0, 2)]
		epreuve["profondeur_obstacle_mm"] = [100.0, 150.0, 50.0][_alea.randi_range(0, 2)]
	if _alea.randf() < 0.4:
		fluide["marge_obstacle_mm"] = _alea_f(0.0, 40.0)
	if _alea.randf() < 0.2:
		fluide["acceleration_laterale_max"] = _alea_f(0.1, 1.5)
	if _alea.randf() < 0.2:
		fluide["anticipation_mm"] = _alea_f(50.0, 250.0)
	if _alea.randf() < 0.2:
		cont["gain_cap"] = _alea_f(0.5, 2.0)
	if _alea.randf() < 0.2:
		cont["braquage_manoeuvre"] = _alea_f(20.0, 45.0)
	if _alea.randf() < 0.2:
		cont["decalage_max_mm"] = _alea_f(50.0, 400.0)
	if _alea.randf() < 0.2:
		cont["cap_retour_deg"] = _alea_f(20.0, 60.0)
	if _alea.randf() < 0.3:
		cont["recherche_en_orbite"] = false
	ov["robot"] = robot
	ov["epreuve"] = epreuve
	ov["fluide"] = fluide
	ov["contournement"] = cont
	return ov


func _q() -> Array:
	var res := []
	for i in 80:
		var t: float = _alea_f(-0.3, 1.3) if i > 3 else [0.0, 1.0, 0.5, -1.0][i]
		res.append({"t": t, "q": ContournementFluide._q(t)})
	return res


## choisir() : rayon choisi (rejeu de la poursuite), vitesse, trajectoire (y1, y2, y3, ecart), x_chemin ; puis
## demarrer() et une suite de calculer() sur des entrees enregistrees.
func _fluide() -> Array:
	var res := []
	for i in 120:
		var ov := _variante()
		var c := _nouveau(ov)
		var d := _alea_f(50.0, 400.0)
		var cote := 1 if _alea.randf() < 0.5 else -1
		var x0 := _alea_f(-100.0, 100.0) if _alea.randf() < 0.5 else 0.0
		var cap0 := _alea_f(-0.5, 0.5) if _alea.randf() < 0.5 else 0.0
		var ok := c.fluide.choisir(d, cote, x0, cap0)
		var r := {"ov": ov, "d": d, "cote": cote, "x0": x0, "cap0": cap0, "ok": ok,
				"rayon": c.fluide.rayon_choisi_mm, "v": c.fluide.vitesse_mm_s,
				"y1": c.fluide._y1, "y2": c.fluide._y2, "y3": c.fluide._y3, "y_face": c.fluide._y_face,
				"ecart": c.fluide._ecart, "x0_int": c.fluide._x0, "cap0_int": c.fluide._cap0,
				"ecart_mm": c.fluide.ecart_mm(), "rayon_retour": c.fluide._rayon_retour_mm()}
		var xs := []
		for k in 12:
			var y: float = -50.0 + k * 90.0
			xs.append([y, c.fluide.x_chemin(y)])
		r["x_chemin"] = xs
		if ok:
			r["pas"] = _pas_fluide(c)
		res.append(r)
	return res


func _pas_fluide(c: Controleur) -> Array:
	c.fluide.demarrer()
	var pas := []
	var dt := 0.02
	for s in 600:
		var n := 0
		var somme := 0.0
		if _alea.randf() < (0.0 if s < 200 else 0.25):
			n = _alea.randi_range(1, 5)
			somme = n * _alea_f(-38.0, 38.0)
		var out: Vector2 = c.fluide.calculer(somme, n)
		c.odo.suivre_servo(out.x, dt)
		c.odo.integrer(out.y, dt)
		pas.append([somme, n, out.x, out.y, c.fluide.termine, c.fluide.fin_sans_ligne, c.fluide.e_reprise,
				c.fluide.nom_phase(), c.odo.x, c.odo.y, c.odo.cap])
		if c.fluide.termine:
			break
	return pas


## choisir_cap() apres demarrer() : cap retenu, _y_fin, _x_requis, _centre, _zones.
func _choisir_cap() -> Array:
	var res := []
	for i in 150:
		var ov := _variante()
		var c := _nouveau(ov)
		var d0 := _alea_f(50.0, 400.0)
		var braq := _alea_f(-30.0, 30.0)
		var marge := _alea_f(0.0, 40.0)
		c.contournement.demarrer(d0, braq)
		var ok := c.contournement.choisir_cap(d0, marge)
		res.append({"ov": ov, "d0": d0, "braq": braq, "marge": marge, "ok": ok,
				"cap": c.contournement._cap_ecart_deg, "cote": c.contournement.cote,
				"y_fin": c.contournement._y_fin, "x_requis": c.contournement._x_requis,
				"centre": [c.contournement._centre.x, c.contournement._centre.y],
				"zones": c.contournement._zones.map(func(z): return [z.x, z.y]),
				"largeur": c.contournement._largeur_obstacle(), "longueur": c.contournement._longueur_obstacle(),
				"rayon_virage": c.contournement._rayon_virage_mm()})
	return res


## Suites de calculer() sur des entrees pseudo-aleatoires mais coherentes (obstacle vu au debut, puis perdu ; ligne
## vue plus tard). L odometrie avance avec les sorties (servo, puis integrer a 4 fois la vitesse de manoeuvre : irreel,
## mais atteint plus de phases en peu de pas).
func _contournement() -> Array:
	var res := []
	for i in 300:
		var ov := _variante()
		var c := _nouveau(ov)
		var cont: Contournement = c.contournement
		var d0 := _alea_f(60.0, 300.0)
		var braq := _alea_f(-30.0, 30.0)
		var hors := _alea.randf() < 0.15
		var cote := 1 if _alea.randf() < 0.5 else -1
		var avec_choix := _alea.randf() < 0.5
		var chemin := []
		if _alea.randf() < 0.5:
			for k in _alea.randi_range(1, 6):
				chemin.append([_alea_f(-60.0, 60.0), _alea_f(-1000.0, 0.0)])
		cont.chemin_entrant.clear()
		for p in chemin:
			cont.chemin_entrant.append(Vector2(p[0], p[1]))
		var ok_cap := true
		if hors:
			cont.demarrer_hors_ligne(d0, cote)
		else:
			cont.demarrer(d0, braq)
			if avec_choix:
				ok_cap = cont.choisir_cap(d0, c.fluide.marge_obstacle_mm)
		var k_vu := _alea.randi_range(0, 40)
		var k_ligne := _alea.randi_range(20, 200)
		var p_ligne: float = [0.0, 0.05, 0.3, 0.7, 1.0][_alea.randi_range(0, 4)]
		var p_obs: float = [0.0, 0.05, 0.3][_alea.randi_range(0, 2)]
		var e_base := _alea_f(-38.0, 38.0) if _alea.randf() < 0.5 else _alea_f(-8.0, 8.0)
		var pas := []
		var dt := 0.02
		var v := cont.vitesse_manoeuvre_mm_s
		for s in 400:
			var n := 0
			var somme := 0.0
			if s >= k_ligne and _alea.randf() < p_ligne:
				n = _alea.randi_range(1, 5)
				somme = n * (e_base + _alea_f(-10.0, 10.0))
			var d := -1.0
			var d_brut := -1.0
			if s < k_vu:
				d = _alea_f(80.0, 200.0)
				d_brut = d
			elif _alea.randf() < p_obs:
				d = _alea_f(100.0, 500.0)
				d_brut = d if _alea.randf() < 0.8 else -1.0
			var out: Vector2 = cont.calculer(somme, n, v, d, d_brut, dt)
			c.odo.suivre_servo(out.x, dt)
			c.odo.integrer(out.y * 4.0, dt)
			v = out.y
			_phases_vues[cont.nom_phase()] = int(_phases_vues.get(cont.nom_phase(), 0)) + 1
			pas.append([somme, n, d, d_brut, out.x, out.y, cont.nom_phase(), cont.termine, cont.echec,
					cont.ecart_ligne_mm(), c.odo.x, c.odo.y, c.odo.cap, cont._x_requis, cont._e_vu, cont._arc])
			if cont.termine or cont.echec:
				break
		res.append({"ov": ov, "d0": d0, "braq": braq, "hors": hors, "cote": cote, "avec_choix": avec_choix,
				"ok_cap": ok_cap, "chemin": chemin, "pas": pas})
	return res
