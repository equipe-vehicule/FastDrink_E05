## Sonde : valeurs de reference des fonctions GDScript utilisees par le controleur, calculees par Godot lui-meme.
## Le module Python fastdrink/conduite/godot.py doit les reproduire (tests/test_godot.py).
## Lancement (depuis la racine du depot ; code_picar/ est ignore par Godot grace a .gdignore, d'ou le chemin absolu) :
##   G --headless --script <chemin absolu>/code_picar/tests/outils/sonde_fonctions_godot.gd -- <fichier de sortie .json>
## Ecrit un JSON en pleine precision (JSON.stringify full_precision : 17 chiffres, INF -> 1e99999, NAN -> null).
extends SceneTree

var _alea := RandomNumberGenerator.new()


func _init() -> void:
	_alea.seed = 20261114
	var sortie := String(OS.get_cmdline_user_args()[0]) if OS.get_cmdline_user_args().size() > 0 else "fonctions_godot.json"
	var res := {
		"double": OS.has_feature("double"),
		"scalaires": _scalaires(),
		"vecteurs": _vecteurs(),
		"entiers": _entiers(),
	}
	var f := FileAccess.open(sortie, FileAccess.WRITE)
	f.store_string(JSON.stringify(res, "", false, true))
	f.close()
	print("ECRIT ", sortie)
	quit()


## Valeurs particulieres (demi-entiers, zeros signes, bornes) + valeurs aleatoires de plusieurs ordres de grandeur.
func _valeurs() -> Array:
	var v := [0.0, -0.0, 0.5, -0.5, 1.5, -1.5, 2.5, -2.5, 0.49999999999999994, -0.49999999999999994, 1e-7, -1e-7,
			1e-300, PI, -PI, TAU, -TAU, PI - 1e-12, -PI + 1e-12, PI + 1e-6, 3.0 * PI, 45.0, 90.0, 135.0, 179.999,
			0.1, 1.0 / 3.0, 100.25, -100.25, 1e10, -7.3, 7.3, 1e-5, -1e-5, 4e-6]
	for i in 120:
		var e := _alea.randi_range(-4, 4)
		v.append(_alea.randf_range(-1.0, 1.0) * pow(10.0, e))
	return v


func _scalaires() -> Array:
	var res := []
	var vals := _valeurs()
	for i in vals.size():
		var a: float = vals[i]
		var b: float = vals[(i * 7 + 3) % vals.size()]
		var c: float = vals[(i * 13 + 5) % vals.size()]
		var w := _alea.randf_range(-0.5, 1.5)
		var lo := minf(b, c)
		var hi := maxf(b, c)
		var r := {
			"a": a, "b": b, "c": c, "w": w,
			"roundf": roundf(a), "round": round(a), "floorf": floorf(a), "ceilf": ceilf(a), "signf": signf(a),
			"absf": absf(a), "int": int(a) if absf(a) < 1e9 else 0,
			"minf": minf(a, b), "maxf": maxf(a, b),
			"move_toward": move_toward(a, b, absf(c)),
			"lerpf": lerpf(a, b, w),
			"clampf": clampf(a, lo, hi),
			"wrapf_pi": wrapf(a, -PI, PI),
			"wrapf": wrapf(a, lo, hi),
			"fposmod": fposmod(a, b) if b != 0.0 else 0.0,
			"fmod": fmod(a, b) if b != 0.0 else 0.0,
			"deg_to_rad": deg_to_rad(a), "rad_to_deg": rad_to_deg(a),
			"sin": sin(a), "cos": cos(a), "tan": tan(a), "atan": atan(a), "atan2": atan2(a, b),
			"sqrt": sqrt(absf(a)), "pow3": pow(a, 3.0), "pow4": pow(a, 4.0),
			"is_inf": is_inf(a), "is_finite": is_finite(a),
		}
		res.append(r)
	# minf / maxf avec des zeros signes et INF
	res.append({"a": 0.0, "b": -0.0, "minf": minf(0.0, -0.0), "maxf": maxf(0.0, -0.0), "special": true})
	res.append({"a": -0.0, "b": 0.0, "minf": minf(-0.0, 0.0), "maxf": maxf(-0.0, 0.0), "special": true})
	res.append({"a": INF, "b": 1.0, "minf": minf(INF, 1.0), "maxf": maxf(INF, 1.0), "special": true})
	return res


func _vecteurs() -> Array:
	var res := []
	var vals := _valeurs()
	for i in 150:
		var ax: float = vals[_alea.randi_range(0, vals.size() - 1)] * 1000.0
		var ay: float = _alea.randf_range(-2000.0, 2000.0)
		var bx: float = _alea.randf_range(-2000.0, 2000.0)
		var by: float = vals[_alea.randi_range(0, vals.size() - 1)] * 100.0
		var s: float = _alea.randf_range(-3.0, 3.0)
		if i == 0:
			ax = 0.0; ay = 0.0   # normalized() du vecteur nul
		var a := Vector2(ax, ay)
		var b := Vector2(bx, by)
		var m := a
		m += b
		var r := {
			"ax": ax, "ay": ay, "bx": bx, "by": by, "s": s,
			"a": [a.x, a.y],
			"plus": _v(a + b), "moins": _v(a - b), "neg": _v(-a), "fois": _v(a * s), "fois_g": _v(s * a),
			"div": _v(a / s), "plus_egal": _v(m), "fois_v": _v(a * b),
			"dot": a.dot(b), "length": a.length(), "length_squared": a.length_squared(),
			"normalized": _v(a.normalized()), "distance_to": a.distance_to(b),
			"distance_squared_to": a.distance_squared_to(b), "is_finite": a.is_finite(),
			"egal": a == Vector2(a.x, a.y),
		}
		res.append(r)
	res.append({"special": "inf", "is_finite": Vector2(INF, INF).is_finite(), "inf_x": Vector2(INF, INF).x,
			"dist_inf": Vector2(INF, INF).distance_to(Vector2(1.0, 2.0))})
	return res


func _entiers() -> Array:
	var res := []
	for a in [-7, -6, -1, 0, 1, 6, 7, 13]:
		for b in [-3, -2, 2, 3, 5]:
			res.append({"a": a, "b": b, "div": a / b, "mod": a % b, "posmod": posmod(a, b),
					"maxi": maxi(a, b), "mini": mini(a, b), "float_div": float(a) / float(b)})
	return res


func _v(v: Vector2) -> Array:
	return [v.x, v.y]
