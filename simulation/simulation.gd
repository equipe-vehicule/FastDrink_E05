## Scene de simulation du PiCar-S (equipe 05).
## Touches : 1-6 choix de piste | R remettre au depart | M manuel/automatique
##           Fleches : conduite manuelle | Espace : arret | C : vue | L : journal CSV on/off
extends Node3D

const PISTES := [
	["Ligne droite (18 mm)", "res://assets/pistes/test_02_ligne_droite.glb"],
	["Courbes de 120 mm", "res://assets/pistes/test_03_courbes_r120.glb"],
	["Arret devant obstacle", "res://assets/pistes/test_01_arret.glb"],
	["Contournement", "res://assets/pistes/test_04_contournement.glb"],
	["Recul", "res://assets/pistes/test_05_recul.glb"],
	["Parcours fourni", "res://assets/pistes/parcours.glb"],
]
# Depart du parcours fourni (pas de repere Depart dans le fichier) : debut du ruban, cap +X
func depart_parcours() -> Transform3D:
	return Transform3D(Basis(Vector3.UP, -PI / 2), Vector3(-1.482, 0, 0.912))

const RAYON_BILLE := 0.0075
# Cuvette du support de bille (plan support_bille_v2) : rayon de courbure 140 mm, bord a 20 mm du centre, 1,5 mm de profondeur
const R_CUVETTE := 0.140
const R_BORD := 0.020
const G := 9.81
@export var masse_bille := 0.010      ## kg. PROVISOIRE : a peser (n'influence pas la chute, utile pour la suite).
@export var amortissement_bille := 1.0 ## 1/s, pertes par roulement. PROVISOIRE : a comparer avec le vrai robot.
@export var piste_initiale := 1        ## index dans PISTES (1 = courbes)
@export var mode_auto := true

var vehicule: Vehicule
var controleur := Controleur.new()
var bille: RigidBody3D
var piste: Node3D
var index_piste := 0
var _hud: Label
var _camera: Camera3D
var _vue_dessus := false
var _cmd_vitesse := 0
var _cmd_angle := 90
var _bille_tombee := false
var _bille_prete := false
var _q := Vector2.ZERO        # position du centre de la bille dans la cuvette (m) : x = droite, y = avant (repere voiture)
var _dq := Vector2.ZERO       # vitesse relative (m/s)
var _p1 := Vector3.ZERO; var _v1 := Vector3.ZERO; var _n_mesures := 0
var _accel_max := 0.0
var _accel_moy := 0.0
var _vitesses: Array = []
var _temps := 0.0
var _journal: FileAccess
var _touches_prec := {}

func _ready() -> void:
	_creer_monde()
	vehicule = Vehicule.new()
	vehicule.name = "Vehicule"
	add_child(vehicule)
	_creer_bille()
	_creer_camera_et_hud()
	charger_piste(piste_initiale)

# ---------------------------------------------------------------- monde
func _creer_monde() -> void:
	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_SKY
	e.sky = Sky.new(); e.sky.sky_material = ProceduralSkyMaterial.new()
	e.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.environment = e
	add_child(env)
	var soleil := DirectionalLight3D.new()
	soleil.rotation_degrees = Vector3(-55, 35, 0); soleil.shadow_enabled = true
	add_child(soleil)
	# Sol 8 x 8 m, dessus a y = 0, couche 3 (ni ruban ni obstacle)
	var sol := StaticBody3D.new(); sol.name = "Sol"
	sol.collision_layer = 4; sol.collision_mask = 0
	var cs := CollisionShape3D.new(); var bx := BoxShape3D.new(); bx.size = Vector3(8, 0.02, 8)
	cs.shape = bx; cs.position.y = -0.01; sol.add_child(cs)
	var mi := MeshInstance3D.new(); var bm := BoxMesh.new(); bm.size = bx.size
	var mat := StandardMaterial3D.new(); mat.albedo_color = Color(0.82, 0.82, 0.80); bm.material = mat
	mi.mesh = bm; mi.position.y = -0.01; sol.add_child(mi)
	add_child(sol)

func charger_piste(i: int) -> void:
	index_piste = clampi(i, 0, PISTES.size() - 1)
	if piste: piste.queue_free()
	piste = (load(PISTES[index_piste][1]) as PackedScene).instantiate()
	add_child(piste)
	_regler_couches(piste)
	remettre_au_depart()

## Ruban -> couche 2 (detecte par les sondes), obstacles -> couche 1 (detectes par l'ultrason).
## Les obstacles recoivent une forme convexe pleine (l'import cree une surface creuse).
func _regler_couches(n: Node) -> void:
	if n is StaticBody3D:
		var est_ruban: bool = n.get_parent().name.begins_with("Ruban")
		n.collision_layer = 2 if est_ruban else 1
		n.collision_mask = 0
		if not est_ruban and n.get_parent() is MeshInstance3D:
			for c in n.get_children():
				if c is CollisionShape3D:
					c.shape = (n.get_parent() as MeshInstance3D).mesh.create_convex_shape(true, false)
	for c in n.get_children(): _regler_couches(c)

func remettre_au_depart() -> void:
	var depart := piste.find_child("Depart", true, false) as Node3D
	var t: Transform3D = depart_parcours() if depart == null else Transform3D(Basis(), depart.global_position)
	vehicule.placer(t)
	vehicule.commander(0, 90)
	_cmd_vitesse = 0; _cmd_angle = 90
	controleur = Controleur.new()
	_temps = 0.0; _accel_max = 0.0; _accel_moy = 0.0; _vitesses.clear(); _bille_tombee = false; _bille_prete = false
	_replacer_bille()

# ---------------------------------------------------------------- bille
func _creer_bille() -> void:
	bille = RigidBody3D.new(); bille.name = "Bille"
	bille.mass = masse_bille
	bille.continuous_cd = true
	bille.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
	bille.freeze = true
	bille.collision_layer = 16; bille.collision_mask = 4 | 8      # sol + vehicule
	var pm := PhysicsMaterial.new(); pm.friction = 0.4; pm.bounce = 0.0
	bille.physics_material_override = pm
	var cs := CollisionShape3D.new(); var sp := SphereShape3D.new(); sp.radius = RAYON_BILLE
	cs.shape = sp; bille.add_child(cs)
	var modele := (load("res://assets/picar/bille.glb") as PackedScene).instantiate()
	for m in modele.find_children("*", "MeshInstance3D"):
		(m as Node3D).position = Vector3.ZERO      # la sphere est centree sur le corps physique
	bille.add_child(modele)
	add_child(bille)

func _replacer_bille() -> void:
	bille.freeze = true
	_q = Vector2.ZERO; _dq = Vector2.ZERO; _n_mesures = 0
	await get_tree().physics_frame
	await get_tree().physics_frame
	bille.linear_velocity = Vector3.ZERO; bille.angular_velocity = Vector3.ZERO
	bille.global_transform = Transform3D(Basis(), vehicule.position_bille(RAYON_BILLE))
	_bille_prete = true

## Bille qui roule sans glisser dans une cuvette spherique, dans le repere de la voiture.
## Equation (sphere pleine) : q'' = 5/7 * ( -g * q / (R - r) - a_cuvette ) - amortissement * q'
## a_cuvette = acceleration reelle du fond de la cuvette (acceleration de la voiture + effet des virages).
## La bille tombe quand son point de contact depasse le bord de la cuvette (20 mm du centre).
func _bille_dans_cuvette(dt: float) -> void:
	var t := vehicule.global_transform
	var p := vehicule.centre_plaquette()
	var a_monde := Vector3.ZERO
	if _n_mesures >= 2:
		var v := (p - _p1) / dt
		a_monde = (v - _v1) / dt
		_v1 = v
	elif _n_mesures == 1:
		_v1 = (p - _p1) / dt
	_p1 = p; _n_mesures += 1
	var a_loc := Vector2(t.basis.x.dot(a_monde), -t.basis.z.dot(a_monde))   # (droite, avant)
	var rc := R_CUVETTE - RAYON_BILLE
	var acc := (5.0 / 7.0) * (-G * _q / rc - a_loc) - amortissement_bille * _dq
	_dq += acc * dt
	_q += _dq * dt
	var contact := _q.length() * R_CUVETTE / rc
	var hauteur := RAYON_BILLE + _q.length_squared() / (2.0 * rc)
	bille.global_transform = Transform3D(Basis(), p + t.basis * Vector3(_q.x, hauteur, -_q.y))
	if contact >= R_BORD:
		_bille_tombee = true
		var v_voiture := _v1
		var v_rel := t.basis * Vector3(_dq.x, 0, -_dq.y)
		bille.freeze = false
		bille.linear_velocity = v_voiture + v_rel
		print("BILLE TOMBEE a t = %.2f s (acceleration max atteinte %.2f m/s2)" % [_temps, _accel_max])

# ---------------------------------------------------------------- boucle
func _physics_process(dt: float) -> void:
	_temps += dt
	_lire_touches()
	if not _bille_prete or _n_mesures < 3:
		# La voiture reste immobile tant que la bille n'est pas posee et que son modele
		# n'a pas mesure la position de la plaquette a l'arret (quelques pas apres le depart).
		vehicule.commander(0, 90)
	elif mode_auto:
		var sortie: Dictionary = controleur.calculer(vehicule.sondes.duplicate(), vehicule.distance_cm, dt)
		vehicule.commander(int(sortie.get("vitesse", 0)), int(sortie.get("angle", 90)))
	else:
		vehicule.commander(_cmd_vitesse, _cmd_angle)
	# Acceleration moyenne sur 50 ms (12 pas) : evite les pics dus aux commandes entieres.
	_vitesses.push_back(vehicule.vitesse)
	if _vitesses.size() > 13: _vitesses.pop_front()
	_accel_moy = (_vitesses[-1] - _vitesses[0]) / (dt * (_vitesses.size() - 1)) if _vitesses.size() > 1 else 0.0
	_accel_max = max(_accel_max, abs(_accel_moy))
	if _bille_prete and not _bille_tombee:
		_bille_dans_cuvette(dt)
	if _journal:
		_journal.store_csv_line(PackedStringArray([str(snapped(_temps, 0.001)), str(vehicule.commande_vitesse), str(vehicule.angle_servo),
			str(snapped(vehicule.vitesse, 0.0001)), str(snapped(vehicule.acceleration, 0.001)), "".join(vehicule.sondes.map(func(s): return str(s))),
			str(snapped(vehicule.distance_cm, 0.1)), str(int(_bille_tombee))]))

func _process(_dt: float) -> void:
	_suivre_camera()
	if _hud == null: return
	var s := vehicule.sondes
	_hud.text = "Piste : %s   |   Mode : %s\nCommande vitesse : %d   Servo : %d   Roues : %.1f deg\nVitesse : %.3f m/s   Acceleration avant : %.2f m/s2 (max %.2f)   laterale : %.2f m/s2   Distance : %.2f m\nSondes : %d %d %d %d %d   Ultrason : %s\nBille : %s   %s\n%s" % [
		PISTES[index_piste][0], "AUTOMATIQUE" if mode_auto else "MANUEL",
		vehicule.commande_vitesse, vehicule.angle_servo, rad_to_deg(vehicule.angle_roues_rad()),
		vehicule.vitesse, _accel_moy, _accel_max, vehicule.vitesse * vehicule.vitesse * tan(vehicule.angle_roues_rad()) / Vehicule.EMPATTEMENT, vehicule.distance_parcourue,
		s[0], s[1], s[2], s[3], s[4], ("%.1f cm" % vehicule.distance_cm) if vehicule.distance_cm >= 0 else "rien",
		"TOMBEE" if _bille_tombee else "en place", "COLLISION !" if vehicule.en_collision else "",
		"[1-6] piste  [R] depart  [M] mode  [fleches] conduite  [Espace] arret  [C] vue  [L] journal" + ("  (journal en cours)" if _journal else "")]

# ---------------------------------------------------------------- entrees
func _appui(k: Key) -> bool:
	var p := Input.is_physical_key_pressed(k)
	var avant: bool = _touches_prec.get(k, false)
	_touches_prec[k] = p
	return p and not avant

func _lire_touches() -> void:
	for i in 6:
		if _appui(KEY_1 + i): charger_piste(i)
	if _appui(KEY_R): remettre_au_depart()
	if _appui(KEY_M): mode_auto = not mode_auto
	if _appui(KEY_C): _vue_dessus = not _vue_dessus
	if _appui(KEY_L): _basculer_journal()
	if _appui(KEY_SPACE): _cmd_vitesse = 0
	if _appui(KEY_UP): _cmd_vitesse = mini(_cmd_vitesse + 10, 100)
	if _appui(KEY_DOWN): _cmd_vitesse = maxi(_cmd_vitesse - 10, -100)
	if Input.is_physical_key_pressed(KEY_LEFT): _cmd_angle = 45
	elif Input.is_physical_key_pressed(KEY_RIGHT): _cmd_angle = 135
	else: _cmd_angle = 90

func _basculer_journal() -> void:
	if _journal:
		_journal.close(); _journal = null; return
	DirAccess.make_dir_recursive_absolute("res://journaux")
	var nom := "res://journaux/essai_%s.csv" % Time.get_datetime_string_from_system().replace(":", "-")
	_journal = FileAccess.open(nom, FileAccess.WRITE)
	_journal.store_csv_line(PackedStringArray(["t_s", "commande_vitesse", "servo", "vitesse_m_s", "acceleration_m_s2", "sondes", "ultrason_cm", "bille_tombee"]))

# ---------------------------------------------------------------- camera et affichage
func _creer_camera_et_hud() -> void:
	_camera = Camera3D.new(); _camera.near = 0.01; _camera.current = true
	add_child(_camera)
	var cl := CanvasLayer.new(); add_child(cl)
	var fond := PanelContainer.new(); cl.add_child(fond)
	_hud = Label.new(); fond.add_child(_hud)

func _suivre_camera() -> void:
	if _camera == null: return
	var t := vehicule.global_transform
	if _vue_dessus:
		_camera.global_position = t.origin + Vector3(0, 1.2, 0.001)
		_camera.look_at(t.origin, -t.basis.z)
	else:
		var cible := t.origin + t.basis.z * 0.45 + Vector3(0, 0.30, 0)
		_camera.global_position = _camera.global_position.lerp(cible, 0.1) if _camera.global_position != Vector3.ZERO else cible
		_camera.look_at(t.origin + Vector3(0, 0.04, 0) - t.basis.z * 0.15, Vector3.UP)
