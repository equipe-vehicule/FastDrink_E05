## Decor fixe : ciel, soleil, sol 8 x 8 m (dessus a y = 0) et paysage autour (herbe, foret, brume).
## Le paysage est purement visuel : seule la collision du sol (couche SOL) compte pour la simulation.
class_name Monde
extends Node3D

const TAILLE_PAYSAGE := 80.0     # m, herbe visible jusqu'a l'horizon (dans la brume)

func _ready() -> void:
	_creer_ciel()
	_creer_soleil()
	_creer_sol()
	_creer_paysage()

func _creer_ciel() -> void:
	var env := WorldEnvironment.new()
	var e := Environment.new()
	var ciel := ProceduralSkyMaterial.new()
	ciel.sky_top_color = Color(0.22, 0.45, 0.85)
	ciel.sky_horizon_color = Color(0.68, 0.8, 0.95)
	ciel.ground_horizon_color = Color(0.6, 0.72, 0.6)
	ciel.ground_bottom_color = Color(0.25, 0.35, 0.2)
	ciel.sun_angle_max = 20.0
	e.background_mode = Environment.BG_SKY
	e.sky = Sky.new(); e.sky.sky_material = ciel
	e.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	e.ambient_light_energy = 0.9
	e.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	e.tonemap_white = 6.0
	e.fog_enabled = true              # brume : cache le bord du paysage
	e.fog_light_color = Color(0.7, 0.8, 0.92)
	e.fog_density = 0.02
	e.fog_aerial_perspective = 0.5
	e.adjustment_enabled = true
	e.adjustment_saturation = 1.15
	env.environment = e
	add_child(env)

func _creer_soleil() -> void:
	var soleil := DirectionalLight3D.new()
	soleil.rotation_degrees = Vector3(-50, 35, 0)
	soleil.light_color = Color(1.0, 0.96, 0.88)
	soleil.light_energy = 1.2
	soleil.shadow_enabled = true
	soleil.directional_shadow_mode = DirectionalLight3D.SHADOW_ORTHOGONAL   # une seule passe d'ombres au lieu de quatre
	soleil.directional_shadow_max_distance = 6.0
	add_child(soleil)

# Sol sur sa propre couche (ni ruban ni obstacle). Sa collision ne change pas ; on le voit en herbe (_creer_paysage).
func _creer_sol() -> void:
	var sol := StaticBody3D.new(); sol.name = "Sol"
	sol.collision_layer = Couches.SOL; sol.collision_mask = 0
	var cs := CollisionShape3D.new(); var bx := BoxShape3D.new(); bx.size = Vector3(8, 0.02, 8)
	cs.shape = bx; cs.position.y = -0.01; sol.add_child(cs)
	add_child(sol)

## Grande prairie (juste sous le ruban et la route) et foret tout autour de la zone de jeu.
func _creer_paysage() -> void:
	var herbe := MeshInstance3D.new(); herbe.name = "Herbe"
	var plan := PlaneMesh.new(); plan.size = Vector2(TAILLE_PAYSAGE, TAILLE_PAYSAGE)
	var mat := StandardMaterial3D.new()
	mat.albedo_texture = DecorPiste.texture_bruit(Color(0.24, 0.48, 0.16), Color(0.42, 0.66, 0.24), 0.04, 11)
	mat.uv1_scale = Vector3(TAILLE_PAYSAGE / 2.0, TAILLE_PAYSAGE / 2.0, 1)   # un motif tous les 2 m
	mat.roughness = 1.0
	plan.material = mat
	herbe.mesh = plan
	herbe.position.y = -0.002        # sous la route (0.0001) et le ruban (0.0005)
	add_child(herbe)
	# foret : hors du sol de jeu (8 x 8 m), pour que la voiture ne la traverse jamais
	var rng := RandomNumberGenerator.new(); rng.seed = 2026
	var places: Array[Vector2] = []
	while places.size() < 500:
		var p := Vector2(rng.randf_range(-20, 20), rng.randf_range(-20, 20))
		if absf(p.x) > 4.6 or absf(p.y) > 4.6:
			places.append(p)
	var foret := DecorPiste.vegetation(places, rng, 2.5, false)   # arbres plus grands au loin : effet de profondeur
	add_child(foret)
