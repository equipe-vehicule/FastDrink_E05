## Decor fixe : ciel, soleil et sol 8 x 8 m (dessus a y = 0).
class_name Monde
extends Node3D

func _ready() -> void:
	_creer_ciel()
	_creer_soleil()
	_creer_sol()

func _creer_ciel() -> void:
	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_SKY
	e.sky = Sky.new(); e.sky.sky_material = ProceduralSkyMaterial.new()
	e.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.environment = e
	add_child(env)

func _creer_soleil() -> void:
	var soleil := DirectionalLight3D.new()
	soleil.rotation_degrees = Vector3(-55, 35, 0); soleil.shadow_enabled = true
	add_child(soleil)

# Sol sur sa propre couche (ni ruban ni obstacle)
func _creer_sol() -> void:
	var sol := StaticBody3D.new(); sol.name = "Sol"
	sol.collision_layer = Couches.SOL; sol.collision_mask = 0
	var cs := CollisionShape3D.new(); var bx := BoxShape3D.new(); bx.size = Vector3(8, 0.02, 8)
	cs.shape = bx; cs.position.y = -0.01; sol.add_child(cs)
	var mi := MeshInstance3D.new(); var bm := BoxMesh.new(); bm.size = bx.size
	var mat := StandardMaterial3D.new(); mat.albedo_color = Color(0.82, 0.82, 0.80); bm.material = mat
	mi.mesh = bm; mi.position.y = -0.01; sol.add_child(mi)
	add_child(sol)
