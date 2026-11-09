## Camera qui suit une cible : vue poursuite (lissee) ou vue de dessus.
class_name CameraSuivi
extends Camera3D

const LISSAGE := 6.0          # 1/s, equivalent de l'ancien lissage a 60 images par seconde

var cible: Node3D
var vue_dessus := false

func _ready() -> void:
	near = 0.01
	far = 80.0          # profondeur plus precise : route, bordures et ruban sont a quelques dixiemes de mm l un de l autre
	current = true

func basculer_vue() -> void:
	vue_dessus = not vue_dessus

func _process(dt: float) -> void:
	if cible == null: return
	var t := cible.global_transform
	if vue_dessus:
		global_position = t.origin + Vector3(0, 1.2, 0.001)
		look_at(t.origin, -t.basis.z)
	else:
		var pos := t.origin + t.basis.z * 0.45 + Vector3(0, 0.30, 0)
		# 1 - exp(-k dt) : le meme retard quelle que soit la fluidite (un lerp fixe suit plus vite quand il y a plus d'images)
		global_position = global_position.lerp(pos, 1.0 - exp(-LISSAGE * dt)) if global_position != Vector3.ZERO else pos
		look_at(t.origin + Vector3(0, 0.04, 0) - t.basis.z * 0.15, Vector3.UP)
