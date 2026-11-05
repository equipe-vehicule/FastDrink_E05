## Lecture du clavier. Les touches "action" emettent un signal ;
## la conduite manuelle est exposee dans cmd_vitesse / cmd_angle.
class_name Entrees
extends RefCounted

signal piste_demandee(index: int)
signal remise_demandee
signal mode_bascule
signal vue_bascule
signal journal_bascule

var cmd_vitesse := 0
var cmd_angle := 90
var _touches_prec := {}

func reinitialiser() -> void:
	cmd_vitesse = 0; cmd_angle = 90

func lire(nb_pistes: int) -> void:
	for i in nb_pistes:
		if _appui(KEY_1 + i): piste_demandee.emit(i)
	if _appui(KEY_R): remise_demandee.emit()
	if _appui(KEY_M): mode_bascule.emit()
	if _appui(KEY_C): vue_bascule.emit()
	if _appui(KEY_L): journal_bascule.emit()
	if _appui(KEY_SPACE): cmd_vitesse = 0
	if _appui(KEY_UP): cmd_vitesse = mini(cmd_vitesse + 10, 100)
	if _appui(KEY_DOWN): cmd_vitesse = maxi(cmd_vitesse - 10, -100)
	if Input.is_physical_key_pressed(KEY_LEFT): cmd_angle = 45
	elif Input.is_physical_key_pressed(KEY_RIGHT): cmd_angle = 135
	else: cmd_angle = 90

# Vrai seulement au moment ou la touche est enfoncee (front montant)
func _appui(k: Key) -> bool:
	var p := Input.is_physical_key_pressed(k)
	var avant: bool = _touches_prec.get(k, false)
	_touches_prec[k] = p
	return p and not avant
