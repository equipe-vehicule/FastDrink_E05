## Acceleration moyenne sur 50 ms (12 pas) : evite les pics dus aux commandes entieres.
class_name MesureAcceleration
extends RefCounted

const TAILLE := 13

var moyenne := 0.0
var maximum := 0.0
var _vitesses: Array = []

func reinitialiser() -> void:
	_vitesses.clear(); moyenne = 0.0; maximum = 0.0

func ajouter(vitesse: float, dt: float) -> void:
	_vitesses.push_back(vitesse)
	if _vitesses.size() > TAILLE: _vitesses.pop_front()
	moyenne = (_vitesses[-1] - _vitesses[0]) / (dt * (_vitesses.size() - 1)) if _vitesses.size() > 1 else 0.0
	maximum = max(maximum, abs(moyenne))
