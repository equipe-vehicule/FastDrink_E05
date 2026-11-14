## Lecture du fichier unique des reglages des algorithmes (reglages_algorithmes.json, racine du depot) : SEULE source des
## valeurs des reglages du Controleur et de ses modules (le code .gd ne porte que le type et la justification).
## Seul code lie a Godot pour ces reglages ; le programme Python du robot fera la meme chose avec json.load + setattr.
## Charge par preload (pas de class_name) : utilisable sans editeur.
##
## Format : un objet par module, nom = propriete du Controleur (suivi, ultrason, fluide, rattrapage, epreuve, robot...,
## les memes noms que --reglage=<module>:<cle>=<valeur>) ; l'objet "controleur" = reglages du Controleur lui-meme.
## Les cles qui commencent par « _ » (_info) sont ignorees. Un reglage est une variable declaree @export dans le module :
## c'est ce marquage qui permet de signaler toute cle manquante ou en trop.
## Ordre d'application : ce fichier d'abord, puis reglages_essai / --reglage= (qui l'emportent).
extends RefCounted

const CHEMIN := "res://reglages_algorithmes.json"

## Remplit les reglages du controleur et de ses modules depuis le fichier. Signale (push_error) : fichier absent ou
## invalide, module absent du fichier ou inconnu, cle manquante ou en trop, type incompatible.
static func appliquer(controleur: RefCounted, chemin := CHEMIN) -> void:
	var texte := FileAccess.get_file_as_string(chemin)
	if texte.is_empty():
		push_error("Reglages des algorithmes : fichier introuvable ou vide (%s)." % chemin)
		return
	var donnees: Variant = JSON.parse_string(texte)
	if not donnees is Dictionary:
		push_error("Reglages des algorithmes : JSON invalide (%s)." % chemin)
		return
	var modules := _modules(controleur)
	for nom in donnees:
		if String(nom).begins_with("_"):
			continue   # _info : explications, pas un module
		if not modules.has(nom):
			push_error("Reglages des algorithmes : module « %s » inconnu (modules : %s)." % [nom, ", ".join(PackedStringArray(modules.keys()))])
			continue
		if not donnees[nom] is Dictionary:
			push_error("Reglages des algorithmes : « %s » doit etre un objet {reglage: valeur}." % nom)
			continue
		_appliquer_module(nom, modules[nom], donnees[nom])
	for nom in modules:
		if not donnees.has(nom):
			push_error("Reglages des algorithmes : module « %s » absent de %s." % [nom, chemin])

## {nom: objet} des objets qui portent des reglages : le controleur (« controleur ») et ceux de ses proprietes qui en ont.
static func _modules(controleur: RefCounted) -> Dictionary:
	var modules := {"controleur": controleur}
	for p in controleur.get_property_list():
		if p.type != TYPE_OBJECT or not (p.usage & PROPERTY_USAGE_SCRIPT_VARIABLE):
			continue
		var module: Variant = controleur.get(p.name)
		if module is RefCounted and not _reglages_de(module).is_empty():
			modules[p.name] = module
	return modules

## {nom: type} des reglages d'un objet (variables @export).
static func _reglages_de(objet: Object) -> Dictionary:
	var res := {}
	for p in objet.get_property_list():
		if (p.usage & PROPERTY_USAGE_SCRIPT_VARIABLE) and (p.usage & PROPERTY_USAGE_STORAGE):
			res[p.name] = p.type
	return res

static func _appliquer_module(nom: String, module: Object, valeurs: Dictionary) -> void:
	var attendus := _reglages_de(module)
	for cle in valeurs:
		if not attendus.has(cle):
			push_error("Reglages des algorithmes : « %s:%s » n'est pas un reglage de ce module." % [nom, cle])
			continue
		var v: Variant = _convertir(valeurs[cle], attendus[cle])
		if v == null:
			push_error("Reglages des algorithmes : « %s:%s » = %s, type incompatible (attendu : %s)." % [nom, cle, str(valeurs[cle]), type_string(attendus[cle])])
			continue
		module.set(cle, v)
	for cle in attendus:
		if not valeurs.has(cle):
			push_error("Reglages des algorithmes : « %s:%s » absent du fichier." % [nom, cle])

## Valeur convertie au type de la variable (JSON ne distingue pas entier et reel), ou null si incompatible.
static func _convertir(valeur: Variant, type: int) -> Variant:
	var t := typeof(valeur)
	match type:
		TYPE_FLOAT:
			return float(valeur) if t == TYPE_INT or t == TYPE_FLOAT else null
		TYPE_INT:
			if t == TYPE_INT or (t == TYPE_FLOAT and valeur == floorf(valeur)):
				return int(valeur)
			return null
		TYPE_BOOL:
			return valeur if t == TYPE_BOOL else null
		TYPE_STRING:
			return valeur if t == TYPE_STRING else null
		TYPE_ARRAY:
			return valeur if t == TYPE_ARRAY else null
	return null
