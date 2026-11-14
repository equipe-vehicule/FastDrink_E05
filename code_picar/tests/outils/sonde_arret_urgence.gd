## Sonde de l'arret d'urgence (14 nov.) : un Controleur regle comme la simulation (reglages_algorithmes.json +
## reglages_robot.json, modele realiste) est amene dans une manoeuvre, puis recoit une distance de 3 cm (30 mm, sous
## distance_securite_mm = 40 mm) pendant plus de arret_urgence_max_s ; une 2e serie s'arrete a 2,9 s par un echo lointain.
## Enregistre, tour par tour : les entrees de calculer(), les sorties, l'etat, la duree continue de l'arret d'urgence
## (_t_urgence) et la distance filtree. tests/test_arret_urgence.py rejoue les memes entrees avec le port Python.
## Lancement (depuis la racine du depot ; chemin absolu, code_picar/ est ignore par Godot grace a .gdignore) :
##   G --headless --path . --script <abs>/code_picar/tests/outils/sonde_arret_urgence.gd -- <abs>/sortie.json
## puis compresser : py -3 -c "import gzip,shutil;shutil.copyfileobj(open('x.json','rb'),gzip.open('x.json.gz','wb',9))"
extends SceneTree

const ReglagesAlgorithmes := preload("res://simulation/reglages_algorithmes.gd")
const FichierReglages := preload("res://simulation/fichier_reglages.gd")

const DT := 0.02                      # 50 Hz (frequence_controle_hz)
const SUR_LIGNE := [0, 0, 1, 0, 0]
const SANS_LIGNE := [0, 0, 0, 0, 0]
const D_DEPART_MM := 300.0            # prelude : obstacle initialement a 300 mm ; la distance suit la vitesse du controleur
const D_URGENCE_CM := 3.0
const D_LOIN_CM := 60.0


func _init() -> void:
	var sortie := String(OS.get_cmdline_user_args()[0]) if OS.get_cmdline_user_args().size() > 0 else "arret_urgence_godot.json"
	var res := {"dt": DT, "series": {}}
	res["series"]["continu"] = _serie(4.0, false, true)    # 4 s d'echo a 30 mm : la manoeuvre doit se terminer a 3 s
	res["series"]["court"] = _serie(2.9, true, true)       # 2,9 s puis echo lointain : la manoeuvre continue
	res["series"]["phases"] = _serie(4.0, false, false)    # contournement par phases (fluide inactif) : RETROUVER
	var f := FileAccess.open(sortie, FileAccess.WRITE)
	f.store_string(JSON.stringify(res, "", false, true))
	f.close()
	print("ECRIT ", sortie)
	quit()


func _nouveau() -> Controleur:
	var c := Controleur.new()
	ReglagesAlgorithmes.appliquer(c)
	c.robot.appliquer(FichierReglages.lire())
	return c


## Une serie : prelude (suivi, arret, recul, contournement), puis duree_s d'echo a 30 mm, puis (si lointain) 1,5 s de
## echo lointain, sinon rien de plus.
func _serie(duree_s: float, puis_loin: bool, fluide: bool) -> Dictionary:
	var c := _nouveau()
	c.contournement_fluide = fluide
	var tours := []
	var i := 0
	var etat_urgence := ""
	var debut := -1
	var fin_prelude := -1
	# Prelude : la voiture roule vers l'obstacle (la distance suit les commandes) : arret, recul, puis la premiere
	# manoeuvre avancante (fluide ou par phases)
	var avance_mm := 0.0
	while i < 3000:
		if c.etat_court() in ["CONTOURNEMENT", "CONTOURNEMENT_FLUIDE"]:
			break
		var r := _tour(c, tours, SUR_LIGNE, (D_DEPART_MM - avance_mm) / 10.0, i)
		avance_mm += c.robot.vitesse_odometrie_mm_s(float(r["vitesse"])) * DT
		i += 1
	fin_prelude = i
	# Manoeuvre : sondes sans ligne, quelques tours a l'echo normal, puis l'echo de 30 mm
	for k in 25:
		_tour(c, tours, SANS_LIGNE, D_DEPART_MM / 10.0, i)
		i += 1
	var debut_urgence := i
	var n := int(round(duree_s / DT))
	for k in n:
		_tour(c, tours, SANS_LIGNE, D_URGENCE_CM, i)
		i += 1
	var n_apres := int(round((1.5 if puis_loin else 0.5) / DT))
	for k in n_apres:
		_tour(c, tours, SANS_LIGNE, D_LOIN_CM, i)
		i += 1
	print("SERIE fin_prelude=", fin_prelude, " debut_urgence=", debut_urgence, " tours=", tours.size(), " etat_final=", c.etat_court())
	return {"fin_prelude": fin_prelude, "debut_urgence": debut_urgence, "tours": tours}


## [sondes, d_cm, dt, mesure_nouvelle, vitesse, angle, etat, t_urgence, d_filtre_mm]
func _tour(c: Controleur, tours: Array, sondes: Array, d_cm: float, i: int) -> Dictionary:
	var nouvelle := i % 3 == 0
	var r := c.calculer(sondes, d_cm, DT, nouvelle)
	tours.append([sondes, d_cm, DT, nouvelle, r["vitesse"], r["angle"], c.etat_court(), c._t_urgence, c.distance_filtree_mm])
	return r
