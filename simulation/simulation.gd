## Scene de simulation du PiCar-S (equipe 05).
## Touches : 1-7 choix de piste (7 = aleatoire) | G nouvelle piste aleatoire | R remettre au depart
##           M manuel/automatique | Fleches : conduite manuelle | Espace : arret | C : vue | L : journal CSV on/off
## Ligne de commande (apres --) : --piste=6  --graine=1234 ; banc sans editeur : voir _lire_ligne_de_commande()
## Valeurs du robot : reglages_robot.json (racine du depot), fichier unique partage avec le code Python.
##
## Ce script ne fait qu'assembler les modules et decider qui conduit la voiture :
##   Monde, Pistes, Vehicule, BilleCuvette, CameraSuivi, Hud, Entrees, MesureAcceleration, JournalCSV
extends Node3D

## Lecture du fichier unique des valeurs du robot (reglages_robot.json, racine du depot).
const FichierReglages := preload("res://simulation/fichier_reglages.gd")
## Scenarios courts et rejouables (option --scenarios=tous ou --scenarios=nom1,nom2).
const Scenarios := preload("res://simulation/scenarios.gd")

@export var piste_initiale := 1        ## index dans Pistes.LISTE (1 = courbes, 6 = aleatoire)
@export var mode_auto := true
@export var parametres_piste: ParametresPiste ## reglages de la piste aleatoire (vide = valeurs par defaut)
## Demarre le journal CSV des le lancement (essais automatiques, sans appuyer sur L).
@export var journal_au_demarrage := false
## Essais : reglages appliques a chaque depart, ex. {"suivi:vitesse_max": 55, "robot:vitesse_servo_deg_s": 200}.
##   "<module>:<reglage>" : reglage du controleur ; "<reglage>" seul : reglage du controleur lui-meme
##       (ex. "recul_fin_ligne_mm") ;
##   "robot:<cle de reglages_robot.json>" : valeur du robot, pour le modele ET le controleur (lue au lancement).
##   "robot:<champ de ReglagesRobot>" hors du fichier : reglage du controleur seulement (ex. "robot:rampe_montee_cmd_par_s",
##       "robot:rampe_freinage_cmd_par_s").
@export var reglages_essai: Dictionary = {}
## Banc d'essai : nombre de pistes aleatoires a enchainer automatiquement (0 = desactive). Resultats : journaux/banc_*.csv
@export var banc_essai_pistes := 0
@export var banc_graine_depart := 1
## Acceleration du temps (essais). ATTENTION : Engine.time_scale agrandit aussi le pas de la physique
## (4 -> pas de 1/60 s au lieu de 1/240 s) : les resultats changent un peu avec cette valeur.
@export var vitesse_temps := 1.0

## Valeurs du robot {cle JSON: valeur} : reglages_robot.json + cles "robot:" de reglages_essai.
var reglages_robot: Dictionary = {}
## Frequence de la boucle du programme du robot (reglages_robot.json) ; 0 = a chaque pas physique.
var frequence_controle_hz := 0.0

var vehicule: Vehicule
var controleur: Controleur
var pistes: Pistes
var bille: BilleCuvette
var camera: CameraSuivi
var hud: Hud
var entrees := Entrees.new()
var mesure := MesureAcceleration.new()
var journal := JournalCSV.new()
var _temps := 0.0
var _t_cadence := 0.0                 # boucle du robot : voir _physics_process
var _t_ecoule := 0.0
var banc := BancEssai.new()
var _quitter_a_la_fin := false        # --quitter : fermer Godot quand le banc est termine (essais sans editeur)
var _scenarios_demandes := ""         # --scenarios= : "tous" ou "nom1,nom2"
var _banc_actif := false
var _cas_a_charger := false           # banc : charger le cas suivant au debut du prochain pas physique
var _reglages_cas: Dictionary = {}    # reglages_essai propres au scenario en cours (ajoutes a reglages_essai)

func _ready() -> void:
	# Par defaut Godot fait au plus 8 pas physiques par image : a 240 pas/s, sous 30 images/s, le temps ralentit
	Engine.max_physics_steps_per_frame = 24
	add_child(Monde.new())
	pistes = Pistes.new(); add_child(pistes)
	if parametres_piste: pistes.parametres = parametres_piste
	_lire_ligne_de_commande()
	_lire_reglages_robot()
	vehicule = Vehicule.new(); vehicule.name = "Vehicule"; vehicule.regler(reglages_robot); add_child(vehicule)
	bille = BilleCuvette.new()
	bille.vehicule = vehicule; bille.regler(reglages_robot)
	add_child(bille)
	controleur = _controleur_regle()
	camera = CameraSuivi.new(); camera.cible = vehicule; add_child(camera)
	hud = Hud.new(); add_child(hud)
	_verifier_coherence()
	_brancher_signaux()
	Engine.time_scale = vitesse_temps
	if journal_au_demarrage:
		journal.basculer(null)
	if _scenarios_demandes != "":
		banc.scenarios = Scenarios.choisir(_scenarios_demandes)
	if banc_essai_pistes > 0 or not banc.scenarios.is_empty():
		banc.nombre_pistes = banc_essai_pistes
		banc.graine_depart = banc_graine_depart
		banc.demarrer()
		_banc_actif = true
		# Le 1er cas est charge comme les suivants, au debut d'un pas physique : chaque cas demarre pareil (rejouable)
		_cas_a_charger = true   # aucune piste provisoire : son replacer() de la bille decalerait le 1er cas
	elif _scenarios_demandes != "" and _quitter_a_la_fin:
		get_tree().quit.call_deferred()   # aucun scenario valide : rien a faire
	else:
		charger_piste(piste_initiale)

## Valeurs du robot : reglages_robot.json, puis les cles "robot:<cle>" de reglages_essai (essais sans toucher au fichier).
func _lire_reglages_robot() -> void:
	reglages_robot = FichierReglages.lire()
	for chemin in reglages_essai:
		var texte := String(chemin)
		if not texte.begins_with("robot:"):
			continue
		var cle := texte.trim_prefix("robot:")
		if reglages_robot.has(cle):
			reglages_robot[cle] = reglages_essai[chemin]
		# sinon : champ de ReglagesRobot hors du fichier (ex. robot:rampe_montee_cmd_par_s), applique par _controleur_regle
	frequence_controle_hz = float(reglages_robot.get("frequence_controle_hz", 0.0))

## Le fichier de reglages et le modele 3D (positions des pieces dans picar.glb) doivent decrire le meme robot :
## sinon les capteurs simules ne sont pas la ou le controleur les croit. On signale chaque ecart.
func _verifier_coherence() -> void:
	if not vehicule.has_method("geometrie_mm"):
		push_error("vehicule.gd n'est pas a jour (pas de servo lent, de cone ultrason ni de geometrie_mm) : remplacer le fichier.")
		return
	var r: ReglagesRobot = controleur.robot
	var g: Dictionary = vehicule.geometrie_mm()
	var comparaisons := [
		# [nom, valeur du fichier de reglages, valeur mesuree sur le modele 3D, tolerance]
		["distance_sondes_mm", r.distance_sondes_mm, g["distance_sondes_mm"], 5.0],
		["longueur_avant_mm", r.longueur_avant_mm, g["longueur_avant_mm"], 5.0],
		["hauteur_ultrason_mm", float(reglages_robot["hauteur_ultrason_mm"]), g["hauteur_ultrason_mm"], 5.0],
	]
	for c in comparaisons:
		if absf(c[1] - c[2]) > c[3]:
			push_warning("Reglages et modele 3D different : %s = %.2f dans reglages_robot.json, %.2f dans picar.glb." % [c[0], c[1], c[2]])

## Ex. : godot --path . -- --piste=6 --graine=1234  (pour rejouer exactement la meme piste)
## Banc sans editeur : godot --headless --path . --fixed-fps 240 -- --banc=40 --graine-depart=1 --quitter
##   --banc=N : N pistes aleatoires ; --graine-depart=S : premiere graine ; --quitter : fermer Godot a la fin du banc ;
##   --vitesse-temps=V : comme vitesse_temps (ATTENTION : Engine.time_scale agrandit aussi le pas de la physique) ;
##   --scenarios=tous ou --scenarios=nom1,nom2 : scenarios de simulation/scenarios.gd au lieu des pistes aleatoires ;
##   --reglage=<chemin>=<valeur> (repetable) : ajoute a reglages_essai, ex. --reglage=suivi:kp=5.4.
##   --parametre-piste=<nom>=<valeur> (repetable) : reglage de la piste aleatoire, ex. --parametre-piste=faces_aleatoires=false.
##   --journal : comme journal_au_demarrage (journal detaille de chaque piste).
func _lire_ligne_de_commande() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--piste="): piste_initiale = int(arg.get_slice("=", 1))
		elif arg.begins_with("--graine="): pistes.parametres.graine = int(arg.get_slice("=", 1))
		elif arg.begins_with("--banc="): banc_essai_pistes = int(arg.get_slice("=", 1))
		elif arg.begins_with("--graine-depart="): banc_graine_depart = int(arg.get_slice("=", 1))
		elif arg.begins_with("--vitesse-temps="): vitesse_temps = float(arg.get_slice("=", 1))
		elif arg.begins_with("--scenarios="): _scenarios_demandes = arg.get_slice("=", 1)
		elif arg.begins_with("--reglage="):   # --reglage=suivi:kp=5.4 : comme une entree de reglages_essai
			var texte := arg.trim_prefix("--reglage=")
			reglages_essai[texte.get_slice("=", 0)] = str_to_var(texte.get_slice("=", 1))
		elif arg.begins_with("--parametre-piste="):   # --parametre-piste=faces_aleatoires=false : ParametresPiste
			var texte := arg.trim_prefix("--parametre-piste=")
			var nom := texte.get_slice("=", 0)
			if nom in pistes.parametres:
				pistes.parametres.set(nom, str_to_var(texte.get_slice("=", 1)))
			else:
				push_error("--parametre-piste : « %s » n'existe pas dans ParametresPiste." % nom)
		elif arg == "--journal": journal_au_demarrage = true   # journal detaille journaux/essai_*.csv
		elif arg == "--quitter": _quitter_a_la_fin = true

func _brancher_signaux() -> void:
	entrees.piste_demandee.connect(func(i): _reglages_cas = {}; charger_piste(i))
	entrees.nouvelle_piste_aleatoire.connect(func(): _reglages_cas = {}; charger_piste(Pistes.INDEX_ALEATOIRE, randi_range(1, 999999)))
	entrees.remise_demandee.connect(remettre_au_depart)
	entrees.mode_bascule.connect(func(): mode_auto = not mode_auto)
	entrees.vue_bascule.connect(camera.basculer_vue)
	entrees.journal_bascule.connect(func(): journal.basculer(pistes.actuelle))
	bille.chute.connect(func(): print("BILLE TOMBEE a t = %.2f s (acceleration max atteinte %.2f m/s2)" % [_temps, mesure.maximum]))

## Les reglages propres a un scenario (_reglages_cas) sont remis a zero par qui charge une autre piste.
func charger_piste(i: int, graine := 0) -> void:
	pistes.place_avant_obstacle = _controleur_regle().place_avant_obstacle_mm() / 1000.0
	pistes.charger(i, graine)
	journal.nouvelle_piste(pistes.actuelle)   # sans effet si le journal est arrete
	remettre_au_depart()

## Banc d'essai : piste aleatoire suivante ou scenario suivant. Un scenario invalide est note et saute.
func _charger_cas_suivant() -> void:
	while not banc.termine:
		var cas = banc.prochain_cas()
		if cas is Dictionary:
			if charger_scenario(cas):
				banc.obstacles = pistes.actuelle.get_meta("obstacles", [])
				banc.fin_ligne = pistes.fin_de_ligne()
				return
			banc.noter_invalide()
		else:
			_reglages_cas = {}
			charger_piste(Pistes.INDEX_ALEATOIRE, cas)
			banc.obstacles = pistes.actuelle.get_meta("obstacles", [])
			banc.fin_ligne = pistes.fin_de_ligne()
			return
	if _quitter_a_la_fin:
		get_tree().quit.call_deferred()

## Scenario de simulation/scenarios.gd, avec ses reglages_essai pour ce cas seulement.
## La place avant les obstacles (« face » : "min") suit le controleur regle pour CE cas.
func charger_scenario(desc: Dictionary) -> bool:
	_reglages_cas = {}
	var r: Dictionary = desc.get("reglages_essai", {})
	for chemin in r:
		if String(chemin).begins_with("robot:"):
			push_error("Scenario %s : « %s » interdit dans un scenario (valeurs du robot lues au lancement seulement)." % [desc["nom"], chemin])
		else:
			_reglages_cas[chemin] = r[chemin]
	if desc.has("piste"):   # piste Blender (Pistes.LISTE), avec les reglages du scenario
		charger_piste(int(desc["piste"]))
		return true
	pistes.place_avant_obstacle = _controleur_regle().place_avant_obstacle_mm() / 1000.0
	if not pistes.charger_scenario(desc):
		return false
	journal.nouvelle_piste(pistes.actuelle)
	remettre_au_depart()
	return true

func remettre_au_depart() -> void:
	vehicule.placer(pistes.transform_depart())
	vehicule.commander(0, 90)
	entrees.reinitialiser()
	controleur = _controleur_regle()
	_regler_mode_piste()
	_temps = 0.0
	_t_cadence = 0.0; _t_ecoule = 0.0   # chaque essai commence de la meme facon (essais rejouables)
	mesure.reinitialiser()
	bille.replacer()

## Mode de demonstration selon la piste de test : seuls des reglages changent, pas la logique du robot.
##   « Arret devant obstacle » : arret a distance predeterminee, sans contournement.
##   « Recul » : a la fin de la ligne, arret puis recul de 180 mm en suivant la ligne (exigence : moins de 200 mm).
func _regler_mode_piste() -> void:
	match Pistes.LISTE[pistes.index][0]:
		"Arret devant obstacle":
			controleur.contournement.actif = false
		"Recul":
			controleur.recul_fin_ligne_mm = 180.0

## Nouveau controleur : valeurs du robot (reglages_robot.json), puis reglages de l'essai (reglages_essai).
## Les modules gardent la meme reference a controleur.robot : appliquer() la remplit sur place.
func _controleur_regle() -> Controleur:
	var c := Controleur.new()
	c.robot.appliquer(reglages_robot)
	var essai := reglages_essai.duplicate()
	essai.merge(_reglages_cas, true)
	for chemin in essai:
		if String(chemin).begins_with("robot:") and reglages_robot.has(String(chemin).trim_prefix("robot:")):
			continue   # cle de reglages_robot.json : deja dans reglages_robot
		if c.get_indexed(NodePath(chemin)) == null:
			push_error("reglages_essai : « %s » ne correspond a aucun reglage du controleur." % chemin)
			continue
		c.set_indexed(NodePath(chemin), essai[chemin])
	return c

func _physics_process(dt: float) -> void:
	if _cas_a_charger:
		_cas_a_charger = false
		_charger_cas_suivant()
		return
	_temps += dt
	entrees.lire(Pistes.LISTE.size())
	# Le programme du robot ne tourne qu'a frequence_controle_hz : entre deux tours, la commande reste la meme
	# _t_cadence : decide quand il tourne (on retire la periode : 50 Hz en moyenne, 4 ou 5 pas de 1/240 s par tour) ;
	# _t_ecoule : temps reellement ecoule depuis le tour precedent, passe au controleur (odometrie, PID).
	_t_cadence += dt
	_t_ecoule += dt
	if frequence_controle_hz <= 0.0 or _t_cadence >= 1.0 / frequence_controle_hz - 1e-6:
		_conduire(_t_ecoule)
		_t_ecoule = 0.0
		_t_cadence = 0.0 if frequence_controle_hz <= 0.0 else _t_cadence - 1.0 / frequence_controle_hz
	mesure.ajouter(vehicule.vitesse, dt)
	bille.mettre_a_jour(dt)   # appele ici (et non dans la bille) pour garder l'ordre : commande -> mesure -> bille
	journal.ecrire(_temps, vehicule, bille.tombee, controleur, bille.taux_bord())
	if _banc_actif and banc.suivre(_temps, dt, vehicule, bille, controleur, _piste_generee()):
		_cas_a_charger = true

## Piste generee (aleatoire ou scenario dessine) ; null pour une piste Blender (le generateur garde la derniere
## piste aleatoire, qui ne correspond pas a la piste chargee).
func _piste_generee() -> GenerateurPiste:
	return pistes.generateur if pistes.index == Pistes.INDEX_ALEATOIRE else null

## Qui conduit : personne tant que la bille n'est pas prete, sinon le controleur ou le clavier.
func _conduire(dt: float) -> void:
	if not bille.est_prete():
		vehicule.commander(0, 90)
	elif mode_auto:
		var sortie: Dictionary = controleur.calculer(vehicule.sondes.duplicate(), vehicule.distance_cm, dt)
		vehicule.commander(int(sortie.get("vitesse", 0)), int(sortie.get("angle", 90)))
	else:
		vehicule.commander(entrees.cmd_vitesse, entrees.cmd_angle)

func _process(_dt: float) -> void:
	hud.afficher(pistes.nom(), mode_auto, vehicule, mesure, bille.tombee, journal.actif(), controleur.nom_etat())
