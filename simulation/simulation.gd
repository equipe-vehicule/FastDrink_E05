## Scene de simulation du PiCar-S (equipe 05).
## Touches : 1-7 choix de piste (7 = aleatoire) | G nouvelle piste aleatoire | R remettre au depart
##           M manuel/automatique | Fleches : conduite manuelle | Espace : arret | C : vue | L : journal CSV on/off
## Ligne de commande (apres --) : --piste=6  --graine=1234
##
## Ce script ne fait qu'assembler les modules et decider qui conduit la voiture :
##   Monde, Pistes, Vehicule, BilleCuvette, CameraSuivi, Hud, Entrees, MesureAcceleration, JournalCSV
extends Node3D

@export var masse_bille := 0.010      ## kg. PROVISOIRE : a peser (n'influence pas la chute, utile pour la suite).
@export var amortissement_bille := 1.0 ## 1/s, pertes par roulement. PROVISOIRE : a comparer avec le vrai robot.
@export var piste_initiale := 1        ## index dans Pistes.LISTE (1 = courbes, 6 = aleatoire)
@export var mode_auto := true
@export var parametres_piste: ParametresPiste ## reglages de la piste aleatoire (vide = valeurs par defaut)
## Frequence de la boucle du programme sur le vrai robot (lecture capteurs -> calcul -> commande).
## Entre deux tours, la commande precedente reste appliquee. PROVISOIRE : a mesurer sur le Pi ; 0 = a chaque pas physique.
@export var frequence_controle_hz := 50.0
## Demarre le journal CSV des le lancement (essais automatiques, sans appuyer sur L).
@export var journal_au_demarrage := false
## Essais : reglages du controleur appliques a chaque depart, ex. {"contournement_fluide": true, "suivi:vitesse_max": 55}.
@export var reglages_essai: Dictionary = {}
## Banc d'essai : nombre de pistes aleatoires a enchainer automatiquement (0 = desactive). Resultats : journaux/banc_*.csv
@export var banc_essai_pistes := 0
@export var banc_graine_depart := 1
## Acceleration du temps (essais) : les calculs restent identiques, seul l'affichage va plus vite.
@export var vitesse_temps := 1.0

var vehicule: Vehicule
var controleur := Controleur.new()
var pistes: Pistes
var bille: BilleCuvette
var camera: CameraSuivi
var hud: Hud
var entrees := Entrees.new()
var mesure := MesureAcceleration.new()
var journal := JournalCSV.new()
var _temps := 0.0
var _t_controle := 0.0
var banc := BancEssai.new()

func _ready() -> void:
	# Par defaut Godot fait au plus 8 pas physiques par image : a 240 pas/s, sous 30 images/s, le temps ralentit
	Engine.max_physics_steps_per_frame = 24
	add_child(Monde.new())
	pistes = Pistes.new(); add_child(pistes)
	if parametres_piste: pistes.parametres = parametres_piste
	_lire_ligne_de_commande()
	vehicule = Vehicule.new(); vehicule.name = "Vehicule"; add_child(vehicule)
	bille = BilleCuvette.new()
	bille.vehicule = vehicule; bille.masse = masse_bille; bille.amortissement = amortissement_bille
	add_child(bille)
	camera = CameraSuivi.new(); camera.cible = vehicule; add_child(camera)
	hud = Hud.new(); add_child(hud)
	_verifier_coherence()
	_brancher_signaux()
	Engine.time_scale = vitesse_temps
	if journal_au_demarrage:
		journal.basculer(null)
	if banc_essai_pistes > 0:
		banc.nombre_pistes = banc_essai_pistes
		banc.graine_depart = banc_graine_depart
		banc.demarrer()
		charger_piste(Pistes.INDEX_ALEATOIRE, banc.prochaine_graine())
	else:
		charger_piste(piste_initiale)

## Le controleur (ReglagesRobot) et le modele (Vehicule) doivent decrire le meme robot :
## sinon l'odometrie du contournement calcule avec de mauvaises dimensions. On signale chaque ecart.
func _verifier_coherence() -> void:
	if not vehicule.has_method("geometrie_mm"):
		push_error("vehicule.gd n'est pas a jour (pas de servo lent, de cone ultrason ni de geometrie_mm) : remplacer le fichier.")
		return
	var r: ReglagesRobot = controleur.robot
	var g: Dictionary = vehicule.geometrie_mm()
	var comparaisons := [
		# [nom, valeur du controleur, valeur du modele, tolerance]
		["empattement_mm", r.empattement_mm, g["empattement_mm"], 2.0],
		["distance_sondes_mm", r.distance_sondes_mm, g["distance_sondes_mm"], 5.0],
		["longueur_avant_mm", r.longueur_avant_mm, g["longueur_avant_mm"], 5.0],
		["angle_roues_max_deg", r.angle_roues_max_deg, vehicule.angle_roues_max_deg, 0.5],
		["vitesse_servo_deg_s", r.vitesse_servo_deg_s, vehicule.vitesse_servo_deg_s, 1.0],
		["demi_angle_ultrason_deg", r.demi_angle_ultrason_deg, vehicule.demi_angle_ultrason_deg, 0.1],
		["mm_s_par_cmd", r.mm_s_par_cmd, vehicule.vitesse_max * 10.0, 0.05],
	]
	for c in comparaisons:
		if absf(c[1] - c[2]) > c[3]:
			push_warning("Controleur et Vehicule different : %s = %.2f dans ReglagesRobot, %.2f dans le modele." % [c[0], c[1], c[2]])

## Ex. : godot --path . -- --piste=6 --graine=1234  (pour rejouer exactement la meme piste)
func _lire_ligne_de_commande() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--piste="): piste_initiale = int(arg.get_slice("=", 1))
		elif arg.begins_with("--graine="): pistes.parametres.graine = int(arg.get_slice("=", 1))

func _brancher_signaux() -> void:
	entrees.piste_demandee.connect(charger_piste)
	entrees.nouvelle_piste_aleatoire.connect(func(): charger_piste(Pistes.INDEX_ALEATOIRE, randi_range(1, 999999)))
	entrees.remise_demandee.connect(remettre_au_depart)
	entrees.mode_bascule.connect(func(): mode_auto = not mode_auto)
	entrees.vue_bascule.connect(camera.basculer_vue)
	entrees.journal_bascule.connect(func(): journal.basculer(pistes.actuelle))
	bille.chute.connect(func(): print("BILLE TOMBEE a t = %.2f s (acceleration max atteinte %.2f m/s2)" % [_temps, mesure.maximum]))

func charger_piste(i: int, graine := 0) -> void:
	pistes.place_avant_obstacle = _controleur_regle().place_avant_obstacle_mm() / 1000.0
	pistes.charger(i, graine)
	journal.nouvelle_piste(pistes.actuelle)   # sans effet si le journal est arrete
	remettre_au_depart()

func remettre_au_depart() -> void:
	vehicule.placer(pistes.transform_depart())
	vehicule.commander(0, 90)
	entrees.reinitialiser()
	controleur = _controleur_regle()
	_regler_mode_piste()
	_temps = 0.0
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

## Nouveau controleur avec les reglages de l'essai (reglages_essai).
func _controleur_regle() -> Controleur:
	var c := Controleur.new()
	for chemin in reglages_essai:
		c.set_indexed(NodePath(chemin), reglages_essai[chemin])
	return c

func _physics_process(dt: float) -> void:
	_temps += dt
	entrees.lire(Pistes.LISTE.size())
	# Le programme du robot ne tourne qu'a frequence_controle_hz : entre deux tours, la commande reste la meme
	_t_controle += dt
	if frequence_controle_hz <= 0.0 or _t_controle >= 1.0 / frequence_controle_hz - 1e-6:
		_conduire(_t_controle)
		_t_controle = 0.0
	mesure.ajouter(vehicule.vitesse, dt)
	bille.mettre_a_jour(dt)   # appele ici (et non dans la bille) pour garder l'ordre : commande -> mesure -> bille
	journal.ecrire(_temps, vehicule, bille.tombee, controleur, bille.taux_bord())
	if banc_essai_pistes > 0 and banc.suivre(_temps, vehicule, bille, controleur, pistes.generateur.obstacles.size()):
		if not banc.termine:
			charger_piste(Pistes.INDEX_ALEATOIRE, banc.prochaine_graine())

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
