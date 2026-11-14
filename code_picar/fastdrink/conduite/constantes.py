"""Constantes et énumérations partagées entre lots de portage (PORTAGE.md) : une constante utilisée par des modules de lots différents
est écrite ici, une seule fois, pour que les lots restent des fichiers disjoints. Le module d'origine la reprend
(ex. SuiviLigne.ERREUR_MAX_MM = constantes.ERREUR_MAX_MM).
"""
import enum

## suivi_ligne.gd : ecart maximal mesurable avec la ligne visible (mm) = position de la sonde extreme.
## Utilise aussi par contournement.gd, contournement_fluide.gd et demi_tour.gd (SuiviLigne.ERREUR_MAX_MM).
ERREUR_MAX_MM = 38.0

# ----- Enumerations utilisees par le Controleur (lot C) et definies dans les lots A et B : memes membres, meme ordre
# que les enum des .gd. Les classes les reprennent comme attributs (ex. SuiviLigne.Ligne = constantes.Ligne).

## suivi_ligne.gd : enum Ligne
Ligne = enum.IntEnum("Ligne", "VUE ENTRE_SONDES PERDUE", start=0)
## recul_ligne.gd : enum Fin
Fin = enum.IntEnum("Fin", "EN_COURS TERMINE LIGNE_PERDUE", start=0)
## rattrapage.gd : enum Reaction
Reaction = enum.IntEnum("Reaction", "AUCUNE RATTRAPER ARRETER", start=0)
## contournement.gd : enum Phase
Phase = enum.IntEnum("Phase", "ECART LONGER RETOUR RECHERCHE CROISEMENT ALIGNEMENT ORBITE SORTIE_ORBITE TRAVERS", start=0)
