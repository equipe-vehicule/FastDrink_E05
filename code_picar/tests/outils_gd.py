"""Outils des tests : lecture des réglages déclarés dans les .gd de la simulation, liste des traces de référence."""
import glob
import os
import re

from fastdrink.conduite.reglages import MODULES, RACINE_DEPOT

DOSSIER_SIMULATION = os.path.join(RACINE_DEPOT, "simulation")
DOSSIER_TRACES = os.path.join(os.path.dirname(__file__), "traces")
TYPES_GD = {"float": float, "int": int, "bool": bool, "String": str, "Array": list}
_EXPORT = re.compile(r"^@export var (\w+)\s*:\s*(\w+)", re.M)


def schema_gd(fichier):
    """{réglage: type Python} des « @export var nom: type » d'un fichier de simulation/ (ordre du fichier)."""
    with open(os.path.join(DOSSIER_SIMULATION, fichier), encoding="utf-8") as f:
        texte = f.read()
    return {nom: TYPES_GD[t] for nom, t in _EXPORT.findall(texte)}


def schemas_gd():
    """{module: schéma} de tous les objets du Contrôleur qui portent des réglages (MODULES)."""
    return {nom: schema_gd(fichier) for nom, (fichier, _, _) in MODULES.items()}


def traces_reference():
    return sorted(glob.glob(os.path.join(DOSSIER_TRACES, "*.jsonl.gz")))
