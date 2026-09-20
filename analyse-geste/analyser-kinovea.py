# -*- coding: utf-8 -*-
"""Lit l'analyse Kinovea du geste humain et en tire des chiffres exploitables.

    python analyser-kinovea.py

Ce que contient le fichier `.kva` produit par Kinovea : deux trajectoires du
médiator, suivies image par image sur une vidéo au ralenti, et une ligne
d'étalonnage qui donne l'échelle (l'écart entre deux cordes, onze millimètres).

Ce que ce script en fait :

  - convertit les points de pixels en millimètres et les horodatages en
    secondes ;
  - trace la trajectoire mesurée, sa vitesse et son accélération ;
  - en sort les chiffres qui contraignent le mécanisme : amplitude du geste,
    vitesse et accélération maximales, durée d'une attaque.

À quoi cela sert : on ne dimensionne pas un robot guitariste sur une intuition.
Le mécanisme doit reproduire ce geste-là. Ces chiffres sont la cible, et les
courbes de `../simulation/` sont ce que le mécanisme sait faire.

Sorties, dans `sorties/` :
    geste-mesure.png            la trajectoire du médiator, en millimètres
    geste-vitesse.png           vitesse et accélération au cours du geste
    mesures.json                les chiffres, pour la fiche du portfolio

Dépendances : numpy, matplotlib.
"""
import json
import os
import re
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

ICI = os.path.dirname(os.path.abspath(__file__))
ANALYSE = os.path.join(ICI, "..", "documents", "zone de travail 2.kva")
SORTIES = os.path.join(ICI, "sorties")
os.makedirs(SORTIES, exist_ok=True)


def lire_couple(texte):
    a, b = texte.split(";")
    return float(a), float(b)


arbre = ET.parse(ANALYSE)
racine = arbre.getroot()


def valeur(chemin, defaut=None):
    noeud = racine.find(chemin)
    return noeud.text if noeud is not None else defaut


# ── L'échelle : la ligne d'étalonnage vaut une longueur connue ─────────────────
longueur_reelle = float(valeur("Calibration/CalibrationLine/Length"))
a = lire_couple(valeur("Calibration/CalibrationLine/Segment/A"))
b = lire_couple(valeur("Calibration/CalibrationLine/Segment/B"))
longueur_pixels = float(np.hypot(b[0] - a[0], b[1] - a[1]))
unite = racine.find("Calibration/Unit").get("Abbreviation", "mm")
echelle = longueur_reelle / longueur_pixels          # unités par pixel

# ── Le temps : Kinovea compte en horodatages internes ─────────────────────────
par_image = float(valeur("AverageTimeStampsPerFrame"))
images_par_seconde = float(valeur("UserFramerate") or valeur("CaptureFramerate"))
par_seconde = par_image * images_par_seconde

print(f"Étalonnage : {longueur_pixels:.1f} px valent {longueur_reelle} {unite} "
      f"({echelle * 1000:.1f} µm par pixel)")
print(f"Cadence : {images_par_seconde:.0f} images par seconde")

# ── Les trajectoires suivies ──────────────────────────────────────────────────
trajectoires = []
for piste in racine.iter("Track"):
    points = [lire_couple(p.text.rsplit(";", 1)[0]) for p in piste.iter("TrackPoint")]
    instants = [float(p.text.rsplit(";", 1)[1]) for p in piste.iter("TrackPoint")]
    if len(points) < 10:
        continue
    xy = np.array(points) * echelle
    xy -= xy[0]                                       # origine au premier point
    xy[:, 1] *= -1                                    # l'image descend, le monde monte
    t = (np.array(instants) - instants[0]) / par_seconde
    trajectoires.append({"nom": piste.get("name", "trajectoire"), "xy": xy, "t": t})

if not trajectoires:
    raise SystemExit("aucune trajectoire suivie dans le fichier Kinovea")

# ── Les chiffres ──────────────────────────────────────────────────────────────
mesures = {"etalonnage": {"pixels": round(longueur_pixels, 1), "reel": longueur_reelle,
                          "unite": unite, "images_par_seconde": images_par_seconde},
           "trajectoires": []}

for trajectoire in trajectoires:
    xy, t = trajectoire["xy"], trajectoire["t"]
    pas = np.diff(t)
    pas[pas == 0] = np.nan                            # deux points sur la même image
    deplacement = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    vitesse = np.concatenate([[0.0], deplacement / pas])
    vitesse = np.nan_to_num(vitesse)
    acceleration = np.gradient(vitesse, t, edge_order=1)
    trajectoire["vitesse"] = vitesse
    trajectoire["acceleration"] = acceleration
    mesures["trajectoires"].append({
        "nom": trajectoire["nom"],
        "points": int(len(t)),
        "duree_s": round(float(t[-1]), 3),
        "amplitude_mm": round(float(np.ptp(xy, axis=0).max()), 1),
        "chemin_mm": round(float(deplacement.sum()), 1),
        "vitesse_max_mm_s": round(float(vitesse.max()), 0),
        "vitesse_moyenne_mm_s": round(float(deplacement.sum() / t[-1]), 0),
        "acceleration_max_mm_s2": round(float(np.abs(acceleration).max()), 0),
    })
    print(f"  {trajectoire['nom']:16s} {len(t):4d} points, {t[-1]:5.2f} s, "
          f"amplitude {np.ptp(xy, axis=0).max():5.1f} {unite}, "
          f"vitesse max {vitesse.max():6.0f} {unite}/s")

json.dump(mesures, open(os.path.join(SORTIES, "mesures.json"), "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)

# ── La trajectoire mesurée ────────────────────────────────────────────────────
figure, axe = plt.subplots(figsize=(7, 5.5))
for rang, trajectoire in enumerate(trajectoires):
    xy = trajectoire["xy"]
    couleur = ["tab:red", "tab:blue", "tab:green"][rang % 3]
    axe.plot(xy[:, 0], xy[:, 1], "-", color=couleur, lw=1.6, label=trajectoire["nom"])
    axe.plot(xy[0, 0], xy[0, 1], "o", color=couleur, ms=7)
    axe.plot(xy[-1, 0], xy[-1, 1], "s", color=couleur, ms=7)
axe.set_xlabel(f"déplacement horizontal ({unite})")
axe.set_ylabel(f"déplacement vertical ({unite})")
axe.set_title("Le geste du médiator, relevé image par image")
axe.grid(alpha=0.3)
axe.axis("equal")
axe.legend()
figure.tight_layout()
figure.savefig(os.path.join(SORTIES, "geste-mesure.png"), dpi=140)
plt.close(figure)

# ── Vitesse et accélération du geste ──────────────────────────────────────────
figure, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
for rang, trajectoire in enumerate(trajectoires):
    couleur = ["tab:red", "tab:blue", "tab:green"][rang % 3]
    axes[0].plot(trajectoire["t"], trajectoire["vitesse"], color=couleur, lw=1.4,
                 label=trajectoire["nom"])
    axes[1].plot(trajectoire["t"], trajectoire["acceleration"] / 1000, color=couleur, lw=1.2)
axes[0].set_ylabel(f"vitesse ({unite}/s)")
axes[0].set_title("Ce que le mécanisme doit reproduire")
axes[0].grid(alpha=0.3)
axes[0].legend()
axes[1].set_ylabel(f"accélération ({unite}/s² × 1000)")
axes[1].set_xlabel("temps (s)")
axes[1].grid(alpha=0.3)
figure.tight_layout()
figure.savefig(os.path.join(SORTIES, "geste-vitesse.png"), dpi=140)
plt.close(figure)

print("Images et mesures écrites dans", SORTIES)
