# -*- coding: utf-8 -*-
"""Produit les figures de la simulation, depuis le modèle du projet.

    python produire-figures.py

Sorties, dans `sorties/` :

    coupe-mediator.png        le mécanisme dans le plan de coupe, avec la corde
    chemin-pointe.png         le chemin de la pointe sur une période, et le contact
    inclinaison-selon-d.png   l'inclinaison totale en fonction de la translation

── Pourquoi ce fichier a été réécrit ────────────────────────────────────────

Il produisait trois images tirées d'un modèle abrégé, à deux corps, en unités
abstraites : un bras d'un mètre vingt de balayage, des axes gradués en mètres.
Le robot, lui, balaie trente-trois millimètres. Les figures étaient donc
fausses, et elles étaient publiées sur le portfolio et déposées par la chaîne
d'intégration comme si elles décrivaient le mécanisme.

Elles viennent maintenant du seul modèle qui fait foi, `essais/test
python32.py` : la potence, la translation, l'inclinaison qui bascule au signe
de la vitesse, et le médiator qui s'efface jusqu'à rester tangent au disque de
la corde. Rien n'est recopié ici, tout est importé de là-bas : deux modèles ne
peuvent pas se contredire s'il n'y en a qu'un.

Dépendances : numpy, matplotlib.
"""
import importlib.util
import os
import sys

import matplotlib
matplotlib.use("Agg")                 # aucune fenêtre : la machine peut être sans écran

import matplotlib.pyplot as plt
import numpy as np

ICI = os.path.dirname(os.path.abspath(__file__))
SORTIES = os.path.join(ICI, "sorties")


def charger_modele():
    """Importe `essais/test python32.py`, dont le nom porte une espace.

    Un import ordinaire ne sait pas lire ce nom de fichier. On passe donc par
    `importlib`, plutôt que de renommer un fichier qui est cité tel quel dans
    le rapport et dans le dépôt.
    """
    chemin = os.path.join(ICI, "essais", "test python32.py")
    if not os.path.exists(chemin):
        raise SystemExit(f"modèle introuvable : {chemin}")
    specification = importlib.util.spec_from_file_location("modele_mediator", chemin)
    module = importlib.util.module_from_spec(specification)
    # Le script trace une animation quand on l'exécute : on ne veut que ses
    # fonctions, alors on le charge sans lui laisser atteindre son `__main__`.
    module.__name__ = "modele_mediator"
    sys.modules["modele_mediator"] = module
    specification.loader.exec_module(module)
    return module


def corde(m):
    """La position de la corde, calculée comme dans le script d'origine."""
    zmin = -(m.C + m.D) * np.cos(m.theta0)
    zmax = m.d1_0 + (m.C + m.D) * np.cos(m.theta0)
    return (zmax + zmin) / 2.0, m.A - m.B - (m.C + m.D) * np.cos(m.theta0)


def poses(m, points=600):
    """Une période entière, échantillonnée, par les fonctions du modèle."""
    t = np.linspace(0.0, m.T, points)
    d = m.d_periodique(t, m.T, m.d1_0, m.delta)
    theta2, _ = m.theta2_en_fonction_de_d(d, m.theta0)
    Zc, Xc = corde(m)
    frames, theta3, theta_total, distances = m.compute_frames_coupled(t, theta2, d, Zc, Xc, m.dc)
    return t, d, theta2, theta3, theta_total, distances, frames


def figure_coupe(m, frames, chemin):
    """Le mécanisme à un instant, dans le plan où la corde est un cercle."""
    pose = frames[len(frames) // 5]
    points = [pose["p0"], pose["pA"], pose["p1"], pose["p2"], pose["p3"], pose["p4"]]
    # Dans le script d'origine, un point est (x, y, z) : le plan de coupe se
    # lit donc en (z, x), et pas l'inverse.
    z = [p[2] for p in points]
    x = [p[0] for p in points]

    figure, axes = plt.subplots(figsize=(7.2, 5.4))
    axes.plot(z[:2], x[:2], color="#64748b", linewidth=5, label="potence")
    axes.plot(z[1:4], x[1:4], color="#0ea5e9", linewidth=4, label="translation et chariot")
    axes.plot(z[3:6], x[3:6], color="#f97316", linewidth=4, label="bras et médiator")
    Zc, Xc = corde(m)
    axes.add_patch(plt.Circle((Zc, Xc), m.dc / 2.0, color="#94a3b8"))
    axes.set_aspect("equal")
    axes.grid(alpha=0.3)
    axes.set_xlabel("Z (mm)")
    axes.set_ylabel("X (mm)")
    axes.set_title("Le mécanisme dans le plan de coupe, et la corde")
    axes.legend(loc="upper right", fontsize=9)
    figure.tight_layout()
    figure.savefig(chemin, dpi=140)
    plt.close(figure)


def figure_chemin(m, frames, distances, chemin):
    """Le chemin de la pointe sur une période, contacts marqués."""
    z = np.array([f["p4"][2] for f in frames])
    x = np.array([f["p4"][0] for f in frames])
    # Le contact se lit sur l'effacement, pas sur la distance finale : le
    # solveur fait pivoter la lame JUSQU'À ce qu'elle ne touche plus, si bien
    # qu'à l'arrivée la distance est toujours supérieure au rayon. Un theta3
    # non nul, en revanche, ne se produit que si la corde a poussé la lame.
    touche = np.abs(np.array([f["theta3"] for f in frames])) > 1e-9

    figure, axes = plt.subplots(figsize=(7.6, 3.4))
    axes.plot(z, x, color="#38bdf8", linewidth=2, label="chemin de la pointe")
    if touche.any():
        axes.plot(z[touche], x[touche], "o", color="#f97316", markersize=3, label="contact")
    Zc, Xc = corde(m)
    axes.add_patch(plt.Circle((Zc, Xc), m.dc / 2.0, color="#94a3b8", alpha=0.7))
    axes.set_aspect("equal")
    axes.grid(alpha=0.3)
    axes.set_xlabel("Z (mm)")
    axes.set_ylabel("X (mm)")
    axes.set_title("Le chemin de la pointe sur une période, et le contact")
    axes.legend(loc="lower left", fontsize=9, framealpha=0.9)
    figure.tight_layout()
    figure.savefig(chemin, dpi=140)
    plt.close(figure)


def figure_inclinaison(d, theta_total, chemin):
    """L'inclinaison totale en fonction de la translation : les deux branches."""
    figure, axes = plt.subplots(figsize=(7.2, 4.2))
    axes.plot(d, np.degrees(theta_total), color="#a78bfa", linewidth=2)
    axes.grid(alpha=0.3)
    axes.set_xlabel("translation d (mm)")
    axes.set_ylabel("inclinaison totale (°)")
    axes.set_title("L'inclinaison bascule en fin de course, et décroche au contact")
    figure.tight_layout()
    figure.savefig(chemin, dpi=140)
    plt.close(figure)


def main():
    os.makedirs(SORTIES, exist_ok=True)
    modele = charger_modele()
    _, d, _, _, theta_total, distances, frames = poses(modele)

    figure_coupe(modele, frames, os.path.join(SORTIES, "coupe-mediator.png"))
    figure_chemin(modele, frames, distances, os.path.join(SORTIES, "chemin-pointe.png"))
    figure_inclinaison(d, theta_total, os.path.join(SORTIES, "inclinaison-selon-d.png"))

    print("figures écrites dans", SORTIES)
    for nom in sorted(os.listdir(SORTIES)):
        print("   ", nom)


if __name__ == "__main__":
    main()
