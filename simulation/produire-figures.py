# -*- coding: utf-8 -*-
"""Produit les images de la simulation cinématique, sans ouvrir de fenêtre.

    python produire-figures.py

Sorties, dans `sorties/` :

    animation-bras.gif          le bras en mouvement, avec theta(t) et d(t)
    trajectoire-outil.png       le chemin parcouru par l'outil dans le plan x-z
    vitesses-accelerations.png  position, vitesse et accélération des deux axes

Pourquoi ces trois-là : le projet n'a pas de site à montrer. Ce qu'un recruteur
peut juger, c'est la modélisation, et elle se juge sur ces courbes. Les vitesses
et les accélérations sont dérivées des mêmes trajectoires par différences
finies, comme dans l'étude d'origine : ce sont elles qui disent si un mouvement
est réalisable par le mécanisme, ou s'il demande au moteur plus qu'il ne peut.

Dépendances : numpy, matplotlib, pillow.
"""
import os

import matplotlib
matplotlib.use("Agg")                 # aucune fenêtre : la machine peut être sans écran

import matplotlib.pyplot as plt
import numpy as np

from matplotlib import animation

from animation_cinematique import cubic_smooth_traj, forward_kinematics


def parcours(theta, d):
    """Les trois points du mécanisme à chaque instant : la base, l'origine du
    bras et l'outil. `forward_kinematics` travaille sur un instant ; on la
    déroule sur toute la trajectoire, sans réécrire la cinématique."""
    calcules = [forward_kinematics(th, dd, A=A, B=B)[:3] for th, dd in zip(theta, d)]
    return (np.array([c[0] for c in calcules]),
            np.array([c[1] for c in calcules]),
            np.array([c[2] for c in calcules]))

SORTIES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sorties")
os.makedirs(SORTIES, exist_ok=True)

# Les deux paramètres géométriques du bras, en mètres : le décalage du prisme
# et la longueur de la dernière liaison.
A, B = 1.0, 0.6
DUREE = 8.0
ECHANTILLONS = 400
t = np.linspace(0.0, DUREE, ECHANTILLONS)

# Trois mouvements représentatifs : un balayage complet, un aller-retour
# entretenu, et un déplacement adouci aux extrémités.
theta_lineaire = np.linspace(0.0, np.pi, ECHANTILLONS)
theta_sinus = np.pi / 2 + 0.8 * np.sin(2 * np.pi * t / DUREE)
theta_adouci = cubic_smooth_traj(t, np.pi, 0.0)

d_lineaire = np.linspace(0.1, 0.5, ECHANTILLONS)
d_sinus = 0.25 + 0.05 * np.cos(4 * np.pi * t / DUREE)
d_adouci = 0.4 + 0.15 * np.sin(1.5 * 2 * np.pi * t / DUREE)

trajectoires = [
    {"theta": theta_lineaire, "d": d_lineaire, "label": "balayage linéaire", "color": "tab:blue"},
    {"theta": theta_sinus, "d": d_sinus, "label": "aller-retour entretenu", "color": "tab:green"},
    {"theta": theta_adouci, "d": d_adouci, "label": "départ et arrivée adoucis", "color": "tab:orange"},
]

# ── 1. L'animation ────────────────────────────────────────────────────────────
# Le bras est redessiné image par image à partir de la cinématique directe du
# module de référence : les trois points p0, p1 et p3 du mécanisme, plus la
# trace laissée par l'outil.
figure = plt.figure(figsize=(11, 5.5))
grille = figure.add_gridspec(2, 3, width_ratios=[2, 1, 1], height_ratios=[2, 1])
vue = figure.add_subplot(grille[:, 0])
axe_theta = figure.add_subplot(grille[0, 1:])
axe_d = figure.add_subplot(grille[1, 1:])

points = [parcours(tr["theta"], tr["d"]) for tr in trajectoires]
bras, traces, reperes_theta, reperes_d = [], [], [], []
for trajectoire, (p0, p1, p3) in zip(trajectoires, points):
    couleur = trajectoire["color"]
    bras.append(vue.plot([], [], "-o", color=couleur, lw=3, ms=7)[0])
    traces.append(vue.plot([], [], "-", color=couleur, lw=1, alpha=0.45)[0])
    axe_theta.plot(t, trajectoire["theta"], color=couleur, lw=1.4, label=trajectoire["label"])
    axe_d.plot(t, trajectoire["d"], color=couleur, lw=1.4)
    reperes_theta.append(axe_theta.plot([], [], "o", color=couleur, ms=5)[0])
    reperes_d.append(axe_d.plot([], [], "o", color=couleur, ms=5)[0])

tous = np.concatenate([np.stack([p3[:, 0], p3[:, 2]], axis=1) for _, _, p3 in points])
marge = 0.25
vue.set_xlim(min(0, tous[:, 0].min()) - marge, tous[:, 0].max() + marge)
vue.set_ylim(tous[:, 1].min() - marge, tous[:, 1].max() + marge)
vue.set_aspect("equal")
vue.grid(alpha=0.3)
vue.set_title("Le bras dans le plan de la corde")
vue.set_xlabel("x (m)")
vue.set_ylabel("z (m)")
axe_theta.set_ylabel("theta (rad)")
axe_theta.grid(alpha=0.3)
axe_theta.legend(fontsize=8)
axe_d.set_ylabel("d (m)")
axe_d.set_xlabel("temps (s)")
axe_d.grid(alpha=0.3)
chrono = vue.text(0.02, 0.96, "", transform=vue.transAxes, va="top", fontsize=9)

TRACE = 60          # longueur de la trace laissée derrière l'outil, en images


def dessiner(image):
    for rang, (p0, p1, p3) in enumerate(points):
        bras[rang].set_data([p0[image, 0], p1[image, 0], p3[image, 0]],
                            [p0[image, 2], p1[image, 2], p3[image, 2]])
        debut = max(0, image - TRACE)
        traces[rang].set_data(p3[debut:image + 1, 0], p3[debut:image + 1, 2])
        reperes_theta[rang].set_data([t[image]], [trajectoires[rang]["theta"][image]])
        reperes_d[rang].set_data([t[image]], [trajectoires[rang]["d"][image]])
    chrono.set_text(f"t = {t[image]:4.1f} s")
    return bras + traces + reperes_theta + reperes_d + [chrono]


figure.tight_layout()
# Une image sur quatre et une résolution modeste : le fichier est publié sur
# le portfolio, et un GIF de plusieurs mégaoctets se télécharge mal.
film = animation.FuncAnimation(figure, dessiner, frames=range(0, ECHANTILLONS, 4), blit=False)
gif = os.path.join(SORTIES, "animation-bras.gif")
film.save(gif, writer="pillow", fps=14, dpi=72)
plt.close(figure)

# ── 2. Le chemin de l'outil ───────────────────────────────────────────────────
figure, axe = plt.subplots(figsize=(7, 5))
for trajectoire in trajectoires:
    _, _, p3 = parcours(trajectoire["theta"], trajectoire["d"])
    axe.plot(p3[:, 0], p3[:, 2], color=trajectoire["color"], label=trajectoire["label"], lw=2)
    axe.plot(p3[0, 0], p3[0, 2], "o", color=trajectoire["color"], ms=6)
axe.set_xlabel("x (m)")
axe.set_ylabel("z (m)")
axe.set_title("Chemin parcouru par l'outil, dans le plan de la corde")
axe.grid(alpha=0.3)
axe.axis("equal")
axe.legend()
figure.tight_layout()
figure.savefig(os.path.join(SORTIES, "trajectoire-outil.png"), dpi=140)
plt.close(figure)

# ── 3. Vitesses et accélérations ──────────────────────────────────────────────
pas = t[1] - t[0]
figure, axes = plt.subplots(3, 2, figsize=(11, 8), sharex=True)
for colonne, (nom, unite) in enumerate([("theta", "rad"), ("d", "m")]):
    for trajectoire in trajectoires:
        valeur = trajectoire[nom]
        vitesse = np.gradient(valeur, pas)
        acceleration = np.gradient(vitesse, pas)
        for ligne, (courbe, libelle) in enumerate([
            (valeur, f"{nom} ({unite})"),
            (vitesse, f"vitesse ({unite}/s)"),
            (acceleration, f"accélération ({unite}/s²)"),
        ]):
            axes[ligne][colonne].plot(t, courbe, color=trajectoire["color"], lw=1.6,
                                      label=trajectoire["label"] if ligne == 0 else None)
            axes[ligne][colonne].set_ylabel(libelle)
            axes[ligne][colonne].grid(alpha=0.3)
    axes[0][colonne].set_title("Articulation rotoïde" if colonne == 0 else "Articulation prismatique")
    axes[2][colonne].set_xlabel("temps (s)")
axes[0][0].legend(fontsize=8)
figure.suptitle("Position, vitesse et accélération des deux axes")
figure.tight_layout()
figure.savefig(os.path.join(SORTIES, "vitesses-accelerations.png"), dpi=140)
plt.close(figure)

# ── Ce que disent les courbes ─────────────────────────────────────────────────
print("Images écrites dans", SORTIES)
for trajectoire in trajectoires:
    vitesse = np.gradient(trajectoire["theta"], pas)
    acceleration = np.gradient(vitesse, pas)
    print(f"  {trajectoire['label']:28s} vitesse max {np.abs(vitesse).max():5.2f} rad/s, "
          f"accélération max {np.abs(acceleration).max():6.2f} rad/s²")
