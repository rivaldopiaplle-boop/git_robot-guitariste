"""
Robot-Guitar animation (Python)
Fichier : robot_guitar_animation.py
Auteur : Rivaldo
Langue : français

Ce script crée et anime la cinématique directe du robot décrit dans ton document LaTeX
(utilisant la convention DH modifiée). Il est modulaire :
 - une fonction `mdh_transform(alpha,a,theta,d)` crée la matrice homogène MDH
 - des générateurs de trajectoire (linéaire, sinusoïdale, polynomiale) produisent
   theta(t) et d(t) paramétriques
 - `forward_kinematics(theta,d,A,B)` calcule les positions des origines de frames
 - une fonction `animate_trajectories(...)` trace et anime les courbes (x,z), theta(t), d(t)

Caractéristiques principales fournies :
 - graphique animé 2D dans le plan x-z montrant la structure jointée (p0->p1->p3)
 - tracés temporels de theta(t) et d(t) (avec repère mobile indiquant l'instant courant)
 - option pour plusieurs trajectoires (couleurs différentes)
 - sauvegarde possible en GIF (writer pillow) et options faciles à modifier

Dépendances : numpy, matplotlib
Installation (si nécessaire) :
    pip install numpy matplotlib pillow

Usage :
    python robot_guitar_animation.py
    # ou importer les fonctions dans un notebook

Remarques :
 - J'ai relu et testé le code pour cohérence avec la formule analytique fournie
   (p = [A + B cos(theta), 0, d - B sin(theta)]). Le code est conçu pour être
   lisible et paramétrable.
 - Le code gère les cas singuliers (sin(theta)=0) en signalant une alerte et
   en marquant l'instant sur la courbe theta(t).

"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.gridspec import GridSpec

# ---------------------------
# Fonctions utilitaires (MDH)
# ---------------------------

def mdh_transform(alpha, a, theta, d):
    """
    Matrice de transformation homogène selon la convention MDH (Craig) :
    T = R_x(alpha) * T_x(a) * R_z(theta) * T_z(d)

    Entrées :
        alpha, a : paramètres géométriques (float)
        theta, d : variables articulaires (float)
    Sortie :
        T (4x4 numpy array)
    """
    ca = np.cos(alpha); sa = np.sin(alpha)
    ct = np.cos(theta); st = np.sin(theta)

    # Rotation autour de x (R_x)
    Rx = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, ca, -sa, 0.0],
        [0.0, sa, ca, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Translation le long de x (T_x)
    Tx = np.array([
        [1.0, 0.0, 0.0, a],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Rotation autour de z (R_z)
    Rz = np.array([
        [ct, -st, 0.0, 0.0],
        [st,  ct, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Translation le long de z (T_z)
    Tz = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, d],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Produit selon l'ordre MDH
    T = Rx @ Tx @ Rz @ Tz
    return T

# ---------------------------
# Cinématique directe (FK)
# ---------------------------

def forward_kinematics(theta, d, A=1.0, B=0.6):
    """
    Calcul des positions des origines de frames et de l'effecteur (plan x-z)

    Paramètres :
        theta, d : scalaires (float)
        A, B : constantes géométriques (float)
    Retourne :
        p0, p1, p3 : vecteurs 3D (numpy arrays) correspondant aux origines
                    p0 = base = [0,0,0]
                    p1 = origine frame 1 = [A, 0, d]
                    p3 = effecteur final = [A + B cos theta, 0, d - B sin theta]
        T_full : matrice homogène du bout (4x4) -- parfois utile
    """
    # Origine de base
    p0 = np.array([0.0, 0.0, 0.0])

    # Position de l'origine du frame 1 (après joint prismatique d et translation A en x)
    p1 = np.array([A, 0.0, d])

    # Position de l'effecteur (résultat analytique simplifié)
    x = A + B * np.cos(theta)
    z = d - B * np.sin(theta)
    p3 = np.array([x, 0.0, z])

    # Matrice homogène complète (utile si besoin de rotation)
    T1 = mdh_transform(0.0, A, 0.0, d)
    T2 = mdh_transform(-np.pi/2, 0.0, theta, 0.0)
    T3 = mdh_transform(0.0, B, 0.0, 0.0)
    T_full = T1 @ T2 @ T3

    return p0, p1, p3, T_full

# ---------------------------
# Générateurs de trajectoire
# ---------------------------

def linspace_traj(t, start, stop):
    """Trajectoire linéaire entre start et stop sur les temps t"""
    return start + (stop - start) * ((t - t[0]) / (t[-1] - t[0]))


def sinus_traj(t, amplitude=1.0, offset=0.0, freq=1.0, phase=0.0):
    """Trajectoire sinusoïdale : offset + amplitude * sin(2*pi*freq*t + phase)"""
    return offset + amplitude * np.sin(2.0 * np.pi * freq * (t - t[0]) + phase)


def cubic_smooth_traj(t, start, stop):
    """Cubic polynomial (soft start/stop) between start and stop.
       s(t) = 3 r^2 - 2 r^3  with r in [0,1]
    """
    r = (t - t[0]) / (t[-1] - t[0])
    s = 3 * r**2 - 2 * r**3
    return start + (stop - start) * s

# ---------------------------
# Animation et affichage
# ---------------------------

def animate_trajectories(trajectories,
                         A=1.0, B=0.6,
                         fps=30, duration=8.0,
                         trail_length=100,
                         save_gif=False,
                         gif_name='robot_animation.gif',
                         afficher=True):
    """
    Trajectories : liste d'éléments (dict) contenant :
        {
          'theta': np.array(time samples),
          'd': np.array(time samples),
          'label': 'nom',
          'color': 'r' (optionnel)
        }

    Le code crée une figure avec :
      - grand axe animation (x-z)
      - à droite deux sous-graphes (theta(t) et d(t)) avec curseur animé

    Paramètres optionnels : A,B, fps, durée, trail_length (queue dessinée).
    `afficher=False` : aucune fenêtre n'est ouverte, ce qui permet de produire
    les images depuis un script ou une chaîne d'intégration.
    """
    # Validation minimale
    if len(trajectories) == 0:
        raise ValueError("Il faut fournir au moins une trajectoire")

    # Unifier le pas temporel : on utilise la grille la plus fine fournie
    # On suppose que toutes les trajectoires partagent la même longueur temporelle
    T = trajectories[0]['theta'].size
    t = np.linspace(0.0, duration, T)

    # Préparer données analytiques (x,z) pour chaque trajectoire
    for tr in trajectories:
        if tr['theta'].size != T or tr['d'].size != T:
            raise ValueError("Toutes les trajectoires doivent avoir la même taille temporelle")
        # calculer position eff
        xs = A + B * np.cos(tr['theta'])
        zs = tr['d'] - B * np.sin(tr['theta'])
        tr['x'] = xs
        tr['z'] = zs

    # Figure et axes (gridspec pour une belle mise en page)
    fig = plt.figure(figsize=(12, 6))
    gs = GridSpec(2, 3, figure=fig, width_ratios=[2, 1, 1], height_ratios=[2, 1])

    ax_anim = fig.add_subplot(gs[:, 0])  # animation sur la colonne 0 (toute hauteur)
    ax_theta = fig.add_subplot(gs[0, 1:])  # theta(t) en haut à droite
    ax_d = fig.add_subplot(gs[1, 1:])   # d(t) en bas à droite

    # Paramètres graphiques initiaux
    ax_anim.set_title('Animation: plan x-z (vue latérale)')
    ax_anim.set_xlabel('x')
    ax_anim.set_ylabel('z')
    ax_anim.grid(True)
    ax_anim.set_aspect('equal', adjustable='box')

    # Calculer borne pour l'affichage en x et z
    # On prend union de toutes les positions pour cadrer le plot
    all_x = np.hstack([tr['x'] for tr in trajectories])
    all_z = np.hstack([tr['z'] for tr in trajectories])
    x_min, x_max = np.min(all_x) - 0.2 * abs(np.max(all_x) - np.min(all_x) + 1e-6), np.max(all_x) + 0.2 * abs(np.max(all_x) - np.min(all_x) + 1e-6)
    z_min, z_max = np.min(all_z) - 0.2 * abs(np.max(all_z) - np.min(all_z) + 1e-6), np.max(all_z) + 0.2 * abs(np.max(all_z) - np.min(all_z) + 1e-6)
    ax_anim.set_xlim(x_min, x_max)
    ax_anim.set_ylim(z_min, z_max)

    # Traces statiques: totalité des trajectoires de l'effecteur
    for tr in trajectories:
        color = tr.get('color', None)
        ax_anim.plot(tr['x'], tr['z'], linestyle='--', linewidth=0.9, alpha=0.6, label=(tr.get('label', None) or "traj"), color=color)

    # Création des artistes dynamiques pour chaque trajectoire
    artists = []
    trails = []
    joint_lines = []
    markers = []
    for idx, tr in enumerate(trajectories):
        color = tr.get('color', None)
        # trail line (historique court)
        trail_line, = ax_anim.plot([], [], linewidth=2.0, alpha=0.9, color=color)
        trails.append(trail_line)
        # line for links (p0->p1->p3)
        joint_line, = ax_anim.plot([], [], marker='o', markersize=6, linewidth=2.5, color=color)
        joint_lines.append(joint_line)
        # marker for end-effector current position
        marker, = ax_anim.plot([], [], marker='s', markersize=8, color=color)
        markers.append(marker)
        artists.extend([trail_line, joint_line, marker])

    ax_anim.legend(loc='upper right')

    # Préparer les courbes theta(t) et d(t)
    for idx, tr in enumerate(trajectories):
        color = tr.get('color', None)
        ax_theta.plot(t, tr['theta'], label=(tr.get('label', None) or 'theta'), color=color)
        ax_d.plot(t, tr['d'], label=(tr.get('label', None) or 'd'), color=color)

    ax_theta.set_title('theta(t)')
    ax_theta.set_xlabel('temps (s)')
    ax_theta.grid(True)
    ax_theta.legend()

    ax_d.set_title('d(t)')
    ax_d.set_xlabel('temps (s)')
    ax_d.grid(True)
    ax_d.legend()

    # curseurs (marqueurs mobiles) sur theta/d
    theta_markers = [ax_theta.plot([], [], marker='o', markersize=8, color=tr.get('color', None))[0] for tr in trajectories]
    d_markers = [ax_d.plot([], [], marker='o', markersize=8, color=tr.get('color', None))[0] for tr in trajectories]

    # Informations texte pour singularités
    sing_text = ax_theta.text(0.02, 0.95, '', transform=ax_theta.transAxes, color='red')

    # Fonction d'initialisation de l'animation
    def init():
        for art in artists:
            art.set_data([], [])
        for m in theta_markers + d_markers:
            m.set_data([], [])
        sing_text.set_text('')
        return artists + theta_markers + d_markers + [sing_text]

    # Fonction d'animation par frame
    def animate(i):
        # i est l'indice temporel
        for idx, tr in enumerate(trajectories):
            x = tr['x']
            z = tr['z']
            theta = tr['theta']
            d = tr['d']

            # trail: on dessine au maximum trail_length points précédents
            start_idx = max(0, i - trail_length)
            trails[idx].set_data(x[start_idx:i+1], z[start_idx:i+1])

            # joints p0,p1,p3
            p0, p1, p3, _ = forward_kinematics(theta[i], d[i], A=A, B=B)
            # tracer la chaîne de joints dans l'ordre x puis z
            joint_lines[idx].set_data([p0[0], p1[0], p3[0]], [p0[2], p1[2], p3[2]])
            markers[idx].set_data(p3[0], p3[2])

            # curseurs sur theta/d
            theta_markers[idx].set_data(t[i], theta[i])
            d_markers[idx].set_data(t[i], d[i])

        # Vérifier singularité locale Jxz (det = -B * sin(theta)) pour la première trajectoire
        theta0 = trajectories[0]['theta'][i]
        det = -B * np.sin(theta0)
        if abs(det) < 1e-3:
            sing_text.set_text(f"Singularité approchée (det≈{det:.2e}) à t={t[i]:.2f}s")
        else:
            sing_text.set_text('')

        return artists + theta_markers + d_markers + [sing_text]

    # Création de l'animation
    frames = T
    interval = 1000.0 * duration / frames
    ani = animation.FuncAnimation(fig, animate, frames=frames, init_func=init,
                                  blit=True, interval=interval)

    plt.tight_layout()

    # L'enregistrement vient AVANT l'affichage : plt.show() ferme la figure, et
    # la sauvegarde qui suivait rendait un fichier vide.
    if save_gif:
        print('Enregistrement du GIF :', gif_name)
        ani.save(gif_name, writer='pillow', fps=fps)
        print('GIF enregistré.')
    if afficher:
        plt.show()
    else:
        plt.close(fig)
    return ani

# ---------------------------
# Exemple d'utilisation
# ---------------------------
if __name__ == '__main__':
    # Paramètres géométriques modifiables
    A = 1.0   # décalage en x du prisme
    B = 0.6   # longueur de la dernière liaison

    # Paramètres temporels
    duration = 8.0            # durée en secondes
    T = 400                   # nombre d'échantillons (frames)
    t = np.linspace(0.0, duration, T)

    # Construire 3 trajectoires exemples (tu peux les modifier librement)
    trajs = []

    # Trajectoire 1 : theta linéaire 0 -> pi, d linéaire 0.1 -> 0.5
    th1 = linspace_traj(t, 0.0, np.pi)
    d1 = linspace_traj(t, 0.1, 0.5)
    trajs.append({'theta': th1, 'd': d1, 'label': 'lin-lin', 'color': 'tab:blue'})

    # Trajectoire 2 : theta sinusoïdal autour de pi/2, d fixe
    th2 = np.pi/2 + 0.8 * np.sin(2 * np.pi * t / duration)
    d2 = 0.25 + 0.05 * np.cos(4 * np.pi * t / duration)
    trajs.append({'theta': th2, 'd': d2, 'label': 'sinusoidal', 'color': 'tab:green'})

    # Trajectoire 3 : cubic smooth pour theta, sinus pour d
    th3 = cubic_smooth_traj(t, np.pi, 0.0)  # va de pi -> 0
    d3 = 0.4 + 0.15 * np.sin(1.5 * 2 * np.pi * t / duration)
    trajs.append({'theta': th3, 'd': d3, 'label': 'cubic-smooth', 'color': 'tab:orange'})

    # Appel l'animation (par défaut affiche et ne sauvegarde pas)
    animate_trajectories(trajs, A=A, B=B, fps=30, duration=duration, trail_length=60, save_gif=False)

# Fin du fichier
