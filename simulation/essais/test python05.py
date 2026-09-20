# ...existing code...
"""
Robot animation complet (MDH) - animation x-z + traces temporelles
Auteur : GitHub Copilot (produit)
Description :
 - Utilise la convention MDH pour 4 joints (prismatique en d1 et trois rotations
   intermédiaires comme spécifié).
 - Trajectoires périodiques fournies pour theta2, theta3 et d (un seul motif suffit).
 - Figure principale : animation plan x-z (vue latérale), points dynamiques P0, PA,
   P1, P2, P3, PE (couleurs différentes), segments reliant les points avec la même
   couleur que les endpoints, trajectoire finale de l'effecteur toujours visible.
 - Vecteur d (flèche) est PA -> P1, sa longueur varie. Arc colorés montrant theta2
   et theta3 autour des origines P2 et P3 respectivement.
 - Trois sous-graphes à droite : theta2(t), theta3(t), et panneau combiné (theta_total,d).
 - Marges et layout optimisés. Utilise uniquement numpy et matplotlib.
Usage :
    python robot_animation_full.py
Dépendances : numpy, matplotlib
"""
import numpy as np
import matplotlib.pyplot as plt
"""
test python05.py - version nettoyée
Animation robot utilisant les signaux fournis theta2_signal, theta3_signal et d_signal.
Affichage : abscisse = z, ordonnée = x. Flèche dynamique (PA->P1), arcs pour θ2 et θ3, tracés temporels.
Dépendances : numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import FancyArrowPatch
from matplotlib.lines import Line2D
from matplotlib.gridspec import GridSpec

# ---------------------------
# Paramètres géométriques (grands pour visibilité)
# ---------------------------
A = 2.5
B = 2.0
C = 1.8
D = 1.2
d1 = 1.0

# Temps
duration = 8.0
N_samples = 600
t_vect = np.linspace(0.0, duration, N_samples)

# Paramètres trajectoires (utiliser exactement les fonctions fournies)
T_period = 6.0
theta0 = np.pi/4
tau = 0.9
sigma = np.pi/6
dmax = 1.6

# Fonctions fournies
def theta2_periodique(t, T=T_period, theta0=theta0, tau=tau):
    theta = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if phase <= T/2 - tau:
            theta[i] = theta0
        elif phase <= T/2:
            theta[i] = theta0 - 2*theta0*(phase - (T/2 - tau))/tau
        elif phase <= T - tau:
            theta[i] = -theta0
        else:
            theta[i] = -theta0 + 2*theta0*(phase - (T - tau))/tau
    return theta


def theta3_triangle(t, T=T_period, sigma=sigma, tau=tau):
    theta3 = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if abs(phase - T/4) <= tau/2:
            theta3[i] = sigma * (1 - 2*abs(phase - T/4)/tau)
        elif abs(phase - 3*T/4) <= tau/2:
            theta3[i] = -sigma * (1 - 2*abs(phase - 3*T/4)/tau)
        else:
            theta3[i] = 0.0
    return theta3


def d_periodique(t, T=T_period, dmax=dmax):
    d = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if phase <= T/2:
            d[i] = d1 + 2*(dmax - d1)*phase/T
        else:
            d[i] = d1 + 2*(dmax - d1)*(-phase/T + 1)
    return d

# Signaux (ce sont ceux que l'utilisateur a fournis et demande d'utiliser)
theta2_signal = theta2_periodique(t_vect)
theta3_signal = theta3_triangle(t_vect)
d_signal = d_periodique(t_vect)
theta_total_signal = theta2_signal + theta3_signal

# ---------------------------
# MDH : Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
# Table fournie par l'utilisateur :
# 1: alpha=0, a=A, d=d1, theta=pi
# 2: alpha=pi/2, a=B, d=0, theta=theta2
# 3: alpha=0, a=C, d=0, theta=theta3
# 4: alpha=-pi/2, a=D, d=0, theta=pi
# ---------------------------

def mdh_transform(alpha, a, theta, d):
    ca = np.cos(alpha); sa = np.sin(alpha)
    ct = np.cos(theta); st = np.sin(theta)
    Rx = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, ca, -sa, 0.0],
        [0.0, sa, ca, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    Tx = np.array([
        [1.0, 0.0, 0.0, a],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    Rz = np.array([
        [ct, -st, 0.0, 0.0],
        [st, ct, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    Tz = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, d],
        [0.0, 0.0, 0.0, 1.0]
    ])
    return Rx @ Tx @ Rz @ Tz


def forward_kinematics(theta2, theta3, d):
    T0_1 = mdh_transform(0.0, A, np.pi, d1)
    T1_2 = mdh_transform(np.pi/2, B, theta2, 0.0)
    T2_3 = mdh_transform(0.0, C, theta3, 0.0)
    T3_4 = mdh_transform(-np.pi/2, D, np.pi, 0.0)
    O0 = np.array([0.0, 0.0, 0.0, 1.0])
    O1 = T0_1 @ O0
    O2 = T0_1 @ T1_2 @ O0
    O3 = T0_1 @ T1_2 @ T2_3 @ O0
    O4 = T0_1 @ T1_2 @ T2_3 @ T3_4 @ O0
    P0 = O0[:3]
    PA = np.array([A, 0.0, d1])
    P1 = O1[:3]
    P2 = O2[:3]
    P3 = O3[:3]
    PE = O4[:3]
    return P0, PA, P1, P2, P3, PE

# ---------------------------
# Plot setup: z horizontal (abscissa), x vertical (ordinate)
# ---------------------------
fig = plt.figure(figsize=(16, 9), constrained_layout=True)
gs = GridSpec(2, 3, figure=fig, width_ratios=[2.6, 1, 1], height_ratios=[1, 1], hspace=0.3)
ax_anim = fig.add_subplot(gs[:, 0])
ax_theta = fig.add_subplot(gs[0, 1])
ax_d = fig.add_subplot(gs[1, 1])
ax_thd = fig.add_subplot(gs[:, 2])

ax_anim.set_title('Animation (z abscisse, x ordonnée)')
ax_anim.set_xlabel('z')
ax_anim.set_ylabel('x')
ax_anim.set_aspect('equal', adjustable='box')
ax_anim.grid(True)

# precompute envelope for limits
all_PE_z = []
all_PE_x = []
for i in range(len(t_vect)):
    _, _, _, _, _, PE = forward_kinematics(theta2_signal[i], theta3_signal[i], d_signal[i])
    all_PE_z.append(PE[2])
    all_PE_x.append(PE[0])
all_PE_z = np.array(all_PE_z)
all_PE_x = np.array(all_PE_x)

z_min, z_max = np.min(all_PE_z), np.max(all_PE_z)
x_min, x_max = np.min(all_PE_x), np.max(all_PE_x)
margin_z = 0.25 * (z_max - z_min + 1e-6)
margin_x = 0.25 * (x_max - x_min + 1e-6)
ax_anim.set_xlim(z_min - margin_z, z_max + margin_z)
ax_anim.set_ylim(x_min - margin_x, x_max + margin_x)

# static final trajectory
ax_anim.plot(all_PE_z, all_PE_x, '--', color='tab:pink', alpha=0.6, label='PE final traj')

# dynamic artists
point_colors = {'P0':'gray','PA':'purple','P1':'blue','P2':'green','P3':'orange','PE':'red'}
point_artists = {}
for name, col in point_colors.items():
    p, = ax_anim.plot([], [], marker='o', color=col, markersize=7, label=name)
    point_artists[name] = p

line_pairs = [('P0','PA'),('PA','P1'),('P1','P2'),('P2','P3'),('P3','PE')]
line_artists = {}
for a,b in line_pairs:
    l, = ax_anim.plot([], [], lw=3, color=point_colors[a])
    line_artists[(a,b)] = l

# history
PE_hist, = ax_anim.plot([], [], 'r-', lw=2, alpha=0.9)

# arrow for prismatic d (PA -> P1) in plotted coords (z,x)
arrow = FancyArrowPatch((0,0),(0,0), mutation_scale=18, color='olive', lw=2, arrowstyle='-|>')

"""
Animation robot complet (MDH) - version enrichie et propre
Points dynamiques colorés, segments, flèche PA->P1, arcs θ2/θ3, légende, trajectoire finale, marges parfaites, quatre sous-figures, figure très grande
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import FancyArrowPatch
from matplotlib.lines import Line2D
from matplotlib.gridspec import GridSpec

# ---------------------------
# Paramètres géométriques
# ---------------------------
A = 2.5
B = 2.0
C = 1.8
D = 1.2
d1 = 1.0

# Temps
duration = 8.0
N_samples = 800
t_vect = np.linspace(0.0, duration, N_samples)

# Trajectoires périodiques (fonctions fournies)
T_period = 6.0
theta0 = np.pi/4
tau = 0.9
sigma = np.pi/6
dmax = 1.6

def theta2_periodique(t, T=T_period, theta0=theta0, tau=tau):
    theta = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if phase <= T/2 - tau:
            theta[i] = theta0
        elif phase <= T/2:
            theta[i] = theta0 - 2*theta0*(phase - (T/2 - tau))/tau
        elif phase <= T - tau:
            theta[i] = -theta0
        else:
            theta[i] = -theta0 + 2*theta0*(phase - (T - tau))/tau
    return theta

def theta3_triangle(t, T=T_period, sigma=sigma, tau=tau):
    theta3 = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if abs(phase - T/4) <= tau/2:
            theta3[i] = sigma * (1 - 2*abs(phase - T/4)/tau)
        elif abs(phase - 3*T/4) <= tau/2:
            theta3[i] = -sigma * (1 - 2*abs(phase - 3*T/4)/tau)
        else:
            theta3[i] = 0.0
    return theta3

def d_periodique(t, T=T_period, dmax=dmax):
    d = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if phase <= T/2:
            d[i] = d1 + 2*(dmax - d1)*phase/T
        else:
            d[i] = d1 + 2*(dmax - d1)*(-phase/T + 1)
    return d

# Signaux à utiliser
theta2_signal = theta2_periodique(t_vect)
theta3_signal = theta3_triangle(t_vect)
d_signal = d_periodique(t_vect)
theta_total_signal = theta2_signal + theta3_signal

# ---------------------------
# MDH convention
# ---------------------------
def mdh_transform(alpha, a, theta, d):
    ca = np.cos(alpha); sa = np.sin(alpha)
    ct = np.cos(theta); st = np.sin(theta)
    Rx = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, ca, -sa, 0.0],
        [0.0, sa, ca, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    Tx = np.array([
        [1.0, 0.0, 0.0, a],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    Rz = np.array([
        [ct, -st, 0.0, 0.0],
        [st, ct, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    Tz = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, d],
        [0.0, 0.0, 0.0, 1.0]
    ])
    return Rx @ Tx @ Rz @ Tz

def forward_kinematics(theta2, theta3, d):
    T0_1 = mdh_transform(0.0, A, np.pi, d1)
    T1_2 = mdh_transform(np.pi/2, B, theta2, 0.0)
    T2_3 = mdh_transform(0.0, C, theta3, 0.0)
    T3_4 = mdh_transform(-np.pi/2, D, np.pi, 0.0)
    O0 = np.array([0.0, 0.0, 0.0, 1.0])
    O1 = T0_1 @ O0
    O2 = T0_1 @ T1_2 @ O0
    O3 = T0_1 @ T1_2 @ T2_3 @ O0
    O4 = T0_1 @ T1_2 @ T2_3 @ T3_4 @ O0
    P0 = O0[:3]
    PA = np.array([A, 0.0, d1])
    P1 = O1[:3]
    P2 = O2[:3]
    P3 = O3[:3]
    PE = O4[:3]
    return P0, PA, P1, P2, P3, PE

# ---------------------------
# Figure et sous-figures
# ---------------------------
fig = plt.figure(figsize=(20, 12), constrained_layout=True)
    paz, pax = coords['PA']
    p1z, p1x = coords['P1']
    arrow.set_positions((paz, pax),(p1z, p1x))
    # arcs theta2 around P2
    v1 = np.array([coords['P1'][0] - coords['P2'][0], coords['P1'][1] - coords['P2'][1]])
    v2 = np.array([coords['P3'][0] - coords['P2'][0], coords['P3'][1] - coords['P2'][1]])
    if np.linalg.norm(v1) > 1e-8 and np.linalg.norm(v2) > 1e-8:
        xs, ys = arc_points_2d(coords['P2'], v1, v2, radius=0.25)
        arc_t2.set_data(xs, ys)
    else:
        arc_t2.set_data([],[])
    # arcs theta3 around P3
    v3 = np.array([coords['P2'][0] - coords['P3'][0], coords['P2'][1] - coords['P3'][1]])
    v4 = np.array([coords['PE'][0] - coords['P3'][0], coords['PE'][1] - coords['P3'][1]])
    if np.linalg.norm(v3) > 1e-8 and np.linalg.norm(v4) > 1e-8:
        xs, ys = arc_points_2d(coords['P3'], v3, v4, radius=0.25)
        arc_t3.set_data(xs, ys)
    else:
        arc_t3.set_data([],[])
    # cursors
    t = t_vect[i]
    cursor_t2.set_data([t],[th2])
    cursor_t3.set_data([t],[th3])
    cursor_tt.set_data([t],[theta_total_signal[i]])
    cursor_d.set_data([t],[d])
    # update right scatter
    ax_thd.lines[0].set_data(d_signal[:i+1], theta_total_signal[:i+1])
    return list(point_artists.values()) + list(line_artists.values()) + [PE_hist, arrow, arc_t2, arc_t3, cursor_t2, cursor_t3, cursor_tt, cursor_d] + list(labels.values())

# animation
ani = animation.FuncAnimation(fig, update, frames=len(t_vect), init_func=init, interval=1000.0*duration/len(t_vect), blit=False)

plt.show()

ax_anim.set_title("Animation (plan x-z) - vues latérales", fontsize=14)
ax_anim.set_xlabel("x")
ax_anim.set_ylabel("z")
ax_anim.grid(True)
ax_anim.set_aspect('equal', adjustable='box')

# Déterminer bornes x,z en prenant l'enveloppe de toutes positions possibles
all_PE_x = []
all_PE_z = []
for i in range(len(t_vect)):
    _, _, _, _, _, PE = forward_kinematics(theta2_signal[i], theta3_signal[i], d_signal[i])
    all_PE_x.append(PE[0])
    all_PE_z.append(PE[2])
all_x = np.array(all_PE_x + [0.0, A, A+B, A+B+C+D])
all_z = np.array(all_PE_z + [0.0, d1 + dmax + 0.5])
x_margin = 0.25 * (np.max(all_x) - np.min(all_x) + 1e-6)
z_margin = 0.25 * (np.max(all_z) - np.min(all_z) + 1e-6)
ax_anim.set_xlim(np.min(all_x) - x_margin, np.max(all_x) + x_margin)
ax_anim.set_ylim(np.min(all_z) - z_margin, np.max(all_z) + z_margin)

# Traces statiques (en pointillés) pour trajectoire complète de l'effecteur
PE_all_x = np.array(all_PE_x)
PE_all_z = np.array(all_PE_z)
ax_anim.plot(PE_all_x, PE_all_z, '--', linewidth=1.1, color='tab:pink', alpha=0.6, label="Traj. finale (PE)")

# Créer artistes dynamiques
# Points
point_artists = {}
for name, col in point_colors.items():
    p, = ax_anim.plot([], [], marker='o', markersize=8, color=col, label=name)
    point_artists[name] = p

# Lignes/segments (une ligne entre chaque paire demandée, la couleur sera la couleur du premier point)
line_artists = {}
for key, (p_from, p_to) in line_color_pairs.items():
    col_from = point_colors[p_from]
    l, = ax_anim.plot([], [], lw=3.0, color=col_from)
    line_artists[key] = l

# Trajectoire effecteur visible tout le temps (historique)
PE_traj_line, = ax_anim.plot([], [], 'r-', lw=2.0, alpha=0.9, label='PE (histo)')

# Flèche dynamique pour d (PA -> P1)
arrow = FancyArrowPatch((0,0), (0,0), mutation_scale=18, lw=2.0, color='tab:olive', arrowstyle='-|>')
ax_anim.add_artist(arrow)

# Arcs pour theta2 (origine P2) et theta3 (origine P3) : on crée des Line2D qui seront mis à jour
arc_t2_line, = ax_anim.plot([], [], color='tab:cyan', lw=2.5, alpha=0.9, label='θ2 (arc)')
arc_t3_line, = ax_anim.plot([], [], color='tab:magenta', lw=2.5, alpha=0.9, label='θ3 (arc)')

# Points de repère labels (texte) près des origines
text_labels = {}
for name in ['P0', 'PA', 'P1', 'P2', 'P3', 'PE']:
    txt = ax_anim.text(0, 0, name, fontsize=10, color=point_colors[name], va='bottom', ha='left')
    text_labels[name] = txt

# Légende spécifique sur axe dédié pour éviter chevauchement
legend_handles = []
legend_labels = []
# ajouter points et ligne traj
for name in ['P0', 'PA', 'P1', 'P2', 'P3', 'PE']:
    legend_handles.append(point_artists[name])
    legend_labels.append(name)
legend_handles.append(PE_traj_line); legend_labels.append('PE (histo)')
legend_handles.append(Line2D([0],[0], color='tab:olive', lw=2)); legend_labels.append('vecteur d (PA->P1)')
ax_legend.legend(legend_handles, legend_labels, loc='center', ncol=4, frameon=False)

# ---------------------------
# Axes temporels (theta2, theta3, combined)
# ---------------------------
ax_t2.plot(t_vect, theta2_signal, color='tab:blue', lw=1.2)
ax_t2.set_title('θ2(t)')
ax_t2.set_xlabel('t [s]'); ax_t2.set_ylabel('rad'); ax_t2.grid(True)
t2_cursor, = ax_t2.plot([], [], marker='o', color='tab:blue', markersize=8)

ax_t3.plot(t_vect, theta3_signal, color='tab:orange', lw=1.2)
ax_t3.set_title('θ3(t)')
ax_t3.set_xlabel('t [s]'); ax_t3.set_ylabel('rad'); ax_t3.grid(True)
t3_cursor, = ax_t3.plot([], [], marker='o', color='tab:orange', markersize=8)

# Combined: theta_total and d
theta_total_signal = theta2_signal + theta3_signal
ax_comb.plot(t_vect, theta_total_signal, color='tab:purple', lw=1.2, label='θ total')
ax_comb.plot(t_vect, d_signal, color='tab:green', lw=1.2, label='d(t)')
ax_comb.set_title('θ total(t) et d(t)')
ax_comb.set_xlabel('t [s]'); ax_comb.grid(True); ax_comb.legend(loc='upper right')
combined_cursor_theta, = ax_comb.plot([], [], marker='o', color='tab:purple', markersize=8)
combined_cursor_d, = ax_comb.plot([], [], marker='o', color='tab:green', markersize=8)

# ---------------------------
# Initialisation
# ---------------------------
trail_length = 300  # nombre maximal de points affichés dans la traînée
history_x = []; history_z = []

def init_anim():
    for p in point_artists.values():
        p.set_data([], [])
    for l in line_artists.values():
        l.set_data([], [])
    PE_traj_line.set_data([], [])
    arrow.set_positions((0,0), (0,0))
    arc_t2_line.set_data([], [])
    arc_t3_line.set_data([], [])
    t2_cursor.set_data([], [])
    t3_cursor.set_data([], [])
    combined_cursor_theta.set_data([], [])
    combined_cursor_d.set_data([], [])
    for txt in text_labels.values():
        txt.set_position((np.nan, np.nan))
    return list(point_artists.values()) + list(line_artists.values()) + [PE_traj_line, arrow, arc_t2_line, arc_t3_line,
                                                                      t2_cursor, t3_cursor, combined_cursor_theta, combined_cursor_d] + list(text_labels.values())

# ---------------------------
# Fonction d'update par frame
# ---------------------------
def update(frame_idx):
    global history_x, history_z
    th2 = theta2_signal[frame_idx]
    th3 = theta3_signal[frame_idx]
    d = d_signal[frame_idx]
    P0, PA, P1, P2, P3, PE = forward_kinematics(th2, th3, d)

    # Extraire coordonnées x,z (plan)
    coords = {
        'P0': (P0[0], P0[2]),
        'PA': (PA[0], PA[2]),
        'P1': (P1[0], P1[2]),
        'P2': (P2[0], P2[2]),
        'P3': (P3[0], P3[2]),
        'PE': (PE[0], PE[2])
    }

    # Mettre à jour points
    for name, art in point_artists.items():
        xz = coords[name]
        art.set_data([xz[0]], [xz[1]])
        # Mettre à jour labels proches des points (décalage vers la droite)
        text_labels[name].set_position((xz[0] + 0.02, xz[1] + 0.02))

    # Mettre à jour segments (chaque segment coloré selon point de début)
    for key, (p_from, p_to) in line_color_pairs.items():
        x1, z1 = coords[p_from]
        x2, z2 = coords[p_to]
        line_artists[key].set_data([x1, x2], [z1, z2])

    # Mettre à jour la trajectoire historique complète (toujours visible)
    history_x.append(coords['PE'][0])
    history_z.append(coords['PE'][1])
    # limiter l'historique à trail_length si souhaité, mais garder la trajectoire finale toujours visible
    if len(history_x) > trail_length:
        history_x = history_x[-trail_length:]
        history_z = history_z[-trail_length:]
    PE_traj_line.set_data(history_x, history_z)

    # Mettre à jour flèche vecteur d (PA -> P1)
    arrow.set_positions((coords['PA'][0], coords['PA'][1]), (coords['P1'][0], coords['P1'][1]))

    # Arcs: calculer vecteurs dans plan x-z pour angles
    # theta2 : angle entre vec(P1->P2) et vec(P2->P3) autour origine P2
    v_p1p2 = np.array([coords['P1'][0] - coords['P2'][0], coords['P1'][1] - coords['P2'][1]])
    v_p2p3 = np.array([coords['P3'][0] - coords['P2'][0], coords['P3'][1] - coords['P2'][1]])
    # theta3 : angle entre vec(P2->P3) et vec(P3->PE) autour origine P3
    v_p2p3_for3 = np.array([coords['P2'][0] - coords['P3'][0], coords['P2'][1] - coords['P3'][1]])
    v_p3pe = np.array([coords['PE'][0] - coords['P3'][0], coords['PE'][1] - coords['P3'][1]])

    # Norms check and compute arc points
    # pour theta2 arc
    if np.linalg.norm(v_p1p2) > 1e-6 and np.linalg.norm(v_p2p3) > 1e-6:
        xs2, zs2 = arc_points((coords['P2'][0], coords['P2'][1]), v_p1p2, v_p2p3, radius=min(0.5, 0.25*(np.linalg.norm(v_p2p3)+np.linalg.norm(v_p1p2))))
        arc_t2_line.set_data(xs2, zs2)
    else:
        arc_t2_line.set_data([], [])

    # pour theta3 arc
    if np.linalg.norm(v_p2p3_for3) > 1e-6 and np.linalg.norm(v_p3pe) > 1e-6:
        xs3, zs3 = arc_points((coords['P3'][0], coords['P3'][1]), v_p2p3_for3, v_p3pe, radius=min(0.5, 0.25*(np.linalg.norm(v_p3pe)+np.linalg.norm(v_p2p3_for3))))
        arc_t3_line.set_data(xs3, zs3)
    else:
        arc_t3_line.set_data([], [])

    # Cursors temporels sur les sous-graphes
    t_current = t_vect[frame_idx]
    t2_cursor.set_data([t_current], [th2])
    t3_cursor.set_data([t_current], [th3])
    combined_cursor_theta.set_data([t_current], [theta_total_signal[frame_idx]])
    combined_cursor_d.set_data([t_current], [d])

    # Retourner artistes qui auront changé
    artists = list(point_artists.values()) + list(line_artists.values()) + [PE_traj_line, arrow, arc_t2_line, arc_t3_line,
                                                                          t2_cursor, t3_cursor, combined_cursor_theta, combined_cursor_d] + list(text_labels.values())
    return artists

# ---------------------------
# Création de l'animation
# ---------------------------
ani = animation.FuncAnimation(fig, update, frames=len(t_vect), init_func=init_anim,
                              interval=1000.0 * duration / len(t_vect), blit=False)

plt.tight_layout()
plt.show()

# Sauvegarde optionnelle (décommentez si besoin ; nécessite pillow pour writer='pillow')
# try:
#     ani.save('robot_animation_full.gif', writer='pillow', fps=30)
# except Exception as e:
#     print("Erreur lors de la sauvegarde:", e)
# ...existing code...