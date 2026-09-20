# -*- coding: utf-8 -*-
"""
Robot-Guitar : θ3 corrigé pour tangence exacte au disque de la corde
Version complète et appliquée — remplace compute_frames_coupled par une version plus rigoureuse
Auteur : Rivaldo (repris et complété)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import FancyArrowPatch, Circle

# ------------------------
# PARAMÈTRES (comme fournis)
# ------------------------
A, B, C, D = 15.0, 3.0, 5.0, 8.0
d1_0 = 25.0
T = 10.0
theta0 = np.pi/6
delta = T/25.0

# animation / résolution
N = 400
dc = 3.0
frame_interval = 40.0
speed_factor = 3.0
eps_tang = 1e-6
max_iter_fixedpoint = 20
tol_theta3 = 1e-6

# ------------------------
# SIGNALS (gardées, légèrement simplifiées)
# ------------------------
def d_periodique(t, T, dmax, delta):
    d = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if phase <= T/2 - delta:
            d[i] = dmax * phase / (T/2 - delta)
        elif phase <= T/2:
            d[i] = dmax
        elif phase <= T - delta:
            d[i] = dmax * (-phase/(T/2 - delta) + (T-delta)/(T/2 - delta))
        else:
            d[i] = 0.0
    return d

def theta2_en_fonction_de_d(d_vect, theta0, eps=1e-6, smooth_window=3):
    Ntot = len(d_vect)
    d_prime = np.gradient(d_vect)
    sign = np.zeros_like(d_prime, dtype=int)
    sign[d_prime > eps] = 1
    sign[d_prime < -eps] = -1
    theta2 = np.zeros_like(d_vect)
    theta2[sign == 1] = -theta0
    theta2[sign == -1] = theta0
    i = 0
    while i < Ntot:
        if sign[i] == 0:
            start = i
            while i < Ntot and sign[i] == 0:
                i += 1
            end = i
            seg_len = end - start
            if seg_len <= 0:
                continue
            if np.max(np.abs(d_vect[start:end])) <= eps:
                theta_seg = np.linspace(theta0, -theta0, seg_len)
            else:
                theta_seg = np.linspace(-theta0, theta0, seg_len)
            theta2[start:end] = theta_seg
        else:
            i += 1
    if smooth_window is not None and smooth_window > 1:
        k = np.ones(smooth_window) / float(smooth_window)
        theta2 = np.convolve(theta2, k, mode='same')
    return theta2, d_prime

# ------------------------
# UTILITAIRES plan Z-X
# ------------------------
def vector_norm(v):
    return np.linalg.norm(v)

def angle_between_vectors_in_xz(v_from, v_to):
    ax = np.array([v_from[2], v_from[0]])
    bx = np.array([v_to[2], v_to[0]])
    ang_from = np.arctan2(ax[1], ax[0])
    ang_to = np.arctan2(bx[1], bx[0])
    diff = ang_to - ang_from
    while diff > np.pi: diff -= 2*np.pi
    while diff <= -np.pi: diff += 2*np.pi
    return ang_from, ang_to, diff

def arc_points(center, ang_start, ang_end, radius, npoints=30):
    thetas = np.linspace(ang_start, ang_end, npoints)
    zs = center[2] + radius * np.cos(thetas)
    xs = center[0] + radius * np.sin(thetas)
    return zs, xs

def norm_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi

# ------------------------
# CINÉMATIQUE DIRECTE (p0..p3) — on garde la tienne
# ------------------------
def forward_kinematics(theta2, d1):
    p0 = np.array([0.0, 0.0, 0.0])
    pA = np.array([A, 0.0, 0.0])
    p1 = pA + np.array([0.0, 0.0, d1])
    p2 = p1 + np.array([-B, 0.0, 0.0])
    p3 = p2 + np.array([-C*np.cos(theta2), 0.0, C*np.sin(theta2)])
    return p0, pA, p1, p2, p3

# ------------------------
# NOUVELLE VERSION RIGOUREUSE: calcul θ3 par pas de temps
# - renvoie frames compatibles avec le reste du code
# - fournit theta3_vect, theta_total_vect et dist_cc_list
# ------------------------
import numpy as np

def compute_frames_coupled(t_vect, theta2_vect, d_vect, Zc, Xc, dc,
                           eps_tang=1e-6,
                           max_iter=20, tol=1e-6):
    N = len(t_vect)
    frames = []
    theta3_vect = np.zeros(N)
    theta_total_vect = np.zeros(N)
    dist_cc_list = np.zeros(N)

    r_corde = dc / 2.0
    Cxy = np.array([Zc, Xc])

    for i in range(N):
        th2 = theta2_vect[i]
        dval = d_vect[i]

        # --- positions de base du bras jusqu'à p3 ---
        p0, pA, p1, p2, p3 = forward_kinematics(th2, dval)
        p3_xz = np.array([p3[2], p3[0]])  # (z,x)

        # hypothèse initiale : theta3 = 0
        theta3 = 0.0
        theta_total = th2 + theta3
        p4 = p3 + np.array([-D * np.cos(theta_total), 0.0, D * np.sin(theta_total)])
        p4_xz = np.array([p4[2], p4[0]])

        # --- Test 1 : hors du domaine de la corde ---
        if Xc < (p4_xz[1] - r_corde) or Xc > (p3_xz[1] + r_corde):
            theta3 = 0.0
            theta_total = th2
        else:
            # --- approche tangentielle ---
            vect_cc = Cxy - p4_xz
            dist_cc = np.linalg.norm(vect_cc)
            z_p4 = p4_xz[0]
            z_c = Zc

            # --- Tant que la corde n'est pas tangente (dist < r_corde) ---
            # On incrémente petit à petit theta3 jusqu’à obtenir dist_cc ≈ r_corde
            step = 1e-3  # petit pas angulaire
            max_iter = 2000  # sécurité
            k = 0
            while dist_cc < r_corde and abs(z_p4 - z_c) > eps_tang and k < max_iter:
                theta3 += step
                theta_total = th2 + theta3
                p4 = p3 + np.array([-D * np.cos(theta_total), 0.0, D * np.sin(theta_total)])
                p4_xz = np.array([p4[2], p4[0]])
                vect_cc = Cxy - p4_xz
                dist_cc = np.linalg.norm(vect_cc)
                z_p4 = p4_xz[0]
                k += 1

            # Si la distance devient supérieure, on recule légèrement (ajustement fin)
            if dist_cc > r_corde:
                while dist_cc > r_corde and abs(z_p4 - z_c) > eps_tang:
                    theta3 -= step / 10
                    theta_total = th2 + theta3
                    p4 = p3 + np.array([-D * np.cos(theta_total), 0.0, D * np.sin(theta_total)])
                    p4_xz = np.array([p4[2], p4[0]])
                    vect_cc = Cxy - p4_xz
                    dist_cc = np.linalg.norm(vect_cc)
                    z_p4 = p4_xz[0]

            # --- Condition de sortie tangente en z ---
            if abs(z_p4 - z_c) <= eps_tang:
                theta3 = 0.0
                theta_total = th2

        # recalcul final du p4 pour affichage
        theta_total = th2 + theta3
        p4 = p3 + np.array([-D * np.cos(theta_total), 0.0, D * np.sin(theta_total)])
        p4_xz = np.array([p4[2], p4[0]])
        dist_cc_list[i] = np.linalg.norm(Cxy - p4_xz)

        # --- arcs décoratifs inchangés ---
        ang_from2, _, signed2 = angle_between_vectors_in_xz(p2 - p1, p3 - p2)
        ang_from3, _, signed3 = angle_between_vectors_in_xz(p3 - p2, p4 - p3)
        r2 = max(0.5, 0.1 * vector_norm(p2)) * (0.5 + 0.5 * dval / 61.0)
        r3 = max(0.4, 0.08 * vector_norm(p3)) * (0.5 + 0.5 * dval / 61.0)
        zs2, xs2 = arc_points(p2, ang_from2, ang_from2 + signed2, r2)
        zs3, xs3 = arc_points(p3, ang_from3, ang_from3 + signed3, r3)

        # --- sauvegarde ---
        frames.append({
            "p0": p0, "pA": pA, "p1": p1, "p2": p2, "p3": p3, "p4": p4,
            "theta2": th2, "theta3": theta3, "theta_total": theta_total, "d": dval,
            "arc2": (zs2, xs2), "arc3": (zs3, xs3),
            "dist_cc": dist_cc_list[i]
        })

        theta3_vect[i] = theta3
        theta_total_vect[i] = theta_total

    return frames, theta3_vect, theta_total_vect, dist_cc_list



# ------------------------
# ANIMATION (repris/adapté de ton code)
# ------------------------
def animate_robot_full(show_plots=True):
    t = np.linspace(0.0, T, N)
    d_signal = d_periodique(t, T, d1_0, delta)
    theta2_signal, d_prime = theta2_en_fonction_de_d(d_signal, theta0)

    zmin = -(C + D) * np.cos(theta0)
    zmax = d1_0 + (C + D) * np.cos(theta0)
    Zc = (zmax + zmin) / 2.0
    Xc = A - B - (C + D) * np.cos(theta0)

    frames, theta3_signal, theta_total, dist_cc_list = compute_frames_coupled(
        t, theta2_signal, d_signal, Zc, Xc, dc
    )

    # Figures
    fig, ((ax_anim, ax_theta), (ax_d, ax_thd)) = plt.subplots(2,2, figsize=(15,12))
    ax_anim.set_title("Robot-Guitar animation Z-X")
    ax_anim.set_xlabel("Z")
    ax_anim.set_ylabel("X")
    ax_anim.grid(True)

    all_z = np.concatenate([np.array([f["p0"][2],f["pA"][2],f["p1"][2],f["p2"][2],f["p3"][2],f["p4"][2]]) for f in frames])
    all_x = np.concatenate([np.array([f["p0"][0],f["pA"][0],f["p1"][0],f["p2"][0],f["p3"][0],f["p4"][0]]) for f in frames])
    margin = max(1.0, 0.25*max(np.max(all_z)-np.min(all_z), np.max(all_x)-np.min(all_x)))
    ax_anim.set_xlim(np.min(all_z)-margin, np.max(all_z)+margin)
    ax_anim.set_ylim(np.min(all_x)-margin, np.max(all_x)+margin)

    colors = ["black","gray","blue","green","red","purple"]
    lines = []
    for c in colors[:-1]:
        l, = ax_anim.plot([],[], '-', color=c, lw=2)
        lines.append(l)

    p0_point, = ax_anim.plot([],[], 'o', color='black', markersize=6)
    pA_point, = ax_anim.plot([],[], 'o', color='gray', markersize=6)
    p1_point, = ax_anim.plot([],[], 'o', color='blue', markersize=6)
    p2_point, = ax_anim.plot([],[], 'o', color='green', markersize=10, markerfacecolor='none')
    p3_point, = ax_anim.plot([],[], 'o', color='red', markersize=10, markerfacecolor='none')

    arc2_line, = ax_anim.plot([],[], color='lime', lw=2)
    arc3_line, = ax_anim.plot([],[], color='orange', lw=2)
    arrow = FancyArrowPatch((0,0),(0,0), color='tab:green', arrowstyle='->', lw=2)
    ax_anim.add_patch(arrow)
    traj_line, = ax_anim.plot([],[], 'm--', lw=1.5)
    cord_circle = Circle((Zc, Xc), radius=dc/2.0, facecolor='gray', alpha=0.45, edgecolor='k')
    ax_anim.add_patch(cord_circle)

    ax_theta.set_title("θ2, θ3 et θ_total")
    ax_theta.plot(t, theta2_signal, label='θ2', color='lime')
    ax_theta.plot(t, theta3_signal, label='θ3', color='orange')
    ax_theta.plot(t, theta_total, label='θ_total', color='purple')
    theta2_cursor, = ax_theta.plot([],[], 'ro')
    theta3_cursor, = ax_theta.plot([],[], 'bo')
    theta_total_cursor, = ax_theta.plot([],[], 'go')
    ax_theta.legend()
    ax_theta.grid(True)

    ax_d.set_title("d(t)")
    ax_d.plot(t, d_signal, color='tab:green')
    d_cursor, = ax_d.plot([],[], 'ro')
    ax_d.grid(True)

    ax_thd.set_title("θ_total(d)")
    ax_thd.plot(d_signal, theta_total, color='purple')
    thd_cursor, = ax_thd.plot([],[], 'ro')
    ax_thd.grid(True)

    # NEW: subplot pour dist_cc (traçage de la distance à la corde)
    fig2, ax_dist = plt.subplots(1,1, figsize=(8,3))
    ax_dist.plot(t, dist_cc_list, label='dist_cc = |z_cc|')
    ax_dist.hlines(dc/2.0, t[0], t[-1], colors='r', linestyles='--', label='rayon corde')
    ax_dist.set_xlabel('t')
    ax_dist.set_ylabel('dist_cc')
    ax_dist.legend()
    ax_dist.grid(True)
    plt.tight_layout()

    def update(i):
        fr = frames[i]
        pts = [fr["p0"], fr["pA"], fr["p1"], fr["p2"], fr["p3"], fr["p4"]]
        zs = [p[2] for p in pts]
        xs = [p[0] for p in pts]
        for k,l in enumerate(lines):
            l.set_data([zs[k], zs[k+1]], [xs[k], xs[k+1]])
        p0_point.set_data([zs[0]],[xs[0]])
        pA_point.set_data([zs[1]],[xs[1]])
        p1_point.set_data([zs[2]],[xs[2]])
        p2_point.set_data([zs[3]],[xs[3]])
        p3_point.set_data([zs[4]],[xs[4]])
        arc2_line.set_data(fr["arc2"][0], fr["arc2"][1])
        arc3_line.set_data(fr["arc3"][0], fr["arc3"][1])
        arrow.set_positions((fr["pA"][2], fr["pA"][0]), (fr["p1"][2], fr["p1"][0]))
        dist_pa_p1 = np.hypot(fr["p1"][0]-fr["pA"][0], fr["p1"][2]-fr["pA"][2])
        arrow.set_mutation_scale(10.0 + 3.0*dist_pa_p1)
        traj_zs = [f["p4"][2] for f in frames[:i+1]]
        traj_xs = [f["p4"][0] for f in frames[:i+1]]
        traj_line.set_data(traj_zs, traj_xs)
        theta2_cursor.set_data([t[i]],[fr["theta2"]])
        theta3_cursor.set_data([t[i]],[fr["theta3"]])
        theta_total_cursor.set_data([t[i]],[fr["theta2"]+fr["theta3"]])
        d_cursor.set_data([t[i]],[fr["d"]])
        thd_cursor.set_data([fr["d"]],[fr["theta2"]+fr["theta3"]])
        artists = []
        artists.extend(lines)
        artists.extend([p0_point, pA_point, p1_point, p2_point, p3_point])
        artists.extend([arc2_line, arc3_line, arrow, traj_line])
        artists.extend([theta2_cursor, theta3_cursor, theta_total_cursor, d_cursor, thd_cursor])
        return artists

    interval_effectif = frame_interval / max(1e-9, speed_factor)
    ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=interval_effectif, blit=True)
    plt.tight_layout()
    plt.show()
    return ani, frames, theta3_signal, theta_total, dist_cc_list

# ------------------------
# LANCEMENT
# ------------------------
if __name__ == "__main__":
    import sys
    # if called with the argument 'save', run headless and save the animation to a GIF
    save_mode = any(arg.lower() == 'save' for arg in sys.argv[1:])
    ani, frames, theta3_sig, theta_total_sig, dist_cc = animate_robot_full(show_plots=not save_mode)

    if save_mode:
        # Try to save as GIF using Pillow (matplotlib's PillowWriter). If Pillow is missing,
        # catch the error and print a helpful message.
        try:
            from matplotlib.animation import PillowWriter
            print('Saving animation to test_python24.gif ...')
            writer = PillowWriter(fps=25)
            ani.save('test_python24.gif', writer=writer)
            print('SAVED: test_python24.gif')
        except Exception as e:
            print('SAVE_ERROR:', type(e).__name__, e)
