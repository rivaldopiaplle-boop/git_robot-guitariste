# -*- coding: utf-8 -*-
"""
Robot-Guitar : simulation complète sans raccourci
- forward_kinematics calcule p0..p3
- θ2 calculée depuis d(t) et sa dérivée (sign(d'))
- θ3, θ_total et p4 calculés dans la même boucle (séquentiel)
- Approche : z_cc * d' < 0
- Tangence : |dist_cc - r_corde| <= eps_tang (et approche)
- Relâchement : z_p4 == Zc (comparaison approchée)
- P2/P3 cercles creux ; P4 sans marque ; trajectoire P4 toujours visible
- Flèche PA->P1, queue en PA, tête en P1, tête élastique (mutation_scale)
- Deux lignes pointillées verticales depuis P2 (C/2) et P3 (D/2)
Auteur : adaptation pour Rivaldo
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import FancyArrowPatch, Circle

# ------------------------
# CONSTANTES (garder tes notations)
# ------------------------
A, B, C, D = 15.0, 3.0, 5.0, 3.0   # géométrie DH (A,B,C,D)
d1_0 = 25.0                        # amplitude prismatique (exemple)
T = 10.0
theta0 = 1.0 * np.pi / 6.0
sigma = np.pi/8.0
tau = T/50.0
delta = T/25.0

# Réglages d'usage / animation
N = 400                 # points temporels
dc = 4.0                # diamètre du disque représentant la corde
frame_interval = 40.0   # base interval (ms)
speed_factor = 1.0      # vitesse (interval_effectif = frame_interval / speed_factor)

# Tolérance tangence (petit)
eps_tang = 1e-3

# ------------------------
# Signaux (d(t), puis θ2(d))
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
    """
    Calcule theta2(t) en fonction de d(t) et de signe(d').
    Algorithme demandé :
     - si signe(d')>0 => θ2 = -θ0
     - si signe(d')<0 => θ2 = θ0
     - sinon (signe≈0) :
         * si d≈0 => θ2 décroît linéairement de θ0 à -θ0 (sur le segment)
         * sinon => θ2 croît linéairement de -θ0 à θ0 (sur le segment)
    """
    Ntot = len(d_vect)
    if Ntot == 0:
        return np.array([]), np.array([])

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
        kernel = np.ones(smooth_window)/float(smooth_window)
        theta2 = np.convolve(theta2, kernel, mode='same')

    return theta2, d_prime

# ------------------------
# UTILITAIRES plan X-Z
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

# ------------------------
# CINÉMATIQUE DIRECTE (p0..p3)
# ------------------------
def forward_kinematics(theta2, d1, A=A, B=B, C=C, D=D):
    """
    Calcule p0, pA, p1, p2, p3 (NE CALCULE PAS p4).
    On reprend ta convention originale (vecteurs [x,y,z]).
    """
    p0 = np.array([0.0, 0.0, 0.0])
    pA = np.array([A, 0.0, 0.0])
    p1 = pA + np.array([0.0, 0.0, d1])
    p2 = p1 + np.array([-B, 0.0, 0.0])
    p3 = p2 + np.array([-C*np.cos(theta2), 0.0, C*np.sin(theta2)])
    return p0, pA, p1, p2, p3

# ------------------------
# FONCTION PRINCIPALE : θ3, θ_total et p4 dans la même boucle (séquentiel)
# ------------------------
def compute_frames_coupled(t_vect, theta2_vect, d_vect, Zc, Xc, dc,
                           eps_tang=1e-3):
    """
    Pour chaque instant i :
     - p0..p3 via forward_kinematics(theta2_i, d_i)
     - utiliser theta3_prev (initialement 0) pour calculer p4_temp
     - détecter approche : z_cc * d' < 0
     - si approche et tangence (|dist_cc - r| <= eps_tang) alors
         calculer theta3 via la géométrie (term1 + term2)
       sinon si relâchement (z_p4 ≈ Zc) => theta3 = 0
       sinon theta3 = 0
     - recalculer p4 final avec θ3 trouvé
    Retour : frames (liste dict), theta3_vect, theta_total_vect
    """
    N = len(t_vect)
    frames = []
    theta3_vect = np.zeros(N)
    theta_total_vect = np.zeros(N)

    # dérivée d' pour signe (aller/retour)
    d_prime = np.gradient(d_vect)

    r_corde = dc / 2.0
    Cxy = np.array([Zc, Xc])

    theta3_prev = 0.0

    for i in range(N):
        th2 = theta2_vect[i]
        dval = d_vect[i]
        th3_i = theta3_prev  # valeur de départ (continuité temporelle)

        # 1) calcul p0..p3 (bras) avec forward_kinematics
        p0, pA, p1, p2, p3 = forward_kinematics(th2, dval)
        # projection Z-X pour calculs géométriques (Z -> index 2, X -> index 0)
        p3_xz = np.array([p3[2], p3[0]])  # [z, x]

        # 2) calcul initial p4_temp avec theta3_prev (temporaire)
        theta_total_temp = th2 + th3_i
        p4_temp = p3 + np.array([-D*np.cos(theta_total_temp), 0.0, D*np.sin(theta_total_temp)])
        p4_xz_temp = np.array([p4_temp[2], p4_temp[0]])

        # 3) vecteur unitaire u et normale n sur la droite p3->p4_temp (plan Z-X)
        v_p3p4 = p4_xz_temp - p3_xz
        norm_v = np.linalg.norm(v_p3p4)
        # norm_v devrait être proche de D > 0 ; on assume non nul
        if norm_v < 1e-12:
            u = np.array([1.0, 0.0])
        else:
            u = v_p3p4 / norm_v
        n = np.array([-u[1], u[0]])

        # 4) distance signée corde -> médiator (z_cc) et dist_cc
        z_cc = np.dot(Cxy - p3_xz, n)   # signe relatif (sert pour approche)
        dist_cc = abs(z_cc)

        # 5) signe de d' pour approche detection (aller/retour)
        sign_d = np.sign(d_prime[i]) if i < len(d_prime) else 0.0
        approaching = (z_cc * sign_d < 0.0)  # True si on s'approche (aller ou retour)
        approaching =(0<D)
        # 6) relâchement check : si tête temp atteint Zc (comparaison approchée)
        #    condition demandée : zp4 == Zc -> relâchement
        if (0>D):#abs(p4_xz_temp[0] - Zc) <= eps_tang:
            th3_i = 0.0

        # 7) tangence check (approach & close to radius) -> calcul géométrique de theta3
        elif approaching and abs(dist_cc - r_corde) <= eps_tang:
            # projection scalaire de (C - p3) sur u
            proj_len = np.dot(Cxy - p3_xz, u)
            p3c_proj = proj_len * u               # vect p3 -> C'
            p3c_perp = (Cxy - p3_xz) - p3c_proj   # CC' perpendiculaire
            norm_proj = np.linalg.norm(p3c_proj)
            norm_perp = np.linalg.norm(p3c_perp)

            # term1 = arctan( ||P3C'|| / ||CC'|| )
            # use arctan2 for robustness: arctan2(norm_proj, norm_perp)
            term1 = np.arctan2(norm_proj, norm_perp + 1e-12)

            # term2 = atan2(z(P3C), x(P3C))
            p3c = Cxy - p3_xz
            term2 = np.arctan2(p3c[1], p3c[0])

            theta_total_geom = term1 + term2
            th3_i = theta_total_geom - th2
        """
        else:
            # pas de contact / pas d'approche → θ3 = 0
            th3_i = 0.0
        """
        # 8) recalculer p4 final avec θ3 définitif
        theta_total = th2 + th3_i
        p4 = p3 + np.array([-D*np.cos(theta_total), 0.0, D*np.sin(theta_total)])
        p4_xz = np.array([p4[2], p4[0]])

        # 9) contrainte "toujours dessus" : si p4 sort en X hors intervalle corde -> relâche
        #    (X est coordonnée p4_xz[1])
        """
        if (p4_xz[1] >= Xc + r_corde) or (p4_xz[1] <= Xc - r_corde):
            th3_i = 0.0
            theta_total = th2
            p4 = p3 + np.array([-D*np.cos(theta_total), 0.0, D*np.sin(theta_total)])
            p4_xz = np.array([p4[2], p4[0]])
        """
        # 10) arcs pour affichage (θ2 et θ3 arcs) : on peut calculer
        ang_from2, ang_to2, signed2 = angle_between_vectors_in_xz(p2 - p1, p3 - p2)
        ang_from3, ang_to3, signed3 = angle_between_vectors_in_xz(p3 - p2, p4 - p3)
        r2 = max(0.5, 0.1*vector_norm(p2)) * (0.5 + 0.5*dval/61.0)
        r3 = max(0.4, 0.08*vector_norm(p3)) * (0.5 + 0.5*dval/61.0)
        zs2, xs2 = arc_points(p2, ang_from2, ang_from2 + signed2, r2)
        zs3, xs3 = arc_points(p3, ang_from3, ang_from3 + signed3, r3)

        # 11) stocker frame
        frames.append({
            "p0": p0, "pA": pA, "p1": p1, "p2": p2, "p3": p3, "p4": p4,
            "theta2": th2, "theta3": th3_i, "theta_total": theta_total, "d": dval,
            "arc2": (zs2, xs2), "arc3": (zs3, xs3)
        })
        
        theta3_vect[i] = th3_i
        theta_total_vect[i] = theta_total
        theta3_prev = th3_i

    return frames, theta3_vect, theta_total_vect

# ------------------------
# ANIMATION COMPLETE
# ------------------------
def animate_robot_full():
    # temps et signaux
    t = np.linspace(0.0, T, N)
    d_signal = d_periodique(t, T, d1_0, delta)
    theta2_signal, d_prime = theta2_en_fonction_de_d(d_signal, theta0, eps=1e-6, smooth_window=3)

    # définir centre corde Zc, Xc (exemple calculé à partir de géométrie)
    zmin = -(C + D) * np.cos(theta0)
    zmax = d1_0 + (C + D) * np.cos(theta0)
    Zc = (zmax + zmin) / 2.0
    Xc = A - B - (C + D) * np.cos(theta0)

    frames, theta3_signal, theta_total = compute_frames_coupled(t, theta2_signal, d_signal, Zc, Xc, dc, eps_tang=eps_tang)

    # figure 2x2
    fig, ((ax_anim, ax_theta), (ax_d, ax_thd)) = plt.subplots(2,2, figsize=(15,12))

    # ---- axe animation Z-X ----
    ax_anim.set_title("Robot-Guitar animation Z-X")
    ax_anim.set_xlabel("Z")
    ax_anim.set_ylabel("X")
    ax_anim.grid(True)

    all_z = np.concatenate([np.array([f["p0"][2],f["pA"][2],f["p1"][2],f["p2"][2],f["p3"][2],f["p4"][2]]) for f in frames])
    all_x = np.concatenate([np.array([f["p0"][0],f["pA"][0],f["p1"][0],f["p2"][0],f["p3"][0],f["p4"][0]]) for f in frames])
    margin = max(1.0, 0.25*max(np.max(all_z)-np.min(all_z), np.max(all_x)-np.min(all_x)))
    ax_anim.set_xlim(np.min(all_z)-margin, np.max(all_z)+margin)
    ax_anim.set_ylim(np.min(all_x)-margin, np.max(all_x)+margin)

    # lignes segments
    colors = ["black","gray","blue","green","red","purple"]
    lines = []
    for c in colors[:-1]:
        l, = ax_anim.plot([],[], '-', color=c, lw=2)
        lines.append(l)

    # points : P0 PA P1 P2(creux) P3(creux)
    p0_point, = ax_anim.plot([],[], 'o', color='black', markersize=6, label='P0')
    pA_point, = ax_anim.plot([],[], 'o', color='gray', markersize=6, label='PA')
    p1_point, = ax_anim.plot([],[], 'o', color='blue', markersize=6, label='P1')
    p2_point, = ax_anim.plot([],[], 'o', color='green', markersize=10, markerfacecolor='none', label='P2 (creux)')
    p3_point, = ax_anim.plot([],[], 'o', color='red', markersize=10, markerfacecolor='none', label='P3 (creux)')

    # arcs
    arc2_line, = ax_anim.plot([],[], color='lime', lw=2)
    arc3_line, = ax_anim.plot([],[], color='orange', lw=2)

    # flèche PA->P1 (queue en PA, tête en P1)
    arrow = FancyArrowPatch((0,0),(0,0), color='tab:green', arrowstyle='->', lw=2)
    ax_anim.add_patch(arrow)

    # trajectoire P4 (toujours visible)
    traj_line, = ax_anim.plot([],[], 'm--', lw=1.5, label='Trajectoire final P4')

    # disque plein corde
    cord_circle = Circle((Zc, Xc), radius=dc/2.0, facecolor='gray', alpha=0.45, edgecolor='k')
    ax_anim.add_patch(cord_circle)

    ax_anim.legend(loc='upper left')

    # ---- θ2/θ3/θ_total subplot ----
    ax_theta.set_title("θ2, θ3 et θ_total")
    ax_theta.plot(t, theta2_signal, label='θ2', color='lime')
    ax_theta.plot(t, theta3_signal, label='θ3', color='orange')
    ax_theta.plot(t, theta_total, label='θ_total', color='purple')
    theta2_cursor, = ax_theta.plot([],[], 'ro')
    theta3_cursor, = ax_theta.plot([],[], 'bo')
    theta_total_cursor, = ax_theta.plot([],[], 'go')
    ax_theta.legend()
    ax_theta.grid(True)

    # ---- d(t) subplot ----
    ax_d.set_title("d(t)")
    ax_d.plot(t, d_signal, color='tab:green')
    d_cursor, = ax_d.plot([],[], 'ro')
    ax_d.grid(True)

    # ---- θ_total(d) subplot ----
    ax_thd.set_title("θ_total(d)")
    ax_thd.plot(d_signal, theta_total, color='purple')
    thd_cursor, = ax_thd.plot([],[], 'ro')
    ax_thd.grid(True)

    # dashed vertical lines artist placeholders
    p2_vline, = ax_anim.plot([], [], linestyle='--', linewidth=1, color='black')
    p3_vline, = ax_anim.plot([], [], linestyle='--', linewidth=1, color='black')

    # update function
    def update(i):
        fr = frames[i]
        pts = [fr["p0"], fr["pA"], fr["p1"], fr["p2"], fr["p3"], fr["p4"]]
        zs = [p[2] for p in pts]
        xs = [p[0] for p in pts]

        # segments
        for k,l in enumerate(lines):
            l.set_data([zs[k], zs[k+1]], [xs[k], xs[k+1]])

        # points
        p0_point.set_data([zs[0]],[xs[0]])
        pA_point.set_data([zs[1]],[xs[1]])
        p1_point.set_data([zs[2]],[xs[2]])
        p2_point.set_data([zs[3]],[xs[3]])
        p3_point.set_data([zs[4]],[xs[4]])
        # p4 not plotted as point

        # arcs
        arc2_line.set_data(fr["arc2"][0], fr["arc2"][1])
        arc3_line.set_data(fr["arc3"][0], fr["arc3"][1])

        # arrow PA->P1
        pa_pos = (fr["pA"][2], fr["pA"][0])
        p1_pos = (fr["p1"][2], fr["p1"][0])
        arrow.set_positions(pa_pos, p1_pos)

        # elastic head scale
        dist_pa_p1 = np.hypot(p1_pos[0]-pa_pos[0], p1_pos[1]-pa_pos[1])
        base_scale = 10.0
        gain = 3.0
        mutation_scale = base_scale + gain * dist_pa_p1
        try:
            arrow.set_mutation_scale(mutation_scale)
        except Exception:
            pass

        # trajectory P4
        traj_zs = [f["p4"][2] for f in frames[:i+1]]
        traj_xs = [f["p4"][0] for f in frames[:i+1]]
        traj_line.set_data(traj_zs, traj_xs)

        # vertical dashed lines from p2 and p3 downward: length C/2 and D/2
        p2 = fr["p2"]
        p3 = fr["p3"]
        # each line is vertical in X (downwards): same Z coordinate, X decreasing
        z_p2 = p2[2]; x_p2 = p2[0]
        z_p3 = p3[2]; x_p3 = p3[0]
        line_p2_z = [z_p2, z_p2]
        line_p2_x = [x_p2, x_p2 - C/2.0]
        line_p3_z = [z_p3, z_p3]
        line_p3_x = [x_p3, x_p3 - D/2.0]
        p2_vline.set_data(line_p2_z, line_p2_x)
        p3_vline.set_data(line_p3_z, line_p3_x)

        # cursors
        theta2_cursor.set_data([t[i]],[fr["theta2"]])
        theta3_cursor.set_data([t[i]],[fr["theta3"]])
        theta_total_cursor.set_data([t[i]],[fr["theta2"]+fr["theta3"]])
        d_cursor.set_data([t[i]],[fr["d"]])
        thd_cursor.set_data([fr["d"]],[fr["theta2"]+fr["theta3"]])

        artists = []
        artists.extend(lines)
        artists.extend([p0_point, pA_point, p1_point, p2_point, p3_point])
        artists.extend([arc2_line, arc3_line, arrow, traj_line, p2_vline, p3_vline])
        artists.extend([theta2_cursor, theta3_cursor, theta_total_cursor, d_cursor, thd_cursor])
        return artists

    interval_effectif = frame_interval / max(1e-9, speed_factor)
    ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=interval_effectif, blit=True)
    plt.tight_layout()
    plt.show()
    return ani

# ------------------------
# MAIN
# ------------------------
if __name__ == "__main__":
    ani = animate_robot_full()
