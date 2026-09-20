# -*- coding: utf-8 -*-
"""
Robot-Guitar : θ3 corrigé pour tangence exacte au disque de la corde
- forward_kinematics calcule p0..p3
- compute_frames_coupled résout le couplage p4 <-> theta3
- arcs θ2/θ3, points creux P2/P3, traits pointillés, trajectoire P4
Auteur : Rivaldo
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import FancyArrowPatch, Circle

# ------------------------
# PARAMÈTRES
# ------------------------
A, B, C, D = 15.0, 3.0, 5.0, 3.0
d1_0 = 25.0
T = 10.0
theta0 = np.pi/6
delta = T/25.0

# animation / résolution
N = 400
dc = 4.0
frame_interval = 40.0
speed_factor = 1.0
eps_tang = 1e-3
max_iter_fixedpoint = 20
tol_theta3 = 1e-6

# ------------------------
# SIGNALS
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

# ------------------------
# CINÉMATIQUE DIRECTE (p0..p3)
# ------------------------
def forward_kinematics(theta2, d1):
    p0 = np.array([0.0, 0.0, 0.0])
    pA = np.array([A, 0.0, 0.0])
    p1 = pA + np.array([0.0, 0.0, d1])
    p2 = p1 + np.array([-B, 0.0, 0.0])
    p3 = p2 + np.array([-C*np.cos(theta2), 0.0, C*np.sin(theta2)])
    return p0, pA, p1, p2, p3

# ------------------------
# COUPLAGE p4 <-> theta3
# ------------------------
def compute_frames_coupled(t_vect, theta2_vect, d_vect, Zc, Xc, dc,
                           eps_tang=eps_tang,
                           max_iter=max_iter_fixedpoint, tol=tol_theta3):
    N = len(t_vect)
    frames = []
    theta3_vect = np.zeros(N)
    theta_total_vect = np.zeros(N)
    d_prime = np.gradient(d_vect)
    r_corde = dc/2.0
    Cxy = np.array([Zc, Xc])
    theta3_prev = 0.0

    for i in range(N):
        th2 = theta2_vect[i]
        dval = d_vect[i]
        th3_k = theta3_prev
        p0, pA, p1, p2, p3 = forward_kinematics(th2, dval)
        p3_xz = np.array([p3[2], p3[0]])
        sign_d = np.sign(d_prime[i]) if i < len(d_prime) else 0.0

        for k in range(max_iter):
            theta_total_k = th2 + th3_k
            p4 = p3 + np.array([-D*np.cos(theta_total_k), 0.0, D*np.sin(theta_total_k)])
            p4_xz = np.array([p4[2], p4[0]])
            v_p3p4 = p4_xz - p3_xz
            norm_v = np.linalg.norm(v_p3p4)
            if norm_v == 0.0:
                u = np.array([1.0, 0.0])
            else:
                u = v_p3p4 / norm_v
            n = np.array([-u[1], u[0]])
            z_cc = np.dot(Cxy - p3_xz, n)
            dist_cc = abs(z_cc)
            #approaching = (z_cc * sign_d < 0.0)
            approaching = (-1 < 0.0)
            theta3_candidate = 0.0

            if approaching:
                if abs(dist_cc - r_corde) <= 2*eps_tang:
                    p3C = Cxy - p3_xz
                    sgn = np.sign(z_cc) if z_cc != 0 else 1.0
                    # vecteur tangent : p3->p4 tangent au cercle
                    u_tangent = p3C - sgn*r_corde*n
                    u_tangent /= np.linalg.norm(u_tangent)
                    theta_total_candidate = np.arctan2(u_tangent[0], -u_tangent[1])
                    theta3_candidate = theta_total_candidate - th2

            if abs(theta3_candidate - th3_k) < tol:
                th3_k = theta3_candidate
                break
            th3_k = theta3_candidate

        theta3_i = th3_k
        theta_total_i = th2 + theta3_i
        p4 = p3 + np.array([-D*np.cos(theta_total_i), 0.0, D*np.sin(theta_total_i)])
        p4_xz = np.array([p4[2], p4[0]])

        # hors corde
        if (p4_xz[1] >= Xc + r_corde) or (p4_xz[1] <= Xc - r_corde):
            theta3_i = 0.0
            theta_total_i = th2
            p4 = p3 + np.array([-D*np.cos(theta_total_i), 0.0, D*np.sin(theta_total_i)])

        ang_from2, ang_to2, signed2 = angle_between_vectors_in_xz(p2 - p1, p3 - p2)
        ang_from3, ang_to3, signed3 = angle_between_vectors_in_xz(p3 - p2, p4 - p3)
        r2 = max(0.5, 0.1*vector_norm(p2)) * (0.5 + 0.5*dval/61.0)
        r3 = max(0.4, 0.08*vector_norm(p3)) * (0.5 + 0.5*dval/61.0)
        zs2, xs2 = arc_points(p2, ang_from2, ang_from2 + signed2, r2)
        zs3, xs3 = arc_points(p3, ang_from3, ang_from3 + signed3, r3)

        frames.append({
            "p0": p0, "pA": pA, "p1": p1, "p2": p2, "p3": p3, "p4": p4,
            "theta2": th2, "theta3": theta3_i, "theta_total": theta_total_i, "d": dval,
            "arc2": (zs2, xs2), "arc3": (zs3, xs3)
        })
        theta3_vect[i] = theta3_i
        theta_total_vect[i] = theta_total_i
        theta3_prev = theta3_i

    return frames, theta3_vect, theta_total_vect

# ------------------------
# ANIMATION
# ------------------------
def animate_robot_full():
    t = np.linspace(0.0, T, N)
    d_signal = d_periodique(t, T, d1_0, delta)
    theta2_signal, d_prime = theta2_en_fonction_de_d(d_signal, theta0)

    zmin = -(C + D) * np.cos(theta0)
    zmax = d1_0 + (C + D) * np.cos(theta0)
    Zc = (zmax + zmin) / 2.0
    Xc = A - B - (C + D) * np.cos(theta0)

    frames, theta3_signal, theta_total = compute_frames_coupled(
        t, theta2_signal, d_signal, Zc, Xc, dc
    )

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
    return ani

# ------------------------
# LANCEMENT
# ------------------------
if __name__ == "__main__":
    ani = animate_robot_full()
