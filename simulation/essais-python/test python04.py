# -*- coding: utf-8 -*-
"""
Animation complète robot-Guitar avec DH, points dynamiques, arcs θ2/θ3, flèche d, légende et trajectoire finale.
Inclut quatre figures : animation, θ2/θ3/θ_total, d(t), θ_total(d)
Auteur : Rivaldo
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import FancyArrowPatch

# ------------------------
# CONSTANTES ROBOT (DH)
# ------------------------
A, B, C, D = 15, 3, 5 , 3
d1_0 = 25  # position initiale prismatique
T = 10    # période du mouvement
theta0 = 1*np.pi/6  # amplitude theta2
sigma = np.pi/8   # amplitude theta3
tau = T/50           # durée montée/descente
delta= T/25 

# ------------------------
# SIGNALS PÉRIODIQUES
# ------------------------
def theta2_periodique(t, T, theta0, delta):
    theta = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if phase <= T/2 - delta:
            theta[i] = -theta0
        elif phase <= T/2:
            theta[i] = -theta0 + 2*theta0*(phase - (T/2 - delta))/delta
        elif phase <= T - delta:
            theta[i] = theta0
        else:
            theta[i] = theta0 - 2*theta0*(phase - (T - delta))/delta
    return theta

def theta3_triangle(t, T, sigma, tau):
    """
    θ3 orienté vers le bas :
    - décroissance sinusoïdale centrée sur T/4 et 3T/4
    - valeur 0 en dehors, retour brusque
    - tau : largeur de la "sinusoïde"
    """
    theta3 = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        
        # zone autour de T/4 (premier pic)
        if abs(phase - T/4) <= tau/2:
            # sinus orienté vers le bas : -sigma * cos(0→pi)
            theta3[i] = -sigma * (np.cos(np.pi * (phase - (T/4))/tau) + 1)/2
        
        # zone autour de 3T/4 (second pic)
        elif abs(phase - 3*T/4) <= tau/2:
            # sinus orienté vers le haut : sigma * cos(0→pi)
            theta3[i] = sigma * (np.cos(np.pi * (phase - (3*T/4))/tau) + 1)/2
        
        else:
            theta3[i] = 0  # retour brusque
    return theta3


def d_periodique(t, T, dmax, delta):
    d = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if phase <= T/2-delta:
            d[i] = dmax*phase/(T/2-delta)
        elif phase <= T/2 :  d[i] = dmax  
        elif phase <= T-delta: d[i] = dmax*(-phase/(T/2-delta) + (T-delta)/(T/2-delta))
        else:
            d[i] = 0
    return d

# ------------------------
# PLAN X-Z, vecteurs et angles
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
# CINÉMATIQUE DIRECTE (DH)
# ------------------------
def forward_kinematics(theta_tuple, d1, A=A, B=B, C=C, D=D):
    theta2, theta3 = theta_tuple
    # P0
    p0 = np.array([0,0,0])
    # P_A
    pA = np.array([A,0,0])
    # P1
    p1 = pA + np.array([0,0,d1])
    # P2
    p2 = p1 + np.array([-B,0,0])
    # P3
    p3 = p2 + np.array([-C*np.cos(theta2),0, C*np.sin(theta2)])
    # P4 (effecteur)
    p4 = p3 + np.array([-D*np.cos(theta2+theta3),0, D*np.sin(theta2+theta3)])
    return p0, pA, p1, p2, p3, p4

# ------------------------
# FRAMES POUR ANIMATION
# ------------------------
def compute_frames(t_vect, theta2_vect, theta3_vect, d_vect):
    frames = []
    for i in range(len(t_vect)):
        th2 = theta2_vect[i]
        th3 = theta3_vect[i]
        dval = d_vect[i]
        p0, pA, p1, p2, p3, p4 = forward_kinematics((th2, th3), dval)
        # vecteurs pour arcs
        v12 = p2 - p1
        v23 = p3 - p2
        v34 = p4 - p3
        # arcs θ2 et θ3
        ang_from2, ang_to2, signed2 = angle_between_vectors_in_xz(v12, v23)
        ang_from3, ang_to3, signed3 = angle_between_vectors_in_xz(v23, v34)
        r2 = max(0.5, 0.1*vector_norm(p2)) * (0.5 + 0.5*dval/61)
        r3 = max(0.4, 0.08*vector_norm(p3)) * (0.5 + 0.5*dval/61)
        zs2, xs2 = arc_points(p2, ang_from2, ang_from2+signed2, r2)
        zs3, xs3 = arc_points(p3, ang_from3, ang_from3+signed3, r3)
        frames.append({
            "p0":p0, "pA":pA, "p1":p1, "p2":p2, "p3":p3, "p4":p4,
            "theta2":th2, "theta3":th3, "d":dval,
            "arc2":(zs2, xs2), "arc3":(zs3, xs3)
        })
    return frames

# ------------------------
# ANIMATION COMPLETE
# ------------------------
def animate_robot_full():
    # temps
    t = np.linspace(0, T, 300)
    theta2_signal = theta2_periodique(t, T, theta0, delta)
    theta3_signal = theta3_triangle(t, T, sigma, tau)
    d_signal = d_periodique(t, T, d1_0, delta)
    frames = compute_frames(t, theta2_signal, theta3_signal, d_signal)

    # figure 2x2
    fig, ((ax_anim, ax_theta),(ax_d, ax_thd)) = plt.subplots(2,2, figsize=(15,12))

    # ------------------------
    # ANIMATION Z-X
    # ------------------------
    ax_anim.set_title("Robot-Guitar animation Z-X")
    ax_anim.set_xlabel("Z")
    ax_anim.set_ylabel("X")
    ax_anim.grid(True)

    # marges parfaites
    all_z = np.concatenate([np.array([f["p0"][2],f["pA"][2],f["p1"][2],f["p2"][2],f["p3"][2],f["p4"][2]]) for f in frames])
    all_x = np.concatenate([np.array([f["p0"][0],f["pA"][0],f["p1"][0],f["p2"][0],f["p3"][0],f["p4"][0]]) for f in frames])
    margin = max(1.0, 0.25*max(np.max(all_z)-np.min(all_z), np.max(all_x)-np.min(all_x)))
    ax_anim.set_xlim(np.min(all_z)-margin, np.max(all_z)+margin)
    ax_anim.set_ylim(np.min(all_x)-margin, np.max(all_x)+margin)

    # couleurs
    colors = ["black","gray","blue","green","red","purple"]
    lines = []
    for c in colors[:-1]:
        l, = ax_anim.plot([],[], '-', color=c, lw=2, label=f'{c} segment')
        lines.append(l)
    points = []
    for c in colors:
        p, = ax_anim.plot([],[], 'o', color=c, markersize=8, label=f'{c} point')
        points.append(p)

    # arcs
    arc2_line, = ax_anim.plot([],[], color='lime', lw=2, label='θ2 arc')
    arc3_line, = ax_anim.plot([],[], color='orange', lw=2, label='θ3 arc')

    # flèche PA->P1
    arrow = FancyArrowPatch((0,0),(0,0), color='tab:green', arrowstyle='->', lw=2)
    ax_anim.add_patch(arrow)

    # trajectoire finale
    traj_line, = ax_anim.plot([],[], 'm--', lw=1.5, label="Trajectoire final PE")

    ax_anim.legend(loc='upper left')

    # ------------------------
    # θ2/θ3/θ_total
    # ------------------------
    theta_total = theta2_signal + theta3_signal
    ax_theta.set_title("θ2, θ3 et θ_total")
    ax_theta.plot(t, theta2_signal, label='θ2', color='lime')
    ax_theta.plot(t, theta3_signal, label='θ3', color='orange')
    ax_theta.plot(t, theta_total, label='θ_total', color='purple')
    theta2_cursor, = ax_theta.plot([],[], 'ro')
    theta3_cursor, = ax_theta.plot([],[], 'bo')
    theta_total_cursor, = ax_theta.plot([],[], 'go')
    ax_theta.legend()
    ax_theta.grid(True)

    # ------------------------
    # d(t)
    # ------------------------
    ax_d.set_title("d(t)")
    ax_d.plot(t, d_signal, color='tab:green')
    d_cursor, = ax_d.plot([],[], 'ro')
    ax_d.grid(True)

    # ------------------------
    # θ_total(d)
    # ------------------------
    ax_thd.set_title("θ_total(d)")
    ax_thd.plot(d_signal, theta_total, color='purple')
    thd_cursor, = ax_thd.plot([],[], 'ro')
    ax_thd.grid(True)

    # ------------------------
    # UPDATE
    # ------------------------
    def update(i):
        fr = frames[i]
        pts = [fr["p0"], fr["pA"], fr["p1"], fr["p2"], fr["p3"], fr["p4"]]
        zs = [p[2] for p in pts]
        xs = [p[0] for p in pts]
        # lignes
        for k,l in enumerate(lines):
            l.set_data([zs[k], zs[k+1]], [xs[k], xs[k+1]])
        # points
        for k,p in enumerate(points):
            p.set_data([zs[k]], [xs[k]])
        # arcs
        arc2_line.set_data(fr["arc2"][0], fr["arc2"][1])
        arc3_line.set_data(fr["arc3"][0], fr["arc3"][1])
        # flèche PA->P1
        arrow.set_positions((fr["pA"][2], fr["pA"][0]), (fr["p1"][2], fr["p1"][0]))
        # trajectoire finale
        traj_zs = [f["p4"][2] for f in frames[:i+1]]
        traj_xs = [f["p4"][0] for f in frames[:i+1]]
        traj_line.set_data(traj_zs, traj_xs)
        # cursors
        theta2_cursor.set_data([t[i]],[fr["theta2"]])
        theta3_cursor.set_data([t[i]],[fr["theta3"]])
        theta_total_cursor.set_data([t[i]],[fr["theta2"]+fr["theta3"]])
        d_cursor.set_data([t[i]],[fr["d"]])
        thd_cursor.set_data([fr["d"]],[fr["theta2"]+fr["theta3"]])
        return lines+points+[arc2_line, arc3_line, arrow, traj_line, theta2_cursor, theta3_cursor, theta_total_cursor, d_cursor, thd_cursor]

    ani = animation.FuncAnimation(fig, update, frames=len(t), interval=40, blit=True)
    plt.tight_layout()
    plt.show()
    return ani

# ------------------------
# LANCEMENT
# ------------------------
if __name__=="__main__":
    ani = animate_robot_full()
