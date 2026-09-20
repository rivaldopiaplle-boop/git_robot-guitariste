"""
Robot-Guitar animation (Python)
Auteur : Rivaldo
Langue : français

Ce programme montre un robot très simplifié (comme un médiator de guitare qui bouge).
On calcule sa cinématique directe avec la convention DH modifiée (Matrice DH),
puis on génère des animations pour voir comment il bouge dans le plan (x,z).
"""

# -------------------------------------------------
# Import des librairies nécessaires
# -------------------------------------------------

import numpy as np                 # numpy sert à faire des maths (sinus, cosinus, etc.)
import matplotlib.pyplot as plt    # matplotlib sert à tracer des graphiques et des figures
from matplotlib import animation   # ce module de matplotlib permet de créer des animations


# -------------------------------------------------
# Fonction utilitaire : Matrice DH modifiée
# -------------------------------------------------

def mdh_transform(alpha, a, theta, d):
    """
    Crée une matrice de transformation homogène 4x4 en utilisant la convention DH modifiée.
    """
    ca, sa = np.cos(alpha), np.sin(alpha)   # cos et sin de alpha
    ct, st = np.cos(theta), np.sin(theta)   # cos et sin de theta

    # Matrice de rotation autour de x + translation en x
    RxTx = np.array([
        [1.0, 0.0, 0.0, a],
        [0.0, ca, -sa, 0.0],
        [0.0, sa, ca, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Matrice de rotation autour de z + translation en z
    RzTz = np.array([
        [ct, -st, 0.0, 0.0],
        [st,  ct, 0.0, 0.0],
        [0.0, 0.0, 1.0, d],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Produit matriciel (ordre MDH : RxTx puis RzTz)
    T = RxTx @ RzTz
    return T


# -------------------------------------------------
# Fonction : Cinématique directe via DH
# -------------------------------------------------

def forward_kinematics(theta, d, A=1.0, B=2.0):
    """
    Calcule les positions des points importants du robot à partir des matrices DH.
    """
    # Base du robot (origine)
    p0 = np.array([0.0, 0.0, 0.0])

    # Étape 1 : Transformation de la base vers le 1er repère
    T1 = mdh_transform(0.0, A, 0.0, d)

    # Étape 2 : Rotation autour de z par theta
    T2 = mdh_transform(-np.pi/2, 0.0, theta, 0.0)

    # Étape 3 : Translation de longueur B
    T3 = mdh_transform(0.0, B, 0.0, 0.0)

    # Matrice totale
    T_full = T1 @ T2 @ T3

    # Point final
    p3_h = T_full @ np.array([0, 0, 0, 1])
    p3 = p3_h[:3]

    # Point intermédiaire (après T1)
    p1_h = T1 @ np.array([0, 0, 0, 1])
    p1 = p1_h[:3]

    return p0, p1, p3


# -------------------------------------------------
# Générateur de mouvement périodique pour d(t)
# -------------------------------------------------

def d_periodique(t, T, D):
    d = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if phase <= T/2:   # montée
            d[i] = 2*D*phase/T
        else:              # descente
            d[i] = 2*D*(-phase/T + 1)
    return d


# -------------------------------------------------
# Générateur de mouvement périodique pour theta(t)
# -------------------------------------------------

def theta_periodique(t, T, theta0, sigma, tau):
    theta = np.zeros_like(t)
    for i, ti in enumerate(t): 
        phase = ti % T
        if (0 <= phase <= 3*T/8-T/32- tau) or (3*T/8-T/32 + tau <= phase <= T/2 - tau):
            theta[i] = theta0
        elif (3*T/8-T/32 - tau <= phase <= 3*T/8-T/32 + tau):
            theta[i] = theta0 + sigma
        elif (T/2 < phase <= 7*T/8-T/32 - tau) or (7*T/8-T/32 + tau <= phase <= T - tau):
            theta[i] = -theta0
        elif (7*T/8 -T/32- tau <= phase <= 7*T/8-T/32 + tau):
            theta[i] = -(theta0 + sigma)
        elif (T/2 - tau <= phase <= T/2):
            theta[i] = theta0*(-2*phase/tau + T/tau - 1)
        elif (T - tau < phase <= T):
            theta[i] = theta0*(2*phase/tau - 2*T/tau + 1)
    return theta


# -------------------------------------------------
# Animation principale
# -------------------------------------------------

def animate_periodic(theta, d, A=1.0, B=2.0, duration=10.0, fps=40, speed=1.0):
    """
    Crée une animation montrant :
      - le robot dans le plan (z-x)
      - un point rectiligne
      - θ(t), d(t)
      - la relation θ(d)

    Paramètre speed : contrôle la vitesse (1.0 normal, >1 plus rapide, <1 plus lent).
    """
    T = theta.size
    t = np.linspace(0, duration, T)

    # Coordonnées effecteur
    x = np.zeros_like(t)
    z = np.zeros_like(t)
    for i in range(T):
        _, _, p3 = forward_kinematics(theta[i], d[i], A, B)
        x[i], z[i] = p3[0], p3[2]

    # Coordonnées rectilignes
    x_lin = np.full_like(d, A)
    z_lin = d

    # Figure avec 4 sous-graphes
    fig, ((ax_anim, ax_theta), (ax_d, ax_thd)) = plt.subplots(2, 2, figsize=(12, 10))

    # --- Plan z-x
    ax_anim.set_title("Trajectoires dans le plan (z-x)")
    ax_anim.set_xlabel("z")
    ax_anim.set_ylabel("x")
    ax_anim.set_aspect("equal")
    ax_anim.grid(True)
    ax_anim.plot(z, x, 'k--', alpha=0.9, label="Trajectoire complexe")
    ax_anim.plot(z_lin, x_lin, 'r--', alpha=0.9, label="Trajectoire rectiligne")
    ax_anim.legend(loc="upper right")

    z_min, z_max = min(z.min(), z_lin.min()), max(z.max(), z_lin.max())
    x_min, x_max = min(x.min(), x_lin.min()), max(x.max(), x_lin.max())
    margin = 0.2 * max(z_max - z_min, x_max - x_min)
    ax_anim.set_xlim(z_min - margin, z_max + margin)
    ax_anim.set_ylim(x_min - margin, x_max + margin)

    # objets dynamiques
    line_robot, = ax_anim.plot([], [], 'o-', lw=2, color="blue", alpha=0.6)
    eff_marker, = ax_anim.plot([], [], 'bs', ms=8)
    lin_marker, = ax_anim.plot([], [], 'ro', ms=6)

    # --- theta(t)
    ax_theta.set_title("θ(t)")
    ax_theta.plot(t, theta, 'b')
    theta_cursor, = ax_theta.plot([], [], 'ro')

    # --- d(t)
    ax_d.set_title("d(t)")
    ax_d.plot(t, d, 'g')
    d_cursor, = ax_d.plot([], [], 'ro')

    # --- theta(d)
    ax_thd.set_title("Relation θ(d)")
    ax_thd.set_xlabel("d")
    ax_thd.set_ylabel("θ")
    ax_thd.plot(d, theta, 'm')
    thd_cursor, = ax_thd.plot([], [], 'ro')

    # init
    def init():
        line_robot.set_data([], [])
        eff_marker.set_data([], [])
        lin_marker.set_data([], [])
        theta_cursor.set_data([], [])
        d_cursor.set_data([], [])
        thd_cursor.set_data([], [])
        return line_robot, eff_marker, lin_marker, theta_cursor, d_cursor, thd_cursor

    # update
    def update(i):
        p0, p1, p3 = forward_kinematics(theta[i], d[i], A, B)
        line_robot.set_data([p0[2], p1[2], p3[2]], [p0[0], p1[0], p3[0]])
        eff_marker.set_data(p3[2], p3[0])
        lin_marker.set_data(z_lin[i], x_lin[i])
        theta_cursor.set_data(t[i], theta[i])
        d_cursor.set_data(t[i], d[i])
        thd_cursor.set_data(d[i], theta[i])
        return line_robot, eff_marker, lin_marker, theta_cursor, d_cursor, thd_cursor

    # animation (ajout du facteur speed)
    frames = T
    interval = (1000.0 * duration / frames) / speed
    ani = animation.FuncAnimation(fig, update, frames=frames, init_func=init,
                                  blit=True, interval=interval)

    plt.tight_layout()
    plt.show()
    return ani


# -------------------------------------------------
# Exemple d’utilisation
# -------------------------------------------------
if __name__ == "__main__":
    A = 1.0
    B = 3.0
    duration = 10.0
    T = 600
    t = np.linspace(0, duration, T)

    D = 10
    theta0 = np.pi/5
    sigma = np.pi/15
    tau = duration/50

    d_vals = d_periodique(t, duration, D)
    theta_vals = theta_periodique(t, duration, theta0, sigma, tau)

    # Choisis la vitesse ici :
    speed = 2  # >1 accélère, <1 ralentit, 1.0 = normal

    animate_periodic(theta_vals, d_vals, A=A, B=B, duration=duration, fps=40, speed=speed)
