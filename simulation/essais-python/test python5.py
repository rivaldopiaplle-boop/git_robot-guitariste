"""
Robot-Guitar animation (Python)
Auteur : Rivaldo (modifié)
Langue : français

Modifications majeures :
 - conservation des générateurs theta_periodique et d_periodique
 - ajout d'une jacobienne analytique 2x2 J(theta,d)
 - calcul des positions (x,z) par intégration de x' = J q' (sans utiliser T3 pour x,z)
 - génération d'une trajectoire "inspirée" et calcul de q via q' = J^{-1} x'
 - deux animations : (1) originale, (2) désirée vs réalisée
 - paramètre 'speed' pour contrôler vitesse de l'animation
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

# -------------------------------------------------
# Fonctions inchangées (d_periodique, theta_periodique)
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

def theta_periodique(t, T, theta0, sigma, tau):
    theta = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if (0 <= phase <= 3*T/8 - T/32 - tau) or (3*T/8 - T/32 + tau <= phase <= T/2 - tau):
            theta[i] = theta0
        elif (3*T/8 - T/32 - tau <= phase <= 3*T/8 - T/32 + tau):
            theta[i] = theta0 + sigma
        elif (T/2 < phase <= 7*T/8 - T/32 - tau) or (7*T/8 - T/32 + tau <= phase <= T - tau):
            theta[i] = -theta0
        elif (7*T/8 - T/32 - tau <= phase <= 7*T/8 - T/32 + tau):
            theta[i] = -(theta0 + sigma)
        elif (T/2 - tau <= phase <= T/2):
            theta[i] = theta0*(-2*phase/tau + T/tau - 1)
        elif (T - tau < phase <= T):
            theta[i] = theta0*(2*phase/tau - 2*T/tau + 1)
    return theta

# -------------------------------------------------
# Cinématique directe (expression analytique)
# --> on garde la fonction forward_kinematics pour tracer
# mais pour obtenir x,z on utilisera la jacobienne et / ou expressions analytiques
# -------------------------------------------------

def forward_kinematics(theta, d, A=1.0, B=2.0):
    """
    Retourne p0, p1, p3 comme auparavant (utile pour afficher le robot).
    Basé sur MDH original — conservé pour le dessin du robot.
    """
    # Reprise du mdh_transform (même code logique que tu avais)
    def mdh_transform(alpha, a, th, dd):
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(th), np.sin(th)
        RxTx = np.array([
            [1.0, 0.0, 0.0, a],
            [0.0, ca, -sa, 0.0],
            [0.0, sa, ca, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        RzTz = np.array([
            [ct, -st, 0.0, 0.0],
            [st,  ct, 0.0, 0.0],
            [0.0, 0.0, 1.0, dd],
            [0.0, 0.0, 0.0, 1.0]
        ])
        return RxTx @ RzTz

    p0 = np.array([0.0, 0.0, 0.0])
    T1 = mdh_transform(0.0, A, 0.0, d)
    T2 = mdh_transform(-np.pi/2, 0.0, theta, 0.0)
    T3 = mdh_transform(0.0, B, 0.0, 0.0)
    T_full = T1 @ T2 @ T3
    p3_h = T_full @ np.array([0,0,0,1])
    p1_h = T1 @ np.array([0,0,0,1])
    p3 = p3_h[:3]
    p1 = p1_h[:3]
    return p0, p1, p3

# -------------------------------------------------
# Expressions analytiques directes pour x,z (évite de re-multiplier T1*T2*T3)
# dérivation faite analytiquement : 
#   x = A + B * cos(theta)
#   z = - B * sin(theta) + d
# -------------------------------------------------

def xz_from_q(theta, d, A=1.0, B=2.0):
    x = A + B * np.cos(theta)
    z = -B * np.sin(theta) + d
    return x, z

# -------------------------------------------------
# Jacobienne analytique 2x2 (J(theta,d)) -> ∂(x,z)/∂(q) avec q = [theta, d]
# Dérivé analytiquement :
#   ∂x/∂theta = -B sin(theta)
#   ∂x/∂d     = 0
#   ∂z/∂theta = -B cos(theta)
#   ∂z/∂d     = 1
# -------------------------------------------------

def jacobian(theta, d, A=1.0, B=2.0):
    """
    Retourne la jacobienne 2x2 évaluée en (theta,d).
    ordre q = [theta, d]
    """
    J = np.array([
        [-B * np.sin(theta), 0.0],
        [-B * np.cos(theta), 1.0]
    ])
    return J

def invert_jacobian(J, tol=1e-8):
    """
    Inverse la jacobienne J (2x2). Si déterminant trop petit, retourne pseudo-inverse.
    """
    try:
        det = J[0,0]*J[1,1] - J[0,1]*J[1,0]
        if abs(det) > tol:
            Jinv = np.linalg.inv(J)
        else:
            # singularité proche -> pseudo-inverse
            Jinv = np.linalg.pinv(J)
        return Jinv
    except Exception:
        return np.linalg.pinv(J)

# -------------------------------------------------
# Calcul de (x,z) à partir de q via intégration x' = J q'
# - q(t) donné (theta_vals, d_vals)
# -------------------------------------------------

def compute_xz_from_q_via_J(theta_vals, d_vals, t, A=1.0, B=2.0):
    """
    theta_vals, d_vals: arrays of same length
    t: time vector (same length)
    retourne x_arr, z_arr calculés par intégration de x' = J q'
    (précision : intégration trapézoïdale)
    """
    theta_vals = np.asarray(theta_vals)
    d_vals = np.asarray(d_vals)
    t = np.asarray(t)
    N = t.size
    dt = np.gradient(t)  # supporte temps non-uniformes

    # calcul des dérivées q'
    theta_dot = np.gradient(theta_vals, t, edge_order=2)
    d_dot = np.gradient(d_vals, t, edge_order=2)

    # initial x0,z0 (on peut utiliser formule analytique sans rappeler T3)
    x0, z0 = xz_from_q(theta_vals[0], d_vals[0], A=A, B=B)
    x = np.zeros((N,))
    z = np.zeros((N,))
    x[0] = x0
    z[0] = z0

    # calc x' = J q' et intégration trapézoïdale
    xdot_prev = None
    for i in range(N):
        J = jacobian(theta_vals[i], d_vals[i], A=A, B=B)
        qdot = np.array([theta_dot[i], d_dot[i]])
        xdot = J @ qdot  # xdot is 2-vector (x_dot, z_dot)
        if i == 0:
            xdot_prev = xdot
            continue
        # trapezoid step for interval dt[i]
        # use dt[i] as the local time increment (np.gradient gives dt per sample)
        dx = 0.5 * (xdot_prev + xdot) * dt[i]
        x[i] = x[i-1] + dx[0]
        z[i] = z[i-1] + dx[1]
        xdot_prev = xdot

    # For the first sample we didn't fill x[0] beyond initial; ensure subsequent samples filled:
    # If N>1 and loop started at i=0, x[1] might still be zero; handle simple forward Euler for i=1
    if N > 1 and x[1] == 0.0:
        # fallback simple Euler first step
        J = jacobian(theta_vals[0], d_vals[0], A=A, B=B)
        qdot0 = np.array([theta_dot[0], d_dot[0]])
        xdot0 = J @ qdot0
        x[1] = x[0] + xdot0[0] * dt[0]
        z[1] = z[0] + xdot0[1] * dt[0]

    # if any zero entries remain (edgecases), fill by direct analytic mapping as last resort
    for i in range(N):
        if x[i] == 0 and z[i] == 0 and not (theta_vals[i]==theta_vals[0] and d_vals[i]==d_vals[0]):
            xa, za = xz_from_q(theta_vals[i], d_vals[i], A=A, B=B)
            x[i] = xa
            z[i] = za

    return x, z

# -------------------------------------------------
# Génération d'une trajectoire "inspirée" à partir de (x,z)
# Ici : on construit x_des = x_orig + modulation sinusoïdale lissée
# et z_des = z_orig (ou légèrement modifiée).
# -------------------------------------------------

def generate_desired_trajectory(x_orig, z_orig, t, amplitude=0.4, freq=2.0):
    """
    Retourne x_des, z_des (mêmes longueurs que x_orig)
    amplitude: amplitude de modulation relative
    freq: fréquence (cycles pendant la durée)
    """
    duration = t[-1] - t[0] if t[-1] > t[0] else 1.0
    s = (t - t[0]) / duration
    # modulation temporelle lissée (enveloppe pour avoir attaque + décroissance)
    envelope = (0.5 * (1 - np.cos(np.pi * s)))  # passe de 0->1 puis reste; lisse l'attaque
    modulation = amplitude * envelope * np.sin(2 * np.pi * freq * s)
    x_des = x_orig + modulation
    # garder z proche de l'original mais on peut ajouter petite variation
    z_des = z_orig + 0.05 * np.sin(2 * np.pi * freq * s * 0.7)
    return x_des, z_des

# -------------------------------------------------
# Calcul de q(t) à partir d'une trajectoire désirée x_des(t) via
# résolution itérative : q' = J^{-1}(q) x'  (intégration avant Euler explicite)
# -------------------------------------------------

def compute_q_from_xdot_via_Jinv(theta0, d0, x_des, z_des, t, A=1.0, B=2.0):
    """
    Intègre q' = J^{-1}(q) x' à partir d'un état initial q0 = (theta0, d0).
    Retourne theta_arr, d_arr et qdot_arr.
    Méthode : Euler explicite avec pas local dt.
    """
    t = np.asarray(t)
    N = t.size
    dt_arr = np.gradient(t)
    x_des = np.asarray(x_des)
    z_des = np.asarray(z_des)

    # calculer xdot (vector) numériquement
    x_dot = np.gradient(x_des, t, axis=0, edge_order=2)
    z_dot = np.gradient(z_des, t, axis=0, edge_order=2)

    theta = np.zeros(N)
    d = np.zeros(N)
    theta[0] = theta0
    d[0] = d0

    qdot_hist = np.zeros((N,2))
    for i in range(N-1):
        J = jacobian(theta[i], d[i], A=A, B=B)
        Jinv = invert_jacobian(J)
        xdot_vec = np.array([x_dot[i], z_dot[i]])
        qdot = Jinv @ xdot_vec  # (theta_dot, d_dot)
        # applique Euler explicite
        theta[i+1] = theta[i] + qdot[0] * dt_arr[i]
        d[i+1] = d[i] + qdot[1] * dt_arr[i]
        qdot_hist[i,:] = qdot

        # si valeurs très extrêmes on clamp (sécurité basique)
        if np.isnan(theta[i+1]) or np.isnan(d[i+1]):
            theta[i+1] = theta[i]
            d[i+1] = d[i]

    # dernière qdot
    # calculer dernier qdot avec dernier xdot
    J = jacobian(theta[-1], d[-1], A=A, B=B)
    qdot_hist[-1,:] = invert_jacobian(J) @ np.array([x_dot[-1], z_dot[-1]])

    return theta, d, qdot_hist

# -------------------------------------------------
# Animation 1 : originale (theta, d, theta(d), x(z) calculés via J)
# -------------------------------------------------

def animate_original(theta_vals, d_vals, t, A=1.0, B=2.0, speed=1.0):
    # calcule x,z via intégration J q'
    x_vals, z_vals = compute_xz_from_q_via_J(theta_vals, d_vals, t, A=A, B=B)

    # pré-tracé
    fig, ((ax_anim, ax_theta), (ax_d, ax_thd)) = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle("Animation 1 — trajectoire calculée via J (original)")

    # plan z-x
    ax_anim.set_title("Trajectoire dans le plan (z - x)")
    ax_anim.set_xlabel("z")
    ax_anim.set_ylabel("x")
    ax_anim.set_aspect("equal")
    ax_anim.grid(True)
    ax_anim.plot(z_vals, x_vals, 'k--', alpha=0.7, label="Trajectoire (via J)")
    ax_anim.legend(loc="upper right")

    # limites
    z_min, z_max = z_vals.min(), z_vals.max()
    x_min, x_max = x_vals.min(), x_vals.max()
    margin = 0.2 * max(z_max - z_min if z_max>z_min else 1.0, x_max - x_min if x_max>x_min else 1.0)
    ax_anim.set_xlim(z_min - margin, z_max + margin)
    ax_anim.set_ylim(x_min - margin, x_max + margin)

    # objets dynamiques
    line_robot, = ax_anim.plot([], [], 'o-', lw=2, alpha=0.8)
    eff_marker, = ax_anim.plot([], [], 'rs', ms=6)
    path_trace, = ax_anim.plot([], [], '-', lw=1, alpha=0.6)

    # theta(t)
    ax_theta.set_title("θ(t)")
    ax_theta.plot(t, theta_vals, 'b')
    theta_cursor, = ax_theta.plot([], [], 'ro')

    # d(t)
    ax_d.set_title("d(t)")
    ax_d.plot(t, d_vals, 'g')
    d_cursor, = ax_d.plot([], [], 'ro')

    # theta(d)
    ax_thd.set_title("Relation θ(d)")
    ax_thd.set_xlabel("d")
    ax_thd.set_ylabel("θ")
    ax_thd.plot(d_vals, theta_vals, 'm')
    thd_cursor, = ax_thd.plot([], [], 'ro')

    # init
    def init():
        line_robot.set_data([], [])
        eff_marker.set_data([], [])
        path_trace.set_data([], [])
        theta_cursor.set_data([], [])
        d_cursor.set_data([], [])
        thd_cursor.set_data([], [])
        return line_robot, eff_marker, path_trace, theta_cursor, d_cursor, thd_cursor

    # update
    trace_z = []
    trace_x = []
    N = t.size
    dt = t[1] - t[0] if t.size > 1 else 1.0
    frames = N
    interval = (1000.0 * (t[-1]-t[0]) / frames) / speed

    for_anim_p0 = np.array([0.0, 0.0, 0.0])  # base

    def update(i):
        # pour le dessin du robot on peut conserver forward_kinematics (visual only)
        p0, p1, p3 = forward_kinematics(theta_vals[i], d_vals[i], A=A, B=B)
        # Le plan z-x : abscisse = z, ordonnée = x (comme ton code original)
        line_robot.set_data([p0[2], p1[2], p3[2]], [p0[0], p1[0], p3[0]])
        eff_marker.set_data(p3[2], p3[0])

        # trace
        trace_z.append(z_vals[i])
        trace_x.append(x_vals[i])
        path_trace.set_data(trace_z, trace_x)

        # curseurs
        theta_cursor.set_data(t[i], theta_vals[i])
        d_cursor.set_data(t[i], d_vals[i])
        thd_cursor.set_data(d_vals[i], theta_vals[i])
        return line_robot, eff_marker, path_trace, theta_cursor, d_cursor, thd_cursor

    ani = animation.FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, interval=interval)
    plt.tight_layout()
    plt.show()
    return ani

# -------------------------------------------------
# Animation 2 : trajectoire désirée (x_des) -> calcul de q via J^-1 -> trajectoire réelle
# -------------------------------------------------

def animate_desired_vs_real(theta0, d0, x_orig, z_orig, t, A=1.0, B=2.0, speed=1.0):
    # generate desired path
    x_des, z_des = generate_desired_trajectory(x_orig, z_orig, t, amplitude=0.4, freq=2.5)

    # compute q from x_des via q' = J^{-1} x'
    theta_des, d_des, qdot_hist = compute_q_from_xdot_via_Jinv(theta0, d0, x_des, z_des, t, A=A, B=B)

    # compute the "real" x,z obtained by mapping theta_des,d_des through analytic mapping (no T3 matrix multiplications)
    x_real, z_real = xz_from_q(theta_des, d_des, A=A, B=B)

    # prepare figure
    fig, ((ax_anim, ax_theta), (ax_d, ax_thd)) = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle("Animation 2 — trajectoire désirée vs réalisée")

    # plan z-x : affichage désiré (pointillé) et réalisé (plein)
    ax_anim.set_title("Trajectoire désirée (pointillé) vs réalisée (plein)")
    ax_anim.set_xlabel("z")
    ax_anim.set_ylabel("x")
    ax_anim.set_aspect("equal")
    ax_anim.grid(True)
    ax_anim.plot(z_des, x_des, 'r--', alpha=0.6, label="désirée")
    ax_anim.plot(z_real, x_real, 'b--', alpha=0.6, label="réelle (prélim.)")
    ax_anim.legend(loc="upper right")

    z_min = min(z_des.min(), z_real.min())
    z_max = max(z_des.max(), z_real.max())
    x_min = min(x_des.min(), x_real.min())
    x_max = max(x_des.max(), x_real.max())
    margin = 0.2 * max(z_max - z_min if z_max>z_min else 1.0, x_max - x_min if x_max>x_min else 1.0)
    ax_anim.set_xlim(z_min - margin, z_max + margin)
    ax_anim.set_ylim(x_min - margin, x_max + margin)

    # objets dynamiques
    des_marker, = ax_anim.plot([], [], 'rs', ms=6)
    real_marker, = ax_anim.plot([], [], 'bo', ms=6)
    real_path, = ax_anim.plot([], [], '-', lw=1, alpha=0.7)
    des_path, = ax_anim.plot([], [], '--', lw=1, alpha=0.5)

    # θ(t) et d(t) -> afficher évolution calculée (theta_des, d_des)
    ax_theta.set_title("θ_des(t)")
    ax_theta.plot(t, theta_des, 'b')
    theta_cursor, = ax_theta.plot([], [], 'ro')

    ax_d.set_title("d_des(t)")
    ax_d.plot(t, d_des, 'g')
    d_cursor, = ax_d.plot([], [], 'ro')

    ax_thd.set_title("Relation θ_des(d_des)")
    ax_thd.set_xlabel("d")
    ax_thd.set_ylabel("θ")
    ax_thd.plot(d_des, theta_des, 'm')
    thd_cursor, = ax_thd.plot([], [], 'ro')

    # init
    def init():
        des_marker.set_data([], [])
        real_marker.set_data([], [])
        real_path.set_data([], [])
        des_path.set_data([], [])
        theta_cursor.set_data([], [])
        d_cursor.set_data([], [])
        thd_cursor.set_data([], [])
        return des_marker, real_marker, real_path, des_path, theta_cursor, d_cursor, thd_cursor

    # update
    N = t.size
    dt = t[1] - t[0] if N>1 else 1.0
    frames = N
    interval = (1000.0 * (t[-1]-t[0]) / frames) / speed

    rz = []
    rx = []
    dz = []
    dx = []

    def update(i):
        des_marker.set_data(z_des[i], x_des[i])
        real_marker.set_data(z_real[i], x_real[i])

        rz.append(z_real[i]); rx.append(x_real[i])
        dz.append(z_des[i]); dx.append(x_des[i])

        real_path.set_data(rz, rx)
        des_path.set_data(dz, dx)

        theta_cursor.set_data(t[i], theta_des[i])
        d_cursor.set_data(t[i], d_des[i])
        thd_cursor.set_data(d_des[i], theta_des[i])

        return des_marker, real_marker, real_path, des_path, theta_cursor, d_cursor, thd_cursor

    ani = animation.FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, interval=interval)
    plt.tight_layout()
    plt.show()
    return ani

# -------------------------------------------------
# Exemple d’utilisation complet
# -------------------------------------------------
if __name__ == "__main__":
    # paramètres robot
    A = 1.0
    B = 3.0

    # paramètres temps / animation
    duration = 10.0
    T = 600
    t = np.linspace(0, duration, T)

    # paramètres signaux (conservés)
    D = 10
    theta0 = np.pi/5
    sigma = np.pi/15
    tau = duration / 50

    d_vals = d_periodique(t, duration, D)
    theta_vals = theta_periodique(t, duration, theta0, sigma, tau)

    # vitesse d'animation (1.0 normal)
    speed = 1.5

    # --- Animation 1 : trajectoire calculée via J (original)
    ani1 = animate_original(theta_vals, d_vals, t, A=A, B=B, speed=speed)

    # --- Préparation pour animation 2
    # calcul x,z originaux (on peut obtenir via xz_from_q)
    x_orig, z_orig = xz_from_q(theta_vals, d_vals, A=A, B=B)

    # choix condition initiale pour l'inverse (on prend la première valeur de theta,d)
    theta_init = theta_vals[0]
    d_init = d_vals[0]

    # --- Animation 2 : trajectoire désirée -> q via J^-1 -> trajectoire réelle
    ani2 = animate_desired_vs_real(theta_init, d_init, x_orig, z_orig, t, A=A, B=B, speed=speed)

    # Note : plt.show() déjà appelé dans fonctions d'animation
