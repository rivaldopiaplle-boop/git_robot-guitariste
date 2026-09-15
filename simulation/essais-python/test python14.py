# -*- coding: utf-8 -*-
"""
Animation complète robot-Guitar (version rigoureuse)
- Résolution couplée p4 <-> theta3 au pas de temps (itération Relaxation)
- Theta2 calculé depuis d(t) avec dérivée anticipée et interpolation locale
- Flèche : queue fixée en P_A, tête toujours à P1 ; la TÊTE est 'élastique' :
          la taille (échelle) de la tête varie avec la distance PA->P1
- P2 et P3 : cercles creux ; P4 : pas de marque ; trajectoire finale P4 toujours visible
- Disque plein (corde) centré en (Zc, Xc) de diamètre dc
Auteur : Rivaldo (adaptation)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import FancyArrowPatch, Circle

# ------------------------
# CONSTANTES ROBOT (DH) & RÉGLAGES GLOBAUX
# ------------------------
A, B, C, D = 15.0, 3.0, 5.0, 3.0   # paramètres géométriques
d1_0 = 25.0                        # amplitude maximale de d
T = 10.0                           # période (s)
theta0 = 1.0 * np.pi / 6.0         # amplitude theta2 (rad)
sigma = np.pi / 8.0                # amplitude éventuelle pour theta3 (non utilisée ici)
tau = T / 50.0
delta = T / 25.0

# RÉGLAGES D'ANIMATION / DISQUE / RÉSOLUTION
N = 800                # nombre d'échantillons temporels (augmente pour plus de précision)
dc = 2.0                # diamètre du disque représentant la corde (plus grand comme demandé)
frame_interval = 40.0   # valeur de base pour interval (ms) - modifiable
speed_factor = 1000.0      # facteur de vitesse (interval_effectif = frame_interval / speed_factor)

# PARAMÈTRES ITÉRATION COUPLÉE p4 <-> theta3
max_iter = 60           # nombre maximum d'itérations par pas de temps
tol = 1e-6              # tolérance de convergence sur theta3 (rad)
alpha_relax = 0.75      # facteur de relaxation (0<alpha<=1). 0.75 = raisonnable

# LISSAGE
smooth_window_theta2 = 3    # fenêtre moyenne mobile pour theta2 (1 = pas de lissage)
smooth_window_theta3 = 3    # fenêtre moyenne mobile pour theta3

# EPS pour seuils numériques
eps = 1e-6

# ------------------------
# SIGNAL d(t) (ton signal périodique initial)
# ------------------------
def d_periodique(t, T, dmax, delta):
    """
    Signal périodique pour d(t) : montée linéaire, palier, descente linéaire, retour à zéro.
    """
    d = np.zeros_like(t)
    for i, ti in enumerate(t):
        phase = ti % T
        if phase <= T/2 - delta:
            d[i] = dmax * phase / (T/2 - delta)
        elif phase <= T/2:
            d[i] = dmax
        elif phase <= T - delta:
            d[i] = dmax * (-phase/(T/2 - delta) + (T-delta)/(T/2-delta))
        else:
            d[i] = 0.0
    return d

# ------------------------
# CALCUL DE theta2 À PARTIR DE d (dérivée anticipée + interpolation locale)
# ------------------------
def theta2_en_fonction_de_d(d_vect, theta0, eps=1e-6, smooth_window=3):
    """
    Calcule theta2(t) à partir de d(t) :
    - calcule d' avec np.gradient (une seule fois)
    - si signe(d') > 0 => theta2 = -theta0
    - si signe(d') < 0 => theta2 = theta0
    - sinon (signe(d') == 0) : on détecte le segment contigu et on applique
      * décroissance locale theta0 -> -theta0 si d ≈ 0 sur le segment
      * croissance locale -theta0 -> theta0 sinon
    - application optionnelle d'une moyenne mobile pour lisser.
    """
    Ntot = len(d_vect)
    if Ntot == 0:
        return np.array([])

    d_prime = np.gradient(d_vect)  # dérivée numérique unique
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
            # test si d ≈ 0 sur le segment
            if np.max(np.abs(d_vect[start:end])) <= eps:
                # décroissance locale theta0 -> -theta0
                theta_seg = np.linspace(theta0, -theta0, seg_len)
            else:
                # croissance locale -theta0 -> theta0
                theta_seg = np.linspace(-theta0, theta0, seg_len)
            theta2[start:end] = theta_seg
        else:
            i += 1

    # lissage optionnel
    if smooth_window is not None and smooth_window > 1:
        kernel = np.ones(smooth_window) / float(smooth_window)
        theta2 = np.convolve(theta2, kernel, mode='same')

    return theta2

# ------------------------
# UTILITAIRES PLAN X-Z
# ------------------------
def vector_norm(v):
    return np.linalg.norm(v)

def angle_between_vectors_in_xz(v_from, v_to):
    """
    Retourne (ang_from, ang_to, diff_signed) pour les vecteurs projetés dans le plan X-Z.
    Note : on construit [z, x] pour arctan2 comme dans ton code d'origine.
    """
    ax = np.array([v_from[2], v_from[0]])
    bx = np.array([v_to[2], v_to[0]])
    ang_from = np.arctan2(ax[1], ax[0])
    ang_to = np.arctan2(bx[1], bx[0])
    diff = ang_to - ang_from
    while diff > np.pi:
        diff -= 2.0 * np.pi
    while diff <= -np.pi:
        diff += 2.0 * np.pi
    return ang_from, ang_to, diff

def arc_points(center, ang_start, ang_end, radius, npoints=30):
    thetas = np.linspace(ang_start, ang_end, npoints)
    zs = center[2] + radius * np.cos(thetas)
    xs = center[0] + radius * np.sin(thetas)
    return zs, xs

# ------------------------
# CINEMATIQUE DIRECTE (DH)
# ------------------------
def forward_kinematics(theta_tuple, d1, A=A, B=B, C=C, D=D):
    """
    Calcule les points p0..p4 dans le plan (x, y, z) -> on utilisera x=0, z=2.
    """
    theta2, theta3 = theta_tuple
    p0 = np.array([0.0, 0.0, 0.0])
    pA = np.array([A, 0.0, 0.0])
    p1 = pA + np.array([0.0, 0.0, d1])
    p2 = p1 + np.array([-B, 0.0, 0.0])
    p3 = p2 + np.array([-C * np.cos(theta2), 0.0, C * np.sin(theta2)])
    p4 = p3 + np.array([-D * np.cos(theta2 + theta3), 0.0, D * np.sin(theta2 + theta3)])
    return p0, pA, p1, p2, p3, p4

# ------------------------
# RÉSOLUTION COUPLÉE p4 <-> theta3 (itérative, sans raccourci)
# ------------------------
def compute_frames_coupled(t_vect, theta2_vect, d_vect, Zc, Xc, dc,
                           eps=1e-6, smooth_window=3,
                           max_iter=50, tol=1e-6, alpha=0.7):
    """
    Résout pour chaque pas i la relation entre p4 et theta3 par itération:
    - theta2_vect et d_vect sont connus
    - on itère sur theta3_i : on calcule p4 à partir de theta3_i et on calcule
      ensuite la theta3_candidate via ton algorithme géométrique; on applique
      relaxation et on répète jusqu'à convergence.
    Retour:
      frames : liste des dictionnaires pour chaque instant (p0..p4, arcs, theta2, theta3, d)
      theta3_vect : vecteur theta3 (longueur N)
      theta_total_vect : theta2 + theta3
    """
    N = len(t_vect)
    frames = []
    theta3_vect = np.zeros(N)
    theta_total_vect = np.zeros(N)

    # signe de d' pré-calculé
    d_prime = np.gradient(d_vect)
    sign_d = np.zeros_like(d_prime, dtype=int)
    sign_d[d_prime > eps] = 1
    sign_d[d_prime < -eps] = -1

    # centre corde en format (X, Z)
    Cxy = np.array([Xc, Zc])

    # initialisation de theta3_prev (bonne initialisation -> accélère la convergence)
    theta3_prev = 0.0

    for i in range(N):
        th2 = theta2_vect[i]
        dval = d_vect[i]

        # initialisation locale de theta3_i
        theta3_i = theta3_prev  # start from previous value for continuity

        converged = False
        for k in range(max_iter):
            # calcul des positions p avec theta3_i courant
            p0, pA, p1, p2, p3, p4 = forward_kinematics((th2, theta3_i), dval)

            # coord X,Z
            p3_xz = np.array([p3[0], p3[2]])
            p4_xz = np.array([p4[0], p4[2]])

            # application des règles (ordre strict)
            # Règle 1 : Xc < x(p4) - dc/2 -> theta_total = theta2 -> theta3 = 0
            if Xc < (p4_xz[0] - dc/2.0):
                theta_total_candidate = th2
                theta3_candidate = 0.0

            # Règle 2 : Xc > x(p3) + dc/2 -> erreur corde trop haute (on stoppe)
            elif Xc > (p3_xz[0] + dc/2.0):
                raise ValueError(f"Corde trop haute à i={i}: Xc={Xc:.4f} > x(p3)+dc/2={p3_xz[0] + dc/2.0:.4f}")

            else:
                # cas intermédiaire : projection orthogonale de C sur la droite p3->p4
                v_p3p4 = p4_xz - p3_xz
                norm_v = np.linalg.norm(v_p3p4)
                if norm_v < 1e-12:
                    # ligne dégénérée -> aucune géométrie fiable
                    theta_total_candidate = th2
                    theta3_candidate = 0.0
                else:
                    u = v_p3p4 / norm_v
                    p3c = Cxy - p3_xz         # vecteur p3 -> C
                    proj_len = np.dot(p3c, u)
                    p3c_proj = proj_len * u
                    cc_perp = p3c - p3c_proj
                    dist_cc = np.linalg.norm(cc_perp)
                    z_cc = cc_perp[1]
                    norm_p3c_proj = np.linalg.norm(p3c_proj)

                    # condition de contact : dist_cc < dc/2 AND z_cc * sign(d') < 0
                    if (dist_cc < dc/2.0) and (z_cc * sign_d[i] < 0):
                        # term1 = atan(dist_cc / norm_p3c_proj) (protect denom)
                        if norm_p3c_proj > 1e-12:
                            term1 = np.arctan(dist_cc / norm_p3c_proj)
                        else:
                            term1 = np.pi/2.0 if dist_cc != 0 else 0.0
                        # term2 = atan2(z_p3c, x_p3c)
                        x_p3c = p3c[0]
                        z_p3c = p3c[1]
                        term2 = np.arctan2(z_p3c, x_p3c)
                        theta_total_candidate = term1 + term2
                        theta3_candidate = theta_total_candidate - th2
                    else:
                        theta_total_candidate = th2
                        theta3_candidate = 0.0

            # relaxation update
            theta3_new = alpha * theta3_candidate + (1.0 - alpha) * theta3_i

            # convergence ?
            if abs(theta3_new - theta3_i) < tol:
                theta3_i = theta3_new
                converged = True
                break

            theta3_i = theta3_new

        # fin itération k
        # si non convergé, on garde la dernière estimation theta3_i (log possible)
        # recalcul des positions finales pour ce pas
        p0, pA, p1, p2, p3, p4 = forward_kinematics((th2, theta3_i), dval)
        ang_from2, ang_to2, signed2 = angle_between_vectors_in_xz(p2 - p1, p3 - p2)
        ang_from3, ang_to3, signed3 = angle_between_vectors_in_xz(p3 - p2, p4 - p3)
        r2 = max(0.5, 0.1 * vector_norm(p2)) * (0.5 + 0.5 * dval / 61.0)
        r3 = max(0.4, 0.08 * vector_norm(p3)) * (0.5 + 0.5 * dval / 61.0)
        zs2, xs2 = arc_points(p2, ang_from2, ang_from2 + signed2, r2)
        zs3, xs3 = arc_points(p3, ang_from3, ang_from3 + signed3, r3)

        frames.append({
            "p0": p0, "pA": pA, "p1": p1, "p2": p2, "p3": p3, "p4": p4,
            "theta2": th2, "theta3": theta3_i, "d": dval,
            "arc2": (zs2, xs2), "arc3": (zs3, xs3)
        })

        theta3_vect[i] = theta3_i
        theta_total_vect[i] = th2 + theta3_i
        theta3_prev = theta3_i  # bonne initialisation pour l'instant suivant

    # lissage optionnel
    """
     if smooth_window is not None and smooth_window > 1:
        k = np.ones(smooth_window) / float(smooth_window)
        theta3_vect = np.convolve(theta3_vect, k, mode='same')
        theta_total_vect = np.convolve(theta_total_vect, k, mode='same')
        """

    return frames, theta3_vect, theta_total_vect

# ------------------------
# FONCTION D'ANIMATION PRINCIPALE
# ------------------------
def animate_robot_full():
    """
    Prépare le temps, calcule d(t), theta2(t) puis résout couplé frames+theta3,
    et crée l'animation matplotlib.
    """
    # vecteur temps
    t = np.linspace(0.0, T, N)

    # d(t)
    d_signal = d_periodique(t, T, d1_0, delta)

    # theta2 depuis d (dérivée anticipée)
    theta2_signal = theta2_en_fonction_de_d(d_signal, theta0, eps=eps, smooth_window=smooth_window_theta2)

    # Calcul Zc,Xc (exemples basés sur tes calculs précédents)
    # zmin/zmax et centre Zc, Xc comme tu utilisais auparavant
    zmin = -(C + D) * np.cos(theta0)
    zmax = d1_0 + (C + D) * np.cos(theta0)
    Zc = (zmax + zmin) / 2.0
    Xc = A - B - (C + D) * np.cos(theta0)

    # Résolution couplée : frames, theta3 et theta_total
    frames, theta3_signal, theta_total = compute_frames_coupled(
        t, theta2_signal, d_signal, Zc, Xc, dc,
        eps=eps, smooth_window=smooth_window_theta3,
        max_iter=max_iter, tol=tol, alpha=alpha_relax
    )

    # Préparation figure 2x2
    fig, ((ax_anim, ax_theta), (ax_d, ax_thd)) = plt.subplots(2, 2, figsize=(15, 12))

    # AXE animation Z-X
    ax_anim.set_title("Robot-Guitar animation Z-X")
    ax_anim.set_xlabel("Z")
    ax_anim.set_ylabel("X")
    ax_anim.grid(True)

    # calcul marges (touts les frames)
    all_z = np.concatenate([np.array([f["p0"][2], f["pA"][2], f["p1"][2], f["p2"][2], f["p3"][2], f["p4"][2]]) for f in frames])
    all_x = np.concatenate([np.array([f["p0"][0], f["pA"][0], f["p1"][0], f["p2"][0], f["p3"][0], f["p4"][0]]) for f in frames])
    margin = max(1.0, 0.25 * max(np.max(all_z) - np.min(all_z), np.max(all_x) - np.min(all_x)))
    ax_anim.set_xlim(np.min(all_z) - margin, np.max(all_z) + margin)
    ax_anim.set_ylim(np.min(all_x) - margin, np.max(all_x) + margin)

    # Tracés segments (on garde la même stratégie visuelle que précédemment)
    colors = ["black", "gray", "blue", "green", "red", "purple"]
    lines = []
    for c in colors[:-1]:
        l, = ax_anim.plot([], [], '-', color=c, lw=2)
        lines.append(l)

    # Points : P0, PA, P1, P2 (creux), P3 (creux)
    p0_point, = ax_anim.plot([], [], 'o', color='black', markersize=6, label='P0')
    pA_point, = ax_anim.plot([], [], 'o', color='gray', markersize=6, label='PA')
    p1_point, = ax_anim.plot([], [], 'o', color='blue', markersize=6, label='P1')
    p2_point, = ax_anim.plot([], [], 'o', color='green', markersize=10, markerfacecolor='none', label='P2 (creux)')
    p3_point, = ax_anim.plot([], [], 'o', color='red', markersize=10, markerfacecolor='none', label='P3 (creux)')

    # Arcs
    arc2_line, = ax_anim.plot([], [], color='lime', lw=2)
    arc3_line, = ax_anim.plot([], [], color='orange', lw=2)

    # Flèche : queue fixée en P_A, tête à P1 ; on utilisera FancyArrowPatch et on adaptera
    # la taille de la tête via mutation_scale proportionnellement à la distance PA->P1.
    arrow = FancyArrowPatch((0, 0), (0, 0), color='tab:green', arrowstyle='->', linewidth=2)
    ax_anim.add_patch(arrow)

    # Trajectoire finale P4 (toujours visible)
    traj_line, = ax_anim.plot([], [], 'm--', lw=1.5, label='Trajectoire final P4')

    # Disque plein (corde) centré en (Zc, Xc) - facecolor remplie
    cord_circle = Circle((Zc, Xc), radius=dc/2.0, facecolor='gray', alpha=0.45, edgecolor='k')
    ax_anim.add_patch(cord_circle)

    ax_anim.legend(loc='upper left')

    # Subplot theta2/theta3/thetatotal
    theta_total_calc = theta2_signal + theta3_signal
    ax_theta.set_title("θ2, θ3 et θ_total")
    ax_theta.plot(t, theta2_signal, label='θ2', color='lime')
    ax_theta.plot(t, theta3_signal, label='θ3', color='orange')
    ax_theta.plot(t, theta_total_calc, label='θ_total', color='purple')
    theta2_cursor, = ax_theta.plot([], [], 'ro')
    theta3_cursor, = ax_theta.plot([], [], 'bo')
    theta_total_cursor, = ax_theta.plot([], [], 'go')
    ax_theta.legend()
    ax_theta.grid(True)

    # Subplot d(t)
    ax_d.set_title("d(t)")
    ax_d.plot(t, d_signal, color='tab:green')
    d_cursor, = ax_d.plot([], [], 'ro')
    ax_d.grid(True)

    # Subplot theta_total(d)
    ax_thd.set_title("θ_total(d)")
    ax_thd.plot(d_signal, theta_total_calc, color='purple')
    thd_cursor, = ax_thd.plot([], [], 'ro')
    ax_thd.grid(True)

    # Fonction d'update (exécutée à chaque frame)
    def update(i):
        fr = frames[i]
        pts = [fr["p0"], fr["pA"], fr["p1"], fr["p2"], fr["p3"], fr["p4"]]
        zs = [p[2] for p in pts]  # Z -> abscisse du plot
        xs = [p[0] for p in pts]  # X -> ordonnée du plot

        # Mettre à jour segments (on aligne les 5 segments en chain)
        for k, l in enumerate(lines):
            l.set_data([zs[k], zs[k+1]], [xs[k], xs[k+1]])

        # Mettre à jour points creux / pleins
        p0_point.set_data([zs[0]], [xs[0]])
        pA_point.set_data([zs[1]], [xs[1]])
        p1_point.set_data([zs[2]], [xs[2]])
        p2_point.set_data([zs[3]], [xs[3]])
        p3_point.set_data([zs[4]], [xs[4]])
        # P4 : pas de point, mais son trajectoire est dessinée ci-après

        # Arcs
        arc2_line.set_data(fr["arc2"][0], fr["arc2"][1])
        arc3_line.set_data(fr["arc3"][0], fr["arc3"][1])

        # Flèche : queue à P_A, tête à P1
        # On met la queue au point P_A et la tête au point P1 (positions en (Z,X))
        pa_pos = (fr["pA"][2], fr["pA"][0])
        p1_pos = (fr["p1"][2], fr["p1"][0])
        arrow.set_positions(pa_pos, p1_pos)

        # Calcul de la distance PA->P1 (euclidienne dans le plan Z-X)
        dist_pa_p1 = np.hypot(p1_pos[0] - pa_pos[0], p1_pos[1] - pa_pos[1])

        # TÊTE élastique : adapter la taille de la tête via mutation_scale
        # on choisit une échelle de base et un gain proportionnel à dist
        base_scale = 10.0
        gain = 3.0
        mutation_scale = base_scale + gain * dist_pa_p1
        # setter de la mutation_scale se fait en ré-créant le style via set_mutation_scale
        try:
            arrow.set_mutation_scale(mutation_scale)
        except Exception:
            # certaines versions matplotlib acceptent set_mutation_scale, d'autres pas.
            # Si indisponible, on ignore silencieusement (la flèche a alors taille par défaut).
            pass

        # Trajectoire finale P4
        traj_zs = [f["p4"][2] for f in frames[:i+1]]
        traj_xs = [f["p4"][0] for f in frames[:i+1]]
        traj_line.set_data(traj_zs, traj_xs)

        # Cursors sur graphes
        theta2_cursor.set_data([t[i]], [fr["theta2"]])
        theta3_cursor.set_data([t[i]], [fr["theta3"]])
        theta_total_cursor.set_data([t[i]], [fr["theta2"] + fr["theta3"]])
        d_cursor.set_data([t[i]], [fr["d"]])
        thd_cursor.set_data([fr["d"]], [fr["theta2"] + fr["theta3"]])

        # Retourner la liste d'artistes mis à jour (nécessaire si blit=True)
        artists = []
        artists.extend(lines)
        artists.extend([p0_point, pA_point, p1_point, p2_point, p3_point])
        artists.extend([arc2_line, arc3_line, arrow, traj_line])
        artists.extend([theta2_cursor, theta3_cursor, theta_total_cursor, d_cursor, thd_cursor])
        return artists

    # Calcul interval effectif
    interval_effectif = frame_interval / max(1e-9, speed_factor)

    ani = animation.FuncAnimation(fig, update, frames=len(t), interval=interval_effectif, blit=True)
    plt.tight_layout()
    plt.show()
    return ani

# ------------------------
# LANCEMENT
# ------------------------
if __name__ == "__main__":
    ani = animate_robot_full()
