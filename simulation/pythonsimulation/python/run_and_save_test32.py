import os
import importlib.util
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from matplotlib.patches import Circle, FancyArrowPatch

# path to the original script
SCRIPT_PATH = r"c:\Users\HP\Desktop\vscode\python\python\test python32.py"

spec = importlib.util.spec_from_file_location("test32", SCRIPT_PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

# Recreate what animate_robot_full does but save outputs instead of showing
T = mod.T
N = mod.N
dc = mod.dc

# time and signals
t = mod.np.linspace(0.0, T, N)
d_signal = mod.d_periodique(t, mod.T, mod.d1_0, mod.delta)
theta2_signal, d_prime = mod.theta2_en_fonction_de_d(d_signal, mod.theta0)

zmin = -(mod.C + mod.D) * mod.np.cos(mod.theta0)
zmax = mod.d1_0 + (mod.C + mod.D) * mod.np.cos(mod.theta0)
Zc = (zmax + 1*zmin) / 2.0
Xc = mod.A - -mod.B - 1*(mod.C + mod.D) * mod.np.cos(mod.theta0)

frames, theta3_signal, theta_total, dist_cc_list = mod.compute_frames_coupled(
    t, theta2_signal, d_signal, Zc, Xc, dc
)

out_dir = os.path.dirname(SCRIPT_PATH)

# 1) save dist_cc plot
fig1, ax1 = plt.subplots(figsize=(8,3))
ax1.plot(t, dist_cc_list, label='dist_cc = |z_cc|')
ax1.hlines(dc/2.0, t[0], t[-1], colors='r', linestyles='--', label='rayon corde')
ax1.set_xlabel('t')
ax1.set_ylabel('dist_cc')
ax1.legend()
ax1.grid(True)
fig1.tight_layout()
fn1 = os.path.join(out_dir, 'dist_cc.png')
fig1.savefig(fn1)
print('Saved', fn1)
plt.close(fig1)

# 2) save theta_total vs d plot
fig2, ax2 = plt.subplots(figsize=(6,4))
ax2.plot(d_signal, theta_total, color='purple')
ax2.set_xlabel('d')
ax2.set_ylabel('theta_total')
ax2.grid(True)
fig2.tight_layout()
fn2 = os.path.join(out_dir, 'theta_total_vs_d.png')
fig2.savefig(fn2)
print('Saved', fn2)
plt.close(fig2)

# 3) save final frame robot configuration
last = frames[-1]
pts = [last['p0'], last['pA'], last['p1'], last['p2'], last['p3'], last['p4']]
zs = [p[2] for p in pts]
xs = [p[0] for p in pts]

fig3, ax3 = plt.subplots(figsize=(8,6))
# robot links
for k in range(len(pts)-1):
    ax3.plot([zs[k], zs[k+1]], [xs[k], xs[k+1]], '-o', lw=2)
# arcs
ax3.plot(last['arc2'][0], last['arc2'][1], color='lime', lw=2)
ax3.plot(last['arc3'][0], last['arc3'][1], color='orange', lw=2)
# cord circle
cord_circle = Circle((Zc, Xc), radius=dc/2.0, facecolor='gray', alpha=0.45, edgecolor='k')
ax3.add_patch(cord_circle)
# c' point
cp = last['cprime']
ax3.plot(cp[2], cp[0], 'ko', markersize=8, markerfacecolor='yellow')

ax3.set_xlabel('Z')
ax3.set_ylabel('X')
ax3.grid(True)
fig3.tight_layout()
fn3 = os.path.join(out_dir, 'final_frame.png')
fig3.savefig(fn3)
print('Saved', fn3)
plt.close(fig3)

print('All done')
