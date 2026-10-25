import math, sys, time
sys.path.insert(0, '.')
import numpy as np
from gen6323_core.path import source_reference, initial_pose, Progress, project, point_at

# 1) Longueurs S0 de l'annexe A.1
for n, ref_val in [("droite", 20.0), ("cercle", 31.416), ("huit", 48.778)]:
    p = source_reference(n)
    print(f"{n:7s} N={len(p.P):5d} closed={p.closed} S0={p.length:.6f} (énoncé ~{ref_val})")

# 2) Poses initiales et récupération
for n in ["droite", "cercle", "huit"]:
    r = source_reference(n)
    print(n, "nominale", np.round(initial_pose(n, r), 5), "récup", np.round(initial_pose(n, r, True), 5))

# 3) Croisement du huit : on parcourt le huit (u de 0 à 2pi) avec un décalage
#    latéral de 0.3 m ; projection avec progression vs recherche globale.
huit = source_reference("huit")
prog = Progress(); s_hist = []; glob_jump = 0; prev_g = 0.0
t0 = time.time()
for u in np.linspace(0, 2*np.pi, 3000, endpoint=False):
    x, y = 8*np.sin(u), 4*np.sin(2*u)
    tx, ty = 8*np.cos(u), 8*np.cos(2*u); nn = np.hypot(tx, ty)
    p = (x - 0.3*ty/nn, y + 0.3*tx/nn)         # 0.3 m à gauche
    prog, C, psi, e = project(huit, p, prog)
    s_hist.append(prog.s_tot)
    # recherche globale naïve (au point le plus proche)
    g = np.argmin(np.hypot(huit.P[:,0]-p[0], huit.P[:,1]-p[1]))
    sg = huit.s_start[g]
    if abs(((sg - prev_g + huit.length/2) % huit.length) - huit.length/2) > 2: glob_jump += 1
    prev_g = sg
dt = time.time() - t0
s_hist = np.array(s_hist)
print(f"huit: s_tot final={s_hist[-1]:.3f} / S0={huit.length:.3f}, "
      f"min ds={np.diff(s_hist).min():.4f}, max ds={np.diff(s_hist).max():.4f}")
print(f"huit: écart signé dernier point = {e:.4f} (attendu +0.3 à gauche)")
print(f"recherche globale : {glob_jump} sauts de branche ; temps moyen projection {dt/3000*1e3:.3f} ms")

# 4) point_at
P, psi, j = point_at(huit, huit.length + 1.0)
print("point_at(S0+1) =", np.round(P,4), "cap", round(psi,4))
dr = source_reference("droite")
print("droite point_at(25) =", point_at(dr, 25.0)[0], "(borné à la fin)")
