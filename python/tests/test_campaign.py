import sys, time; sys.path.insert(0, '.')
from gen6323_core.path import source_reference, initial_pose
from gen6323_core.sim import run_trial
G = {"pp": {"l0": 0.6, "tau": 0.3}, "st": {"ks": 2.0, "v0": 1.0}}
t0 = time.time()
for name in ["droite", "cercle", "huit"]:
    ref = source_reference(name)
    for law in ["pp", "st"]:
        for v, rec in [(1, False), (2, False), (3, False), (2, True)]:
            r = run_trial(ref, ref, initial_pose(name, ref, rec), v, law, G[law])
            print(f"{name:6s} {law} v={v} {'récup' if rec else 'nomin'} {r['status']:12s} "
                  f"T={r['T']:6.2f} eRMS={r['e_rms']:.4f} emax={r['e_max']:.3f} "
                  f"Jd={r['J_d']:.4f} Jv={r['J_v']:.5f} frac={r['s_frac']:.3f}")
print("temps total", round(time.time()-t0,1), "s")
