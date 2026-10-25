import math, sys
sys.path.insert(0, '.')
from gen6323_core.vehicle import Limits, step, saturate

lim = Limits()

def analytic(q0, v, delta, t):
    x0, y0, p0 = q0
    if delta == 0:
        return (x0 + v*t*math.cos(p0), y0 + v*t*math.sin(p0), p0)
    R = lim.L / math.tan(delta)
    w = v / R
    cx, cy = x0 - R*math.sin(p0), y0 + R*math.cos(p0)
    p = p0 + w*t
    return (cx + R*math.sin(p), cy - R*math.cos(p), p)

def run(q0, v, delta, T, method):
    q = q0; n = round(T/lim.dt); emax = 0
    for k in range(1, n+1):
        q = step(q, v, delta, lim, method=method)
        qa = analytic(q0, v, delta, k*lim.dt)
        emax = max(emax, math.hypot(q[0]-qa[0], q[1]-qa[1]))
    return q, emax

for name, delta in [("droite", 0.0), ("cercle d=0.2", 0.2), ("cercle d=-0.4189", -0.4189)]:
    for m in ["exact", "euler"]:
        q0 = (1.0, -2.0, 0.7)
        q, e = run(q0, 2.0, delta, 10.0, m)
        print(f"{name:18s} {m:5s} erreur pos max = {e:.3e} m")
R = lim.L/math.tan(0.2); print("R théorique (d=0.2) =", R)

v=d=0.0; vs=[]
for k in range(5):
    v, d, f = saturate(10.0, 1.0, v, d, lim)
    vs.append((round(v,4), round(d,4)))
print("5 premiers pas consigne (10 m/s, 1 rad) ->", vs)
v=d=0.0
for k in range(400): v,d,f = saturate(10.0, -1.0, v, d, lim)
print("après 4 s :", v, d, f)
