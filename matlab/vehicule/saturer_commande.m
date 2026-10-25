function [vc, dc, actif] = saturer_commande(v_des, d_des, v_prev, d_prev, P)
%SATURER_COMMANDE Limites d'amplitude puis de variation (section 2.1).
%   v_prev, d_prev : consignes appliquées au pas précédent (0 au démarrage).
%   actif : struct logique des limites actives (fraction de temps, sect. 3.3).
dt  = P.dt;
v_a = min(max(v_des, 0), P.v_max);
d_a = min(max(d_des, -P.delta_max), P.delta_max);
dv  = min(max(v_a - v_prev, -P.a_max*dt),      P.a_max*dt);
dd  = min(max(d_a - d_prev, -P.ddelta_max*dt), P.ddelta_max*dt);
vc  = v_prev + dv;
dc  = d_prev + dd;
actif.v_amp  = (v_a ~= v_des);
actif.d_amp  = (d_a ~= d_des);
actif.v_rate = (dv ~= v_a - v_prev);
actif.d_rate = (dd ~= d_a - d_prev);
end
