function [delta, prog_f, p_proj] = stanley(q, v, C, prog_f, gains, P)
%STANLEY Loi géométrique (3) de l'énoncé, version de base (sans anticipation de courbure).
%   q      : pose [x; y; psi] du milieu de l'essieu arrière
%   v      : vitesse courante du véhicule [m/s], v >= 0
%   prog_f : état de progression du point AVANT (distinct de celui du point arrière)
%   gains  : struct ks [1/s] > 0, v0 [m/s] > 0 (régularisation à basse vitesse)
%
%   delta = -e_psi - atan(ks*e_f/(v + v0)), avec
%   e_f   = n'(p_f - p_Gamma), n normale gauche (> 0 si le point avant est à gauche)
%   e_psi = wrap(psi - psi_Gamma)            (> 0 si le véhicule pointe à gauche du chemin)

% 1. Point avant : milieu de l'essieu avant
pf = q(1:2) + P.L*[cos(q(3)); sin(q(3))];

% 2. Projection avec progression : point projeté, cap tangent, écart signé
[prog_f, p_proj, psi_t, e_f] = projeter_progression(C, pf, prog_f);

% 3. Erreur de cap ramenée dans [-pi, pi[
e_psi = wrap_angle(q(3) - psi_t);

% 4. Loi de commande
delta = -e_psi - atan(gains.ks*e_f/(max(v, 0) + gains.v0));
end
