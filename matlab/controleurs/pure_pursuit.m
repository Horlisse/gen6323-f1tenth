function [delta, prog, cible] = pure_pursuit(q, v, C, prog, gains, P)
%PURE_PURSUIT Loi géométrique (2) de l'énoncé.
%   q     : pose [x; y; psi] du milieu de l'essieu arrière
%   v     : vitesse courante du véhicule [m/s]
%   C     : chemin suivi (creer_chemin)
%   prog  : état de progression du point arrière (struct seg, s, s_tot)
%   gains : struct l0 [m], tau [s] ; avance le long du chemin l(v) = l0 + tau*v
%   Retourne le braquage géométrique AVANT saturation, le nouvel état et la cible.
%
%   La cible est prise à une ABSCISSE CURVILIGNE en avant de la projection du
%   point arrière (et non à une distance droite) : avance et corde sont distinctes.

% 1. Projection du point arrière, en respectant la progression (pas de saut de branche)
prog = projeter_progression(C, q(1:2), prog);

% 2. Cible le long du chemin (bornée à la fin d'un chemin ouvert)
cible = point_abscisse(C, prog.s_tot + gains.l0 + gains.tau*v);

% 3. Changement de repère : carte -> repère de l'essieu arrière (xB vers l'avant, yB à gauche)
d  = cible - q(1:2);
xB =  cos(q(3))*d(1) + sin(q(3))*d(2);
yB = -sin(q(3))*d(1) + cos(q(3))*d(2);

% 4. Courbure de l'arc tangent à xB passant par la cible, puis braquage
d2 = xB^2 + yB^2;
if d2 < 1e-12                 % cible confondue avec le véhicule (fin de la droite)
    kappa = 0;
else
    kappa = 2*yB/d2;
end
delta = atan(P.L*kappa);
end
