function A = analyser_chemin(Pts, ferme)
%ANALYSER_CHEMIN Indicateurs de qualité utiles au choix du prétraitement.
%   dpsi(i) : changement de cap au sommet entre deux segments consécutifs [rad]
%   kappa   : courbure discrète dpsi / longueur moyenne des deux segments [1/m]
if ferme, Q = [Pts; Pts(1, :)]; else, Q = Pts; end
d = diff(Q, 1, 1);
seg = hypot(d(:, 1), d(:, 2));
psi = atan2(d(:, 2), d(:, 1));
if ferme
    dpsi = wrap_angle(psi([2:end 1]) - psi);
    lmoy = 0.5*(seg + seg([2:end 1]));
else
    dpsi = wrap_angle(diff(psi));
    lmoy = 0.5*(seg(1:end-1) + seg(2:end));
end
A.n = size(Pts, 1);
A.S0 = sum(seg);
A.seg = seg; A.seg_min = min(seg); A.seg_moy = mean(seg); A.seg_max = max(seg);
A.dpsi = dpsi; A.dpsi_max = max(abs(dpsi));
A.kappa = dpsi./lmoy; A.kappa_max = max(abs(A.kappa));
A.dkappa_max = max(abs(diff(A.kappa)));
aire = 0.5*sum(Pts(:, 1).*Pts([2:end 1], 2) - Pts([2:end 1], 1).*Pts(:, 2));
if aire > 0, A.sens = 'antihoraire'; else, A.sens = 'horaire'; end
end
