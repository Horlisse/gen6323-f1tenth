function [pt, psi_t, j] = point_abscisse(C, s_tot)
%POINT_ABSCISSE Point et cap à l'abscisse déroulée s_tot (cibles Pure Pursuit).
%   Chemin ouvert : s borné à [0, longueur] (la cible s'arrête en fin de chemin).
if C.ferme, s = mod(s_tot, C.longueur);
else,       s = min(max(s_tot, 0), C.longueur); end
j = find(C.s_start <= s, 1, 'last');
j = min(max(j, 1), C.nseg);
t = (s - C.s_start(j)) / C.seg_len(j);
A = C.P(j, :)';  B = C.P(mod(j, size(C.P,1)) + 1, :)';
pt = A + t*(B - A);
psi_t = C.seg_psi(j);
end
