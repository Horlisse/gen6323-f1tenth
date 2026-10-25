function [prog, pc, psi_t, e_lat] = projeter_progression(C, p, prog, arriere, avant)
%PROJETER_PROGRESSION Projection sur les segments dans une fenêtre
%   [s - arriere, s + avant] autour de la projection précédente.
%   prog : struct (seg, s, s_tot) ; init : struct('seg',1,'s',0,'s_tot',0).
%   Sorties : nouvel état, point projeté, cap tangent, écart signé n'(p - pc).
%   Une recherche globale est volontairement évitée (croisement du huit).
if nargin < 4, arriere = 1.0; end
if nargin < 5, avant = 3.0; end
rs = C.s_start - prog.s;
if C.ferme, rs = mod(rs + C.longueur/2, C.longueur) - C.longueur/2; end
re = rs + C.seg_len;
masque = (re >= -arriere) & (rs <= avant);
masque(prog.seg) = true;
idx = find(masque);

N  = size(C.P, 1);
A  = C.P(idx, :);
B  = C.P(mod(idx, N) + 1, :);        % idx est 1-indexé : segment i -> point i+1
AB = B - A;
t  = ((p(1) - A(:,1)).*AB(:,1) + (p(2) - A(:,2)).*AB(:,2)) ./ C.seg_len(idx).^2;
t  = min(max(t, 0), 1);
Pc = A + t.*AB;
[~, k] = min((p(1) - Pc(:,1)).^2 + (p(2) - Pc(:,2)).^2);
j = idx(k);

s_new = C.s_start(j) + t(k)*C.seg_len(j);
ds = s_new - prog.s;
if C.ferme, ds = mod(ds + C.longueur/2, C.longueur) - C.longueur/2; end
prog = struct('seg', j, 's', s_new, 's_tot', prog.s_tot + ds);
pc = Pc(k, :)';
psi_t = C.seg_psi(j);
e_lat = -sin(psi_t)*(p(1) - pc(1)) + cos(psi_t)*(p(2) - pc(2));
end
