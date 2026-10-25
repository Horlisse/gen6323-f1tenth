function C = creer_chemin(P, ferme)
%CREER_CHEMIN Ligne brisée à partir de points (N x 2).
%   Pour un chemin fermé, le premier point n'est PAS dupliqué : le segment
%   N -> 1 est ajouté implicitement (annexe A.1).
if ~all(isfinite(P(:))), error('creer_chemin:nonfini', 'Valeurs non finies.'); end
if ferme, Q = [P; P(1,:)]; else, Q = P; end
d = diff(Q, 1, 1);
lg = hypot(d(:,1), d(:,2));
if any(lg <= 1e-12), error('creer_chemin:segnul', 'Segment de longueur nulle.'); end
C.P        = P;
C.ferme    = ferme;
C.seg_len  = lg;
C.s_start  = [0; cumsum(lg(1:end-1))];
C.seg_psi  = atan2(d(:,2), d(:,1));
C.longueur = sum(lg);
C.nseg     = numel(lg);
end
