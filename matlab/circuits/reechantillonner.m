function R = reechantillonner(Pts, ds, ferme)
%REECHANTILLONNER Points à pas uniforme ~ds [m] le long de la ligne brisée (aucun lissage).
if ferme, Q = [Pts; Pts(1, :)]; else, Q = Pts; end
s = [0; cumsum(hypot(diff(Q(:, 1)), diff(Q(:, 2))))];
n = max(round(s(end)/ds), 3);
u = linspace(0, s(end), n + 1)';
if ferme, u(end) = []; end            % pas de point dupliqué sur un chemin fermé
R = [interp1(s, Q(:, 1), u), interp1(s, Q(:, 2), u)];
end
