function libre = points_libres(carte, x, y)
%POINTS_LIBRES Vrai si le point (x, y) [m, repère carte] tombe sur un pixel libre.
%   Vérification nécessaire mais NON suffisante : elle ne garantit pas le
%   passage de l'ensemble du véhicule (annexe A.4).
res = carte.resolution; ox = carte.origin(1); oy = carte.origin(2);
col = floor((x(:) - ox)/res) + 1;
lig = carte.H - floor((y(:) - oy)/res);
dedans = lig >= 1 & lig <= carte.H & col >= 1 & col <= carte.W;
libre = false(numel(x), 1);
libre(dedans) = carte.libre(sub2ind([carte.H, carte.W], lig(dedans), col(dedans)));
end
