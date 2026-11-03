function Q = lisser_gauss(Pts, sigma, ds)
%LISSER_GAUSS Lissage d'un chemin FERMÉ : rééchantillonnage uniforme puis
%   filtre gaussien circulaire d'écart type sigma [m] appliqué à x et à y.
%   Atténue le bruit de numérisation de longueur d'onde << sigma. Déplace
%   légèrement la ligne vers l'intérieur des virages : écart à quantifier
%   avec ecart_chemins. (Une spline d'INTERPOLATION ne lisse pas : elle
%   passe par chaque point, bruit compris.)
R = reechantillonner(Pts, ds, true);
demi = ceil(4*sigma/ds);
if demi >= size(R, 1), error('lisser_gauss:sigma', 'sigma trop grand pour ce chemin.'); end
k = exp(-0.5*(((-demi:demi)*ds)/sigma).^2);
k = k(:)/sum(k);
Q = zeros(size(R));
for j = 1:2
    ext = [R(end - demi + 1:end, j); R(:, j); R(1:demi, j)];   % bouclage
    Q(:, j) = conv(ext, k, 'valid');
end
end
