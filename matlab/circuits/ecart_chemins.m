function e = ecart_chemins(A, B, ferme)
%ECART_CHEMINS Distance de chaque point de A à la ligne brisée B (projection sur les segments).
%   Écart géométrique introduit par un prétraitement : max(ecart(A,B), ecart(B,A)).
if ferme, Bq = [B; B(1, :)]; else, Bq = B; end
a = Bq(1:end - 1, :);
ab = diff(Bq, 1, 1);
l2 = sum(ab.^2, 2);
e = zeros(size(A, 1), 1);
for i = 1:size(A, 1)
    t = ((A(i, 1) - a(:, 1)).*ab(:, 1) + (A(i, 2) - a(:, 2)).*ab(:, 2))./l2;
    t = min(max(t, 0), 1);
    e(i) = sqrt(min((a(:, 1) + t.*ab(:, 1) - A(i, 1)).^2 + (a(:, 2) + t.*ab(:, 2) - A(i, 2)).^2));
end
end
