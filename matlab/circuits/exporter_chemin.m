function exporter_chemin(Pts, ferme, fichier, description)
%EXPORTER_CHEMIN Export de l'annexe A.4 : colonnes s_m,x_m,y_m,psi_rad, 17 chiffres significatifs.
%   s : abscisse curviligne calculée à partir des points exportés, première valeur nulle.
%   psi : cap par différence centrée (point suivant - point précédent) ; chemin fermé :
%   voisins pris cycliquement ; chemin ouvert : différence décentrée aux extrémités.
%   Un fichier <nom>_meta.txt indique ouvert/fermé, longueur totale et méthode.
n = size(Pts, 1);
if ferme
    suiv = Pts([2:n 1], :); prec = Pts([n 1:n - 1], :);
else
    suiv = Pts([2:n n], :); prec = Pts([1 1:n - 1], :);
end
psi = atan2(suiv(:, 2) - prec(:, 2), suiv(:, 1) - prec(:, 1));
s = [0; cumsum(hypot(diff(Pts(:, 1)), diff(Pts(:, 2))))];

fid = fopen(fichier, 'w');
if fid < 0, error('exporter_chemin:ouverture', 'Écriture impossible : %s', fichier); end
fprintf(fid, 's_m,x_m,y_m,psi_rad\n');
fprintf(fid, '%.17g,%.17g,%.17g,%.17g\n', [s, Pts, psi]');
fclose(fid);

C = creer_chemin(Pts, ferme);       % vérifie aussi : valeurs finies, aucun segment nul
[dossier, base] = fileparts(fichier);
fid = fopen(fullfile(dossier, [base '_meta.txt']), 'w');
if ferme, type = 'ferme (le dernier point rejoint le premier)'; else, type = 'ouvert'; end
fprintf(fid, 'chemin : %s\npoints : %d\nlongueur totale [m] : %.6f\nmethode : %s\n', ...
    type, n, C.longueur, description);
fprintf(fid, 'cap : difference centree des points voisins\ngenere le : %s\n', ...
    char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm')));
fclose(fid);
end
