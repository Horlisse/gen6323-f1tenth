function C = charger_chemin_prepare(nom, dossier)
%CHARGER_CHEMIN_PREPARE Chemin exporté à l'étape 4 (colonnes s_m,x_m,y_m,psi_rad).
%   Le chemin est fermé : le dernier point rejoint le premier (non dupliqué).
f = fullfile(dossier, sprintf('%s_chemin.csv', nom));
if ~isfile(f), error('charger_chemin_prepare:absent', 'Fichier introuvable : %s', f); end
T = readmatrix(f, 'NumHeaderLines', 1);
C = creer_chemin(T(:, 2:3), true);
C.nom = [nom ' (préparé)'];
end
