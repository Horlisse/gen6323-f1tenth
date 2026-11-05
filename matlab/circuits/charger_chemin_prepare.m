function C = charger_chemin_prepare(nom, dossier)
%CHARGER_CHEMIN_PREPARE Chemin exporté à l'étape 4 (colonnes s_m,x_m,y_m,psi_rad).
%   Le chemin est fermé : le dernier point rejoint le premier (non dupliqué).
%   Le séparateur est imposé : la détection automatique de readmatrix peut échouer.
f = fullfile(dossier, sprintf('%s_chemin.csv', nom));
if ~isfile(f), error('charger_chemin_prepare:absent', 'Fichier introuvable : %s', f); end
T = readmatrix(f, 'FileType', 'text', 'Delimiter', ',', 'NumHeaderLines', 1);
if size(T, 2) < 3
    error('charger_chemin_prepare:format', 'Format inattendu dans %s : %d colonne(s) lue(s).', f, size(T, 2));
end
C = creer_chemin(T(:, 2:3), true);
C.nom = [nom ' (préparé)'];
end
