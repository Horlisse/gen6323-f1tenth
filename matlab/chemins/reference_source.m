function C = reference_source(nom, dossier_csv)
%REFERENCE_SOURCE Références sources de l'annexe A.1 (non modifiées).
%   dossier_csv : racine de f1tenth_racetracks (circuits seulement).
switch nom
    case 'droite'
        j = (0:400)';
        C = creer_chemin([0.05*j, zeros(size(j))], false);
    case {'cercle', 'huit'}
        u = 2*pi*(0:9999)'/10000;
        if strcmp(nom, 'cercle'), P = [5*cos(u), 5*sin(u)];
        else,                     P = [8*sin(u), 4*sin(2*u)]; end
        C = creer_chemin(P, true);
    case {'Spielberg', 'Silverstone'}
        f = fullfile(dossier_csv, nom, [nom '_centerline.csv']);
        D = lire_csv_centerline(f);
        C = creer_chemin(D(:,1:2), true);   % ordre du CSV conservé
        C.largeurs = D(:,3:4);              % [w_droite, w_gauche]
    otherwise
        error('Chemin inconnu : %s', nom);
end
C.nom = nom;
end

function D = lire_csv_centerline(f)
% Ignore les lignes de commentaire commençant par '#'.
fid = fopen(f, 'r');
if fid < 0, error('Fichier introuvable : %s', f); end
nettoyage = onCleanup(@() fclose(fid));
D = zeros(0, 4);
while true
    l = fgetl(fid);
    if ~ischar(l), break; end
    l = strtrim(l);
    if isempty(l) || l(1) == '#', continue; end
    D(end+1, :) = sscanf(strrep(l, ',', ' '), '%f')'; %#ok<AGROW>
end
end
