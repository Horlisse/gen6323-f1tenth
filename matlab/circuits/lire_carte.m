function carte = lire_carte(racine, nom)
%LIRE_CARTE Carte au format ROS map_server : <nom>_map.yaml + image.
%   Le YAML donne la résolution [m/pixel] et l'origine [x y lacet] du coin
%   INFÉRIEUR GAUCHE de l'image. La ligne 1 de l'image est le HAUT de la carte.
%   carte.libre(r, c) = true si le pixel est libre (probabilité d'occupation < free_thresh).
fy = fullfile(racine, nom, [nom '_map.yaml']);
txt = fileread(fy);
cle = @(k) regexp(txt, ['(?m)^\s*' k '\s*:\s*(.+?)\s*$'], 'tokens', 'once');

tok = cle('image');      carte.image = strtrim(strrep(strrep(tok{1}, '''', ''), '"', ''));
tok = cle('resolution'); carte.resolution = str2double(tok{1});
tok = cle('origin');     carte.origin = str2double(regexp(tok{1}, '[-+0-9.eE]+', 'match'));
tok = cle('negate');     carte.negate = ~isempty(tok) && str2double(tok{1}) ~= 0;
tok = cle('occupied_thresh'); carte.occupied_thresh = lire_ou(tok, 0.65);
tok = cle('free_thresh');     carte.free_thresh = lire_ou(tok, 0.196);
if numel(carte.origin) >= 3 && abs(carte.origin(3)) > 1e-12
    error('lire_carte:lacet', 'Origine avec lacet non nul : non pris en charge.');
end

[X, cmap] = imread(fullfile(racine, nom, carte.image));
if ~isempty(cmap)                      % image indexée
    img = 255*mean(ind2rgb(X, cmap), 3);
elseif islogical(X)
    img = 255*double(X);
else
    img = double(X);
    if isa(X, 'uint16'), img = img/257; end
    if size(img, 3) > 1, img = mean(img(:, :, 1:3), 3); end
end
if carte.negate, p = img/255; else, p = (255 - img)/255; end
carte.libre = p < carte.free_thresh;
[carte.H, carte.W] = size(carte.libre);
end

function v = lire_ou(tok, defaut)
if isempty(tok), v = defaut; else, v = str2double(tok{1}); end
end
