function afficher_carte(carte)
%AFFICHER_CARTE Affiche la carte dans le repère métrique (sans recentrer le CSV).
res = carte.resolution; ox = carte.origin(1); oy = carte.origin(2);
xv = ox + ((1:carte.W) - 0.5)*res;
yv = oy + (carte.H - (1:carte.H) + 0.5)*res;   % ligne 1 = haut de l'image = y maximal
imagesc(xv, yv, carte.libre);
colormap(gca, [0.25 0.25 0.25; 1 1 1]);         % occupé : gris foncé, libre : blanc
axis xy equal tight; hold on; xlabel('x [m]'); ylabel('y [m]');
end
