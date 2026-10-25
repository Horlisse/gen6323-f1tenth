%TEST_CHEMINS Contrôles d'importation (S0, poses) et test du croisement du huit.
clear;
attendu = struct('droite', 20.0, 'cercle', 31.416, 'huit', 48.778);
for nom = {'droite', 'cercle', 'huit'}
    C = reference_source(nom{1});
    fprintf('%-7s N=%5d ferme=%d S0=%.6f (enonce ~%.3f)\n', nom{1}, ...
        size(C.P,1), C.ferme, C.longueur, attendu.(nom{1}));
    disp([pose_initiale(nom{1}, C)'; pose_initiale(nom{1}, C, true)']);
end
% Croisement du huit : progression vs recherche globale
H = reference_source('huit');
prog = struct('seg', 1, 's', 0, 's_tot', 0);
u = linspace(0, 2*pi, 3001); u(end) = [];
s_hist = zeros(size(u)); sg_hist = zeros(size(u));
for i = 1:numel(u)
    T = [8*cos(u(i)); 8*cos(2*u(i))]; T = T/norm(T);
    p = [8*sin(u(i)); 4*sin(2*u(i))] + 0.3*[-T(2); T(1)];
    [prog, ~, ~, e] = projeter_progression(H, p, prog);
    s_hist(i) = prog.s_tot;
    [~, g] = min(hypot(H.P(:,1) - p(1), H.P(:,2) - p(2)));
    sg_hist(i) = H.s_start(g);
end
dsg = mod(diff(sg_hist) + H.longueur/2, H.longueur) - H.longueur/2;
fprintf('Progression : s_tot final = %.3f, min ds = %.4f, ecart signe = %.4f\n', ...
    s_hist(end), min(diff(s_hist)), e);
fprintf('Recherche globale : %d sauts de branche\n', sum(abs(dsg) > 2));
figure; plot(u, s_hist, u, mod(sg_hist, H.longueur), '--');
legend('avec progression', 'recherche globale'); xlabel('u'); ylabel('s [m]');
title('Huit : abscisse projetée');
