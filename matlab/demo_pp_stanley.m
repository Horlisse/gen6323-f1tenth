%DEMO_PP_STANLEY Compare Pure Pursuit et Stanley sur un essai et trace les résultats.
% Lancer depuis le dossier matlab/ :  addpath(genpath(pwd)); demo_pp_stanley
clear; close all;
P = params_vehicule();

nom = 'huit';          % 'droite', 'cercle' ou 'huit'
v_cible = 2;           % 1, 2 ou 3 m/s
recuperation = false;  % true : départ décalé de 0,5 m et cap +10°

ref = reference_source(nom);
q0  = pose_initiale(nom, ref, recuperation);
R.pp      = simuler_essai(ref, ref, q0, v_cible, 'pp',      struct('l0', 0.6, 'tau', 0.3), P);
R.stanley = simuler_essai(ref, ref, q0, v_cible, 'stanley', struct('ks', 2.0, 'v0', 1.0), P);

lois = {'pp', 'stanley'}; noms = {'Pure Pursuit', 'Stanley'};
couleurs = {[0.07 0.54 0.54], [0.14 0.27 0.49]};
for i = 1:2
    r = R.(lois{i});
    fprintf('%-12s %-14s T = %6.2f s  e_RMS = %.4f m  e_max = %.4f m  J_delta = %.5f  J_v = %.5f\n', ...
        noms{i}, r.statut, r.T, r.e_rms, r.e_max, r.J_d, r.J_v);
end

figure('Name', sprintf('%s à %g m/s', nom, v_cible), 'Position', [100 100 1100 650]);
subplot(3, 2, [1 3 5]); hold on; axis equal; grid on;
plot(ref.P(:, 1), ref.P(:, 2), '--', 'Color', [0.55 0.6 0.66], 'DisplayName', 'Référence');
for i = 1:2
    r = R.(lois{i});
    plot(r.X(:, 1), r.X(:, 2), 'Color', couleurs{i}, 'LineWidth', 1.4, 'DisplayName', noms{i});
end
plot(q0(1), q0(2), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'Départ');
xlabel('x [m]'); ylabel('y [m]'); legend('Location', 'best'); title('Trajectoires du point arrière');

signaux = {'e', 'Erreur au point arrière [m]'; 'dc', '\delta_c [rad]'; 'vc', 'v_c [m/s]'};
for s = 1:3
    subplot(3, 2, 2*s); hold on; grid on;
    for i = 1:2
        r = R.(lois{i});
        plot(r.t, r.(signaux{s, 1}), 'Color', couleurs{i}, 'LineWidth', 1.1);
    end
    if strcmp(signaux{s, 1}, 'dc'), yline([-P.delta_max, P.delta_max], 'r:'); end
    ylabel(signaux{s, 2});
end
xlabel('t [s]');
