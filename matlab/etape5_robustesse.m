%ETAPE5_ROBUSTESSE Série « Robustesse » du tableau 2 : un essai repris, deux niveaux
% d'un même facteur, mêmes niveaux et réalisations pour les deux lois.
%
% Facteur retenu : RETARD DE COMMANDE (déterministe, pertinent pour ROS 2).
% Essai repris   : huit, 2 m/s, pose nominale.
% Niveaux        : 10 pas (100 ms) et 20 pas (200 ms) ; le niveau 0 est l'essai repris.
%
% IMPORTANT : les gains ci-dessous sont les réglages de DÉMONSTRATION. Pour la
% campagne officielle (étape 7), remplacer par les réglages FIGÉS à l'étape 6.
%
% Lancer depuis matlab/ :  addpath(genpath(pwd)); etape5_robustesse
clear; close all;
P = params_vehicule();

% ---------------------------------------------------------------- paramètres
nom = 'huit'; v_cible = 2; recuperation = false;
niveaux = [0 10 20];                       % retard en pas de dt (0 = essai repris)
gains.pp      = struct('l0', 0.6, 'tau', 0.3);
gains.stanley = struct('ks', 2.0, 'v0', 1.0);
verifier_concordance = true;              % comparaison au jumeau Python (gains de démonstration)

% Valeurs du jumeau Python : loi, retard, T, e_RMS, e_max, J_delta, J_v
ATTENDU = {
    'pp', 0, 24.67, 0.0266209537, 0.0541942588, 0.0041348697, 0.2423996757;
    'pp', 10, 24.80, 0.0142319234, 0.0295663656, 0.0047584527, 0.2411290323;
    'pp', 20, 24.93, 0.0039023174, 0.0120022909, 0.0061345832, 0.2398716406;
    'stanley', 0, 24.63, 0.0138466174, 0.0309489209, 0.0071577246, 0.2427933415;
    'stanley', 10, 24.76, 0.0128812963, 0.0252927693, 0.0079292254, 0.2415185784;
    'stanley', 20, 25.50, 0.0649139079, 0.1957436827, 2.3721198020, 0.2345098039;
};

% ---------------------------------------------------------------- essais
ref = reference_source(nom);
q0 = pose_initiale(nom, ref, recuperation);
lois = {'pp', 'stanley'};
R = struct();
fprintf('\nRobustesse : %s, %g m/s, retard de commande (Δt = %.2f s)\n\n', nom, v_cible, P.dt);
fprintf('%-8s %6s %8s %-16s %7s %9s %9s %9s %8s %9s %s\n', 'Loi', 'Retard', '[ms]', 'Statut', ...
    'T [s]', 'eRMS [cm]', 'emax [cm]', 'J_delta', 'J_v', 'δ sat. %', 'Concordance');
n_ok = 0; n_tot = 0;
for il = 1:numel(lois)
    loi = lois{il};
    for in = 1:numel(niveaux)
        d = niveaux(in);
        r = simuler_essai(ref, ref, q0, v_cible, loi, gains.(loi), P, struct('retard', d));
        R.(loi)(in) = r;
        verdict = '';
        if verifier_concordance
            i = find(strcmp(ATTENDU(:, 1), loi) & [ATTENDU{:, 2}]' == d);
            att = [ATTENDU{i, 3:7}];
            ec = abs([r.T, r.e_rms, r.e_max, r.J_d, r.J_v] - att);
            ok = ec(1) <= 0.006 && all(ec(2:5) <= 1e-6);
            n_ok = n_ok + ok; n_tot = n_tot + 1;
            if ok, verdict = 'OK'; else, verdict = sprintf('ÉCART %.1e', max(ec(2:5))); end
        end
        fprintf('%-8s %6d %8d %-16s %7.2f %9.2f %9.2f %9.4f %8.4f %9.1f %s\n', loi, d, round(1000*d*P.dt), ...
            r.statut, r.T, 100*r.e_rms, 100*r.e_max, r.J_d, r.J_v, 100*r.lim_frac(3), verdict);
    end
end
if verifier_concordance
    fprintf('\n%d/%d essais concordent avec le jumeau Python.\n', n_ok, n_tot);
end
fprintf('(δ sat. %% : fraction du temps où la limite d''amplitude du braquage est active.)\n');

% ---------------------------------------------------------------- figures
coul = {[0.55 0.6 0.66], [0.07 0.54 0.54], [0.80 0.30 0.10]};
noms_lois = {'Pure Pursuit', 'Stanley'};
figure('Name', 'Robustesse au retard', 'Position', [60 60 1150 700]);
for il = 1:2
    loi = lois{il};
    subplot(3, 2, il); hold on; axis equal; grid on;
    plot(ref.P(:, 1), ref.P(:, 2), 'k:', 'DisplayName', 'Référence');
    for in = 1:numel(niveaux)
        r = R.(loi)(in);
        plot(r.X(:, 1), r.X(:, 2), 'Color', coul{in}, 'LineWidth', 1.2, ...
            'DisplayName', sprintf('%d ms', round(1000*niveaux(in)*P.dt)));
    end
    title(sprintf('%s : trajectoires', noms_lois{il})); legend('Location', 'southoutside', 'Orientation', 'horizontal');
    subplot(3, 2, 2 + il); hold on; grid on;
    for in = 1:numel(niveaux)
        r = R.(loi)(in); plot(r.t, 100*r.e, 'Color', coul{in}, 'LineWidth', 1.1);
    end
    ylabel('erreur [cm]'); title('Erreur au point arrière (référence source)');
    subplot(3, 2, 4 + il); hold on; grid on;
    for in = 1:numel(niveaux)
        r = R.(loi)(in); plot(r.t, r.dc, 'Color', coul{in}, 'LineWidth', 1.1);
    end
    yline([-P.delta_max, P.delta_max], 'r:');
    ylabel('\delta_c [rad]'); xlabel('t [s]'); title('Consigne de braquage envoyée');
end

% ---------------------------------------------------------------- résultats bruts
sortie = fullfile(fileparts(mfilename('fullpath')), '..', 'resultats');
if ~isfolder(sortie), mkdir(sortie); end
fichier = fullfile(sortie, sprintf('etape5_robustesse_%s.mat', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))));
meta = struct('chemin', nom, 'v_cible', v_cible, 'niveaux_pas', niveaux, 'dt', P.dt, ...
    'facteur', 'retard de commande', 'gains', gains, 'matlab', version);
save(fichier, 'R', 'meta');
fprintf('Résultats bruts horodatés : %s\n', fichier);
