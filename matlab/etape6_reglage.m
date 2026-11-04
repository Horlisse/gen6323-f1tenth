%ETAPE6_REGLAGE Réglage équitable de Pure Pursuit et de Stanley (docs/plan_reglage.md).
%   Même budget pour les deux lois : grille 6 x 6 = 36 combinaisons, 6 essais chacune.
%   Critère d'admissibilité et score fixés AVANT les résultats (plan commité).
%
% Lancer depuis matlab/ :  addpath(genpath(pwd)); etape6_reglage
% mode_rapide = true : grille 2 x 2 (coins) pour tester le script et chronométrer.
clear; close all;
P = params_vehicule();
mode_rapide = false;

% ---------------------------------------------------------------- grilles (plan, section 1)
G.pp      = struct('noms', {{'l0', 'tau'}}, 'v1', [0.3 0.45 0.6 0.8 1.0 1.3], 'v2', [0 0.1 0.2 0.3 0.45 0.6]);
G.stanley = struct('noms', {{'ks', 'v0'}},  'v1', [0.5 1 1.5 2 3 5],         'v2', [0.25 0.5 1 1.5 2 3]);
if mode_rapide
    for l = {'pp', 'stanley'}, G.(l{1}).v1 = G.(l{1}).v1([1 end]); G.(l{1}).v2 = G.(l{1}).v2([1 end]); end
end

% ---------------------------------------------------------------- essais (plan, section 2)
ici = fileparts(mfilename('fullpath'));
racine = fullfile(ici, '..', 'data', 'f1tenth_racetracks');
huit = reference_source('huit');
spl_ref = reference_source('Spielberg', racine);
spl_C = charger_chemin_prepare('Spielberg', fullfile(ici, '..', 'data', 'chemins_prepares'));
q_huit = pose_initiale('huit', huit);
E = struct('nom', {'E1 huit 1 m/s', 'E2 huit 2 m/s', 'E3 huit 3 m/s', 'E4 huit récup 2 m/s', ...
                   'E5 Spielberg 2 m/s', 'E6 huit 2 m/s retard 100 ms'}, ...
    'ref', {huit, huit, huit, huit, spl_ref, huit}, 'C', {huit, huit, huit, huit, spl_C, huit}, ...
    'q0', {q_huit, q_huit, q_huit, pose_initiale('huit', huit, true), pose_initiale('Spielberg', spl_ref), q_huit}, ...
    'v', {1, 2, 3, 2, 2, 2}, 'retard', {0, 0, 0, 0, 0, 10});
nE = numel(E);

% ---------------------------------------------------------------- simulations
lois = {'pp', 'stanley'};
res = struct();
t0 = tic;
for il = 1:2
    loi = lois{il}; g = G.(loi);
    n1 = numel(g.v1); n2 = numel(g.v2);
    M = struct('ok', false(n1, n2, nE), 'e_rms', nan(n1, n2, nE), 'e_max', nan(n1, n2, nE), ...
               'J_d', nan(n1, n2, nE), 'T', nan(n1, n2, nE));
    for a = 1:n1
        for b = 1:n2
            gains = struct(g.noms{1}, g.v1(a), g.noms{2}, g.v2(b));
            for e = 1:nE
                r = simuler_essai(E(e).ref, E(e).C, E(e).q0, E(e).v, loi, gains, P, struct('retard', E(e).retard));
                M.ok(a, b, e) = r.reussi; M.e_rms(a, b, e) = r.e_rms; M.e_max(a, b, e) = r.e_max;
                M.J_d(a, b, e) = r.J_d; M.T(a, b, e) = r.T;
            end
            fprintf('%-8s %s = %-5g %s = %-5g  terminé (%4.0f s écoulées)\n', loi, g.noms{1}, g.v1(a), ...
                g.noms{2}, g.v2(b), toc(t0));
        end
    end
    res.(loi) = M;
end
fprintf('\nDurée totale des simulations : %.1f min\n', toc(t0)/60);

% ---------------------------------------------------------------- critère (plan, section 3)
choix = struct();
for il = 1:2
    loi = lois{il}; g = G.(loi); M = res.(loi);
    adm = all(M.ok(:, :, 1:5), 3) & M.e_max(:, :, 5) <= 0.30 & M.ok(:, :, 6) & ...
          M.J_d(:, :, 6) <= 3*M.J_d(:, :, 2);
    S = nan(size(adm));
    if any(adm(:))
        se = zeros(size(adm)); sj = zeros(size(adm));
        for e = 1:5
            Ee = M.e_rms(:, :, e); Je = M.J_d(:, :, e);
            se = se + Ee/median(Ee(adm))/5;
            sj = sj + Je/median(Je(adm))/5;
        end
        S = 0.5*se + 0.5*sj;
        S(~adm) = NaN;
    end
    res.(loi).adm = adm; res.(loi).S = S;

    fprintf('\n=== %s : %d / %d combinaisons admissibles ===\n', upper(loi), sum(adm(:)), numel(adm));
    elim = {'tour incomplet (E1-E5)', ~all(M.ok(:, :, 1:5), 3); 'e_max Spielberg > 0,30 m', M.e_max(:, :, 5) > 0.30; ...
            'retard : tour incomplet', ~M.ok(:, :, 6); 'retard : J_delta > 3 x', M.J_d(:, :, 6) > 3*M.J_d(:, :, 2)};
    for k = 1:size(elim, 1), fprintf('  éliminées par « %s » : %d\n', elim{k, 1}, sum(elim{k, 2}(:))); end
    if ~any(adm(:)), fprintf('  AUCUNE combinaison admissible : élargir la grille et le documenter.\n'); continue; end

    [~, ordre] = sort(S(:));
    ordre = ordre(~isnan(S(ordre)));
    fprintf('\n  Rang  %-5s %-5s  Score   eRMS moy [cm]  J_d moy   emax Spielberg [cm]  J_d retard/J_d\n', g.noms{:});
    for k = 1:min(5, numel(ordre))
        [a, b] = ind2sub(size(S), ordre(k));
        fprintf('  %4d  %-5g %-5g  %.3f   %13.2f  %8.4f  %19.1f  %14.2f\n', k, g.v1(a), g.v2(b), S(a, b), ...
            100*mean(M.e_rms(a, b, 1:5)), mean(M.J_d(a, b, 1:5)), 100*M.e_max(a, b, 5), M.J_d(a, b, 6)/M.J_d(a, b, 2));
    end
    % Égalité à 1 % près : départage par le J_delta avec retard (E6)
    proches = ordre(S(ordre) <= 1.01*S(ordre(1)));
    J6 = M.J_d(:, :, 6);
    [~, k] = min(J6(proches));
    [a, b] = ind2sub(size(S), proches(k));
    choix.(loi) = struct(g.noms{1}, g.v1(a), g.noms{2}, g.v2(b));
    fprintf('\n  >> Choix %s : %s = %g, %s = %g (score %.3f)\n', loi, g.noms{1}, g.v1(a), g.noms{2}, g.v2(b), S(a, b));

    % Carte du score (NaN = non admissible)
    figure('Name', ['Réglage ' loi], 'Position', [80 + 560*(il - 1) 120 540 460]);
    im = imagesc(S'); set(im, 'AlphaData', ~isnan(S')); set(gca, 'Color', [0.85 0.85 0.85]);
    axis xy; colorbar; hold on;
    plot(a, b, 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');
    set(gca, 'XTick', 1:numel(g.v1), 'XTickLabel', string(g.v1), 'YTick', 1:numel(g.v2), 'YTickLabel', string(g.v2));
    xlabel(g.noms{1}); ylabel(g.noms{2});
    title(sprintf('%s : score (gris = non admissible, étoile = choix)', loi));
end

% ---------------------------------------------------------------- résultats bruts
sortie = fullfile(ici, '..', 'resultats');
if ~isfolder(sortie), mkdir(sortie); end
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
if mode_rapide, stamp = [stamp '_rapide']; end
save(fullfile(sortie, ['etape6_reglage_' stamp '.mat']), 'res', 'G', 'E', 'choix', 'mode_rapide');
lignes = {};
for il = 1:2
    loi = lois{il}; g = G.(loi); M = res.(loi);
    for a = 1:numel(g.v1), for b = 1:numel(g.v2), for e = 1:nE
        lignes(end + 1, :) = {loi, g.v1(a), g.v2(b), E(e).nom, M.ok(a, b, e), M.T(a, b, e), ...
            M.e_rms(a, b, e), M.e_max(a, b, e), M.J_d(a, b, e)}; %#ok<SAGROW>
    end, end, end
end
Tb = cell2table(lignes, 'VariableNames', {'loi', 'param1', 'param2', 'essai', 'reussi', 'T_s', 'e_rms_m', 'e_max_m', 'J_delta'});
writetable(Tb, fullfile(sortie, ['etape6_reglage_' stamp '.csv']));
fprintf('\nRésultats bruts : resultats/etape6_reglage_%s.mat et .csv\n', stamp);
if isfield(choix, 'pp') && isfield(choix, 'stanley') && ~mode_rapide
    fprintf('\nÀ reporter dans reglages_figes.m après vérification par l''équipe :\n');
    fprintf('G.pp      = struct(''l0'', %g, ''tau'', %g);\n', choix.pp.l0, choix.pp.tau);
    fprintf('G.stanley = struct(''ks'', %g, ''v0'', %g);\n', choix.stanley.ks, choix.stanley.v0);
end
