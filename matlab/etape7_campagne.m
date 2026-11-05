%ETAPE7_CAMPAGNE Campagne MATLAB obligatoire (tableau 2), réglages GELÉS (reglages_figes.m).
%   Série Vitesse      : 5 chemins x 1, 2, 3 m/s, pose nominale        -> 15 essais par loi
%   Série Récupération : 5 chemins à 2 m/s, +0,50 m normale gauche, +10° -> 5 essais par loi
%   Série Robustesse   : huit 2 m/s (essai repris) + retard 100 et 200 ms -> 2 essais par loi
%   Total : 22 essais par loi, 44 essais. Les échecs sont conservés et signalés.
%
% Lancer depuis matlab/ :  addpath(genpath(pwd)); etape7_campagne
clear; close all;
P = params_vehicule();
G = reglages_figes();
if ~G.gele, error('Réglages non gelés : terminer l''étape 6 avant la campagne.'); end
gains.pp = G.pp; gains.stanley = G.stanley;
fprintf('Réglages gelés : PP l0 = %g m, tau = %g s ; Stanley ks = %g, v0 = %g m/s\n', ...
    G.pp.l0, G.pp.tau, G.stanley.ks, G.stanley.v0);

% ---------------------------------------------------------------- chemins
ici = fileparts(mfilename('fullpath'));
racine = fullfile(ici, '..', 'data', 'f1tenth_racetracks');
noms = {'droite', 'cercle', 'huit', 'Spielberg', 'Silverstone'};
for k = 1:numel(noms)
    nom = noms{k};
    if k <= 3
        ch.(nom).ref = reference_source(nom); ch.(nom).C = ch.(nom).ref;
    else
        ch.(nom).ref = reference_source(nom, racine);
        % Prétraitement FIGÉ à l'étape 4 (gauss sigma = 0,3 m, pas 0,10 m) ; identique au CSV exporté
        ch.(nom).C = creer_chemin(lisser_gauss(ch.(nom).ref.P, 0.3, 0.10), true);
    end
end

% ---------------------------------------------------------------- liste des essais
% serie, chemin, v_cible, recuperation, retard (pas)
L = {};
for k = 1:5, for v = [1 2 3], L(end + 1, :) = {'Vitesse', noms{k}, v, false, 0}; end, end %#ok<SAGROW>
for k = 1:5, L(end + 1, :) = {'Récupération', noms{k}, 2, true, 0}; end %#ok<SAGROW>
L(end + 1, :) = {'Robustesse', 'huit', 2, false, 10};
L(end + 1, :) = {'Robustesse', 'huit', 2, false, 20};
nL = size(L, 1);

% Valeurs du jumeau Python (chemins analytiques) : loi, chemin, v, récup, retard, T, eRMS, emax, J_d, J_v
ATTENDU = {
    'pp', 'droite', 1, false, 0, 20.33, 0.0000000000, 0.0000000000, 0.0000000000, 0.2754147832;
    'pp', 'droite', 2, false, 0, 10.69, 0.0000000000, 0.0000000000, 0.0000000000, 1.0383838444;
    'pp', 'droite', 3, false, 0, 7.72, 0.0000000000, 0.0000000000, 0.0000000000, 2.1534196700;
    'pp', 'droite', 2, true, 0, 10.74, 0.1276412163, 0.5127791395, 0.1977966698, 1.0356040943;
    'pp', 'cercle', 1, false, 0, 31.58, 0.0000006968, 0.0000050789, 0.0064980212, 0.0943635212;
    'pp', 'cercle', 2, false, 0, 16.04, 0.0000008825, 0.0000051974, 0.0127934849, 0.3728179551;
    'pp', 'cercle', 3, false, 0, 10.97, 0.0000010486, 0.0000051974, 0.0187062438, 0.8204193254;
    'pp', 'cercle', 2, true, 0, 16.05, 0.1017379035, 0.5109630400, 0.1494494327, 0.3725856698;
    'pp', 'huit', 1, false, 0, 48.94, 0.0023808052, 0.0053628974, 0.0014887249, 0.0608908868;
    'pp', 'huit', 2, false, 0, 24.71, 0.0061176711, 0.0132366083, 0.0054445300, 0.2420072845;
    'pp', 'huit', 3, false, 0, 16.75, 0.0120500936, 0.0255038254, 0.0108790948, 0.5373134328;
    'pp', 'huit', 2, true, 0, 24.76, 0.0843064274, 0.5127843605, 0.0908654408, 0.2415185784;
    'pp', 'huit', 2, false, 10, 24.82, 0.0008364105, 0.0023544892, 0.0061421267, 0.2409347301;
    'pp', 'huit', 2, false, 20, 26.20, 0.1119254646, 0.2837698227, 2.0891064556, 0.2282442748;
    'stanley', 'droite', 1, false, 0, 20.33, 0.0000000000, 0.0000000000, 0.0000000000, 0.2754147832;
    'stanley', 'droite', 2, false, 0, 10.69, 0.0000000000, 0.0000000000, 0.0000000000, 1.0383838444;
    'stanley', 'droite', 3, false, 0, 7.72, 0.0000000000, 0.0000000000, 0.0000000000, 2.1534196700;
    'stanley', 'droite', 2, true, 0, 10.72, 0.1371620377, 0.5127791395, 0.1934743939, 1.0375982401;
    'stanley', 'cercle', 1, false, 0, 31.52, 0.0106142759, 0.0109177034, 0.0084125947, 0.0945431472;
    'stanley', 'cercle', 2, false, 0, 16.01, 0.0104985153, 0.0109195145, 0.0163131642, 0.3735165522;
    'stanley', 'cercle', 3, false, 0, 10.95, 0.0103536337, 0.0109347221, 0.0234475995, 0.8219178082;
    'stanley', 'cercle', 2, true, 0, 15.96, 0.1121405711, 0.5109630400, 0.1383134164, 0.3746867168;
    'stanley', 'huit', 1, false, 0, 48.77, 0.0140357945, 0.0313513911, 0.0026540130, 0.0611031372;
    'stanley', 'huit', 2, false, 0, 24.63, 0.0138994730, 0.0309651150, 0.0072120192, 0.2427933415;
    'stanley', 'huit', 3, false, 0, 16.70, 0.0136944487, 0.0306579500, 0.0144660301, 0.5389221557;
    'stanley', 'huit', 2, true, 0, 24.67, 0.0913793908, 0.5127843605, 0.0910477004, 0.2423996757;
    'stanley', 'huit', 2, false, 10, 24.75, 0.0133105703, 0.0275105626, 0.0078354086, 0.2416161616;
    'stanley', 'huit', 2, false, 20, 26.20, 0.1074969859, 0.3301263700, 3.3781758111, 0.2282442748;
};

% ---------------------------------------------------------------- simulations
lois = {'pp', 'stanley'}; R = struct(); t0 = tic;
lignes = {}; n_conc = 0; n_conc_ok = 0;
for il = 1:2
    loi = lois{il};
    for i = 1:nL
        [serie, nom, v, recup, d] = L{i, :};
        c = ch.(nom);
        q0 = pose_initiale(nom, c.ref, recup);
        r = simuler_essai(c.ref, c.C, q0, v, loi, gains.(loi), P, struct('retard', d));
        R.(loi)(i).serie = serie; R.(loi)(i).chemin = nom; R.(loi)(i).res = r;
        % concordance (chemins analytiques)
        conc = '—';
        j = find(strcmp(ATTENDU(:, 1), loi) & strcmp(ATTENDU(:, 2), nom) & [ATTENDU{:, 3}]' == v ...
            & [ATTENDU{:, 4}]' == recup & [ATTENDU{:, 5}]' == d);
        if ~isempty(j)
            ec = abs([r.T, r.e_rms, r.e_max, r.J_d, r.J_v] - [ATTENDU{j, 6:10}]);
            ok = ec(1) <= 0.006 && all(ec(2:5) <= 1e-6);
            n_conc = n_conc + 1; n_conc_ok = n_conc_ok + ok;
            if ok, conc = 'OK'; else, conc = sprintf('ÉCART %.1e', max(ec(2:5))); end
        end
        if recup, dep = 'récup'; else, dep = 'nominal'; end
        lignes(end + 1, :) = {loi, serie, nom, v, dep, 10*d, r.statut, r.reussi, r.T, r.portion, ...
            r.e_rms, r.e_max, r.J_d, r.J_v, r.lim_frac(1), r.lim_frac(2), r.lim_frac(3), r.lim_frac(4), conc}; %#ok<SAGROW>
        fprintf('%-8s %-13s %-11s %g m/s %-7s %3d ms  %-26s (%4.0f s)\n', loi, serie, nom, v, dep, 10*d, r.statut, toc(t0));
    end
end
fprintf('\nDurée totale : %.1f min. Concordance (chemins analytiques) : %d/%d.\n', toc(t0)/60, n_conc_ok, n_conc);
Tb = cell2table(lignes, 'VariableNames', {'loi', 'serie', 'chemin', 'v_cible', 'depart', 'retard_ms', ...
    'statut', 'reussi', 'T_s', 'portion', 'e_rms_m', 'e_max_m', 'J_delta', 'J_v', ...
    'frac_v_amp', 'frac_v_var', 'frac_d_amp', 'frac_d_var', 'concordance'});

% ---------------------------------------------------------------- tableaux par chemin et vitesse
fprintf('\n%-11s %-13s %-7s %4s | %-15s %7s %8s %8s %8s | %-15s %7s %8s %8s %8s\n', 'Chemin', 'Série', ...
    'Départ', 'v', 'PP statut', 'T [s]', 'eRMS cm', 'emax cm', 'J_delta', 'ST statut', 'T [s]', 'eRMS cm', 'emax cm', 'J_delta');
for i = 1:nL
    a = R.pp(i).res; b = R.stanley(i).res;
    if L{i, 4}, dep = 'récup'; elseif L{i, 5} > 0, dep = sprintf('%d ms', 10*L{i, 5}); else, dep = 'nominal'; end
    fprintf('%-11s %-13s %-7s %4g | %-15s %7.2f %8.2f %8.2f %8.4f | %-15s %7.2f %8.2f %8.2f %8.4f\n', L{i, 2}, ...
        L{i, 1}, dep, L{i, 3}, abrege(a.statut), a.T, 100*a.e_rms, 100*a.e_max, a.J_d, ...
        abrege(b.statut), b.T, 100*b.e_rms, 100*b.e_max, b.J_d);
end

% ---------------------------------------------------------------- échecs (conservés)
fprintf('\nÉchecs et interruptions :\n');
nb = 0;
for il = 1:2
    for i = 1:nL
        r = R.(lois{il})(i).res;
        if ~r.reussi
            nb = nb + 1;
            fprintf('  %-8s %-13s %-11s %g m/s : %s après %.2f s, %.1f %% du parcours\n', lois{il}, L{i, 1}, ...
                L{i, 2}, L{i, 3}, r.statut, r.T, 100*r.portion);
        end
    end
end
if nb == 0, fprintf('  aucun (44 essais menés à terme)\n'); end

% ---------------------------------------------------------------- limites actives (moyenne par loi)
for il = 1:2
    F = reshape([arrayfun(@(x) x.res.lim_frac, R.(lois{il}), 'UniformOutput', false)]', [], 1);
    F = vertcat(F{:});
    fprintf('%-8s fraction moyenne du temps : v amplitude %.1f %%, v variation %.1f %%, δ amplitude %.1f %%, δ variation %.1f %%\n', ...
        lois{il}, 100*mean(F, 1));
end

% ---------------------------------------------------------------- figures
coulPP = [0.07 0.54 0.54]; coulST = [0.14 0.27 0.49];
iv2 = find(strcmp(L(:, 1), 'Vitesse') & [L{:, 3}]' == 2);
figure('Name', 'Trajectoires à 2 m/s', 'Position', [40 40 1400 760]);
for k = 1:5
    subplot(2, 3, k); hold on;
    nom = L{iv2(k), 2};
    if k >= 4, afficher_carte(lire_carte(racine, nom)); else, axis equal; grid on; end
    h1 = plot(ch.(nom).ref.P(:, 1), ch.(nom).ref.P(:, 2), 'k:');
    a = R.pp(iv2(k)).res; b = R.stanley(iv2(k)).res;
    h2 = plot(a.X(:, 1), a.X(:, 2), 'Color', coulPP, 'LineWidth', 1.2);
    h3 = plot(b.X(:, 1), b.X(:, 2), 'Color', coulST, 'LineWidth', 1.0);
    title(sprintf('%s, 2 m/s', nom));
end
legend([h1 h2 h3], {'Référence', 'Pure Pursuit', 'Stanley'}, 'Position', [0.72 0.25 0.12 0.1]);

figure('Name', 'Précision et régularité', 'Position', [60 60 1300 520]);
iv = find(strcmp(L(:, 1), 'Vitesse'));
etiq = arrayfun(@(i) sprintf('%s %g', L{i, 2}, L{i, 3}), iv, 'UniformOutput', false);
subplot(1, 2, 1);
bar(100*[arrayfun(@(i) R.pp(i).res.e_rms, iv), arrayfun(@(i) R.stanley(i).res.e_rms, iv)]);
set(gca, 'XTick', 1:numel(iv), 'XTickLabel', etiq, 'XTickLabelRotation', 60); ylabel('e_{RMS} [cm]');
legend('Pure Pursuit', 'Stanley'); grid on; title('Erreur au point arrière (série Vitesse)');
subplot(1, 2, 2);
bar([arrayfun(@(i) R.pp(i).res.J_d, iv), arrayfun(@(i) R.stanley(i).res.J_d, iv)]);
set(gca, 'XTick', 1:numel(iv), 'XTickLabel', etiq, 'XTickLabelRotation', 60, 'YScale', 'log'); ylabel('J_\delta [rad^2/s^2]');
legend('Pure Pursuit', 'Stanley'); grid on; title('Régularité du braquage (série Vitesse)');

i_sp = iv2(4);
figure('Name', 'Spielberg 2 m/s', 'Position', [80 80 1100 650]);
sig = {'e', 100, 'erreur [cm]'; 'dc', 1, '\delta_c [rad]'; 'vc', 1, 'v_c [m/s]'};
for s = 1:3
    subplot(3, 1, s); hold on; grid on;
    plot(R.pp(i_sp).res.t, sig{s, 2}*R.pp(i_sp).res.(sig{s, 1}), 'Color', coulPP);
    plot(R.stanley(i_sp).res.t, sig{s, 2}*R.stanley(i_sp).res.(sig{s, 1}), 'Color', coulST);
    ylabel(sig{s, 3});
    if s == 2, yline([-P.delta_max P.delta_max], 'r:'); end
end
xlabel('t [s]'); subplot(3, 1, 1); legend('Pure Pursuit', 'Stanley'); title('Spielberg, 2 m/s, pose nominale');

% ---------------------------------------------------------------- résultats bruts
sortie = fullfile(ici, '..', 'resultats');
if ~isfolder(sortie), mkdir(sortie); end
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
writetable(Tb, fullfile(sortie, ['etape7_campagne_' stamp '.csv']));
save(fullfile(sortie, ['etape7_campagne_' stamp '.mat']), 'R', 'L', 'G', 'Tb', '-v7.3');
fprintf('\nRésultats bruts : resultats/etape7_campagne_%s.csv et .mat\n', stamp);

function s = abrege(statut)
s = strrep(strrep(statut, 'interrompu : ', 'INT. '), 'temps maximal atteint', 'TEMPS MAX');
end
