%ETAPE4_CIRCUITS Import, vérification et préparation de Spielberg et Silverstone.
%   1. Import des CSV sources (ordre conservé, fichiers non modifiés)
%   2. Contrôles de l'annexe A : nombre de points, S0, fermeture, cap initial
%   3. Poses initiales nominale et de récupération (valeurs pour sim_gen6323.yaml)
%   4. Superposition à la carte (PNG + YAML, sans recentrer le CSV)
%   5. Qualité des données et comparaison de prétraitements, y compris l'effet sur la commande
%   6. Export du chemin retenu : s_m,x_m,y_m,psi_rad
%
% Lancer depuis le dossier matlab/ du dépôt :  addpath(genpath(pwd)); etape4_circuits
clear; close all;
P = params_vehicule();

% ---------------------------------------------------------------- paramètres
ici = fileparts(mfilename('fullpath'));
racine = fullfile(ici, '..', 'data', 'f1tenth_racetracks');   % sous-module au commit b95c4ef
sortie = fullfile(ici, '..', 'data', 'chemins_prepares');
circuits = {'Spielberg', 'Silverstone'};
attendu = struct( ...
    'Spielberg',   struct('n', 864,  'S0', 343.323, 'psi0', -2.87898), ...
    'Silverstone', struct('n', 1178, 'S0', 457.925, 'psi0',  0.94440));
ds = 0.10;                                % pas de rééchantillonnage [m]
candidats = {'aucun', 0; 'gauss', 0.5; 'gauss', 1.5};   % méthode, sigma [m]
comparer_commande = true;                 % essais PP et Stanley à 2 m/s par candidat
gains_pp = struct('l0', 0.6, 'tau', 0.3); % réglages de DÉMONSTRATION (le gel se fait à l'étape 6)
gains_st = struct('ks', 2.0, 'v0', 1.0);

% CHOIX RETENU : à fixer par l'équipe APRÈS lecture du tableau, puis figer.
% Le même chemin préparé servira aux deux lois, à toutes les vitesses et à ROS 2.
choix = struct('methode', 'gauss', 'sigma', 0.5);

if ~isfolder(racine)
    error(['Données introuvables : %s\nRécupérer le sous-module : ' ...
        'git submodule update --init'], racine);
end
if ~isfolder(sortie), mkdir(sortie); end
kappa_vehicule = tan(P.delta_max)/P.L;    % courbure maximale réalisable
demi_largeur = 0.31/2;                    % largeur F1TENTH 0,31 m

for ic = 1:numel(circuits)
    nom = circuits{ic};
    fprintf('\n=========================== %s ===========================\n', nom);

    % ------------------------------------------------ 1-2. import et contrôles
    ref = reference_source(nom, racine);
    A = analyser_chemin(ref.P, true);
    q0 = pose_initiale(nom, ref);
    qr = pose_initiale(nom, ref, true);
    att = attendu.(nom);
    controles = {
        'nombre de points',      A.n == att.n,                      sprintf('%d (attendu %d)', A.n, att.n)
        'longueur S0 [m]',       abs(A.S0 - att.S0) < 1e-3,         sprintf('%.6f (attendu ~%.3f)', A.S0, att.S0)
        'cap initial [rad]',     abs(q0(3) - att.psi0) < 1e-5,      sprintf('%.8f (attendu ~%.5f)', q0(3), att.psi0)
        'position initiale',     norm(q0(1:2)) < 1e-9,              sprintf('(%.6g, %.6g)', q0(1), q0(2))
        'premier point non dupliqué', norm(ref.P(1,:) - ref.P(end,:)) > 1e-9, 'segment de fermeture implicite'
        'valeurs finies, aucun segment nul', true,                  sprintf('segment min %.4f m', A.seg_min)
        };
    for i = 1:size(controles, 1)
        if controles{i, 2}, etat = 'OK'; else, etat = 'ÉCART'; end
        fprintf('  %-36s %-6s %s\n', controles{i, 1}, etat, controles{i, 3});
    end
    fprintf('  sens de parcours (ordre du CSV)       %s\n', A.sens);

    % ------------------------------------------------ 3. poses pour sim_gen6323.yaml
    fprintf('\n  Pose nominale (sim_gen6323.yaml) : sx: %.10f  sy: %.10f  stheta: %.10f\n', q0);
    fprintf('  Pose de récupération             : sx: %.10f  sy: %.10f  stheta: %.10f\n', qr);

    % ------------------------------------------------ 4. carte
    carte = lire_carte(racine, nom);
    libres = points_libres(carte, ref.P(:, 1), ref.P(:, 2));
    fprintf('\n  Carte : %d x %d pixels, résolution %.5f m, origine (%.4f, %.4f)\n', ...
        carte.W, carte.H, carte.resolution, carte.origin(1), carte.origin(2));
    fprintf('  Points de la ligne centrale en zone libre : %d / %d\n', sum(libres), numel(libres));
    w = ref.largeurs;
    fprintf('  Demi-largeur de piste min : droite %.2f m, gauche %.2f m (demi-largeur véhicule %.3f m)\n', ...
        min(w(:, 1)), min(w(:, 2)), demi_largeur);

    figure('Name', [nom ' : carte'], 'Position', [60 60 900 700]);
    afficher_carte(carte);
    plot(ref.P(:, 1), ref.P(:, 2), '-', 'Color', [0.07 0.54 0.54], 'LineWidth', 1.2);
    plot(ref.P(~libres, 1), ref.P(~libres, 2), 'rx', 'MarkerSize', 8);
    quiver(q0(1), q0(2), 4*cos(q0(3)), 4*sin(q0(3)), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    plot(q0(1), q0(2), 'ro', 'MarkerFaceColor', 'r');
    title(sprintf('%s : ligne centrale source, départ et sens', nom));

    % ------------------------------------------------ 5. qualité et prétraitements
    fprintf('\n  Qualité des points sources : segments %.3f / %.3f / %.3f m (min / moy / max)\n', ...
        A.seg_min, A.seg_moy, A.seg_max);
    fprintf('  |dpsi| max par sommet %.4f rad, |kappa| max %.3f 1/m (limite véhicule %.3f 1/m)\n', ...
        A.dpsi_max, A.kappa_max, kappa_vehicule);

    fprintf('\n  %-16s %8s %10s | %9s %9s | %9s %9s | %s\n', 'Prétraitement', 'kmax', 'écart max', ...
        'PP eRMS', 'PP J_d', 'ST eRMS', 'ST J_d', 'statuts');
    chemins = cell(size(candidats, 1), 1);
    for c = 1:size(candidats, 1)
        [Pc, etiquette] = preparer(ref.P, candidats{c, 1}, candidats{c, 2}, ds);
        chemins{c} = Pc;
        Ac = analyser_chemin(Pc, true);
        if strcmp(candidats{c, 1}, 'aucun'), ecart = 0;
        else, ecart = max([ecart_chemins(Pc, ref.P, true); ecart_chemins(ref.P, Pc, true)]); end
        ligne = sprintf('  %-16s %8.3f %8.2f cm', etiquette, Ac.kappa_max, 100*ecart);
        if comparer_commande
            C = creer_chemin(Pc, true);
            rp = simuler_essai(ref, C, q0, 2, 'pp', gains_pp, P);
            rs = simuler_essai(ref, C, q0, 2, 'stanley', gains_st, P);
            ligne = sprintf('%s | %9.4f %9.4f | %9.4f %9.4f | %s / %s', ligne, rp.e_rms, rp.J_d, ...
                rs.e_rms, rs.J_d, rp.statut, rs.statut);
        end
        fprintf('%s\n', ligne);
    end

    % ------------------------------------------------ 6. chemin retenu : figures et export
    [Pch, etiquette] = preparer(ref.P, choix.methode, choix.sigma, ds);
    Ach = analyser_chemin(Pch, true);
    ecarts = ecart_chemins(Pch, ref.P, true);
    libres_ch = points_libres(carte, Pch(:, 1), Pch(:, 2));
    fprintf('\n  Chemin retenu : %s, %d points, longueur %.3f m, écart max %.2f cm, moyen %.2f cm\n', ...
        etiquette, size(Pch, 1), sum(hypot(diff([Pch(:,1); Pch(1,1)]), diff([Pch(:,2); Pch(1,2)]))), ...
        100*max(ecarts), 100*mean(ecarts));
    fprintf('  Points du chemin retenu en zone libre : %d / %d\n', sum(libres_ch), numel(libres_ch));

    % Figure avant / après autour du virage le plus serré (annexe A.2)
    [~, imax] = max(abs(A.kappa));
    centre = ref.P(imax, :);
    figure('Name', [nom ' : prétraitement'], 'Position', [80 80 1100 450]);
    subplot(1, 2, 1); hold on; axis equal; grid on;
    plot(ref.P(:, 1), ref.P(:, 2), 'o-', 'Color', [0.55 0.6 0.66], 'MarkerSize', 3, 'DisplayName', 'Source');
    plot(Pch(:, 1), Pch(:, 2), '-', 'Color', [0.07 0.54 0.54], 'LineWidth', 1.4, 'DisplayName', etiquette);
    xlim(centre(1) + [-4 4]); ylim(centre(2) + [-4 4]); legend('Location', 'best');
    title('Avant / après, virage le plus serré'); xlabel('x [m]'); ylabel('y [m]');
    subplot(1, 2, 2); hold on; grid on;
    s_src = [0; cumsum(A.seg(1:end - 1))];
    s_ch = [0; cumsum(Ach.seg(1:end - 1))];
    plot(s_src, A.kappa, 'Color', [0.55 0.6 0.66], 'DisplayName', 'Source');
    plot(s_ch, Ach.kappa, 'Color', [0.07 0.54 0.54], 'LineWidth', 1.2, 'DisplayName', etiquette);
    yline([-kappa_vehicule, kappa_vehicule], 'r:', 'DisplayName', 'Limite véhicule');
    xlabel('s [m]'); ylabel('\kappa [1/m]'); legend('Location', 'best'); title('Courbure discrète');

    fichier = fullfile(sortie, sprintf('%s_chemin.csv', nom));
    exporter_chemin(Pch, true, fichier, sprintf('%s, pas %.2f m, depuis %s_centerline.csv (b95c4ef)', ...
        etiquette, ds, nom));
    fprintf('  Exporté : %s\n', fichier);
end

function [Pc, etiquette] = preparer(Psrc, methode, sigma, ds)
switch methode
    case 'aucun'
        Pc = Psrc; etiquette = 'aucun (source)';
    case 'gauss'
        Pc = lisser_gauss(Psrc, sigma, ds); etiquette = sprintf('gauss s=%.1f m', sigma);
    otherwise
        error('Méthode inconnue : %s', methode);
end
end
