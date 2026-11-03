%CAMPAGNE_REFERENCE 24 essais (3 chemins x 2 lois x 4 conditions) comparés au jumeau Python.
% Sert de vérification de concordance MATLAB-Python (exigée en phase 2) :
% mêmes algorithmes, mêmes réglages, donc mêmes résultats à l'arrondi près.
% Réglages de DÉMONSTRATION : ils ne remplacent pas le réglage documenté de l'étape 5.
clear; P = params_vehicule();
gains.pp      = struct('l0', 0.6, 'tau', 0.3);
gains.stanley = struct('ks', 2.0, 'v0', 1.0);

% chemin, loi, v_cible, récupération, T, e_RMS, e_max, J_delta, J_v (jumeau Python)
ATTENDU = {
    'droite', 'pp', 1, false, 20.33, 0.0000000000, 0.0000000000, 0.0000000000, 0.2754147832;
    'droite', 'pp', 2, false, 10.69, 0.0000000000, 0.0000000000, 0.0000000000, 1.0383838444;
    'droite', 'pp', 3, false, 7.72, 0.0000000000, 0.0000000000, 0.0000000000, 2.1534196700;
    'droite', 'pp', 2, true, 10.72, 0.1415003274, 0.5131370907, 0.1547585334, 1.0360794129;
    'droite', 'stanley', 1, false, 20.33, 0.0000000000, 0.0000000000, 0.0000000000, 0.2754147832;
    'droite', 'stanley', 2, false, 10.69, 0.0000000000, 0.0000000000, 0.0000000000, 1.0383838444;
    'droite', 'stanley', 3, false, 7.72, 0.0000000000, 0.0000000000, 0.0000000000, 2.1534196700;
    'droite', 'stanley', 2, true, 10.71, 0.1459444657, 0.5127791395, 0.1883591911, 1.0366666228;
    'cercle', 'pp', 1, false, 31.58, 0.0000017666, 0.0000096320, 0.0064975429, 0.0943635212;
    'cercle', 'pp', 2, false, 16.04, 0.0000022636, 0.0000105141, 0.0127925438, 0.3728179551;
    'cercle', 'pp', 3, false, 10.97, 0.0000025955, 0.0000105233, 0.0187048681, 0.8204193254;
    'cercle', 'pp', 2, true, 16.01, 0.1123351831, 0.5112754654, 0.1118334125, 0.3735165522;
    'cercle', 'stanley', 1, false, 31.52, 0.0105665695, 0.0109176386, 0.0087952554, 0.0945431472;
    'cercle', 'stanley', 2, false, 16.01, 0.0103868865, 0.0109201771, 0.0170618547, 0.3735165522;
    'cercle', 'stanley', 3, false, 10.95, 0.0101967703, 0.0109365302, 0.0245411585, 0.8219178082;
    'cercle', 'stanley', 2, true, 15.93, 0.1197744825, 0.5109630400, 0.1368706294, 0.3753923415;
    'huit', 'pp', 1, false, 48.90, 0.0125961483, 0.0262771658, 0.0012363516, 0.0609406953;
    'huit', 'pp', 2, false, 24.67, 0.0266209537, 0.0541942588, 0.0041348697, 0.2423996757;
    'huit', 'pp', 3, false, 16.69, 0.0461879709, 0.0923403159, 0.0077280871, 0.5392450569;
    'huit', 'pp', 2, true, 24.70, 0.0969414896, 0.5131395701, 0.0711151898, 0.2421052632;
    'huit', 'stanley', 1, false, 48.77, 0.0140288811, 0.0313319895, 0.0026515635, 0.0611031372;
    'huit', 'stanley', 2, false, 24.63, 0.0138466174, 0.0309489209, 0.0071577246, 0.2427933415;
    'huit', 'stanley', 3, false, 16.70, 0.0136064080, 0.0305010060, 0.0143582772, 0.5389221557;
    'huit', 'stanley', 2, true, 24.67, 0.0968960539, 0.5127843605, 0.0886533285, 0.2423996757;
};

tol = struct('T', 0.006, 'e', 1e-6, 'J', 1e-6);
n_ok = 0;
fprintf('%-7s %-8s %3s %-6s %-14s %7s %9s %8s %9s %9s  %s\n', 'Chemin', 'Loi', 'v', 'Départ', 'Statut', ...
    'T [s]', 'eRMS [m]', 'emax [m]', 'J_delta', 'J_v', 'Concordance');
refs = struct();
tic;
for i = 1:size(ATTENDU, 1)
    [nom, loi, v, recup] = ATTENDU{i, 1:4};
    if ~isfield(refs, nom), refs.(nom) = reference_source(nom); end
    ref = refs.(nom);
    R = simuler_essai(ref, ref, pose_initiale(nom, ref, recup), v, loi, gains.(loi), P);
    att = [ATTENDU{i, 5:9}];
    obt = [R.T, R.e_rms, R.e_max, R.J_d, R.J_v];
    ecart = abs(obt - att);
    ok = ecart(1) <= tol.T && all(ecart(2:3) <= tol.e) && all(ecart(4:5) <= tol.J);
    n_ok = n_ok + ok;
    if recup, depart = 'récup'; else, depart = 'nomin'; end
    if ok, verdict = 'OK'; else, verdict = sprintf('ÉCART max %.2e', max(ecart)); end
    fprintf('%-7s %-8s %3d %-6s %-14s %7.2f %9.5f %8.4f %9.5f %9.5f  %s\n', nom, loi, v, depart, ...
        R.statut, R.T, R.e_rms, R.e_max, R.J_d, R.J_v, verdict);
end
fprintf('\n%d/%d essais concordent avec le jumeau Python (%.1f s).\n', n_ok, size(ATTENDU, 1), toc);
