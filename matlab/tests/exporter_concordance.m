%EXPORTER_CONCORDANCE Jeu de cas pour la concordance MATLAB-Python du cœur Pure Pursuit
% (section 3.2 : « poses, vitesses, cibles, paramètres et état de progression »).
% Pour chaque cas : entrées (pose, vitesse, état de progression, consignes précédentes,
% vitesse désirée) et sorties (braquage géométrique, cible, nouvel état, consignes saturées).
% Réglages GELÉS (reglages_figes.m). Graine fixe : jeu reproductible.
% Chemins : huit (croisement) et Spielberg (chemin préparé, gauss 0,3 m, pas 0,10 m).
% Sortie : data/concordance/pp_cas.csv (17 chiffres significatifs).
%
% Lancer depuis matlab/ :  addpath(genpath(pwd)); exporter_concordance
clear;
P = params_vehicule(); G = reglages_figes();
if ~G.gele, error('Réglages non gelés.'); end
ici = fileparts(mfilename('fullpath'));
racine = fullfile(ici, '..', '..', 'data', 'f1tenth_racetracks');
spl_ref = reference_source('Spielberg', racine);
chemins = {'huit', reference_source('huit'); ...
           'Spielberg', creer_chemin(lisser_gauss(spl_ref.P, 0.3, 0.10), true)};
flux = RandStream('mt19937ar', 'Seed', 2026);
n_par_chemin = 25;
vitesses = [0 0.5 1 2 3];

lignes = {};
for ic = 1:size(chemins, 1)
    nom = chemins{ic, 1}; C = chemins{ic, 2};
    for k = 1:n_par_chemin
        j = randi(flux, C.nseg);                         % segment de départ
        t = rand(flux);
        s_in = C.s_start(j) + t*C.seg_len(j);
        A = C.P(j, :)'; B = C.P(mod(j, size(C.P, 1)) + 1, :)';
        base = A + t*(B - A); psi_t = C.seg_psi(j);
        off = 0.8*rand(flux) - 0.4;                       % écart latéral [-0,4 ; 0,4] m
        dpsi = 0.6*rand(flux) - 0.3;                      % écart de cap [-0,3 ; 0,3] rad
        q = [base + off*[-sin(psi_t); cos(psi_t)]; wrap_angle(psi_t + dpsi)];
        v = vitesses(randi(flux, numel(vitesses)));
        vc_prev = 3*rand(flux); dc_prev = 0.8*rand(flux) - 0.4;
        v_des = 2;
        prog_in = struct('seg', j, 's', s_in, 's_tot', s_in);
        [d_geom, prog_out, cible] = pure_pursuit(q, v, C, prog_in, G.pp, P);
        [vc, dc] = saturer_commande(v_des, d_geom, vc_prev, dc_prev, P);
        lignes(end + 1, :) = {nom, q(1), q(2), q(3), v, prog_in.seg, prog_in.s, prog_in.s_tot, ...
            G.pp.l0, G.pp.tau, vc_prev, dc_prev, v_des, ...
            d_geom, cible(1), cible(2), prog_out.seg, prog_out.s, prog_out.s_tot, vc, dc}; %#ok<SAGROW>
    end
end

dossier = fullfile(ici, '..', '..', 'data', 'concordance');
if ~isfolder(dossier), mkdir(dossier); end
f = fullfile(dossier, 'pp_cas.csv');
fid = fopen(f, 'w');
fprintf(fid, ['chemin,x,y,psi,v,seg_in,s_in,s_tot_in,l0,tau,vc_prev,dc_prev,v_des,' ...
              'delta_geom,cible_x,cible_y,seg_out,s_out,s_tot_out,vc,dc\n']);
for i = 1:size(lignes, 1)
    fprintf(fid, '%s', lignes{i, 1});
    fprintf(fid, ',%.17g', lignes{i, 2:end});
    fprintf(fid, '\n');
end
fclose(fid);
fprintf('%d cas exportés dans %s\n', size(lignes, 1), f);
fprintf('Indices de segment : convention MATLAB (à partir de 1).\n');
