function R = simuler_essai(ref, C, q0, v_cible, loi, gains, P)
%SIMULER_ESSAI Un essai de la campagne MATLAB (section 3.1).
%   ref     : référence SOURCE (mesures communes, longueur S0 pour Tmax)
%   C       : chemin fourni au contrôleur (= ref si aucun prétraitement)
%   q0      : pose initiale [x; y; psi] ; départ au repos, braquage nul
%   v_cible : vitesse cible [m/s]
%   loi     : 'pp' ou 'stanley' ; gains : struct de la loi
%   Actionnement idéal du modèle cinématique : v = vc et delta = dc à chaque pas.

Tmax = 2*ref.longueur/v_cible + 10;
Nmax = ceil(Tmax/P.dt) + 2;
t = zeros(Nmax, 1); X = zeros(Nmax, 3); e = zeros(Nmax, 1);
vc_h = zeros(Nmax, 1); dc_h = zeros(Nmax, 1); cible = nan(Nmax, 2); lim = false(Nmax, 4);

init = struct('seg', 1, 's', 0, 's_tot', 0);
prog = init; prog_f = init; prog_arr = init; prog_ref = init;
q = q0(:); vc = 0; dc = 0;
[prog_ref, ~, ~, e0] = projeter_progression(ref, q(1:2), prog_ref);
X(1, :) = q'; e(1) = abs(e0);
fin = ref.P(end, :)';                 % fin d'un chemin ouvert (droite : (20, 0))
statut = 'temps maximal atteint';
k = 1;

while (k - 1)*P.dt < Tmax - 1e-9
    % --- Loi de commande (la vitesse courante est vc : actionnement idéal)
    switch loi
        case 'pp'
            [d_geom, prog, c] = pure_pursuit(q, vc, C, prog, gains, P);
            prog_arr = prog;
        case 'stanley'
            [d_geom, prog_f, c] = stanley(q, vc, C, prog_f, gains, P);
            prog_arr = projeter_progression(C, q(1:2), prog_arr);
        otherwise
            error('Loi inconnue : %s', loi);
    end
    % --- Vitesse désirée, saturations, intégration
    v_des = profil_vitesse(v_cible, C.longueur - prog_arr.s_tot, C.ferme);
    [vc, dc, actif] = saturer_commande(v_des, d_geom, vc, dc, P);
    q = modele_pas(q, vc, dc, P);
    k = k + 1;

    % --- Mesure au point arrière sur la référence source, en respectant la progression
    [prog_ref, ~, ~, ek] = projeter_progression(ref, q(1:2), prog_ref);
    t(k) = (k - 1)*P.dt; X(k, :) = q'; e(k) = abs(ek);
    vc_h(k) = vc; dc_h(k) = dc; cible(k, :) = c';
    lim(k, :) = [actif.v_amp, actif.v_rate, actif.d_amp, actif.d_rate];

    % --- Conditions de fin ou d'interruption
    if ~all(isfinite(q)),            statut = 'interrompu : valeur non finie'; break; end
    if abs(ek) > 2,                  statut = 'interrompu : erreur > 2 m';     break; end
    if ref.ferme && prog_ref.s_tot >= ref.longueur, statut = 'tour complet'; break; end
    if ~ref.ferme && norm(q(1:2) - fin) < 0.20 && vc < 0.05, statut = 'arrêt atteint'; break; end
end

idx = 1:k;
R = mesures(t(idx), e(idx), vc_h(idx), dc_h(idx));
R.statut   = statut;
R.reussi   = any(strcmp(statut, {'tour complet', 'arrêt atteint'}));
R.portion  = min(prog_ref.s_tot/ref.longueur, 1);
R.lim_frac = mean(lim(2:k, :), 1);    % [v amplitude, v variation, delta amplitude, delta variation]
R.t = t(idx); R.X = X(idx, :); R.e = e(idx); R.vc = vc_h(idx); R.dc = dc_h(idx); R.cible = cible(idx, :);
R.loi = loi; R.gains = gains; R.v_cible = v_cible;
end
