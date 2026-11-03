%TEST_SIGNES Conventions de signe imposées par l'énoncé (section 2.3).
% Droite orientée vers +x ; véhicule parallèle situé à GAUCHE du chemin :
% les deux lois doivent commander un braquage vers la DROITE (delta < 0).
clear; P = params_vehicule();
C = reference_source('droite');
gains_pp = struct('l0', 0.6, 'tau', 0.3);
gains_st = struct('ks', 2.0, 'v0', 1.0);
prog = struct('seg', 1, 's', 0, 's_tot', 0);
v = 2.0;

% Positionner les états de progression près de x = 5 m
for x = 0:0.05:5
    prog = projeter_progression(C, [x; 0], prog);
end
prog_f = prog;
for x = 5:0.05:5 + P.L
    prog_f = projeter_progression(C, [x; 0], prog_f);
end

cas = {'à gauche (y = +0,5)', 0.5, -1; 'à droite (y = -0,5)', -0.5, +1};
ok = true;
for i = 1:size(cas, 1)
    q = [5; cas{i, 2}; 0];
    d_pp = pure_pursuit(q, v, C, prog, gains_pp, P);
    d_st = stanley(q, v, C, prog_f, gains_st, P);
    attendu = cas{i, 3};
    r = sign(d_pp) == attendu && sign(d_st) == attendu;
    ok = ok && r;
    fprintf('Véhicule %-20s PP : %+.6f  Stanley : %+.6f  -> %s\n', cas{i, 1}, d_pp, d_st, ...
        ternaire(r, 'OK', 'ÉCHEC'));
end
fprintf('Valeurs attendues (jumeau Python), véhicule à gauche : PP -0.192954, Stanley -0.321751\n');
assert(ok, 'Convention de signe non respectée.');
disp('Conventions de signe respectées.');

function s = ternaire(c, a, b)
if c, s = a; else, s = b; end
end
