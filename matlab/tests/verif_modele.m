%VERIF_MODELE Vérification du modèle (1) contre la solution analytique.
% Mouvement rectiligne et circulaire à commande constante (sans saturation :
% on vérifie le modèle, pas un essai). Compare intégration exacte et Euler.
clear; P = params_vehicule();
q0 = [1; -2; 0.7]; v = 2.0; T = 10; n = round(T/P.dt);
cas = {'droite', 0; 'cercle \delta=0.2', 0.2; 'cercle \delta=-0.4189', -0.4189};
methodes = {'exact', 'euler'};
figure('Name','Vérification du modèle'); hold on; axis equal; grid on;
for c = 1:size(cas,1)
    delta = cas{c,2};
    for m = 1:2
        q = q0; traj = zeros(3, n+1); traj(:,1) = q; emax = 0;
        for k = 1:n
            q = modele_pas(q, v, delta, P, methodes{m});
            traj(:,k+1) = q;
            qa = solution_analytique(q0, v, delta, k*P.dt, P.L);
            emax = max(emax, norm(q(1:2) - qa(1:2)));
        end
        fprintf('%-22s %-5s erreur position max = %.3e m\n', cas{c,1}, methodes{m}, emax);
        if m == 1, plot(traj(1,:), traj(2,:), 'DisplayName', cas{c,1}); end
    end
    if delta ~= 0
        fprintf('   R theorique = L/tan(delta) = %.6f m\n', P.L/tan(delta));
    end
end
legend show; xlabel('x [m]'); ylabel('y [m]');

function qa = solution_analytique(q0, v, delta, t, L)
if delta == 0
    qa = [q0(1) + v*t*cos(q0(3)); q0(2) + v*t*sin(q0(3)); q0(3)];
    return
end
R = L/tan(delta); c = [q0(1) - R*sin(q0(3)); q0(2) + R*cos(q0(3))];
p = q0(3) + v/R*t;
qa = [c(1) + R*sin(p); c(2) - R*cos(p); p];
end
