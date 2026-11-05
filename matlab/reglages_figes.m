
function G = reglages_figes()
%REGLAGES_FIGES Réglages GELÉS après l'étape 6 (un seul jeu par loi, docs/plan_reglage.md).
%   À utiliser sans modification pour toute la campagne et le transfert ROS 2.
%   Choix obtenus par la règle déclarée dans le plan (commit e77d6e7) : score minimal,
%   départage à 1 % près par le J_delta avec 100 ms de retard (essai E6).
%   Résultats bruts : resultats/etape6_reglage_20261105_160116.mat et .csv
G.pp      = struct('l0', 0.3, 'tau', 0.2);   % score 0,864 (rang 1 : 0,860, écart 0,5 %)
G.stanley = struct('ks', 5, 'v0', 3);        % score 1,002 (rang 1 : 0,994, écart 0,8 %)
G.gele    = true;
G.source  = 'étape 6, réglage complet du 5 novembre 2026 (MATLAB R2026b)';
end