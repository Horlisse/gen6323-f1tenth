function G = reglages_figes()
%REGLAGES_FIGES Réglages GELÉS après l'étape 6 (un seul jeu par loi, docs/plan_reglage.md).
%   À utiliser sans modification pour toute la campagne et le transfert ROS 2.
%   ÉTAT : NON GELÉ. Remplacer ces valeurs par celles retenues par etape6_reglage.m,
%   puis passer G.gele à true et commiter.
G.pp      = struct('l0', 0.6, 'tau', 0.3);
G.stanley = struct('ks', 2.0, 'v0', 1.0);
G.gele    = false;
G.source  = 'réglages de démonstration (à remplacer après l''étape 6)';
end
