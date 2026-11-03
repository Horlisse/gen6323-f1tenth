function M = mesures(t, e, vc, dc)
%MESURES Indicateurs communs de la section 3.3, équations (4) et (5).
%   t : horodatages t_0 < ... < t_K ; e : erreur au point arrière (référence source)
%   vc, dc : consignes APRÈS saturation effectivement appliquées.
%   e_k est évalué à la fin de chaque intervalle [t_{k-1}, t_k].
dt = diff(t);
if any(dt <= 0), error('mesures:temps', 'Horodatages non strictement croissants.'); end
T = t(end) - t(1);
M.T     = T;
M.e_rms = sqrt(sum(e(2:end).^2 .* dt)/T);
M.e_max = max(e);
M.J_d   = sum(diff(dc).^2 ./ dt)/T;       % rad^2 s^-2
M.J_v   = sum(diff(vc).^2 ./ dt)/T;       % m^2 s^-4
end
