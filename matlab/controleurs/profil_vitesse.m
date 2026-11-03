function v_des = profil_vitesse(v_cible, s_restant, ferme, a_frein)
%PROFIL_VITESSE Vitesse désirée avant saturation.
%   Chemin fermé : vitesse cible constante.
%   Chemin ouvert : freinage anticipé, v <= sqrt(2*a_frein*s_restant), pour
%   s'arrêter à moins de 0,20 m de la fin avec v < 0,05 m/s.
%   a_frein = 2,5 m/s^2 < 3 m/s^2 : marge sous la limite de variation imposée.
if nargin < 4, a_frein = 2.5; end
if ferme
    v_des = v_cible;
elseif s_restant < 0.03
    v_des = 0;
else
    v_des = min(v_cible, sqrt(2*a_frein*s_restant));
end
end
