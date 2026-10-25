function P = params_vehicule()
%PARAMS_VEHICULE Paramètres communs imposés par l'énoncé (section 2.1).
P.L          = 0.3302;   % empattement [m]
P.v_max      = 5.0;      % [m/s]
P.delta_max  = 0.4189;   % [rad]
P.a_max      = 3.0;      % |dv_c/dt| max [m/s^2]
P.ddelta_max = 3.2;      % |ddelta_c/dt| max [rad/s]
P.dt         = 0.01;     % pas de simulation [s]
end
