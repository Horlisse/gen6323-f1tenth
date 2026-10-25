function q0 = pose_initiale(nom, ref, recuperation)
%POSE_INITIALE Pose nominale (tableau 1 / annexe A.3) ou perturbée.
if nargin < 3, recuperation = false; end
switch nom
    case 'droite', q0 = [0; 0; 0];
    case 'cercle', q0 = [5; 0; pi/2];
    case 'huit',   q0 = [0; 0; pi/4];
    otherwise   % circuits : premier point et cap du premier segment
        a0 = ref.P(1,:); a1 = ref.P(2,:);
        q0 = [a0(1); a0(2); atan2(a1(2)-a0(2), a1(1)-a0(1))];
end
if recuperation
    psi = q0(3);
    q0 = [q0(1) - 0.5*sin(psi); q0(2) + 0.5*cos(psi); wrap_angle(psi + pi/18)];
end
end
