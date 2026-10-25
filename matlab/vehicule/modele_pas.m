function q1 = modele_pas(q, v, delta, P, methode)
%MODELE_PAS Un pas du modèle cinématique (1), commande constante sur dt.
%   q = [x; y; psi] au milieu de l'essieu arrière.
%   methode : 'exact' (arc de cercle exact, par défaut) ou 'euler'.
if nargin < 5, methode = 'exact'; end
dt = P.dt;  x = q(1); y = q(2); psi = q(3);
w = v*tan(delta)/P.L;
if strcmp(methode, 'euler')
    q1 = [x + v*cos(psi)*dt; y + v*sin(psi)*dt; psi + w*dt];
    return
end
psi1 = psi + w*dt;
if abs(w*dt) < 1e-9          % quasi rectiligne : point milieu (ordre 2)
    q1 = [x + v*dt*cos(psi + 0.5*w*dt); y + v*dt*sin(psi + 0.5*w*dt); psi1];
else
    q1 = [x + v/w*(sin(psi1) - sin(psi)); y - v/w*(cos(psi1) - cos(psi)); psi1];
end
end
