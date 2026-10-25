function a = wrap_angle(a)
%WRAP_ANGLE Ramène un angle dans [-pi, pi[.
a = mod(a + pi, 2*pi) - pi;
end
