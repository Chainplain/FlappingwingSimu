%%%Chianplain Just for test the rotation align e_3 to a specific vector in
%%%3
e_3 = [1,0,0]';
u_1 = [1,1,0]';
u_1_normalized = u_1 / norm(u_1);

k = cross(e_3, u_1_normalized);
c = e_3' * u_1_normalized;
s = norm(k);

R = eye(3)+ skew(k) + (1 - c) / s^2 *  skew(k) *  skew(k)

function R = skew( vec )
R = [     0, -vec(3),  vec(2);...
      vec(3),      0, -vec(1);...
     -vec(2), vec(1),       0];
end