R = [1,0,0;...
     0,0,-1;...
     0,1,0];
R_mid =1 / (1 + trace(R)) * ( R' - R );
R_norm = inv(eye(3) + R_mid) * (eye(3) - R_mid);

a = 0: 0.1: 2*pi;
x = 1 - 1 * cos(a) ./ (1 + sin(a).^2 );
y = 1 * sin(a) .* cos(a) ./ (1 + sin(a).^2 );

plot3(x,y,z)