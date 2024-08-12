
syms alpha
syms beta
syms gamma


Rx = [1,      0,       0;
      0, cos(gamma), -sin(gamma);
      0, sin(gamma),  cos(gamma)];

Ry = [cos(beta), 0, sin(beta);
           0, 1,     0;
     -sin(beta), 0, cos(beta)];

Rz = [cos(alpha),-sin(alpha), 0;
      sin(alpha), cos(alpha), 0;
           0,      0, 1];

R = Rx*Ry*Rz;