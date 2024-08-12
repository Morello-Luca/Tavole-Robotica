%% Tavola 2 %%
clc
clear all

syms m I

% variabili di stato
    % World
        syms x z theta
    % Body
        syms u w q
    % Coeff resistenza avanzamento
        syms Clu Cqu Clw Cqw Clq Cqq
    % Ingressi (coppie)
        syms tau_u tau_q
    % Stato sistema  
        syms x_w x_b xi
    % derivata stato
        syms dx_w dx_b dxi
    % non linear 
        syms f g1 g2
% matrici dinamiche 

M = [m,0,0;
     0,m,0
     0,0,I];

C = [ 0 0 m*w;
       0 0 -m*u;
       -m*w m*u 0];
D = [Clu+Cqu*abs(u) 0 0;
     0              Clw+Cqw*abs(w) 0;
     0              0   Clq+Cqq*abs(q)];
I = [tau_u;0;tau_q];


R = [cos(theta) sin(theta) 0;
     -sin(theta) cos(theta) 0;
      0               0     1;];

% stati init

x_w = [x;z;theta];
x_b = [u;w;q];
xi = [x_w;x_b];
% def deriv stati
dx_w = R*x_b;

dx_b = - inv(M)*C*x_b - inv(M)*D*x_b   ;  

f = [ dx_w;dx_b ];
g1 = [0;0;0;1;0;0];
g2 = [0;0;0;0;0;1];
dxi = f + g1*tau_u + g2*tau_q;

% Controllabilità
% lie bracket [f,g]
   lie1 = jacobian(g1,xi)*f - jacobian(f,xi)*g1;
   lie2 = jacobian(g2,xi)*f - jacobian(f,xi)*g2;
% lie bracket [f,[f,g]]
   dlie1 = jacobian(lie1,xi)*f-jacobian(f,xi)*lie1;
   dlie2 = jacobian(lie2,xi)*f-jacobian(f,xi)*lie2;
% lie bracket [f,[f,[f,g]]]
   ddlie1 = jacobian(dlie1,xi)*f-jacobian(f,xi)*dlie1;
   ddlie2 = jacobian(dlie2,xi)*f-jacobian(f,xi)*dlie2;   

   Ran = [g1,g2,lie1,lie2,dlie1,dlie2,ddlie1,ddlie2];
  rank(Ran);

 % Osservabilità

 % init vettore h
    syms h
    h = x+z+theta;
    omega0 = jacobian(h,xi);
    delta = [f,g1,g2];
    omega1_1 = jacobian((omega0*f),xi);
    omega1_2 = jacobian((omega0*g1),xi);
    omega1_3 = jacobian((omega0*g2),xi);

    omega2_1 = jacobian((omega1_1*f),xi);
    omega2_2 = jacobian((omega1_1*g1),xi);
    omega2_3 = jacobian((omega1_1*g2),xi);

    omega3_1 = jacobian((omega2_1*f),xi);
    omega3_2 = jacobian((omega2_1*g1),xi);
    omega3_3 = jacobian((omega2_1*g2),xi);

    omega3_1_ = jacobian((omega2_2*f),xi);
    omega3_2_ = jacobian((omega2_2*g1),xi);
    omega3_3_ = jacobian((omega2_2*g2),xi);

    Omega = [omega0;
              omega1_1;
              %omega1_2;
              %omega1_3; 
              omega2_1;
              omega2_2;
              %omega2_3;
              omega3_1;
              omega3_2;
              omega3_3;
              omega3_1_;
              %omega3_2_;
              %omega3_3_
              ];




   
   