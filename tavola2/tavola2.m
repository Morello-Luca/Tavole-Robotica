%% Tavola 2 %%
clc
clear all

%% variabili di stato
    % World                 % Body
      syms x z theta;         syms u w q
    % Coeff resistenza avanzamento
       syms Clu Cqu Clw Cqw Clq Cqq
    % Ingressi (coppie)
        syms tau_u tau_q
    % Stato sistema         % derivata
        syms x_w x_b xi;      syms dx_w dx_b dxi;
    % non linear 
        syms f g1 g2
    % dynamic param
        syms m I


% %% Massa e Inerzia del modello
%     m = 115;        %[kg]
%     I = 5.98;       %[kg*m^2]
% 
% %% Coefficienti di resistenza
%     Clu=17.24;     % [kg/s]
%     Cqu=106.03;    % [kg/m]
% 
%     Clw=38.06;     % [kg/s]
%     Cqw=84.1;      % [kg/m]
% 
%     Clq=1.18;      % [kg*m^2]
%     Cqq=7.51;      % [kg*m^2/rad^2]        

%% matrici dinamiche 

% massa
M = [ m, 0, 0  ;
      0, m, 0  ;
      0, 0, I ];
% coriolis
C = [  0    0   m*w ;
       0    0  -m*u ;
     -m*w  m*u   0 ];
% resistenza avanzamento
D = [ Clu+Cqu*abs(u)        0             0;
           0         Clw+Cqw*abs(w)       0;
           0                0       Clq+Cqq*abs(q)];
% ingressi
I = [ tau_u; 0 ; tau_q];

% rotazione body->world
R = [ cos(theta) sin(theta)  0;
     -sin(theta) cos(theta)  0;
          0           0      1;];

% stati init
x_w = [x;z;theta];
x_b = [u;w;q];
xi = [x_w;x_b];

% def deriv stati
dx_w = R*x_b;
dx_b = - inv(M)*C*x_b - inv(M)*D*x_b   ;  

f  = [dx_w ; dx_b];
g1 = [0; 0; 0; 1; 0; 0];
g2 = [0; 0; 0; 0; 0; 1];
dxi = f + g1*tau_u + g2*tau_q;

%% Controllabilità
    % Lie bracket [f, g]
        lie1 = jacobian(g1,xi)*f - jacobian(f,xi)*g1;
        lie2 = jacobian(g2,xi)*f - jacobian(f,xi)*g2;
    % Lie bracket [f, [f,g]]
        dlie1 = jacobian(lie1,xi)*f - jacobian(f,xi)*lie1;
        dlie2 = jacobian(lie2,xi)*f - jacobian(f,xi)*lie2;
    % Lie bracket [f, [f,[f,g]]]
        ddlie1 = jacobian(dlie1,xi)*f - jacobian(f,xi)*dlie1;
        ddlie2 = jacobian(dlie2,xi)*f - jacobian(f,xi)*dlie2;   
    % < Delta | Delta_0>
        accessibility = [g1,g2,lie1,lie2,dlie1,dlie2,ddlie1,ddlie2];
        simplify(accessibility);
        rank_controllo= rank(accessibility);
% check state theta and q

   Z=subs(accessibility,theta,0);
   rank_controllo_theta_null= rank(Z);
    
   ninety = acos(0);
   N= subs(accessibility,theta,ninety);
   rank_controllo_theta_ninety= rank(N);
    
   % ho la q al denominatore da qualche parte
   %V = subs(accessibility,q,0);

% subs(accessibility,Clu,17.24);
% subs(accessibility,Cqu,106.03);
% subs(accessibility,Clw,38.06);
% subs(accessibility,Cqw,84.1);
% subs(accessibility,Clq,1.18);
% subs(accessibility,Cqq,7.51);
% subs(accessibility,m,115);
% %subs(accessibility,I,5.98);
% simplify(accessibility);

 %% Osservabilità

 % init vettore h
    syms h    
    h = [z;theta];

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

    Omega = [ omega0;
              omega1_1;
              omega1_2;
              omega1_3; 
              omega2_1;
              omega2_2;
              omega2_3;
              omega3_1;
              omega3_2;
              omega3_3;
              omega3_1_;
              omega3_2_;
              omega3_3_
              ];


%     Omega = [omega0;
%               omega1_1;
%               omega2_1;
%               omega2_2;
%               omega3_1;
%               omega3_2;
%               omega3_3;
%               omega3_1_;
%               ];


rango_omega = rank(Omega);

   
null(Omega);