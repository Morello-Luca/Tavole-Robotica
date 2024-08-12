%% Tavola 3 %%

%% Comment and objective
    % feedback linearization for the system described in the 2-th section
%% Clear workspace
    clc
    clear all
%% Init variables
    % State Variables
            % Mass/Inertia
                    syms m I
            % World
                    syms x z theta
            % Body
                    syms u w q
            % Advance resistence 
                    syms Clu Cqu Clw Cqw Clq Cqq
            % Inputs 
                    syms tau_u tau_q
            % states  
                    syms x_w x_b xi
            % state derivative
                    syms dx_w dx_b dxi
            % non linear components 
                    syms f g1 g2
%% Matrix 
    % Mass
        M = [m,0,0;
             0,m,0
             0,0,I];
    % Coriolis
        C = [ 0   0  m*w;
              0   0 -m*u;
            -m*w m*u  0 ];
    % Resistence 
        D = [Clu+Cqu*abs(u)      0              0;
                0          Clw+Cqw*abs(w)       0;
                0                0       Clq+Cqq*abs(q)];
    % Inputs
        I = [tau_u;0;tau_q];
    % Convertion from body to ned
        R = [cos(theta) sin(theta) 0;
            -sin(theta) cos(theta) 0;
                0            0     1];
%% Init state
    % world
            x_w = [x;z;theta];
    % body
            x_b = [u;w;q];
    % Extended state
            xi = [x_w;x_b];
%% Kinematic
    % world
            dx_w = R*x_b;
    % body
            dx_b = - inv(M)*C*x_b - inv(M)*D*x_b;  
    % Non-linear sistem
            f = [ dx_w;dx_b];
            g1 = [0;0;0;1;0;0];
            g2 = [0;0;0;0;0;1];
            dxi = f + g1*tau_u + g2*tau_q;

%% LINEARIZZAZIONE in retroazione MIMO
    % Outputs 
        % y1 = x
        % y2 = z
        % y3 = theta
    % Procedure
        % dh = jacobian (h,x);
        % Lfh = dh*f; 
        % dy = Lfh+  dh*g*tau;



 %--- First --------------------------------------------------------------- 
          h_1 = x; 
         dh_1 = jacobian(h_1,xi);       Lfh_1 =  dh_1*f;
        d2h_1 = jacobian(Lfh_1,xi);    Lf2h_1 = d2h_1*f;
        d3h_1 = jacobian(Lf2h_1,xi);   Lf3h_1 = d3h_1*f;
        d4h_1 = jacobian(Lf3h_1,xi);   Lf4h_1 = d4h_1*f;

                %---------------------------------------------------------%
           y1 = h_1;
         d_y1 = Lfh_1     +dh_1*g1*tau_u      +dh_1*g2 *tau_q;     
        d2_y1 = Lf2h_1   +d2h_1*g1*tau_u     +d2h_1*g2 *tau_q; 
        d3_y1 = Lf3h_1   +d3h_1*g1*tau_u     +d3h_1*g2 *tau_q; 
        d4_y1 = Lf4h_1   +d4h_1*g1*tau_u     +d4h_1*g2 *tau_q; %%
 %--- Second --------------------------------------------------------------
          h_2 = z; 
         dh_2 = jacobian(h_2,xi);         Lfh_2 = dh_2*f;
        d2h_2 = jacobian(Lfh_2,xi);      Lf2h_2 = d2h_2*f;
        d3h_2 = jacobian(Lf2h_2,xi);     Lf3h_2 = d3h_2*f;
        d4h_2 = jacobian(Lf3h_2,xi);     Lf4h_2 = d4h_2*f;
                %---------------------------------------------------------%
           y2 = h_2;
         d_y2 = Lfh_2    +dh_2*g1*tau_u      +dh_2*g2 *tau_q;     
        d2_y2 = Lf2h_2  +d2h_2*g1*tau_u     +d2h_2*g2 *tau_q;  % input <> 0
        d3_y2 = Lf3h_2  +d3h_2*g1*tau_u     +d3h_2*g2 *tau_q;
        d4_y2 = Lf4h_2  +d4h_2*g1*tau_u     +d4h_2*g2 *tau_q;
 %--- Third ---------------------------------------------------------------
          h_3 = theta; 
         dh_3 = jacobian(h_3,xi);        Lfh_3 = dh_3*f;
        d2h_3 = jacobian(Lfh_3,xi);     Lf2h_3 = d2h_3*f;
                %---------------------------------------------------------%
           y3 = h_3;
         d_y3 = Lfh_3    +dh_3*g1*tau_u      +dh_3*g2 *tau_q;     
        d2_y3 = Lf2h_3  +d2h_3*g1*tau_u     +d2h_3*g2 *tau_q;  % input <> 0

%% Matrix Form
        y1_ = Lf2h_1 + d2h_1*g1*tau_u;
        y2_ = Lf2h_2 + d2h_2*g1*tau_u;
        y3_ = Lf2h_3 + d2h_3*g2*tau_q;

 % Retroazione 
        syms ni1 ni2 ni3
        syms u1 u2 u3
        syms alpha1 beta1
        syms alpha2 beta2
        syms alpha3 beta3

        alpha1 = -Lf2h_1/(d2h_1*g1);
        beta1 =  1/(d2h_1*g1);
        u1 = alpha1 +beta1*ni1;

        alpha2 = -Lf2h_2/(d2h_2*g1);
        beta2 =  1/(d2h_2*g1);
        u2 = alpha2 +beta2*ni2;

        alpha3 = -Lf2h_3/(d2h_3*g2);
        beta3 =  1/(d2h_3*g2);
        u3 = alpha3 +beta3*ni3;

        yr1 =  simplifyFraction(Lf2h_1 + d2h_1*g1*u1);
        yr2 =  simplifyFraction(Lf2h_2 + d2h_2*g1*u2);
        yr3 =  simplifyFraction(Lf2h_3 + d2h_3*g2*u3);


syms    v__1 v__2
       v_=[v__1;v__2];
        E = [d2h_1*g1,0;
             0,d2h_3*g2  ];

        gamma = [Lf2h_1;Lf2h_3];

        input = -inv(E)*gamma+inv(E)*v_;



        Y = simplify(gamma +E*input);

%% Change variables 

% every output has grade 2, it doesn't matter which output we chose
    
    % First     output x
        syms    gi1 dgi1
        
         gi1 = y1;
        dgi1 = d_y1;
    % Second    output theta
        syms    gi2 dgi2

         gi2 = y3;
        dgi2 = d_y3;
        syms    v_1 v_2

    % alpha vector beta matrix
        
        alpha=[alpha1;alpha3];
        beta = diag ([beta1,beta3]);
        
        u_ = [u1;u3];
        v_ = [v_1;v_2];
        A0 = [0,1;
              0,0];
        A = blkdiag(A0,A0);

        B0 = [0;1];
        B = blkdiag(B0,B0);
    % nes
        syms Gi1 Gi2 
        syms dGi1 dGi2 
        Gi1 = [gi1;dgi1];
        Gi2 = [gi2;dgi2];
        

        Gi = [Gi1;Gi2];
        
        dGi = A*Gi+B*v_;




















%% Massa e Inerzia del modello
    m = 115;        %[kg]
    I = 5.98;       %[kg*m^2]

%% Coefficienti di resistenza
    Clu=17.24;     % [kg/s]
    Cqu=106.03;    % [kg/m]

    Clw=38.06;     % [kg/s]
    Cqw=84.1;      % [kg/m]

    Clq=1.18;      % [kg*m^2]
    Cqq=7.51;      % [kg*m^2/rad^2]

%% Dimensioni ambiente di lavoro
    D1=300;         %[m]
    D2=50;          %[m]

% plot(out.x.data(1,:),out.z.data(1,:))
% hold on
% plot(out.xnom.data(1,:),out.znom.data(1,:))
% 
% 
% plot(out.x.time',out.xnom.data(1,:)-out.x.data(1,:))
% hold on
% plot(out.z.time',out.znom.data(1,:)-out.z.data(1,:))



 
   