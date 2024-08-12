% % Clear
%     clear all
%     close all
%     clc
% % Traiettoria
%     fprintf('Choose trajectory: \n');
%     fprintf('1: Circumference \n');
%     fprintf('2: Helix \n');           
%     choiche = input(' ... ');
% % Robot Model Setup
    load('robot.mat')
    load('robot1.mat')
    load('robotmodel.mat')
%     grey = [0.5, 0.5, 0.5];
%     orange = [0.8, 0.6, 0];
% % Simulation Parameters Setup
%     t_in = 0;           % [s]
%     t_fin = 10;         % [s]
%     delta_t = 0.001;    % [s]
%     timeSpan= 10;
%     t = t_in:delta_t:t_fin;
% % Robot Parameters Setup
%     num_of_joints = 7;
%     Q = zeros(num_of_joints,length(t));
%     dQ = zeros(num_of_joints,length(t));
%     ddQ = zeros(num_of_joints,length(t));
%     TAU = zeros(num_of_joints,length(t));
% % Select Trajectory
% switch choiche                  
%     case 1 % Circonferenza
%         q0 = [0 pi/3 0 pi/6 0 0 0];
%         q_dot0 = [0 0 0 0 0 0 0];
%         pos0 = PANDA.fkine(q0).t;
%         radius = 0.1; % raggio dell'elica [m]
%         center = pos0 - [radius;0;0];
% 
%         x = center(1) + radius * cos(t/t(end)*2*pi);
%         y = center(2) * ones(size(x));
%         z = center(3) + radius * sin(t/t(end)*2*pi);
%         theta = 0.1*sin(t/3*2*pi);
%         
%         phi = zeros(size(x));
%         psi = zeros(size(x));
% 
%         xi = [x; y; z; theta; phi; psi]; % twist
%         
%         q_des= generate_trajectory(xi,q0,PANDA);
%         dq_des=gradient(q_des)*1000;
%         ddq_des=gradient(dq_des)*1000;
% 
%         
%     case 2 % Traiettoria elicoidale
%         
%         q0 = [0 pi/3 0 pi/6 0 0 0];
%         q_dot0 = [0 0 0 0 0 0 0];
%         pos0 = PANDA.fkine(q0).t;
%        
%         shift = 0; % passo dell'elica [m] 
%         radius = 0.1; % raggio dell'elica [m]
%         num = 2; % numero di giri [#]
%         center = pos0 - [radius;0;0];
% 
%         x = center(1) + radius * cos(t/t(end)*num*2*pi);
%         y = center(2) + t/t(end)*num*shift;
%         z = center(3) + radius * sin(t/t(end)*num*2*pi);
%         theta = zeros(size(x));
%         phi = zeros(size(x));
%         psi = zeros(size(x));
% 
%         xi = [ x ;y; z; theta ;phi; psi]; % twist
%         
%         
%         q_des= generate_trajectory(xi,q0,PANDA);
%         dq_des=gradient(q_des)*1000;
%         ddq_des=gradient(dq_des)*1000;
%         
% 
% 
% end
% 
% %% Visualize desired trajectory
% 
% figure
% 
% plot3(x,y,z,'r','Linewidth',1.5)
% 
% grid on
% PANDA.plotopt = {'workspace',[-0.75,0.75,-0.75,0.75,0,1]};
% hold on
% for i=1:100:length(q_des)
%     
%     PANDA.plot(transpose(q_des(:,i)),'floorlevel',0,'fps',100,'trail','-k','linkcolor',orange,'jointcolor',grey)
% 
% end
% 
% %% Trajectory Tracking: Computed Torque Method
% 
% % Gain circumference parameters matrix
% Kp = 1*diag([0 1 3 3 5 3 30]);
% Kv = 0.5*diag([48 18 27 20 20 20 20]);
% 
% % Good Helix parameters matrix
% % Kp = 200*diag([3 3 3 3 5 3 5]);
% % Kv = 25*diag([1 1 1 1 70 2 70]);
% 
% results_computed_torque = q0;
% index = 1;
% q=q0 ;
% dq=q_dot0;
% ddq=[0 0 0 0 0 0 0];
% for i=1:length(t)
% 
%    % Error and derivate of the error   
%     err = transpose(q_des(:,i)) - q;
%     derr = transpose(dq_des(:,i)) - dq;
%     
%     %Get dynamic matrices
%    
%     G = get_GravityVector(q);
%     C= get_CoriolisVector(q,dq);
%     M = get_MassMatrix(q);
% 
%     % Computed Torque Controller
%     
%     tau = ( M*(ddq_des(:,1) + Kv*(derr') + Kp*(err')) + C + G )';
%       
%     % Robot joint accelerations
%     ddq_old = ddq;
%     ddq = (pinv(M)*(tau - C'- G')')';
%         
%     % Tustin integration
%     dq_old = dq;
%     dq = dq + (ddq_old + ddq) * delta_t / 2;
%     q = q + (dq + dq_old) * delta_t /2;
%     
%     % Store result for the final plot
%     results_computed_torque(index,:) = q;
%     index = index + 1;
% 
% end
% 
% 
% %% Plot computed torque results for trajectory tracking
% 
figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t(1:10001),results_computed_torque(1:10001,j))
%     legend ()
    hold on
    plot (t,q_des(j,1:length(t)))
    legend ('Computed Torque','Desired angle')
    grid;
end
% 
% 
% %% Trajectory tracking: Backstepping control
% 
% 
% % Good Circumference parameters
% Kp = 1* diag([1 1 1 1 3 1 1]);
% 
% % Good Helix parameters
% % Kp = diag([1 1 1 1 3 1 1]);
% 
% 
% results_backstepping = q0;
% index = 1;
% q=q0;
% dq=q_dot0;
% ddq=[0 0 0 0 0 0 0];
% for i=1:length(t)
% 
%    % Error and derivate of the error   
%     err = transpose(q_des(:,i)) - q;
%     derr = transpose(dq_des(:,i)) - dq;
%     
%     dqr = transpose(dq_des(:,i)) + err*(Kp);
%     ddqr = transpose(ddq_des(:,i)) + derr*(Kp);
%     s = derr + err*(Kp');
%      
%     %Get dynamic matrices
% 
%     G = get_GravityVector(q);
%     C = get_CoriolisMatrix(q,dq);
%     M = get_MassMatrix(q);
% 
% 
%     % Backstepping Controller
%     tau = (M*(ddqr') + C*(dqr') + G + Kp*(s') + err')';      
%     
%     % Robot joint accelerations
%     ddq_old = ddq;
%     ddq = (pinv(M)*(tau - transpose(C*(dq'))- G')')';
%         
%     % Tustin integration
%     dq_old = dq;
%     dq = dq + (ddq_old + ddq) * delta_t / 2;
%     q = q + (dq + dq_old) * delta_t /2;
%     
%     % Store result for the final plot
%     results_backstepping(index,  :) = q;
%     index = index + 1;
% 
% end
% 
% %% Plot computed torque results for backstepping control
% 
% figure
% for j=1:num_of_joints
%     subplot(4,2,j);
%     plot(t,results_backstepping(:,j))
%     hold on
%     plot (t,q_des(j,1:length(t)))
% %     hold on
% %     plot(t,results_computed_torque(:,j))
%     xlabel('time [s]');
%     ylabeltext = sprintf('_%i [rad]',j);
%     ylabel(['Joint position' ylabeltext]);
%     grid;
%     legend ('Backstepping Results','Desired angle')
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Selezione del controllo da eseguire
fprintf('Selezionare il tipo di controllo: \n');
fprintf('     #1: adaptive computed torque \n');
fprintf('     #2 backstepping \n');
sel2 = input(' ... ');
% Numero di giunti
n=5;
%% Perturbazione iniziale dei parametri
int = 10; % Percentuale della perturbazione sui parametri
for j = 1:n 
    KUKAmodel.links(j).m = KUKAmodel.links(j).m .* (1+int/100); 
end
%% Simulazione
    % Init
        % Init posizione giunti
            q0 = [-0.1047    1.5184   -1.7279    0.1571   -0.2618]; % partiamo in una posizione diversa da quella di inizio traiettoria
            q = zeros(length(t),n); 
            q(1,:) = q0; 
        % Init velocità giunti
            q_dot = zeros(length(t),n);
            q_dot(1,:) = q_dot0(1:5); 
        % Init coppia
            tau = zeros(length(t),n); 
        % Vettore dei parametri dinamici 
            piArray = zeros(length(t),n*10); 
            pi0 = zeros(1,n*10); 
        % Backstepping init
            qr_dot = zeros(length(t),n); 
            qr_ddot = zeros(length(t),n); 
    % Caricare parametri dinamici
        % Ciclo for da 10 in 10
            for j = 1:n
                pi0((j-1)*10+1:j*10) = [KUKAmodel.links(j).m KUKAmodel.links(j).m*KUKAmodel.links(j).r KUKAmodel.links(j).I(1,1) 0 0 KUKAmodel.links(j).I(2,2) 0 KUKAmodel.links(j).I(3,3)];
            end
        % Loading Params
            piArray(1,:) = pi0; 
    % pesi Computed torque 
            Kp = 1*diag([200 200 200 20 10]);
            Kv = 0.1*diag([200 200 200 10 1]); 
    % Pesi Backstepping
            Kd = 0.1*diag([200 200 200 20 1]);
            lambda = diag([200, 200, 200, 200, 200])*0.03;
    % P e R fanno parte della candidata di Lyapunov, quindi devono essere definite positive
            R = diag(repmat([1e1 repmat(1e3,1,3) 1e2 1e7 1e7 1e2 1e7 1e2],1,n)); 
            P = 0.01*eye(10);
tic
for i = 2:length(t)
    %% Interruzione della simulazione se q diverge
    if any(isnan(q(i-1,:)))
        fprintf('Simulazione interrotta! \n')
        return
    end

    % Loading posizione/velocità/accelerazione desiderata
        qd_dot=dq_des(1:5,:)';
        qd_ddot=ddq_des(1:5,:)';
        qd=q_des(1:5,:)';

    % Calcolo dell'errore: e, e_dot
        e = qd(i-1,:) - q(i-1,:); 
        e_dot = qd_dot(i-1,:) - q_dot(i-1,:); 
        % Backstepping
            s = (e_dot + e*lambda);
    % Velocità riferimento Backsteping
        qr_dot(i-1,:) = qd_dot(i-1,:) + e*lambda;
    % Accelerazione riferimento Backstepping
        if (i > 2)
            qr_ddot(i-1,:) = (qr_dot(i-1) - qr_dot(i-2)) / delta_t;
        end
    
    %% Calcolo della coppia (a partire dal modello)
        % Aggiorno Stima massa
            KUKAmodel.links(j).m = piArray(i-1,(j-1)*10+1); % elemento 1 di pi
        % Determino matrici dinamiche stimate 
            Mtilde = KUKAmodel.inertia(q(i-1,:)); 
            Ctilde = KUKAmodel.coriolis(q(i-1,:),q_dot(i-1,:)); 
            Gtilde = KUKAmodel.gravload(q(i-1,:)); 
        % Determino la coppia in base alla scelta
            switch sel2
                case 1  %   Computed torque
                    tau(i,:) = qd_ddot(i-1,:)*Mtilde' + q_dot(i-1,:)*Ctilde' + Gtilde + e_dot*Kv' + e*Kp'; 
                case 2  %   Backstepping
                    tau(i,:) = qr_ddot(i-1,:)*Mtilde' + qr_dot(i-1,:)*Ctilde' + Gtilde + s*Kd' + e*Kp'; 
            end
    %% Dinamica del manipolatore (reale)
    % entrano tau, q e q_dot, devo calcolare M, C e G e ricavare q_ddot
        M = KUKA.inertia(q(i-1,:)); 
        C = KUKA.coriolis(q(i-1,:),q_dot(i-1,:)); 
        G = KUKA.gravload(q(i-1,:)); 
    
        q_ddot = (tau(i,:) - q_dot(i-1,:)*C' - G) * (M')^(-1); 
    % integro q_ddot due volte e ricavo q e q_dot
        q_dot(i,:) = q_dot(i-1,:) + delta_t*q_ddot; 
        q(i,:) = q(i-1,:) + delta_t*q_dot(i,:); 
    %% Dinamica dei parametri
        % 1                      % 2                      % 3                      % 4                      % 5
        q1 = q(i,1);             q2 = q(i,2);             q3 = q(i,3);             q4 = q(i,4);             q5 = q(i,5);
        q1_dot = q_dot(i,1);     q2_dot = q_dot(i,2);     q3_dot = q_dot(i,3);     q4_dot = q_dot(i,4);     q5_dot = q_dot(i,5);
        qd1_dot = qd_dot(i,1);   qd2_dot = qd_dot(i,2);   qd3_dot = qd_dot(i,3);   qd4_dot = qd_dot(i,4);   qd5_dot = qd_dot(i,5);
        qd1_ddot = qd_ddot(i,1); qd2_ddot = qd_ddot(i,2); qd3_ddot = qd_ddot(i,3); qd4_ddot = qd_ddot(i,4); qd5_ddot = qd_ddot(i,5);
        g = 9.81;
        regressor2;
        switch sel2
             case 1  %   Computed torque
                            piArray_dot = ( R^(-1) * Y' * (Mtilde')^(-1) * [zeros(n) eye(n)] * P * [e e_dot]' )'; 
                            piArray(i,:) = piArray(i-1,:) + delta_t*piArray_dot; 
             case 2  %   Backstepping
                            piArray_dot = (R^(-1) * Y' * s')';  
                            piArray(i,:) = piArray(i-1,:) + delta_t*piArray_dot; 
        end    
    %% Progresso Simulazione
    if mod(i,100) == 0
        
        fprintf('Percent complete: %0.1f%%.',100*i/(length(t)-1));
        hms = fix(mod(toc,[0, 3600, 60])./[3600, 60, 1]);
        fprintf(' Elapsed time: %0.0fh %0.0fm %0.0fs. \n', ...
            hms(1),hms(2),hms(3));
    end
    
end
figure
for j=1:5
    subplot(4,2,j);
    plot(t(1:10001),q(1:10001,j))
%     legend ()
    hold on
    plot (t,q_des(j,1:length(t)))
    legend ('Computed Torque','Desired angle')
    grid;
end
%% Plot Dynamics parameter
figure
subplot(5,1,1);
plot(t(1:10001),piArray(1:10001,1))
legend ('Mass Link 1')
hold on
subplot(5,1,2);
plot(t(1:10001),piArray(1:10001,11))
legend ('Mass Link 2')
subplot(5,1,3);
plot(t(1:10001),piArray(1:10001,21))
legend ('Mass Link 3')
subplot(5,1,4);
plot(t(1:10001),piArray(1:10001,31))
legend ('Mass Link 4')
subplot(5,1,5);
plot(t(1:10001),piArray(1:10001,41))
% plot(t(1:10001),piArray(1:10001,51))
legend ('Mass Link 5')
grid;


