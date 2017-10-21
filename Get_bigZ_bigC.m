function [bigZ, bigC] = Get_bigZ_bigC(z_x, z_y, ...
    x0_robot, y0_robot, p_hat, M_robot, K_robot, T_robot, ...
    R_psi, R_cocf_psi, V)
%Returns the big Z matrix as well as the big C (right hand side) vector

% - x0_robot and y0_robot are the current values for the x and y parameters
% of the robot
% - p_hat is used especially when computing Ad_g matrices and it is easier 
% to have it as a parameter passed to the function rather than compute it
% here
% M_robot is the metric tensor of the robot
% K_robot is the curvature tensor of the robot
% T_robot is the torsion form of the robot
% R_psi is another matrix necessary for the computations
% R_cocf_psi is there for the same reason as R_psi
% V is the velocity vector

%% The surface:
% The surface is approximated to a plane:
% @(x,y) [x+cst; y+cst; z_x*x + z_y*y + cst]
    
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Initializing a few matrices and vectors before getting              %%%
%%%             to rewriting the system of equations                    %%%
I_surf = [1+z_x^2, z_x*z_y; z_x*z_y, 1+z_y^2]; %The first fundamental form
M_surf = sqrtm(I_surf);                        %The metric tensor
R_fcf = [-sin(x0_robot)*cos(y0_robot), -sin(y0_robot), -cos(x0_robot)*cos(y0_robot)
         -sin(x0_robot)*sin(y0_robot),  cos(y0_robot), -cos(x0_robot)*sin(y0_robot)
         cos(x0_robot)                      0               -sin(x0_robot)];
Adg_fcf = [R_fcf, p_hat*R_fcf; zeros(3), R_fcf];


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Part three:   Rewriting the system of equations                     %%%
A = (M_robot^(-1))*(K_robot)^(-1);
B = (M_surf^(-1))*R_psi*(K_robot)^(-1);
C_f = T_robot*M_robot;
RR = [R_cocf_psi, zeros(3); zeros(3), R_cocf_psi];
J = [M_surf; zeros(4,2)];
L = eye(6)-((Adg_fcf)^(-2));
Q1 = [0 0 0 0 -1 0
    0 0 0 1 0 0];
Q2 = [1 0 0 0 0 0
    0 1 0 0 0 0];
Q3 = [0 0 0 0 0 1];
Q4 = [0; 0; 0; 0; 0; 1];
S1 = Q1*L*RR*J;
S2 = Q2*L*RR*J;
S3 = Q3*L*RR*J;
T1 = Q1*L*Q4;
T2 = Q2*L*Q4;
T3 = Q3*L*Q4;
V1 = Q1*((Adg_fcf)^(-1))*V;
V2 = Q2*((Adg_fcf)^(-1))*V;
V3 = Q3*((Adg_fcf)^(-1))*V;


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Part four:   Getting the big Z matrix and the big C vector          %%%
bigZ = [eye(2), -A*S1, -A*T1
        zeros(2), eye(2)-B*(S1+K_robot*S2), -B*(T1+K_robot*T2)
        -C_f, -S3, 1-T3];
bigC = [A*V1; B*(V1+K_robot*V2); V3];


end

