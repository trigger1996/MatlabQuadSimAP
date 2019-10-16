% Wil Selby
% Washington, DC
% May 30, 2015

% This function implements a Proportional Integral Derivative Controller
% (PID) for the quadrotor. A lower level controller takes those inputs and 
% controls the error between the deisred and actual Euler angles. 

function attitude_Backstepping

persistent z_error_sum;
persistent phi_error_sum;
persistent theta_error_sum;
persistent psi_error_sum;

persistent phi_des_last   p_des_last;
persistent theta_des_last q_des_last;
persistent psi_des_last   r_des_last;

global Quad

C1 = 5;
C2 = 5;
A1 = (Quad.Jy - Quad.Jz) / Quad.Jx;
B1 = -Quad.Jp*Quad.p*Quad.Obar / Quad.Jx;
D1 = Quad.l / Quad.Jx;

C3 = 5;
C4 = 5;
A2 = (Quad.Jz - Quad.Jx) / Quad.Jy;
B2 = Quad.Jp*Quad.p*Quad.Obar / Quad.Jy;
D2 = Quad.l / Quad.Jy;

C5 = 6;
C6 = 6;
A3 = (Quad.Jx - Quad.Jy) / Quad.Jz;
D3 = 1 / Quad.Jz;

% initialize persistent variables at beginning of simulation
if Quad.init==0
    z_error_sum = 0;
    phi_error_sum = 0;
    theta_error_sum = 0;
    psi_error_sum = 0;
    
    Quad.p_des = 0;
    Quad.p_des = 0;
    Quad.r_des = 0;
    
    phi_des_last = 0;
    theta_des_last = 0;
    psi_des_last = 0;
    
    p_des_last = 0;
    q_des_last = 0;
    r_des_last = 0;
end

% % Measurement Model
% if(Quad.ground_truth)
%     phi = Quad.phi;
%     theta = Quad.theta;
%     psi = Quad.psi;
% end
% 
% if(Quad.sensor_unfiltered)
%     phi = Quad.phi_meas;
%     theta = Quad.theta_meas;
%     psi = Quad.psi_meas;
% end
% 
% if(Quad.sensor_kf)
%     phi = Quad.phi;
%     theta = Quad.theta;
%     psi = Quad.psi;
% end

phi = Quad.phi;             % attitude
theta = Quad.theta;
psi = Quad.psi;

p = Quad.p;                 % attitude rate
q = Quad.q;
r = Quad.r;

%% Z Position PID Controller/Altitude Controller
z_error = Quad.Z_des_GF-Quad.Z_BF;
if(abs(z_error) < Quad.Z_KI_lim)
    z_error_sum = z_error_sum + z_error;
end
cp = Quad.Z_KP*z_error;         %Proportional term
ci = Quad.Z_KI*Quad.Ts*z_error_sum; %Integral term
ci = min(Quad.U1_max, max(Quad.U1_min, ci));    %Saturate ci
cd = Quad.Z_KD*Quad.Z_dot;                  %Derivative term
Quad.U1 = -(cp + ci + cd)/(cos(theta)*cos(phi)) + (Quad.m * Quad.g)/(cos(theta)*cos(phi));   %Negative since Thurst and Z inversely related
Quad.U1 = min(Quad.U1_max, max(Quad.U1_min, Quad.U1));


%% Attitude Controller

% Roll PID Controller
phi_error = Quad.phi_des - phi;
if(abs(phi_error) < Quad.phi_KI_lim)
    phi_error_sum = phi_error_sum + phi_error;
end
phi_error_diff = Quad.phi_des - phi_des_last;

Quad.p_des = phi_error_diff + C1 * phi_error;
p_error = Quad.p_des - Quad.p;
p_des_diff = Quad.p_des - p_des_last;

Quad.U2 = phi_error + C2 * p_error + p_des_diff - B1 * Quad.p - Quad.q * Quad.r * A1 + phi_error_sum;
Quad.U2 = Quad.U2 / D1;
%Quad.U2 = Quad.U2 / 10;
Quad.U2 = min(Quad.U2_max, max(Quad.U2_min, Quad.U2));

phi_des_last = Quad.phi_des;
p_des_last   = Quad.p_des;

% Pitch PID Controller
theta_error = Quad.theta_des - theta;
if(abs(theta_error) < Quad.theta_KI_lim)
    theta_error_sum = theta_error_sum + theta_error;
end
theta_error_diff = Quad.theta_des - theta_des_last;

Quad.q_des = theta_error_diff + C3 * theta_error;
q_error = Quad.q_des - Quad.q;
q_des_diff = Quad.q_des - q_des_last;

Quad.U3 = theta_error + C4 * q_error + q_des_diff - B2 * Quad.q - Quad.p * Quad.r * A2 + theta_error_sum;
Quad.U3 = Quad.U3 / D2;
%Quad.U3 = Quad.U3 / 10;
Quad.U3 = min(Quad.U3_max, max(Quad.U3_min, Quad.U3));

theta_des_last = Quad.theta_des;
q_des_last   = Quad.q_des;

% Yaw PID Controller
psi_error = Quad.psi_des - psi;
if(abs(psi_error) < Quad.psi_KI_lim)
    psi_error_sum = psi_error_sum + psi_error;
end
psi_error_diff = Quad.psi_des - psi_des_last;

Quad.r_des = psi_error_diff + C5 * psi_error;
r_error = Quad.r_des - Quad.r;
r_des_diff = Quad.r_des - r_des_last;

Quad.U4 = psi_error + C6 * r_error + r_des_diff - Quad.p * Quad.q * A3 + psi_error_sum;
Quad.U4 = Quad.U4 / D3;
%Quad.U4 = Quad.U4 / 10;
Quad.U4 = min(Quad.U4_max, max(Quad.U4_min, Quad.U4));

psi_des_last = Quad.psi_des;
r_des_last   = Quad.r_des;


end
