% Wil Selby
% Washington, DC
% May 30, 2015

% This function implements a Proportional Integral Derivative Controller
% (PID) for the quadrotor. A high level controller outputs desired roll and
% pitch angles based on errors between the Global and desired X and Y
% positions. A lower level controller takes those inputs and controls the
% error between the deisred and actual Euler angles. After the control
% inputs are calculated, the desired motor speeds are calculated. See
% www.wilselby.com for derivations of these equations.

function outer_Backstepping

persistent x_error_sum vx_error_sum;
persistent y_error_sum vy_error_sum;

global Quad

persistent y_des_last vy_des_last;
persistent x_des_last vx_des_last;

C1 = 0.5;
C2 = 0.5;
A1 = -[cos(Quad.phi)*sin(Quad.theta)*cos(Quad.psi)+sin(Quad.phi)*sin(Quad.psi)] / Quad.m;
B1 = -Quad.Kdx / Quad.m;

C3 = 0.5;
C4 = 0.5;
A2 =  -[cos(Quad.phi)*sin(Quad.psi)*sin(Quad.theta)-cos(Quad.psi)*sin(Quad.phi)] / Quad.m;
B2 = -Quad.Kdy / Quad.m;

vx_KI_lim = 0.05;
vy_KI_lim = 0.05;

% initialize persistent variables at beginning of simulation
if Quad.init==0
    x_error_sum = 0;
    y_error_sum = 0;

    vx_error_sum = 0;
    vy_error_sum = 0;

    x_des_last = 0;
    y_des_last = 0;

    vx_des_last = 0;
    vy_des_last = 0;
end

%% High Level Position Controller

% Measurement Model
% if(Quad.ground_truth)
%     x = Quad.X;
%     y = Quad.Y;
%     z = Quad.Z;
%     phi = Quad.phi;
%     theta = Quad.theta;
%     psi = Quad.psi;
% end
% 
% if(Quad.sensor_unfiltered)
%     x = Quad.X_meas;
%     y = Quad.Y_meas;
%     z = Quad.Z_meas;
%     phi = Quad.phi_meas;
%     theta = Quad.theta_meas;
%     psi = Quad.psi_meas;
% end
% 
% if(Quad.sensor_kf)
%     x = Quad.X;
%     y = Quad.Y;
%     z = Quad.Z;
%     phi = Quad.phi;
%     theta = Quad.theta;
%     psi = Quad.psi;
% end

x = Quad.X;
y = Quad.Y;
z = Quad.Z;
phi = Quad.phi;
theta = Quad.theta;
psi = Quad.psi;

%%
% Rotate Desired Position from GF to BF (Z axis rotation only)
%[Quad.X_des,Quad.Y_des,Quad.Z_des] = rotateGFtoBF(Quad.X_des_GF,Quad.Y_des_GF,Quad.Z_des_GF,0*phi,0*theta,psi);

% Rotate Current Position from GF to BF
%[Quad.X_BF,Quad.Y_BF,Quad.Z_BF] = rotateGFtoBF(x,y,z,phi,theta,psi);

% Rotate Current Velocity from GF to BF
%[Quad.X_BF_dot,Quad.Y_BF_dot,Quad.Z_BF_dot] = rotateGFtoBF(Quad.X_dot,Quad.Y_dot,Quad.Z_dot,phi,theta,psi);

%%
% Ux solver
x_error = Quad.X_des_GF - x;
if(abs(x_error) < Quad.X_KI_lim)
    x_error_sum = x_error_sum + x_error;
end
x_des_diff = Quad.X_des_GF - x_des_last;

vx_error = Quad.X_dot - x_des_diff - C1 * x_error;
z9  = x_error;
z10 = vx_error;

Ux = -Quad.m / Quad.U1 * (-C2*z10 + z9 - C1 * (C1 * z9 + z10));
if Quad.U1 == 0
    Ux = 0;
end
x_des_last  = Quad.X_des_GF;


%%
% Uy solver
y_error = Quad.Y_des_GF - y;
if(abs(y_error) < Quad.Y_KI_lim)
    y_error_sum = y_error_sum + y_error;
end
y_des_diff = Quad.Y_des_GF - y_des_last;

vy_error = Quad.Y_dot - y_des_diff - C3 * y_error;
z11 = y_error;
z12 = vy_error;

Uy = -Quad.m / Quad.U1 * (-C4*z12 + z11 - C3 * (C3 * z10 + z12));
if Quad.U1 == 0
    Uy = 0;
end
y_des_last  = Quad.Y_des_GF;

%%
Quad.phi_des = asin(Ux*sin(Quad.psi) - Uy*cos(Quad.psi));
Quad.theta_des   = asin(Ux / (cos(Quad.phi) * cos(Quad.psi))- (sin(Quad.phi) * sin(Quad.psi)) / (cos(Quad.phi) * cos(Quad.psi)));
aaa = Ux*sin(Quad.psi) - Uy*cos(Quad.psi)
if abs(aaa) > 1
    bbb = 1;
end

Quad.phi_des = min(Quad.phi_max, max(-Quad.phi_max, Quad.phi_des));
Quad.theta_des = min(Quad.theta_max, max(-Quad.theta_max, Quad.theta_des));

end
