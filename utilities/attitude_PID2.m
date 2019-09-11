% Wil Selby
% Washington, DC
% May 30, 2015

% This function implements a Proportional Integral Derivative Controller
% (PID) for the quadrotor. A lower level controller takes those inputs and 
% controls the error between the deisred and actual Euler angles. 

function attitude_PID2

persistent z_error_sum;
persistent phi_error_sum;
persistent theta_error_sum;
persistent psi_error_sum;

% NLPID for rpy control
% roll
persistent x1_phi x2_phi x1_phi_last x2_phi_last
persistent z1_phi z2_phi z3_phi z1_phi_last z2_phi_last z3_phi_last
persistent u_phi

% pitch
persistent x1_theta x2_theta x1_theta_last x2_theta_last
persistent z1_theta z2_theta z3_theta z1_theta_last z2_theta_last z3_theta_last
persistent u_theta

% yaw
persistent x1_psi x2_psi x1_psi_last x2_psi_last
persistent z1_psi z2_psi z3_psi z1_psi_last z2_psi_last z3_psi_last
persistent u_psi

global Quad

% initialize persistent variables at beginning of simulation
if Quad.init==0
    z_error_sum = 0;
    phi_error_sum = 0;
    theta_error_sum = 0;
    psi_error_sum = 0;
    
	% roll
        x1_phi = 0;
        x2_phi = 0;
        x1_phi_last = 0;
        x2_phi_last = 0;

        z1_phi = 0;
        z2_phi = 0;
        z3_phi  =0;
        z1_phi_last = 0;
        z2_phi_last = 0;
        z3_phi_last = 0;
        
        u_phi = 0;
        
    % pitch
        x1_theta = 0;
        x2_theta = 0;
        x1_theta_last = 0;
        x2_theta_last = 0;

        z1_theta = 0;
        z2_theta = 0;
        z3_theta  =0;
        z1_theta_last = 0;
        z2_theta_last = 0;
        z3_theta_last = 0;
        
        u_theta = 0;

    % yaw
        x1_psi = 0;
        x2_psi = 0;
        x1_psi_last = 0;
        x2_psi_last = 0;

        z1_psi = 0;
        z2_psi = 0;
        z3_psi  =0;
        z1_psi_last = 0;
        z2_psi_last = 0;
        z3_psi_last = 0;
        
        u_psi = 0;
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

phi = Quad.phi;
theta = Quad.theta;
psi = Quad.psi;

% re-estimate parameters
Quad.phi_KP = 1.5;      % KP value in roll control 2
Quad.phi_KI = 0;       % KI value in roll control   1        
Quad.phi_KD = -0.5;     % KD value in roll control  -.5
Quad.phi_max = pi/4;   % Maximum roll angle commanded
Quad.phi_KI_lim = 2*(2*pi/360);  % Error to start calculating integral 

Quad.theta_KP = 1.5;    % KP value in pitch control 2
Quad.theta_KI = 0;     % KI value in pitch control 1
Quad.theta_KD = -0.5;   % KD value in pitch control -.5
Quad.theta_max = pi/4; % Maximum pitch angle commanded
Quad.theta_KI_lim = 2*(2*pi/360);  % Error to start calculating integral 

Quad.psi_KP = 0.30;     % KP value in yaw control
Quad.psi_KI = 0.0;        % KI value in yaw control .75
Quad.psi_KD = -0.65;     % KD value in yaw control -.5
Quad.psi_KI_lim = 8*(2*pi/360);  % Error to start calculating integral 

% nl segment parameters
    ts = Quad.Ts;
    % roll rate ADRC
        %快速因子
        r_phi = 50.0;
        %滤波因子
        h0_phi = 0.01;
        %eso b0因子
        b0_phi = 0.75;
    % roll rate ADRC
        %快速因子
        r_theta = 50.0;
        %滤波因子
        h0_theta = 0.01;
        %eso b0因子
        b0_theta = 0.75;
    % yaw rate ADRC
        %快速因子
        r_psi = 50.0;
        %滤波因子
        h0_psi = 0.01;
        %eso b0因子
        b0_psi = 1.75;

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
    input_phi = Quad.phi_des;
    feedback_phi = phi;
    [x1_phi_last,x2_phi_last] = td3(x1_phi_last,x2_phi_last,input_phi,r_phi,ts,h0_phi);
    x1_phi = x1_phi_last;
    x2_phi = x2_phi_last;

	[z1_phi_last,z2_phi_last,z3_phi_last] = leso3(z1_phi_last,z2_phi_last,z3_phi_last, ...
                                            50, ...
                                            u_phi,b0_phi,feedback_phi,ts);
    z1_phi = z1_phi_last;
    z2_phi = z2_phi_last;
    z3_phi = z3_phi_last;
    
%phi_error = Quad.phi_des - phi;
phi_error = x1_phi - z1_phi;
if(abs(phi_error) < Quad.phi_KI_lim)
    phi_error_sum = phi_error_sum + phi_error;
end
cp = Quad.phi_KP*phi_error;
ci = Quad.phi_KI*Quad.Ts*phi_error_sum;
ci = min(Quad.p_max, max(-Quad.p_max, ci));
cd = Quad.phi_KD*Quad.p;
Quad.p_des = cp + ci + cd;
Quad.p_des = min(Quad.p_max, max(-Quad.p_max, Quad.p_des)) * 0.45;

u_phi = Quad.p_des;

% Pitch PID Controller
    input_theta = Quad.theta_des;
    feedback_theta = theta;
    [x1_theta_last,x2_theta_last] = td3(x1_theta_last,x2_theta_last,input_theta,r_theta,ts,h0_theta);
    x1_theta = x1_theta_last;
    x2_theta = x2_theta_last;

	[z1_theta_last,z2_theta_last,z3_theta_last] = leso3(z1_theta_last,z2_theta_last,z3_theta_last, ...
                                            50, ...
                                            u_theta,b0_theta,feedback_theta,ts);
    z1_theta = z1_theta_last;
    z2_theta = z2_theta_last;
    z3_theta = z3_theta_last;

%theta_error = Quad.theta_des - theta;
theta_error = x1_theta - z1_theta;
if(abs(theta_error) < Quad.theta_KI_lim)
    theta_error_sum = theta_error_sum + theta_error;
end
cp = Quad.theta_KP*theta_error;
ci = Quad.theta_KI*Quad.Ts*theta_error_sum;
ci = min(Quad.q_max, max(-Quad.q_max, ci));
cd = Quad.theta_KD*Quad.q;
Quad.q_des = cp + ci + cd;
Quad.q_des = min(Quad.q_max, max(-Quad.q_max, Quad.q_des)) * 0.45;

u_theta = Quad.q_des;

% Yaw PID Controller
    input_psi = Quad.psi_des;
    feedback_psi = psi;
    [x1_psi_last,x2_psi_last] = td3(x1_psi_last,x2_psi_last,input_psi,r_psi,ts,h0_psi);
    x1_psi = x1_psi_last;
    x2_psi = x2_psi_last;

	[z1_psi_last,z2_psi_last,z3_psi_last] = leso3(z1_psi_last,z2_psi_last,z3_psi_last, ...
                                            50, ...
                                            u_psi,b0_psi,feedback_psi,ts);
    z1_psi = z1_psi_last;
    z2_psi = z2_psi_last;
    z3_psi = z3_psi_last;

%psi_error = Quad.psi_des - psi;
psi_error = x1_psi - z1_psi;
if(abs(psi_error) < Quad.psi_KI_lim)
    psi_error_sum = psi_error_sum + psi_error;
end
cp = Quad.psi_KP*psi_error;
ci = Quad.psi_KI*Quad.Ts*psi_error_sum;
ci = min(Quad.r_max, max(-Quad.r_max, ci));
cd = Quad.psi_KD*Quad.r;
Quad.r_des = cp + ci + cd;
Quad.r_des = min(Quad.r_max, max(-Quad.r_max, Quad.r_des)) * 0.45;

u_psi = Quad.q_des;

end


	function [x1_new,x2_new] = td3(x1_last,x2_last,input,r,h,h0)
        x1_new = x1_last+h*x2_last;
        x2_new = x2_last+h*fhan(x1_last-input,x2_last,r,h0);
    end
    
    function [z1_new,z2_new,z3_new] = leso3(z1_last,z2_last,z3_last, ...
                                        w, ...
                                        input,b0,output,h)
        beta_01 = 3*w;
        beta_02 = 3*w^2;
        beta_03 = w^3;
        e = z1_last-output;
        z1_new = z1_last+h*(z2_last-beta_01*e);
        z2_new = z2_last + h*(z3_last-beta_02*e+b0*input);
        z3_new = z3_last + h*(-beta_03*e);
    end
    
    function fh = fhan(x1_last,x2_last,r,h0)
        d = r*h0;
        d0 = h0*d;
        x1_new = x1_last+h0*x2_last;
        a0 = sqrt(d^2+8*r*abs(x1_new));
        if abs(x1_new)>d0
            a=x2_last+(a0-d)/2*sign(x1_new);
        else
            a = x2_last+x1_new/h0;
        end
        fh = -r*sat(a,d);
    end

    function M=sat(x,delta)
        if abs(x)<=delta
            M=x/delta;
        else
            M=sign(x);
        end
    end















