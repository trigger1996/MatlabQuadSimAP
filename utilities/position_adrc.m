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

function position_adrc

persistent x_error_sum;
persistent y_error_sum;

persistent x1_X x2_X x1_X_last x2_X_last
persistent z1_X z2_X z3_X z1_X_last z2_X_last z3_X_last
persistent u_X

persistent x1_Y x2_Y x1_Y_last x2_Y_last
persistent z1_Y z2_Y z3_Y z1_Y_last z2_Y_last z3_Y_last
persistent u_Y

global Quad

% initialize persistent variables at beginning of simulation
if Quad.init==0
    x_error_sum = 0;
    y_error_sum = 0;
    
	% X
    x1_X = 0;
    x2_X = 0;
	x1_X_last = 0;
    x2_X_last = 0;
    z1_X = 0;
    z2_X = 0;
	z3_X = 0;
    z1_X_last = 0;
	z2_X_last = 0;
    z3_X_last = 0;
    
    u_X = 0;
    
    % Y
    x1_Y = 0;
    x2_Y = 0;
	x1_Y_last = 0;
    x2_Y_last = 0;
    z1_Y = 0;
    z2_Y = 0;
	z3_Y = 0;
    z1_Y_last = 0;
	z2_Y_last = 0;
    z3_Y_last = 0;
    
	u_Y = 0;
    
end

Quad.X_KP = .05;                % KP value in X position control
Quad.X_KI = .05;                % KI value in X position control
Quad.X_KD = -.33;               % KD value in X position control
Quad.X_KI_lim = .15;            % Error to start calculating integral term

Quad.Y_KP = .05;                % KP value in Y position control
Quad.Y_KI = .05;                % KI value in Y position control
Quad.Y_KD = -.33;               % KD value in Y position control
Quad.Y_KI_lim = .15;            % Error to start calculating integral term

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

% Rotate Desired Position from GF to BF (Z axis rotation only)
[Quad.X_des,Quad.Y_des,Quad.Z_des] = rotateGFtoBF(Quad.X_des_GF,Quad.Y_des_GF,Quad.Z_des_GF,0*phi,0*theta,psi);

% Rotate Current Position from GF to BF
[Quad.X_BF,Quad.Y_BF,Quad.Z_BF] = rotateGFtoBF(x,y,z,phi,theta,psi);

% Rotate Current Velocity from GF to BF
[Quad.X_BF_dot,Quad.Y_BF_dot,Quad.Z_BF_dot] = rotateGFtoBF(Quad.X_dot,Quad.Y_dot,Quad.Z_dot,phi,theta,psi);

%%
    % X
    input_X  = Quad.X_des;
    feedback_X = Quad.X_BF;

    %td 模块 构造过渡过程
    [x1_X_last,x2_X_last] = td3(x1_X_last,x2_X_last,input_X,Quad.r_X,Quad.Ts,Quad.h0_X);
    x1_X = x1_X_last;
    x2_X = x2_X_last;
    %线性eso模块
    [z1_X_last,z2_X_last,z3_X_last] = leso3(z1_X_last,z2_X_last,z3_X_last, ...
                                            Quad.w_X, ...
                                            u_X,Quad.b0_X,feedback_X,Quad.Ts);
    z1_X = z1_X_last;
    z2_X = z2_X_last;
    z3_X = z3_X_last;    

% X Position PID controller 
x_error = x1_X - z1_X;
if(abs(x_error) < Quad.X_KI_lim)
    x_error_sum = x_error_sum + x_error;
end
cp = Quad.X_KP*x_error;    %Proportional term
ci = Quad.X_KI*Quad.Ts*x_error_sum;
ci = min(Quad.theta_max, max(-Quad.theta_max, ci));    %Saturate ci
cd = Quad.X_KD*Quad.X_BF_dot;                     %Derivative term
Quad.theta_des =  - (cp + ci + cd);   %Theta and X inversely related
Quad.theta_des = fal2(Quad.theta_des, 2, 0.707*Quad.theta_max);

Quad.theta_des = min(Quad.theta_max, max(-Quad.theta_max, Quad.theta_des));
Quad.theta_des = Quad.theta_des * 0.33;
u_X = Quad.theta_des;

%%
    input_Y  = Quad.Y_des;
    feedback_Y = Quad.Y_BF;

    %ts 模块 构造过渡过程
    [x1_Y_last,x2_Y_last] = td3(x1_Y_last,x2_Y_last,input_Y,Quad.r_Y,Quad.Ts,Quad.h0_Y);
    x1_Y = x1_Y_last;
    x2_Y = x2_Y_last;
	%线性eso模块
    [z1_Y_last,z2_Y_last,z3_Y_last] = leso3(z1_Y_last,z2_Y_last,z3_Y_last, ...
                                            Quad.w_Y, ...
                                            u_Y,Quad.b0_Y,feedback_Y,Quad.Ts);
    z1_Y = z1_Y_last;
    z2_Y = z2_Y_last;
    z3_Y = z3_Y_last;  

% Y Position PID controller    
y_error = x1_Y - z1_Y;
if(abs(y_error) < Quad.Y_KI_lim)
    y_error_sum = y_error_sum + y_error;
end
cp = Quad.Y_KP*y_error;    %Proportional term
ci = Quad.Y_KI*Quad.Ts*y_error_sum;
ci = min(Quad.phi_max, max(-Quad.phi_max, ci));    %Saturate ci
cd = Quad.Y_KD*Quad.Y_BF_dot;                      %Derivative term
Quad.phi_des = cp + ci + cd;
Quad.phi_des = fal2(Quad.phi_des, 2, 0.707*Quad.phi_max);


Quad.phi_des = min(Quad.phi_max, max(-Quad.phi_max, Quad.phi_des));
Quad.phi_des = Quad.phi_des * 0.33;
u_Y = Quad.phi_des;

end

    function [x1_new,x2_new] = td3(x1_last,x2_last,input,r,h,h0)
        x1_new = x1_last+h*x2_last;
        x2_new = x2_last+h*fhan(x1_last-input,x2_last,r,h0);
    end

    function u0 = nlsef3(e1,e2,c,r,h1)
        u0 = -fhan(e1,c*e2,r,h1);
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
    
    function fe = fal2(error, pow_, threshold)
        if abs(error) > threshold
            fe = abs(error)^pow_ * sign(error);
        else
            fe = error;         % error / threshold^pow_
        end
    end
