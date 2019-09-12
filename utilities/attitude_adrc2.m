
function attitude_adrc2

persistent x1_p x2_p x3_p x1_p_last x2_p_last x3_p_last
persistent z1_p z2_p z3_p z4_p z1_p_last z2_p_last z3_p_last z4_p_last

persistent x1_q x2_q x3_q x1_q_last x2_q_last x3_q_last
persistent z1_q z2_q z3_q z4_q z1_q_last z2_q_last z3_q_last z4_q_last

persistent x1_r x2_r x3_r x1_r_last x2_r_last x3_r_last
persistent z1_r z2_r z3_r z4_r z1_r_last z2_r_last z3_r_last z4_r_last

persistent z_error_sum;

global Quad

ts = Quad.Ts;

if Quad.init==0
    % z
        z_error_sum = 0;
    
    % roll
        x1_p = 0;
        x2_p = 0;
        x3_p = 0;
        x1_p_last = 0;
        x2_p_last = 0;
        x3_p_last = 0;

        z1_p = 0;
        z2_p = 0;
        z3_p = 0;
        z4_p = 0;
        z1_p_last = 0;
        z2_p_last = 0;
        z3_p_last = 0;
        z4_p_last = 0;     
  
    % pitch
        x1_q = 0;
        x2_q = 0;
        x3_q = 0;
        x1_q_last = 0;
        x2_q_last = 0;
        x3_q_last = 0;

        z1_q = 0;
        z2_q = 0;
        z3_q = 0;
        z4_q = 0;
        z1_q_last = 0;
        z2_q_last = 0;
        z3_q_last = 0;
        z4_q_last = 0;     
        
    % yaw
        x1_r = 0;
        x2_r = 0;
        x3_r = 0;
        x1_r_last = 0;
        x2_r_last = 0;
        x3_r_last = 0;

        z1_r = 0;
        z2_r = 0;
        z3_r = 0;
        z4_r = 0;
        z1_r_last = 0;
        z2_r_last = 0;
        z3_r_last = 0;
        z4_r_last = 0;             
end

% initiate params
    % roll rate ADRC
        %快速因子
        r1_p = 50.0;
        r2_p = 50.0;
        %滤波因子
        h0_p = 0.01;
        %eso b0因子
        b0_p = 0.75;
    % roll rate ADRC
        %快速因子
        r1_q = 50.0;
        r2_q = 50.0;        
        %滤波因子
        h0_q = 0.01;
        %eso b0因子
        b0_q = 0.75;
    % yaw rate ADRC
        %快速因子
        r1_r = 50.0;
        r2_r = 50.0;        
        %滤波因子
        h0_r = 0.01;
        %eso b0因子
        b0_r = 1.75;   

%% Z Position PID Controller/Altitude Controller
phi = Quad.phi;
theta = Quad.theta;
%psi = Quad.psi;

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
        
%% Roll       
    roll_exp = Quad.phi_des;   
    roll     = Quad.phi;
    w_roll   = Quad.p;
    
    x1_p_last = x1_p;
	x2_p_last = x2_p;
	x3_p_last = x3_p;
	z1_p_last = z1_p;
	z2_p_last = z2_p;
    z3_p_last = z3_p;
    z4_p_last = z4_p;
    
    [u_p, x1_p, x2_p, x3_p, z1_p, z2_p, z3_p, z4_p] = attitude_adrc(roll_exp, roll, 0, ...
                                                                    x1_p_last, x2_p_last, x3_p_last, ...
                                                                    z1_p_last, z2_p_last, z3_p_last, z4_p_last, ...
                                                                    r1_p, r2_p, ts, h0_p, 0.3, 5, b0_p, 0.3, 0.3);
%% Pitch
    pitch_exp = Quad.theta_des;
    pitch     = Quad.theta;
    w_pitch   = Quad.q;    

    x1_q_last = x1_q;
	x2_q_last = x2_q;
	x3_q_last = x3_q;
	z1_q_last = z1_q;
	z2_q_last = z2_q;
    z3_q_last = z3_q;
    z4_q_last = z4_q;
    
    [u_q, x1_q, x2_q, x3_q, z1_q, z2_q, z3_q, z4_q] = attitude_adrc(pitch_exp, pitch, 0, ...
                                                                    x1_q_last, x2_q_last, x3_q_last, ...
                                                                    z1_q_last, z2_q_last, z3_q_last, z4_q_last, ...
                                                                    r1_q, r2_q, ts, h0_q, 0.3, 5, b0_q, 0.3, 0.3);    
    
%% Yaw    
    yaw_exp = Quad.psi_des;      
    yaw     = Quad.psi;
    w_yaw   = Quad.r;
    
    x1_r_last = x1_r;
	x2_r_last = x2_r;
	x3_r_last = x3_r;
	z1_r_last = z1_r;
	z2_r_last = z2_r;
    z3_r_last = z3_r;
    z4_r_last = z4_r;
    
    [u_r, x1_r, x2_r, x3_r, z1_r, z2_r, z3_r, z4_r] = attitude_adrc(yaw_exp, yaw, 0, ...
                                                                    x1_r_last, x2_r_last, x3_r_last, ...
                                                                    z1_r_last, z2_r_last, z3_r_last, z4_r_last, ...
                                                                    r1_r, r2_r, ts, h0_r, 0.3, 5, b0_r, 0.3, 0.3);    

%% Mixer
% roll
    Quad.U2 = u_p;
    Quad.U2 = min(Quad.U2_max, max(Quad.U2_min, Quad.U2));
    Quad.U2 =  Quad.U2 * 1.0 * 10^(-3);
% pitch
    Quad.U3 = u_q;
    Quad.U3 = min(Quad.U3_max, max(Quad.U3_min, Quad.U3));
    Quad.U3 =  Quad.U3 * 1.0 * 10^(-3);
% yaw
    Quad.U4 = u_r;
    Quad.U4 = min(Quad.U4_max, max(Quad.U4_min, Quad.U4));
	Quad.U2 =  Quad.U4 * 1.0 * 10^(-3);
    
end

function [u, x1, x2, x3, z1, z2, z3, z4] = attitude_adrc(angle_exp, angle, angular_vel, ...
                                                         x1_last, x2_last, x3_last, ...
                                                         z1_last, z2_last, z3_last, z4_last, ...
                                                         r1, r2, h, h0, c, w, b0, kp, kd)
    [x1, x2, x3] = td4(x1_last,x2_last, x3_last,angle_exp,r1, r2, h, h0);
    
    e1 = x1 - z1_last;
    e2 = x2 - z2_last;
    e3 = x3 - z3_last;
    
    u0 = nlsef4(e1, e2, e3, 0.3, 5, 5, 0.03, kp, kd);
    u  = u0 - z4_last * 1/b0;
    
    [z1, z2, z3, z4] = leso4(z1_last, z2_last, z3_last, z4_last, w, u, b0, angle, angular_vel, h);
    
end

	function [x1_new,x2_new,x3_new] = td4(x1_last, x2_last, x3_last, input, r1, r2, h, h0)
        x1_new = x1_last + h*x2_last;                                                       % angluar integration
        x2_new = x2_last + h*fhan(fhan(x1_last-input,x2_last,r1,h0), x3_last, r1, h0);      % angle
        x3_new = x3_last + h*fhan(x2_last,x3_last,r2,h0);                                   % angular velocity
    end

    function u0 = nlsef4(e1,e2,e3,c,r1, r2,h1,kp,kd)
        u0 = -kp*fhan(e1,c*e2,r1,h1) - kd*fhan(e2,c*e3,r2,h1);
    end
    
    function [z1_new,z2_new,z3_new,z4_new] = leso4(z1_last,z2_last,z3_last,z4_last, ...
                                                   w, ...
                                                   u,b0,x_in_1,x_in_2,h)
        beta_01 = 3*w;
        beta_02 = 3*w^2;
        beta_03 = w^3;
        beta_04 = 1/3 * w^4;
        e1 = z1_last-x_in_1;
        e2 = z2_last-x_in_2;
        z1_new = z1_last + h*(z2_last-beta_01*(e1+e2));                                      % angle integration
        z2_new = z2_last + h*(z3_last-beta_02*(e1+e2));                                      % angle
        z3_new = z3_last + h*(z4_last-beta_03*(e1+e2)+b0*u);                                 % angular vel
        z4_new = z4_last + h*(-beta_04*e2);                                                  % disturbance
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