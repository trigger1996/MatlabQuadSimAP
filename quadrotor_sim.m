%-----------------------------------------------------------------------%
%                                                                       %
%   This script simulates quadrotor dynamics and implements a control   %                                %
%   algrotihm.                                                          %
%   Developed by: Wil Selby                                             %
%                                                                       %
%                                                                       %
%-----------------------------------------------------------------------%

% Add Paths
addpath utilities

%% Initialize Workspace
clear all;
close all;
clc;

global Quad;

%% Initialize the plot
init_plot;
plot_quad_model;

%% Initialize Variables
quad_variables;
quad_dynamics_nonlinear;   

%% Run The Simulation Loop
while Quad.t_plot(Quad.counter-1)< max(Quad.t_plot);    
    
    % Measure Parameters (for simulating sensor errors)
      sensor_meas;

    % Filter Measurements
%     Kalman_phi2;
%     Kalman_theta2;
%     Kalman_psi2;
%     Kalman_Z2;
%     Kalman_X2;
%     Kalman_Y2;
    
    % Implement Controller
    position_PID;
    %position_Backstepping;     % 这个东西做错了，把推力U1当成求解量而不是角度。如果根据地面坐标系控制角度的话，角度存在在DCM矩阵内，所以非常难分离，很难做到单独求解，所以这边还是PID控制
    %Quad.phi_des = 0;
    %Quad.theta_des = 0;
    %Quad.psi_des = 0;
    %attitude_PID;
    %rate_PID;
    attitude_Backstepping;
    
    % Calculate Desired Motor Speeds
    quad_motor_speed;
    
    % Update Position With The Equations of Motion
    quad_dynamics_nonlinear;    
    
    % Plot the Quadrotor's Position
    if(mod(Quad.counter,3)==0)
        plot_quad 
        
%         campos([A.X+2 A.Y+2 A.Z+2])
%         camtarget([A.X A.Y A.Z])
%         camroll(0);
        Quad.counter;
        drawnow
    end
    
    Quad.init = 1;  %Ends initialization after first simulation iteration
end

%% Plot Data
plot_data
