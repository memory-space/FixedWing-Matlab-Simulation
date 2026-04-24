clear; clc;
close all;

%% ========================================================================
% Fixed-Wing UAV Simulation (Aerosonde)
% Based on: Lecture 09 Fixed-wing Part 3 (POSTECH MECH701A-01)
%           Beard & McLain, Small Unmanned Aircraft, 2012
% ========================================================================

%% ====================================================================
% 1. Load Parameters
% ====================================================================
param = init_params(); % Load Aerosonde UAV parameters

%% ====================================================================
% 2. Simulation Time Settings
% ====================================================================
% tspan: time vector for ode45
% Smaller step = more accurate but slower
tspan = param.simulation_start_time : param.simulation_step_size  : param.simulation_end_time;    % [0, 0.01, 0.02, ... , 20] sec

%% ====================================================================
% 3. Initial State x0 (12x1)
%    x = [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]'
%    Aerosonde starts on the ground, moving forward at 15 m/s
%    (minimum airspeed needed for wing to generate lift)
% ====================================================================
pn_0    = 0;     % [m]     Initial North position
pe_0    = 0;     % [m]     Initial East position
pd_0    = -100;  % [m]     Initial altitude (NED: negative = up, 50m altitude)
u_0     = 25;    % [m/s]   Initial forward velocity (airspeed for lift)
v_0     = 0;     % [m/s]   Initial lateral velocity
w_0     = 0;     % [m/s]   Initial vertical velocity
phi_0   = 0;     % [rad]   Initial roll angle
theta_0 = 0;     % [rad]   Initial pitch angle
psi_0   = 0;     % [rad]   Initial yaw angle (heading North)
p_0     = 0;     % [rad/s] Initial roll rate
q_0     = 0;     % [rad/s] Initial pitch rate
r_0     = 0;     % [rad/s] Initial yaw rate

x0 = [pn_0; pe_0; pd_0; u_0; v_0; w_0; phi_0; theta_0; psi_0; p_0; q_0; r_0];

%% ====================================================================
% 4. Numerical Integration (ODE45)
%    ode45: Runge-Kutta 4th/5th order adaptive step solver
%    fixedwing_system_wrapper connects control inputs and dynamics
%    x_dot = f(x, u) is solved at each time step
% ====================================================================
[t_out, x_out] = ode45(@(t, x) fixedwing_system_wrapper(t, x, param), tspan, x0);

%% ====================================================================
% # Print Summary (Used for debugging)
% ====================================================================
% fprintf('=== Simulation Summary ===\n');
% fprintf('Final position  : North=%.2f m, East=%.2f m, Alt=%.2f m\n', x_out(end,1), x_out(end,2), -x_out(end,3));
% fprintf('Final velocity  : u=%.2f m/s, v=%.2f m/s, w=%.2f m/s\n', x_out(end,4), x_out(end,5), x_out(end,6));
% fprintf('Final attitude  : phi=%.2f deg, theta=%.2f deg, psi=%.2f deg\n', rad2deg(x_out(end,7)), rad2deg(x_out(end,8)), rad2deg(x_out(end,9)));

%% ====================================================================
% 5. Animation
% ====================================================================
animate(t_out, x_out, param);

% 7. Results Plot
u_log = zeros(length(t_out), 6);
for k = 1:length(t_out)
    u_log(k,:) = get_control_inputs(t_out(k), x_out(k,:)', param)';
end
plot_results(t_out, x_out, u_log, param);

% ========================================================================
% Author: Lim Minseok (minseoklim@postech.ac.kr)
% ========================================================================
