function x_dot = fixedwing_system_wrapper(t, x, param)
    % Wrapper function that connects control inputs and dynamics
    % Called by ode45 at every time step
    %
    % Flow:
    %   1. get_control_inputs → compute u = [Tp, delta_a, delta_e, delta_r]
    %   2. fixedwing_dynamics → compute x_dot from x and u
    %   3. Ground constraint  → prevent aircraft from going below ground

    %% ====================================================================
    % 1. Compute Control Inputs
    %    Choose Option - (motor physics)
    % ====================================================================
    u_control = get_control_inputs(t, x, param); % Option A: motor physics

    %% ====================================================================
    % 2. Compute State Derivatives
    %    fixedwing_dynamics computes x_dot = f(x, u)
    % ====================================================================
    x_dot = fixedwing_dynamics(t, x, u_control, param);

    %% ====================================================================
    % 3. Ground Constraint
    %    Prevent aircraft from going below ground (pd >= 0 in NED)
    %    NED: z-axis points DOWN, so pd >= 0 means at or below ground
    %    x(3) : current altitude (pd)
    %    x_dot(3) : vertical velocity (pd_dot) — positive = moving down
    %    x_dot(6) : vertical acceleration (w_dot) — positive = accelerating down
    % ====================================================================
    if x(3) >= 0 && x_dot(3) > 0
        x_dot(3) = 0; % Stop downward velocity
        x_dot(6) = 0; % Stop downward acceleration
    end
end


% ========================================================================
% Author: Lim Minseok (minseoklim@postech.ac.kr)
% ========================================================================
