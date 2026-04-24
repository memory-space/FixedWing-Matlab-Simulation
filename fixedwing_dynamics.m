function x_dot = fixedwing_dynamics(t, x, control_inputs, param)
    % State vector x: [p^l; V^b; Phi; omega_b] (12x1)
    % p^l    : NED position in local frame [pn, pe, pd]     
    % V^b    : Body frame velocity         [u, v, w]        
    % Phi    : Euler angles                [phi, theta, psi]
    % omega_b: Body angular rate           [p, q, r]        
    %
    % control_inputs: [Tp, delta_a, delta_e, delta_r, omega_p, kQ]' (6x1)
    % Tp      : Propeller thrust [N]
    % delta_a : Aileron deflection angle  [rad]
    % delta_e : Elevator deflection angle [rad]
    % delta_r : Rudder deflection angle   [rad]
    % omega_p : Propeller angular velocity [rad/s]
    % kQ      : Propeller torque coefficient [N*m/rad^2]

    %% ====================================================================
    % 1. State Extraction
    % ====================================================================
    V_b     = x(4:6);       % Body frame velocity [u; v; w]
    u       = x(4);         % Body x-velocity
    v       = x(5);         % Body y-velocity
    w       = x(6);         % Body z-velocity
    phi     = x(7);         % Roll angle  [rad]
    theta   = x(8);         % Pitch angle [rad]
    psi     = x(9);         % Yaw angle   [rad]
    omega_b = x(10:12);     % Body angular rate [p; q; r]
    p       = x(10);        % Roll rate   [rad/s]
    q       = x(11);        % Pitch rate  [rad/s]
    r       = x(12);        % Yaw rate    [rad/s]

    % Control inputs
    Tp      = control_inputs(1); % Propeller thrust [N]
    delta_a = control_inputs(2); % Aileron  [rad]
    delta_e = control_inputs(3); % Elevator [rad]
    delta_r = control_inputs(4); % Rudder   [rad]
    omega_p = control_inputs(5); % Propeller angular velocity [rad/s]
    kQ      = control_inputs(6); % Propeller torque coefficient [N*m/rad^2]

    % Parameters
    % Everything is in init_params.m
    m   = param.mass;   % [kg]      Total mass
    J   = param.J;      % [kg*m^2]  Inertia matrix
    rho = param.rho;    % [kg/m^3]  Air density
    S   = param.S;      % [m^2]     Wing planform area
    b   = param.b;      % [m]       Wing span
    c   = param.c;      % [m]       Mean aerodynamic chord
    g   = param.g;      % [m/s^2]   Gravitational acceleration

    %% ====================================================================
    % 2. Airspeed, Angle of Attack, Sideslip Angle
    %    Geometric definition from body frame velocity
    % ====================================================================
    % Va: Airspeed magnitude
    Va = sqrt(u^2 + v^2 + w^2);

    % Numerical guard: prevent division by zero at Va = 0 (e.g. simulation start)
    if Va < 0.1
        alpha = 0;
        beta  = 0;
        dyna_pres = 0;
        CL = 0; 
        CD = 0; 
        Cm = 0; 
        CY = 0; 
        Cl = 0; 
        Cn = 0;
    else
        % alpha: Angle of Attack — longitudinal
        % Geometric angle between body x-axis and airspeed vector in xz-plane
        alpha = atan2(w, u);

        % beta: Sideslip angle — lateral
        % Geometric angle between airspeed vector and body xz-plane
        beta = asin(v / Va);

        % Dynamic pressure: (1/2)*rho*Va^2 [N/m^2]
        % Kinetic energy per unit volume
        dyna_pres = 0.5 * rho * Va^2;
    end

    %% ====================================================================
    % 3. Stall Model
    %    Linear model valid only for small alpha range
    %    Flat plate model used above stall: CL = 2*sin^2(alpha)*cos(alpha)
    %    Switch between two models at alpha0 (critical angle of attack)
    % ====================================================================
    alpha0 = param.alpha0;  % Critical angle of attack [rad]

    % CL_alpha: alpha-dependent part of lift coefficient
    % Normal flight (|alpha| < alpha0): linear model
    % Stall         (|alpha| >= alpha0): flat plate model
    if abs(alpha) < alpha0
        % Normal flight: linear model
        CL_alpha = param.C_L0 + param.C_La * alpha;
    else
        % Stall: flat plate model
        % CL = 2*sin^2(alpha)*cos(alpha)
        CL_alpha = 2 * sin(alpha)^2 * cos(alpha);
    end

    %% ====================================================================
    % 4. Longitudinal Aerodynamics
    %    F_lift = dynamic pressure * S * CL(alpha, q, delta_e)
    %    F_drag = dynamic pressure * S * CD(alpha, q, delta_e)
    %    Ma_y   = dynamic pressure * S * c * Cm(alpha, q, delta_e)
    % ====================================================================

    % --- Lift Coefficient ---
    % CL = CL0 + CLa*alpha + CLq*(c/2Va)*q + CLde*delta_e
    % Note: CL0 + CLa*alpha = CL_alpha (computed in Section 3, with stall check)
    %       q, delta_e terms added separately as they are stall-independent
    CL = CL_alpha + param.C_Lq * (c / (2*Va)) * q + param.C_Lde * delta_e;

    % --- Drag Coefficient ---
    % CD = CD0 + CDa*alpha + CDq*(c/2Va)*q + CDde*|delta_e|
    CD = param.C_D0  + param.C_Da  * alpha + param.C_Dq  * (c / (2*Va)) * q + param.C_Dde * abs(delta_e);

    % --- Pitch Moment Coefficient ---
    % Cm = Cm0 + Cma*alpha + Cmq*(c/2Va)*q + Cmde*delta_e
    Cm = param.C_m0  + param.C_ma  * alpha + param.C_mq  * (c / (2*Va)) * q + param.C_mde * delta_e;

    % --- Aerodynamic Forces in Stability Frame ---
    F_lift = dyna_pres * S * CL;     % Lift force    [N]
    F_drag = dyna_pres * S * CD;     % Drag force    [N]
    Ma_y   = dyna_pres * S * c * Cm; % Pitch moment  [N*m]

    % --- Stability Frame -> Body Frame Transformation (NED) ---
    % [fx_a]   [cos(alpha)  -sin(alpha)] [-F_drag]
    % [fz_a] = [sin(alpha)   cos(alpha)] [-F_lift]
    % fx_a: A component that is pulled forward by lift - A component that is pushed back by drag
    fx_a = -cos(alpha)*F_drag + sin(alpha)*F_lift;
    
    % fz_a: Component (-z) that the lift floats upward - component that the drag presses downward (+z)
    fz_a = -sin(alpha)*F_drag - cos(alpha)*F_lift;

    %% ====================================================================
    % 5. Lateral-Directional Aerodynamics
    %    fa_y = Dynamic pressure * S * CY(beta, p, r, delta_a, delta_r)
    %    Ma_x = Dynamic pressure * S * b * Cl(beta, p, r, delta_a, delta_r)
    %    Ma_z = Dynamic pressure * S * b * Cn(beta, p, r, delta_a, delta_r)
    % ====================================================================

    % --- Side Force Coefficient ---
    % CY = CY0 + CYb*beta + CYp*(b/2Va)*p + CYr*(b/2Va)*r + CYda*delta_a + CYdr*delta_r
    CY = param.C_Y0  + param.C_Yb  * beta + param.C_Yp  * (b / (2*Va)) * p + param.C_Yr  * (b / (2*Va)) * r + param.C_Yda * delta_a + param.C_Ydr * delta_r;

    % --- Roll Moment Coefficient ---
    % Cl = Cl0 + Clb*beta + Clp*(b/2Va)*p + Clr*(b/2Va)*r + Clda*delta_a + Cldr*delta_r
    Cl = param.C_l0  + param.C_lb  * beta + param.C_lp  * (b / (2*Va)) * p + param.C_lr  * (b / (2*Va)) * r + param.C_lda * delta_a + param.C_ldr * delta_r;

    % --- Yaw Moment Coefficient ---
    % Cn = Cn0 + Cnb*beta + Cnp*(b/2Va)*p + Cnr*(b/2Va)*r + Cnda*delta_a + Cndr*delta_r
    Cn = param.C_n0  + param.C_nb  * beta + param.C_np  * (b / (2*Va)) * p + param.C_nr  * (b / (2*Va)) * r + param.C_nda * delta_a + param.C_ndr * delta_r;

    % --- Lateral-Directional Forces and Moments ---
    fa_y = dyna_pres * S * CY;       % Side force   [N]
    Ma_x = dyna_pres * S * b * Cl;   % Roll moment  [N*m]
    Ma_z = dyna_pres * S * b * Cn;   % Yaw moment   [N*m]

    %% ====================================================================
    % 6. Propulsion Forces and Moments
    %    Fp^b = [Tp; 0; 0]  — thrust along body x-axis (forward)
    %    Mp^b = [kQ*wp^2; 0; 0] — propeller reaction torque
    %    Note: Tp is directly given as control input
    % ====================================================================
    Fp_b = [Tp; 0; 0];               % Propeller thrust in body frame [N]
    Mp_b = [kQ * omega_p^2; 0; 0];   % Propeller reaction torque [N*m]

    %% ====================================================================
    % 7. Gravity Force in Body Frame
    %    Fg^b = Rl^b * [0; 0; mg]
    %    NED: gravity acts in +z direction of local frame
    % ====================================================================
    R_b_l = body2world_xyz(phi, theta, psi); % R: body -> local (reused from quadrotor)
    R_l_b = R_b_l';                           % R^T = R^-1: local -> body

    Fg_b = R_l_b * [0; 0; m * g]; % Gravity in body frame [N]

    %% ====================================================================
    % 8. Total Force and Moment in Body Frame
    %    F^b = Fa^b + Fp^b + Fg^b
    %    M^b = Ma^b + Mp^b
    % ====================================================================
    Fa_b = [fx_a; fa_y; fz_a]; % Aerodynamic force  in body frame [N]
    Ma_b = [Ma_x; Ma_y; Ma_z]; % Aerodynamic moment in body frame [N*m]

    F_b = Fa_b + Fp_b + Fg_b;  % Total force  in body frame [N]
    M_b = Ma_b + Mp_b;          % Total moment in body frame [N*m]

    %% ====================================================================
    % 9. Equations of Motion — Body Frame
    %    Translational: V_dot^b = F^b/m - omega^b x V^b
    %    Rotational:    omega_dot^b = J^-1 * (M^b - omega^b x J*omega^b)
    % ====================================================================

    % --- Translational EOM ---
    % V_dot^b = F^b/m - omega^b x V^b
    % cross(omega_b, V_b): Gyroscopic term — arises because body frame is rotating
    V_dot_b = F_b / m - cross(omega_b, V_b);

    % --- Rotational EOM ---
    % omega_dot^b = J^-1 * (M^b - omega^b x (J*omega^b))
    % J\(...): numerically more stable than inv(J)*(...)
    omega_dot_b = J \ (M_b - cross(omega_b, J * omega_b));

    %% ====================================================================
    % 10. Kinematics (Position & Attitude)
    %     Borrowed from quadrotor_dynamics.m — frame-independent geometry
    % ====================================================================

    % --- Position Kinematics ---
    % p_dot^l = R_b^l * V^b
    % Rate of change of NED position = body velocity rotated to local frame
    p_dot_l = R_b_l * V_b;

    % --- Attitude Kinematics ---
    % omega_b = W * Phi_dot  =>  Phi_dot = W^-1 * omega_b
    Rx = [1,       0,        0;
          0,  cos(phi),  sin(phi);
          0, -sin(phi),  cos(phi)];    % Roll rotation

    Ry = [cos(theta), 0, -sin(theta);
               0,     1,      0;
          sin(theta), 0,  cos(theta)]; % Pitch rotation

    col1 = [1; 0; 0];            % Roll axis
    col2 = Rx * [0; 1; 0];       % Pitch axis after Roll
    col3 = Rx * Ry * [0; 0; 1];  % Yaw axis after Pitch & Roll

    W = [col1, col2, col3];       % Forward mapping: Phi_dot -> omega_b
    Phi_dot = W \ omega_b;        % Inverse: omega_b -> Phi_dot

    %% ====================================================================
    % 11. Assemble x_dot (12x1)
    %     x_dot = [p_dot^l; V_dot^b; Phi_dot; omega_dot^b]
    % ====================================================================
    x_dot = [p_dot_l;      % dp/dt    : NED position rate        (3x1)
             V_dot_b;      % dV^b/dt  : Body velocity rate       (3x1)
             Phi_dot;      % dPhi/dt  : Euler angle rate         (3x1)
             omega_dot_b]; % domega/dt: Body angular rate change (3x1)
end


% ========================================================================
% [Purpose] Fixed-Wing Aircraft State Differential Equations
% Based on Newton-Euler equations and aerodynamic models,
% this function computes x_dot given current state x and control inputs u.
%
% Equations of Motion source: Lecture 09 Fixed-wing Part 3 (POSTECH MECH701A-01)
%   - Longitudinal aerodynamics 
%   - Stall model            
%   - Lateral aerodynamics    
%   - Other forces & moments  
%   - EOM summary             
%   - State-space form        
%
% Kinematics (position & attitude) borrowed from quadrotor_dynamics.m
% as they are frame-independent rigid body geometry.
%
% Reference:
%   Beard, Randal W., and Timothy W. McLain.
%   Small Unmanned Aircraft: Theory and Practice.
%   Princeton University Press, 2012.
%
% Author: Lim Minseok (minseoklim@postech.ac.kr)
% ========================================================================
