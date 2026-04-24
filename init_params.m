 function [param] = init_params()
    % This function initializes all physical parameters for the fixed-wing UAV.
    % All variable names follow full-name conventions for clarity.
    % Based on Aerosonde UAV parameters from Beard & McLain Appendix E.

    %% ====================================================================
    % 1. Physical Parameters (Table E.1)
    % ====================================================================
    % Mass
    param.mass = 11.0;          % [kg] Total mass of the aircraft (m)

    % Inertia moments
    % Jx: Resistance to Roll  (tilting left/right)
    % Jy: Resistance to Pitch (tilting forward/backward)
    % Jz: Resistance to Yaw   (spinning in place)
    % Jxz: Cross product of inertia (asymmetry between x and z)
    param.Jx  = 0.824;          % [kg*m^2] Moment of inertia about x-axis
    param.Jy  = 1.135;          % [kg*m^2] Moment of inertia about y-axis
    param.Jz  = 1.759;          % [kg*m^2] Moment of inertia about z-axis
    param.Jxz = 0.120;          % [kg*m^2] Cross moment of inertia (xz)

    % Full Inertia Matrix J (3x3)
    % Note: Aerosonde is symmetric in xy and yz planes
    param.J = [ param.Jx,    0,       -param.Jxz;
                   0,     param.Jy,      0;
               -param.Jxz,   0,        param.Jz ];

    % Wing geometry
    param.S = 0.55;             % [m^2] Wing planform area (S)
    param.b = 2.90;             % [m]   Wing span (b)
    param.c = 0.19;             % [m]   Mean aerodynamic chord length (c)
    param.e = 0.9;              % [-]   Oswald efficiency factor

    %% ====================================================================
    % 2. Environmental Parameters
    % ====================================================================
    param.rho = 1.268;          % [kg/m^3] Air density (rho) at sea level
    param.g   = 9.81;           % [m/s^2]  Gravitational acceleration

    % Gravity vector in local (NED) frame
    % NED: z-axis points DOWN, so gravity is positive in z
    param.gravity_vector = [0;
                             0;
                             param.g];

    %% ====================================================================
    % 3. Motor & Propulsion Parameters (Table E.1)
    %    Based on: AXI Motors 5345/20 + APC Propellers 20x10E
    %    Reference: Beard & McLain Appendix E
    % ====================================================================
    param.V_max    = 44.4;      % [V]       Maximum motor voltage
    param.D_prop   = 0.508;     % [m]       Propeller diameter
    param.KV       = 0.0659;    % [V*s/rad] Motor back-EMF constant
    param.KQ       = 0.0659;    % [N*m]     Motor torque constant
    param.R_motor  = 0.042;     % [Ohm]     Motor resistance
    param.i0       = 1.5;       % [A]       Motor no-load current

    % Propeller thrust coefficients (quadratic fit: CT = CT2*J^2 + CT1*J + CT0)
    % J: Advance ratio
    param.C_T2 = -0.1079;
    param.C_T1 = -0.06044;
    param.C_T0 =  0.09357;

    % Propeller torque coefficients (quadratic fit: CQ = CQ2*J^2 + CQ1*J + CQ0)
    param.C_Q2 = -0.01664;
    param.C_Q1 =  0.004970;
    param.C_Q0 =  0.005230;

    %% ====================================================================
    % 4. Longitudinal Aerodynamic Coefficients (Table E.2)
    %    Reference: Slide 7, 9 (Lecture 09)
    % ====================================================================

    % --- Lift Coefficients (CL) ---
    % F_lift = (1/2)*rho*Va^2*S*(CL0 + CLa*alpha + CLq*(c/2Va)*q + CLde*delta_e)
    param.C_L0      =  0.23;    % [-] Zero-alpha lift coefficient
    param.C_La      =  5.61;    % [-] Lift curve slope (per rad)
    param.C_Lq      =  7.95;    % [-] Pitch rate lift derivative (per rad)
    param.C_Lde     =  0.13;    % [-] Elevator lift derivative (per rad)

    % --- Drag Coefficients (CD) ---
    % F_drag = (1/2)*rho*Va^2*S*(CD0 + CDa*alpha + CDq*(c/2Va)*q + CDde*|delta_e|)
    param.C_D0      =  0.0424;  % [-] Zero-lift drag coefficient
    param.C_Da      =  0.132;   % [-] Drag due to angle of attack (per rad)
    param.C_Dq      =  0;       % [-] Pitch rate drag derivative (per rad)
    param.C_Dde     =  0.0135;  % [-] Elevator drag derivative (per rad)
    param.C_Dp      =  0.043;   % [-] Parasitic drag coefficient

    % --- Pitch Moment Coefficients (Cm) ---
    % My = (1/2)*rho*Va^2*S*c*(Cm0 + Cma*alpha + Cmq*(c/2Va)*q + Cmde*delta_e)
    param.C_m0      =  0.0135;  % [-] Zero-alpha pitch moment coefficient
    param.C_ma      = -2.74;    % [-] Pitch moment due to alpha (per rad) — negative = stable
    param.C_mq      = -38.21;   % [-] Pitch damping derivative (per rad)
    param.C_mde     = -0.99;    % [-] Elevator pitch moment derivative (per rad)

    % --- Stall Model Parameters (Slide 8) ---
    % sigmoid blending between linear model and flat plate model
    % sigma(alpha): blending function
    % CL = (1-sigma)*(CL0 + CLa*alpha) + sigma*(2*sin(alpha)*cos(alpha))
    param.M        =  50;       % [-]  Stall transition sharpness (blending factor)
    param.alpha0   =  0.47;     % [rad] Stall critical angle of attack (~27 deg)

    %% ====================================================================
    % 5. Lateral-Directional Aerodynamic Coefficients (Table E.2)
    %    Reference: Slide 11 (Lecture 09)
    % ====================================================================

    % --- Side Force Coefficients (CY) ---
    % fa_y = (1/2)*rho*Va^2*S*(CY0 + CYb*beta + CYp*(b/2Va)*p + CYr*(b/2Va)*r + CYda*delta_a + CYdr*delta_r)
    param.C_Y0      =  0;       % [-] Zero side force coefficient
    param.C_Yb      = -0.83;    % [-] Side force due to sideslip (per rad)
    param.C_Yp      =  0;       % [-] Side force due to roll rate (per rad)
    param.C_Yr      =  0;       % [-] Side force due to yaw rate (per rad)
    param.C_Yda     =  0.075;   % [-] Side force due to aileron (per rad)
    param.C_Ydr     =  0.19;    % [-] Side force due to rudder (per rad)

    % --- Roll Moment Coefficients (Cl) ---
    % Mx = (1/2)*rho*Va^2*S*b*(Cl0 + Clb*beta + Clp*(b/2Va)*p + Clr*(b/2Va)*r + Clda*delta_a + Cldr*delta_r)
    param.C_l0      =  0;       % [-] Zero roll moment coefficient
    param.C_lb      = -0.13;    % [-] Roll moment due to sideslip (per rad) — dihedral effect
    param.C_lp      = -0.51;    % [-] Roll damping derivative (per rad)
    param.C_lr      =  0.25;    % [-] Roll moment due to yaw rate (per rad)
    param.C_lda     =  0.17;    % [-] Aileron roll moment derivative (per rad)
    param.C_ldr     =  0.0024;  % [-] Rudder roll moment derivative (per rad)

    % --- Yaw Moment Coefficients (Cn) ---
    % Mz = (1/2)*rho*Va^2*S*b*(Cn0 + Cnb*beta + Cnp*(b/2Va)*p + Cnr*(b/2Va)*r + Cnda*delta_a + Cndr*delta_r)
    param.C_n0      =  0;       % [-] Zero yaw moment coefficient
    param.C_nb      =  0.073;   % [-] Yaw moment due to sideslip (per rad) — weathercock stability
    param.C_np      = -0.069;   % [-] Yaw moment due to roll rate (per rad)
    param.C_nr      = -0.095;   % [-] Yaw damping derivative (per rad)
    param.C_nda     = -0.011;   % [-] Aileron yaw moment derivative (per rad)
    param.C_ndr     = -0.069;   % [-] Rudder yaw moment derivative (per rad)

    %% ====================================================================
    % 6. Simulation Settings
    % ====================================================================
    param.simulation_start_time = 0;    % [sec]
    param.simulation_end_time   = 20;   % [sec] (Adjustable)
    param.simulation_step_size  = 0.01; % [sec]

end


% ========================================================================
% Reference:
%   Beard, Randal W., and Timothy W. McLain.
%   Small Unmanned Aircraft: Theory and Practice.
%   Princeton University Press, 2012. Appendix E.
%
%   Lecture 09: Fixed-wing Part 3, POSTECH MECH701A-01
%   (Slides 7, 8, 9, 11, 12, 13, 14)
%
% Author: Lim Minseok (minseoklim@postech.ac.kr)
% ========================================================================
