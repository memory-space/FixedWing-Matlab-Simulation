function control_inputs = get_control_inputs(t, x, param)
    % Option A: Motor-Propeller Physics Model
    % Thrust Tp is computed from throttle using motor physics equations
    % Reference: Beard & McLain Chapter 4, Appendix E

    %% 0. Current State Extraction
    u      = x(4);  
    v_body = x(5);  
    w      = x(6);  
    phi    = x(7);  
    theta  = x(8);  
    p      = x(10); 
    q      = x(11); 
    r      = x(12); 
    
    Va = sqrt(u^2 + v_body^2 + w^2); 

    %% 1. User Direct Input — Flight Scenario
    if t < 2.0
        % 0~2s: Standby
        throttle     = 1.0;
        base_delta_a = 0.2;
        base_delta_e = -0.2;
        base_delta_r = -0.1;
    elseif t >= 2.0 && t < 7.0
        throttle     = 1.0;
        base_delta_a = 0.25;
        base_delta_e = -0.25; 
        base_delta_r = -0.3;
    elseif t >= 7.0 && t < 12.0
        throttle     = 1.0;
        base_delta_a = 0.3;
        base_delta_e = -0.42;
        base_delta_r = -0.3;
    elseif t >= 12.0 && t < 15.0
        throttle     = 0.7;
        base_delta_a = -0.2;
        base_delta_e = -0.31;
        base_delta_r = 0.3;
    else
        throttle     = 0.6;
        base_delta_a = -0.2;
        base_delta_e = -0.51;
        base_delta_r = 0.2;
    end

    %% 2. Thrust Calculation 
    V_in = throttle * param.V_max; % [V]

    % Beard & McLain Eq 4.14 
    a_m = (param.rho * param.D_prop^5 / (4 * pi^2)) * param.C_Q0;
    
    b_m = (param.rho * param.D_prop^4 / (2 * pi)) * param.C_Q1 * Va ...
          + (param.KQ * param.KV / param.R_motor);
          
    c_m = param.rho * param.D_prop^3 * param.C_Q2 * Va^2 ...
          - (param.KQ / param.R_motor) * V_in ...
          + param.KQ * param.i0;

    discriminant = b_m^2 - 4 * a_m * c_m;
    if discriminant < 0
        omega_p = 0;
    else
        omega_p = (-b_m + sqrt(discriminant)) / (2 * a_m); % [rad/s]
    end
    omega_p = max(0, omega_p); 

    if omega_p < 1e-6
        J_p = 0;
    else
        J_p = (2 * pi * Va) / (omega_p * param.D_prop);
    end

    C_T = param.C_T2 * J_p^2 + param.C_T1 * J_p + param.C_T0;
    Tp = param.rho * (omega_p / (2*pi))^2 * param.D_prop^4 * C_T; 
    Tp = max(0, Tp); 

    C_Q = param.C_Q2 * J_p^2 + param.C_Q1 * J_p + param.C_Q0;
    kQ = param.rho * (1/(2*pi))^2 * param.D_prop^4 * C_Q;
    kQ = -max(0, kQ);
    
    %% 3. Stabilization Controller (PD Control)
    Kp_roll  = 1;
    Kd_roll  = 0.3;
    Kp_pitch = 1;
    Kd_pitch = 0.2;
    Kd_yaw   = 0.3;

    roll_corr  = Kp_roll  * phi   + Kd_roll  * p; 
    pitch_corr = Kp_pitch * theta + Kd_pitch * q; 
    yaw_corr   =                    Kd_yaw   * r; 

    %% 4. Final Control Inputs 
    % C_lda > 0, so if you get a Roll error, take it out the other way (-)
    delta_a = base_delta_a - roll_corr;
    
    % Because C_mde < 0, the Pitch error raises the positive control surface to positive to create a negative moment (+)
    delta_e = base_delta_e + pitch_corr; 
    
    % C_ndr < 0, so if a yaw rate error occurs, raise the control surface to positive to create the opposite moment (+)
    delta_r = base_delta_r + yaw_corr; 

     % 여기에 추가 ↓
    %fprintf('t=%.2f | omega_p=%.1f rad/s | Tp=%.2f N | Mp=%.2f Nm\n', t, omega_p, Tp, param.KQ * omega_p^2);

    delta_max = 30 * pi / 180; % [rad]
    delta_a = max(-delta_max, min(delta_max, delta_a));
    delta_e = max(-delta_max, min(delta_max, delta_e));
    delta_r = max(-delta_max, min(delta_max, delta_r));

    %% 5. Assemble Output
    control_inputs = [Tp; delta_a; delta_e; delta_r; omega_p; kQ];
end




%% ====================================================================
% 1. How to use
% 
% throttle = 0.0;  % Engine Off (Free Gliding)
% throttle = 0.5;  % Cruise (approximately 30-40N thrust)
% throttle = 1.0;  % Maximum thrust (take off/up)
% 
% base_delta_a = -0.15;  % Roll to the left → Turn to the left
% base_delta_a =  0.00;  % Keep the wings level
% base_delta_a = +0.15;  % Roll to the right → Turn to the right
% 
% base_delta_e = -0.10;  % Rated up → up
% base_delta_e =  0.00;  % Staying level
% base_delta_e = +0.10;  % Lower base → lower base
%
% base_delta_r = -0.05;  % a left turn of the rider
% base_delta_r =  0.00;  % Go straight
% base_delta_r = +0.05;  % right turn of the rider
% 
% ====================================================================