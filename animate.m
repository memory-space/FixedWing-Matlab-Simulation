function animate(t, x, param)
    % Fixed-Wing Aircraft 3D Animation
    % Renders Aerosonde UAV shape with:
    %   - Fuselage
    %   - Left/Right wings 
    %   - Horizontal tail 
    %   - Vertical tail 
    %   - Rotating propeller 
    %   - Flight trace 
    %   - Heading arrow 

    %% ====================================================================
    % 0. Aircraft Geometry (Aerosonde scale)
    %    All dimensions in body frame [m]
    %    x: forward, y: right, z: down (NED)
    % ====================================================================

    % 0. Aircraft Geometry 
    scale = 15.0;

    % --- Fuselage ---
    % Aerosonde total length ~ 1.7m
    fuse_nose = 0.85 * scale;   % [m] nose tip from center (front, +x)
    fuse_tail = -0.85 * scale;  % [m] tail end from center (rear, -x)

    % --- Wings ---
    % b = 2.9m (param.b), c = 0.19m (param.c)
    half_span   = param.b / 2 * scale;     % [m] half wing span (y direction)
    wing_chord  = param.c * scale;         % [m] wing chord length
    wing_x_root = 0.0;             % [m] wing root x position (center)
    wing_x_tip  = -0.05 * scale;           % [m] wing tip swept slightly back

    % --- Horizontal Tail ---
    htail_span  = 0.9 * scale;    % [m] total horizontal tail span
    htail_x     = -0.75 * scale;  % [m] tail position (rear, -x)
    htail_chord = 0.10 * scale;   % [m] tail chord

    % --- Vertical Tail ---
    vtail_height = 0.35 * scale;  % [m] vertical tail height (z direction, upward = -z NED)
    vtail_x      = -0.75 * scale; % [m] same x as horizontal tail

    % --- Propeller ---
    R_prop = param.D_prop / 2 * scale; % [m] propeller radius (0.254m)
    prop_x = fuse_nose;        % [m] propeller at nose tip

    %% ====================================================================
    % 1. Figure Setup
    % ====================================================================
    fig = figure('Name', 'Fixed-Wing UAV Animation', ...
                 'Color', [0.08 0.08 0.12], ...
                 'Position', [100, 100, 1100, 700]);

    ax = axes('Parent', fig, ...
              'Color', [0.08 0.08 0.12], ...
              'XColor', [0.6 0.6 0.6], ...
              'YColor', [0.6 0.6 0.6], ...
              'ZColor', [0.6 0.6 0.6], ...
              'GridColor', [0.25 0.25 0.3], ...
              'GridAlpha', 0.5);

    xlabel('North (m)', 'Color', [0.7 0.7 0.7]);
    ylabel('East (m)',  'Color', [0.7 0.7 0.7]);
    zlabel('Down (m)',  'Color', [0.7 0.7 0.7]);

    view(45, 25);
    axis equal;
    grid on;
    hold on;

    % NED: z-axis reversed so altitude goes upward visually
    set(ax, 'ZDir', 'reverse');

    % Axis range from simulation trajectory
    x_range = max(x(:,1)) - min(x(:,1));
    y_range = max(x(:,2)) - min(x(:,2));
    margin  = max([x_range, y_range, 5]) * 0.3;

    axis([min(x(:,1)) - margin,  max(x(:,1)) + margin, ...
          min(x(:,2)) - margin,  max(x(:,2)) + margin, ...
          min(x(:,3)) - 3.0,     max(x(:,3)) + 1.0]);

    %% ====================================================================
    % 2. Initialize Graphic Objects
    %    All objects created once, then updated in animation loop (faster)
    % ====================================================================

    % --- Flight trace ---
    trace = plot3(ax, x(1,1), x(1,2), x(1,3), ':', 'Color', [0.3 0.6 1.0 0.5], 'LineWidth', 1.0);

    % --- Fuselage ---
    h_fuse = plot3(ax, [0 0], [0 0], [0 0], '-', 'Color', [0.9 0.9 0.95], 'LineWidth', 3);

    % --- Left Wing ---
    h_wing_left = fill3(ax, [0 0 0 0], [0 0 0 0], [0 0 0 0], [0.2 0.5 0.9], 'FaceAlpha', 0.85, 'EdgeColor', [0.5 0.7 1.0], 'LineWidth', 1);

    % --- Right Wing ---
    h_wing_right = fill3(ax, [0 0 0 0], [0 0 0 0], [0 0 0 0], [0.2 0.5 0.9], 'FaceAlpha', 0.85, 'EdgeColor', [0.5 0.7 1.0], 'LineWidth', 1);

    % --- Horizontal Tail ---
    h_htail = fill3(ax, [0 0 0 0], [0 0 0 0], [0 0 0 0], [0.15 0.4 0.75], 'FaceAlpha', 0.85, 'EdgeColor', [0.4 0.6 0.9], 'LineWidth', 1);

    % --- Vertical Tail ---
    h_vtail = fill3(ax, [0 0 0], [0 0 0], [0 0 0], [0.15 0.4 0.75], 'FaceAlpha', 0.85, 'EdgeColor', [0.4 0.6 0.9], 'LineWidth', 1);

    % --- Propeller blade 1 ---
    h_prop1 = plot3(ax, [0 0], [0 0], [0 0], '-', 'Color', [1.0 0.85 0.2], 'LineWidth', 3.5);

    % --- Propeller blade 2 ---
    h_prop2 = plot3(ax, [0 0], [0 0], [0 0], '-', 'Color', [1.0 0.85 0.2], 'LineWidth', 3.5);

    % --- Propeller hub ---
    h_hub = plot3(ax, 0, 0, 0, 'o', 'Color', [1.0 0.5 0.1], 'MarkerFaceColor', [1.0 0.5 0.1], 'MarkerSize', 7);

    % --- Heading arrow ---
    h_arrow = quiver3(ax, 0, 0, 0, 1, 0, 0, 'Color', [0.2 1.0 0.4], 'LineWidth', 2.5, 'MaxHeadSize', 2, 'AutoScale', 'off');

    % --- Title ---
    h_title = title(ax, 'Initializing...', 'Color', [0.9 0.9 0.9], 'FontSize', 12);

    %% ====================================================================
    % 3. Animation Loop
    % ====================================================================
    skip      = 5;    % Frame skip (increase if too slow)
    prop_angle = 0;   % Propeller rotation angle [rad]
    prop_speed = 0.8; % Propeller rotation speed per frame [rad]
                      % (visual only — not tied to actual omega_p)

    for i = 1:skip:length(t)

        % --- Current state ---
        pos   = x(i, 1:3)';  % NED position [pn; pe; pd]
        phi   = x(i, 7);     % Roll
        theta = x(i, 8);     % Pitch
        psi   = x(i, 9);     % Yaw

        % --- Rotation matrix: body -> local (NED) ---
        % Reusing the same R as body2world_xyz
        cphi = cos(phi);   sphi = sin(phi);
        cthe = cos(theta); sthe = sin(theta);
        cpsi = cos(psi);   spsi = sin(psi);

        R = [cthe*cpsi,  sphi*sthe*cpsi - cphi*spsi,  cphi*sthe*cpsi + sphi*spsi;
             cthe*spsi,  sphi*sthe*spsi + cphi*cpsi,  cphi*sthe*spsi - sphi*cpsi;
             -sthe,      sphi*cthe,                    cphi*cthe               ];

        % ----------------------------------------------------------------
        % 3.1 Fuselage
        %     Line from nose to tail in body frame, rotated to world
        % ----------------------------------------------------------------
        nose_b = [fuse_nose; 0; 0];  % Nose in body frame
        tail_b = [fuse_tail; 0; 0];  % Tail in body frame

        nose_w = R * nose_b + pos;   % Nose in world frame
        tail_w = R * tail_b + pos;   % Tail in world frame

        set(h_fuse, 'XData', [nose_w(1), tail_w(1)], ...
                    'YData', [nose_w(2), tail_w(2)], ...
                    'ZData', [nose_w(3), tail_w(3)]);

        % ----------------------------------------------------------------
        % 3.2 Left Wing
        %     Trapezoid shape: 4 corner points in body frame
        %     root_front → root_rear → tip_rear → tip_front
        % ----------------------------------------------------------------
        % Wing corners in body frame [x; y; z]
        wl_root_front = R * [wing_x_root;                      0; 0] + pos;
        wl_root_rear  = R * [wing_x_root-wing_chord;           0; 0] + pos;
        wl_tip_rear   = R * [wing_x_tip - wing_chord; -half_span; 0] + pos;
        wl_tip_front  = R * [wing_x_tip;              -half_span; 0] + pos;

        set(h_wing_left, ...
            'XData', [wl_root_front(1), wl_root_rear(1), wl_tip_rear(1), wl_tip_front(1)], ...
            'YData', [wl_root_front(2), wl_root_rear(2), wl_tip_rear(2), wl_tip_front(2)], ...
            'ZData', [wl_root_front(3), wl_root_rear(3), wl_tip_rear(3), wl_tip_front(3)]);

        % ----------------------------------------------------------------
        % 3.3 Right Wing 
        %     Mirror of left wing (y → +y)
        % ----------------------------------------------------------------
        wr_root_front = R * [wing_x_root;                     0; 0] + pos;
        wr_root_rear  = R * [wing_x_root-wing_chord;          0; 0] + pos;
        wr_tip_rear   = R * [wing_x_tip - wing_chord; half_span; 0] + pos;
        wr_tip_front  = R * [wing_x_tip;              half_span; 0] + pos;

        set(h_wing_right, ...
            'XData', [wr_root_front(1), wr_root_rear(1), wr_tip_rear(1), wr_tip_front(1)], ...
            'YData', [wr_root_front(2), wr_root_rear(2), wr_tip_rear(2), wr_tip_front(2)], ...
            'ZData', [wr_root_front(3), wr_root_rear(3), wr_tip_rear(3), wr_tip_front(3)]);

        % ----------------------------------------------------------------
        % 3.4 Horizontal Tail
        %     Smaller wing at rear, same structure
        % ----------------------------------------------------------------
        ht_half = htail_span / 2;
        ht_rf = R * [htail_x;               0;         0] + pos;
        ht_rr = R * [htail_x - htail_chord; 0;         0] + pos;
        ht_tr = R * [htail_x - htail_chord; -ht_half;  0] + pos;
        ht_tf = R * [htail_x;               -ht_half;  0] + pos;
        ht_rf2= R * [htail_x;                ht_half;  0] + pos;
        ht_rr2= R * [htail_x - htail_chord;  ht_half;  0] + pos;

        % Draw both sides as one fill (6 points forming both sides)
        set(h_htail, ...
            'XData', [ht_tr(1), ht_tf(1), ht_rf(1), ht_rf2(1), ht_rr2(1), ht_rr(1)], ...
            'YData', [ht_tr(2), ht_tf(2), ht_rf(2), ht_rf2(2), ht_rr2(2), ht_rr(2)], ...
            'ZData', [ht_tr(3), ht_tf(3), ht_rf(3), ht_rf2(3), ht_rr2(3), ht_rr(3)]);

        % ----------------------------------------------------------------
        % 3.5 Vertical Tail
        %     Triangle pointing upward (-z in NED)
        % ----------------------------------------------------------------
        vt_base_front = R * [vtail_x;               0;  0           ] + pos;
        vt_base_rear  = R * [vtail_x - htail_chord; 0;  0           ] + pos;
        vt_tip        = R * [vtail_x - htail_chord; 0; -vtail_height] + pos;
        % NED: -z = upward

        set(h_vtail, ...
            'XData', [vt_base_front(1), vt_base_rear(1), vt_tip(1)], ...
            'YData', [vt_base_front(2), vt_base_rear(2), vt_tip(2)], ...
            'ZData', [vt_base_front(3), vt_base_rear(3), vt_tip(3)]);

        % ----------------------------------------------------------------
        % 3.6 Propeller
        %     Two blades rotating in yz-plane at nose tip
        %     prop_angle increases each frame → rotation effect
        % ----------------------------------------------------------------
        prop_angle = prop_angle + prop_speed * skip;

        % Blade 1: tip in +y/+z direction
        prop_tip1_b = [prop_x;  R_prop * cos(prop_angle);  R_prop * sin(prop_angle)];
        prop_tip2_b = [prop_x; -R_prop * cos(prop_angle); -R_prop * sin(prop_angle)];
        prop_hub_b  = [prop_x; 0; 0];

        prop_tip1_w = R * prop_tip1_b + pos;
        prop_tip2_w = R * prop_tip2_b + pos;
        prop_hub_w  = R * prop_hub_b  + pos;

        set(h_prop1, 'XData', [prop_hub_w(1), prop_tip1_w(1)], ...
                     'YData', [prop_hub_w(2), prop_tip1_w(2)], ...
                     'ZData', [prop_hub_w(3), prop_tip1_w(3)]);

        set(h_prop2, 'XData', [prop_hub_w(1), prop_tip2_w(1)], ...
                     'YData', [prop_hub_w(2), prop_tip2_w(2)], ...
                     'ZData', [prop_hub_w(3), prop_tip2_w(3)]);

        set(h_hub,   'XData', prop_hub_w(1), ...
                     'YData', prop_hub_w(2), ...
                     'ZData', prop_hub_w(3));

        % ----------------------------------------------------------------
        % 3.7 Heading Arrow
        %     Green arrow showing aircraft heading (body x-axis direction)
        % ----------------------------------------------------------------
        arrow_len = param.b * 0.1 * scale; % Arrow length proportional to wing span
        arrow_vec = R * [arrow_len; 0; 0];

        set(h_arrow, 'XData', pos(1), ...
                     'YData', pos(2), ...
                     'ZData', pos(3), ...
                     'UData', arrow_vec(1), ...
                     'VData', arrow_vec(2), ...
                     'WData', arrow_vec(3));

        % ----------------------------------------------------------------
        % 3.8 Flight Trace
        %     Update trace up to current time step
        % ----------------------------------------------------------------
        set(trace, 'XData', x(1:i, 1), ...
                   'YData', x(1:i, 2), ...
                   'ZData', x(1:i, 3));

        % ----------------------------------------------------------------
        % 3.9 Title update
        % ----------------------------------------------------------------
        Va_now = sqrt(x(i,4)^2 + x(i,5)^2 + x(i,6)^2);
        alt_now = -x(i, 3); % NED: altitude = -pd

        set(h_title, 'String', sprintf('Time: %.2f s  |  Altitude: %.1f m  |  Airspeed: %.1f m/s', t(i), alt_now, Va_now));

        drawnow;
        if ~ishandle(fig), break; end
    end
end


% ========================================================================
% [Purpose] 3D Animation of Fixed-Wing UAV (Aerosonde)
%
% Graphic elements:
%   - Fuselage        : body frame x-axis line
%   - Wings           : fill3 trapezoid (span = param.b)
%   - Horizontal tail : fill3 smaller trapezoid at rear
%   - Vertical tail   : fill3 triangle at rear
%   - Propeller       : two rotating blades at nose (D = param.D_prop)
%   - Heading arrow   : quiver3 showing body x-axis direction
%   - Flight trace    : dotted trajectory line
%
% Coordinate: NED (North-East-Down), ZDir reversed for visual clarity
%
% Reference:
%   Beard, Randal W., and Timothy W. McLain.
%   Small Unmanned Aircraft: Theory and Practice.
%   Princeton University Press, 2012.
%
% Author: Lim Minseok (minseoklim@postech.ac.kr)
% ========================================================================
