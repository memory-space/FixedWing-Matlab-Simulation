function plot_results(t, x, u_log, param)
% PLOT_RESULTS  Fixed-Wing Simulation Results Visualization
%
% Usage:
%   plot_results(t_out, x_out, u_log, param);
%
% Layout:
%   Left  (5 subplots): Altitude / Va+alpha / Attitude / Control Surfaces / Thrust
%   Right (full height): 3D Trajectory

    %% ====================================================================
    % 0. Derived Variables
    % ====================================================================
    N        = length(t);
    altitude = -x(:, 3);
    Va       = sqrt(x(:,4).^2 + x(:,5).^2 + x(:,6).^2);

    phi_deg   = rad2deg(x(:, 7));
    theta_deg = rad2deg(x(:, 8));
    psi_deg   = rad2deg(x(:, 9));

    alpha_deg = zeros(N, 1);
    for k = 1:N
        if Va(k) > 0.1
            alpha_deg(k) = rad2deg(atan2(x(k,6), x(k,4)));
        end
    end

    Tp_log      = u_log(:, 1);
    delta_a_deg = rad2deg(u_log(:, 2));
    delta_e_deg = rad2deg(u_log(:, 3));

    %% ====================================================================
    % 1. Figure & Style
    % ====================================================================
    figure('Name', 'Simulation Results', 'Color', [0.18 0.20 0.25], 'Position', [30, 30, 1200, 700]);

    % Colors
    C_alt   = [0.20 0.88 0.65];
    C_Va    = [0.95 0.78 0.20];
    C_alpha = [0.95 0.55 0.18];
    C_phi   = [0.90 0.35 0.35];
    C_theta = [0.35 0.68 0.98];
    C_psi   = [0.78 0.45 0.98];
    C_da    = [1.00 0.55 0.10];
    C_de    = [0.20 0.90 0.45];
    C_Tp    = [0.98 0.88 0.20];

    ax_style = {'Color',     [0.22 0.24 0.30], 'XColor',    [0.80 0.82 0.86], 'YColor',    [0.80 0.82 0.86], 'GridColor', [0.35 0.37 0.45], 'GridAlpha', 0.55, 'FontSize',  8};

    lw   = 1.6;
    lw_s = 1.0;
    dlim = 32;

    % Position constants
    left_x  = 0.05;
    left_w  = 0.36;
    right_x = 0.46;
    right_w = 0.50;

    n_rows  = 5;
    h_each  = 0.14;
    h_gap   = 0.025;
    bot     = @(row) 0.06 + (n_rows - row) * (h_each + h_gap);

    %% ====================================================================
    % 2. Left Subplots (5 rows)
    % ====================================================================

    % ── Row 1: Altitude ──────────────────────────────────────────────────
    ax1 = axes('Position', [left_x, bot(1), left_w, h_each], ax_style{:});
    hold on;
    plot(t, altitude, 'Color', C_alt, 'LineWidth', lw);
    yline(altitude(1), '--', 'Color', [C_alt 0.4], 'LineWidth', lw_s, 'HandleVisibility', 'off');
    ylabel('[m]', 'Color', [0.80 0.82 0.86]);
    title('Altitude', 'Color', C_alt, 'FontSize', 9);
    set(ax1, 'XTickLabel', []);
    grid on;

    % ── Row 2: Va + alpha ────────────────────────────────────────────────
    ax2 = axes('Position', [left_x, bot(2), left_w, h_each], ax_style{:});
    yyaxis left;
    plot(t, Va, 'Color', C_Va, 'LineWidth', lw);
    ylabel('Va [m/s]', 'Color', C_Va);
    ax2.YAxis(1).Color = C_Va;
    yyaxis right;
    plot(t, alpha_deg, 'Color', C_alpha, 'LineWidth', lw, 'LineStyle', '--');
    ylabel('\alpha [deg]', 'Color', C_alpha);
    ax2.YAxis(2).Color = C_alpha;
    title('Airspeed Va  /  Angle of Attack \alpha', 'Color', [0.93 0.93 0.95], 'FontSize', 9);
    set(ax2, 'XTickLabel', []);
    grid on;

    % ── Row 3: Attitude ──────────────────────────────────────────────────
    ax3 = axes('Position', [left_x, bot(3), left_w, h_each], ax_style{:});
    hold on;
    plot(t, phi_deg,   'Color', C_phi,   'LineWidth', lw, 'DisplayName', '\phi  Roll');
    plot(t, theta_deg, 'Color', C_theta, 'LineWidth', lw, 'DisplayName', '\theta  Pitch');
    plot(t, psi_deg,   'Color', C_psi,   'LineWidth', lw, 'DisplayName', '\psi  Yaw');
    yline(0, '-', 'Color', [0.5 0.5 0.5 0.35], 'LineWidth', 0.8, 'HandleVisibility', 'off');
    ylabel('[deg]', 'Color', [0.80 0.82 0.86]);
    title('Attitude \phi / \theta / \psi', 'Color', [0.93 0.93 0.95], 'FontSize', 9);
    leg = legend('Location', 'best', 'FontSize', 7);
    leg.TextColor = [0.85 0.87 0.90];
    leg.Color     = [0.18 0.20 0.26];
    leg.EdgeColor = [0.35 0.37 0.45];
    set(ax3, 'XTickLabel', []);
    grid on;

    % ── Row 4: Control Surfaces ──────────────────────────────────────────
    ax4 = axes('Position', [left_x, bot(4), left_w, h_each], ax_style{:});
    hold on;
    plot(t, delta_a_deg, 'Color', C_da, 'LineWidth', lw, 'DisplayName', '\delta_a  Aileron');
    plot(t, delta_e_deg, 'Color', C_de, 'LineWidth', lw, 'DisplayName', '\delta_e  Elevator');
    yline( dlim, ':', 'Color', [0.9 0.3 0.3 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    yline(-dlim, ':', 'Color', [0.9 0.3 0.3 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    yline(0, '-', 'Color', [0.5 0.5 0.5 0.35], 'LineWidth', 0.8, 'HandleVisibility', 'off');
    ylim([-dlim-2, dlim+2]);
    ylabel('[deg]', 'Color', [0.80 0.82 0.86]);
    title('Control Surfaces \delta_a / \delta_e', 'Color', [0.93 0.93 0.95], 'FontSize', 9);
    leg = legend('Location', 'best', 'FontSize', 7);
    leg.TextColor = [0.85 0.87 0.90];
    leg.Color     = [0.18 0.20 0.26];
    leg.EdgeColor = [0.35 0.37 0.45];
    set(ax4, 'XTickLabel', []);
    grid on;

    % ── Row 5: Thrust ────────────────────────────────────────────────────
    ax5 = axes('Position', [left_x, bot(5), left_w, h_each], ax_style{:});
    hold on;
    plot(t, Tp_log, 'Color', C_Tp, 'LineWidth', lw);
    ylabel('[N]',      'Color', [0.80 0.82 0.86]);
    xlabel('Time [s]', 'Color', [0.70 0.72 0.76]);
    title('Thrust Tp', 'Color', C_Tp, 'FontSize', 9);
    grid on;

    %% ====================================================================
    % 3. Right: 3D Trajectory (full height)
    % ====================================================================
    ax3d = axes('Position', [right_x, 0.06, right_w, 0.87], ax_style{:});
    ax3d.ZColor = [0.80 0.82 0.86];

    surface([x(:,1) x(:,1)], ...
            [x(:,2) x(:,2)], ...
            [altitude altitude], ...
            [altitude altitude], ...
            'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 2.5);
    colormap(ax3d, turbo);
    hold on;

    % Ground projection
    plot3(x(:,1), x(:,2), zeros(N,1), ...
          '-', 'Color', [0.55 0.65 0.85 0.22], 'LineWidth', 1.0);

    % Start ● End ■
    plot3(x(1,1),   x(1,2),   altitude(1),   'o', ...
          'Color', C_alt, 'MarkerFaceColor', C_alt, 'MarkerSize', 11);
    plot3(x(end,1), x(end,2), altitude(end), 's', ...
          'Color', C_Va,  'MarkerFaceColor', C_Va,  'MarkerSize', 11);

    xlabel('North [m]',    'Color', [0.80 0.82 0.86]);
    ylabel('East [m]',     'Color', [0.80 0.82 0.86]);
    zlabel('Altitude [m]', 'Color', [0.80 0.82 0.86]);
    title('3D Trajectory  (color = altitude)', 'Color', [0.93 0.93 0.95], 'FontSize', 11);
    view(45, 25);
    grid on;
    cb = colorbar(ax3d);
    cb.Color        = [0.80 0.82 0.86];
    cb.Label.String = 'Altitude [m]';
    cb.Label.Color  = [0.80 0.82 0.86];
end


% ========================================================================
% Layout:
%   Left  (5 rows): Altitude / Va+alpha / Attitude / delta_a+delta_e / Tp
%   Right (full  ): 3D Trajectory
%
% Author: Lim Minseok (minseoklim@postech.ac.kr)
% ========================================================================