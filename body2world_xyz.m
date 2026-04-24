function R_b_w = body2world_xyz(phi, theta, psi)
    %% ROTATION_XYZ - Combined ZYX Euler angle rotation matrix
    % This operation will take right-handed rule
    % Inputs:
    %   phi   - Roll angle  (rotation about X-axis) in radians
    %   theta - Pitch angle (rotation about Y-axis) in radians
    %   psi   - Yaw angle   (rotation about Z-axis) in radians
    %
    % Output:
    %   R_b_w - 3x3 rotation matrix from body frame to world frame
    %
    %% TODO: Fill in the rotation matrix for rotation about X,Y,Z-axis
    %       and combine the individual rotation matrices in the correct order
    %
    % CRITICAL CONCEPT: Body → World transformation
    %   - ZYX Euler: World frame rotates Z → Y → X to become Body frame
    %   - To go BACK (Body → World), we apply INVERSE in REVERSE order
    %   - Since R^(-1) = R^T for rotation: R_b_w = (R_x * R_y * R_z)^T
    %   - Using (AB)^T = B^T * A^T: R_b_w = R_z^T * R_y^T * R_x^T = R_z * R_y * R_x
    %
    % Revised Scenario for Body → World:
    %   - You have a vector v in body frame
    %   - First, undo Roll (X)  → v' = R_x * v
    %   - Then, undo Pitch (Y)  → v''  = R_y * v' = R_y * R_x * v
    %   - Finally, undo Yaw (Z) → v''' = R_z * v'' = R_z * R_y * R_x * v  

    %% Get individual rotation matrices
    R_x = @(phi)[1,        0,         0;
                 0,  cos(phi), -sin(phi);
                 0,  sin(phi),  cos(phi)];       % Roll about X

    R_y = @(theta)[ cos(theta), 0, sin(theta);
                          0,    1,      0;
                   -sin(theta), 0, cos(theta)];    % Pitch about Y

    R_z = @(psi)[cos(psi), -sin(psi), 0;
                 sin(psi),  cos(psi), 0;
                     0,         0,    1]; % Yaw about Z
     
    %% TODO: What is the correct multiplication order?   
    %HINT use ' to 
    R_w_b = R_x(phi)' * R_y(theta)' * R_z(psi)';
    R_b_w = R_w_b';

    %% Uncomment and write body2world_xyz_professor(1, 3, 2) to command then check1 from isequal 
    % ! Check and comment again with Ctrl R
    % Verify_R_b_w = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
    %                 cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
    %                     -sin(theta),                          sin(phi)*cos(theta),                              cos(phi)*cos(theta)        ];
    % isequal(R_b_w, Verify_R_b_w)
end
