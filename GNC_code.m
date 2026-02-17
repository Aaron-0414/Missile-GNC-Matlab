%% ========================================================================
%  MISSILE GNC SIMULATION - HYBRID 6DOF
%  Uses proven 3DOF kinematics + realistic attitude dynamics for visualization
%  
%  Author: [Your Name]
%  Date: February 2026
%  
%  STRATEGY: Decouple guidance/kinematics (which work) from attitude (for realism)
% =========================================================================

clear all; close all; clc;

fprintf('======================================\n');
fprintf('  HYBRID 6DOF MISSILE GNC\n');
fprintf('======================================\n\n');

%% ===================== SIMULATION PARAMETERS =====================

dt = 0.01;              
t_max = 30;             
time = 0:dt:t_max;
n_steps = length(time);

%% ===================== INITIAL CONDITIONS =====================

% Missile translational state (inertial frame)
missile.pos = [0; 0; 1000];           
missile.vel = [250; 0; 0];            
missile.speed = 250;                   % Maintain constant speed

% Missile attitude (for visualization)
missile.euler = [0; 0; 0];            
missile.omega = [0; 0; 0];            

% Missile properties
missile.mass = 100;                    
missile.Ixx = 10;                      
missile.Iyy = 50;                      
missile.Izz = 50;                      

% Target
target.pos = [4000; 1000; 1000];      
target.vel = [0; 40; 0];              

%% ===================== PARAMETERS =====================

guidance.N = 4;                        
guidance.max_accel = 80;               

%% ===================== DATA STORAGE =====================

missile_traj = zeros(3, n_steps);
target_traj = zeros(3, n_steps);
range_hist = zeros(1, n_steps);
accel_cmd_hist = zeros(1, n_steps);
euler_hist = zeros(3, n_steps);
velocity_hist = zeros(1, n_steps);
miss_distance = inf;

%% ===================== MAIN LOOP =====================

fprintf('Running hybrid 6DOF simulation...\n');

for i = 1:n_steps
    t = time(i);
    
    % Store states
    missile_traj(:, i) = missile.pos;
    target_traj(:, i) = target.pos;
    euler_hist(:, i) = missile.euler;
    velocity_hist(i) = norm(missile.vel);
    
    % Relative geometry
    R_rel = target.pos - missile.pos;
    range = norm(R_rel);
    range_hist(i) = range;
    
    % Check intercept
    if range < 10
        miss_distance = range;
        fprintf('\n*** INTERCEPT! ***\n');
        fprintf('Time: %.2f s\n', t);
        fprintf('Miss: %.2f m\n\n', miss_distance);
        
        missile_traj = missile_traj(:, 1:i);
        target_traj = target_traj(:, 1:i);
        range_hist = range_hist(1:i);
        accel_cmd_hist = accel_cmd_hist(1:i);
        euler_hist = euler_hist(:, 1:i);
        velocity_hist = velocity_hist(1:i);
        time = time(1:i);
        break;
    end
    
    if mod(i, 1000) == 0
        fprintf('  t = %.1f s, Range = %.1f m\n', t, range);
    end
    
    %% ========== GUIDANCE LAW (PROVEN) ==========
    
    if range > 1
        LOS = R_rel / range;
        V_rel = target.vel - missile.vel;
        range_rate = dot(V_rel, LOS);
        V_c = -range_rate;
        
        LOS_rate = (V_rel - range_rate * LOS) / range;
        
        % Proportional Navigation
        accel_cmd = guidance.N * V_c * LOS_rate;
        
        accel_mag = norm(accel_cmd);
        if accel_mag > guidance.max_accel
            accel_cmd = accel_cmd * (guidance.max_accel / accel_mag);
        end
        accel_cmd_hist(i) = accel_mag;
    else
        accel_cmd = [0; 0; 0];
        accel_cmd_hist(i) = 0;
    end
    
    %% ========== 3DOF KINEMATICS (PROVEN) ==========
    
    % Update velocity with commanded acceleration
    missile.vel = missile.vel + accel_cmd * dt;
    
    % Maintain constant speed (energy management)
    current_speed = norm(missile.vel);
    if current_speed > 0
        missile.vel = missile.vel * (missile.speed / current_speed);
    end
    
    % Update position
    missile.pos = missile.pos + missile.vel * dt;
    
    %% ========== ATTITUDE DYNAMICS (FOR REALISM) ==========
    
    % Desired body orientation: align with velocity vector
    V_mag = norm(missile.vel);
    if V_mag > 1
        vel_hat = missile.vel / V_mag;
        
        % Desired Euler angles from velocity vector
        % Assuming velocity = [Vx, Vy, Vz]
        % Yaw: arctan(Vy / Vx)
        % Pitch: arctan(-Vz / sqrt(Vx^2 + Vy^2))
        
        desired_yaw = atan2(vel_hat(2), vel_hat(1));
        xy_mag = sqrt(vel_hat(1)^2 + vel_hat(2)^2);
        desired_pitch = atan2(-vel_hat(3), xy_mag);
        desired_roll = 0;  % Keep wings level
        
        desired_euler = [desired_roll; desired_pitch; desired_yaw];
        
        % Simple first-order lag to desired attitude
        tau_attitude = 0.2;  % Time constant (seconds)
        euler_error = desired_euler - missile.euler;
        
        % Wrap angle errors to [-pi, pi]
        euler_error = wrapToPi(euler_error);
        
        % First-order response
        euler_rate = euler_error / tau_attitude;
        
        % Update Euler angles
        missile.euler = missile.euler + euler_rate * dt;
        missile.euler = wrapToPi(missile.euler);
        
        % Compute body rates from Euler rate
        missile.omega = euler_rate_to_omega(missile.euler, euler_rate);
    end
    
    %% ========== TARGET DYNAMICS ==========
    
    if t > 5
        target_accel = [0; 10*sin(0.2*t); 0];
    else
        target_accel = [0; 0; 0];
    end
    
    target.vel = target.vel + target_accel * dt;
    target.pos = target.pos + target.vel * dt;
end

%% ===================== RESULTS =====================

if miss_distance == inf
    miss_distance = range_hist(end);
    fprintf('\nSimulation ended.\n');
    fprintf('Final range: %.2f m\n\n', miss_distance);
end

fprintf('======================================\n');
fprintf('  RESULTS\n');
fprintf('======================================\n');
fprintf('Miss Distance: %.2f m\n', miss_distance);
fprintf('Flight Time:   %.2f s\n', time(end));
fprintf('Final Speed:   %.2f m/s\n', velocity_hist(end));
fprintf('Max Accel Cmd: %.2f m/s^2\n', max(accel_cmd_hist));
fprintf('Final Pitch:   %.2f deg\n', rad2deg(missile.euler(2)));
fprintf('Final Yaw:     %.2f deg\n', rad2deg(missile.euler(3)));
fprintf('======================================\n\n');

%% ===================== PLOTS =====================

fprintf('Generating plots...\n');

screen_size = get(0, 'ScreenSize');
fig_width = 1600;
fig_height = 900;
fig_x = (screen_size(3) - fig_width) / 2;
fig_y = (screen_size(4) - fig_height) / 2;

figure('Position', [fig_x, fig_y, fig_width, fig_height]);

% 3D Trajectory
subplot(2, 4, 1);
plot3(missile_traj(1,:), missile_traj(2,:), missile_traj(3,:), 'b-', 'LineWidth', 3);
hold on;
n_mark = min(20, size(target_traj, 2));
mark_int = max(1, floor(size(target_traj, 2) / n_mark));
mark_idx = 1:mark_int:size(target_traj, 2);
plot3(target_traj(1,:), target_traj(2,:), target_traj(3,:), 'r--', 'LineWidth', 2);
plot3(target_traj(1,mark_idx), target_traj(2,mark_idx), target_traj(3,mark_idx), ...
      'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r');
plot3(missile_traj(1,1), missile_traj(2,1), missile_traj(3,1), ...
      'bs', 'MarkerSize', 14, 'MarkerFaceColor', 'b', 'LineWidth', 2);
plot3(missile_traj(1,end), missile_traj(2,end), missile_traj(3,end), ...
      'bx', 'MarkerSize', 20, 'LineWidth', 4);
grid on; axis equal;
xlabel('X (m)', 'FontWeight', 'bold');
ylabel('Y (m)', 'FontWeight', 'bold');
zlabel('Z (m)', 'FontWeight', 'bold');
title('3D Trajectory', 'FontSize', 12, 'FontWeight', 'bold');
legend('Missile', 'Target Path', 'Target Pos', 'Start', 'Intercept', 'Location', 'best', 'FontSize', 9);
view(45, 25);

% Range
subplot(2, 4, 2);
plot(time, range_hist, 'k-', 'LineWidth', 2.5);
grid on;
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Range (m)', 'FontWeight', 'bold');
title(sprintf('Range (Miss: %.1f m)', miss_distance), 'FontSize', 12, 'FontWeight', 'bold');

% Acceleration Command
subplot(2, 4, 3);
plot(time, accel_cmd_hist, 'b-', 'LineWidth', 2.5);
hold on;
yline(guidance.max_accel, 'r--', 'Limit', 'LineWidth', 2);
grid on;
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Accel (m/s^2)', 'FontWeight', 'bold');
title('Commanded Acceleration', 'FontSize', 12, 'FontWeight', 'bold');

% Velocity (should be constant)
subplot(2, 4, 4);
plot(time, velocity_hist, 'g-', 'LineWidth', 2.5);
yline(missile.speed, 'k--', 'Target', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Speed (m/s)', 'FontWeight', 'bold');
title('Missile Velocity', 'FontSize', 12, 'FontWeight', 'bold');

% Top View
subplot(2, 4, 5);
plot(missile_traj(1,:), missile_traj(2,:), 'b-', 'LineWidth', 2.5);
hold on;
plot(target_traj(1,:), target_traj(2,:), 'r--', 'LineWidth', 2);
plot(target_traj(1,mark_idx), target_traj(2,mark_idx), ...
     'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r');
grid on; axis equal;
xlabel('X (m)', 'FontWeight', 'bold');
ylabel('Y (m)', 'FontWeight', 'bold');
title('Top View (X-Y)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Missile', 'Target Path', 'Target Pos', 'Location', 'best', 'FontSize', 9);

% Side View
subplot(2, 4, 6);
plot(missile_traj(1,:), missile_traj(3,:), 'b-', 'LineWidth', 2.5);
hold on;
plot(target_traj(1,:), target_traj(3,:), 'r--', 'LineWidth', 2);
grid on; axis equal;
xlabel('X (m)', 'FontWeight', 'bold');
ylabel('Z (m)', 'FontWeight', 'bold');
title('Side View (X-Z)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Missile', 'Target', 'Location', 'best', 'FontSize', 9);

% Euler Angles
subplot(2, 4, 7);
plot(time, rad2deg(euler_hist(1,:)), 'r-', 'LineWidth', 2); hold on;
plot(time, rad2deg(euler_hist(2,:)), 'g-', 'LineWidth', 2);
plot(time, rad2deg(euler_hist(3,:)), 'b-', 'LineWidth', 2);
grid on;
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Angle (deg)', 'FontWeight', 'bold');
title('Attitude (Euler Angles)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best', 'FontSize', 9);

% Closing Velocity
subplot(2, 4, 8);
if length(range_hist) > 1
    closing_vel = [-range_hist(1), -diff(range_hist)/dt];
    plot(time, closing_vel, 'k-', 'LineWidth', 2);
end
grid on;
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Closing Vel (m/s)', 'FontWeight', 'bold');
title('Closing Velocity', 'FontSize', 12, 'FontWeight', 'bold');

sgtitle('Missile GNC - Hybrid 6DOF (Proven Kinematics + Realistic Attitude)', ...
        'FontSize', 14, 'FontWeight', 'bold');

fprintf('Complete!\n\n');

%% ===================== HELPER FUNCTIONS =====================

function omega = euler_rate_to_omega(euler, euler_rate)
    % Convert Euler angle rates to body angular rates
    phi = euler(1);
    theta = euler(2);
    
    cphi = cos(phi); sphi = sin(phi);
    cth = cos(theta); sth = sin(theta);
    
    % Avoid singularity
    if abs(cth) < 0.01
        cth = 0.01 * sign(cth);
    end
    
    % Inverse transformation
    T_inv = [1, 0,           -sth;
             0, cphi,         sphi*cth;
             0, -sphi,        cphi*cth];
    
    omega = T_inv * euler_rate;
end