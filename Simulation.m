clc; clear all; close all;

%% === Attention ======================================================= %%
% You need to work only in the section "Closed-Loop Simulation".
% Therein, all places where the code has to be completed are marked.

% Change the following variable according to the task you are working on 
% and the terein used system, respectively.
global system 

% Task 1 and 2: 'active'
% Task 3 and 4: 'semiactive'
system = 'active'; 

%% === Initialization ================================================== %%

if ~(strcmp(system, 'active') || strcmp(system, 'semiactive'))
    error('Infeasible value for variable "system". Use "active" or "semiactive".');
end

% Load parameters
if strcmp(system, 'active')
    Parameters_ActiveSystem
elseif strcmp(system, 'semiactive')
    Parameters_SemiactiveSystem
end

% Make simulation time and storing variables for states, outputs and input.  
% The initial state is the origin (= system is in the steady-state).
t = p.T0 : p.Ts : p.Tf;

% Uncontrolled / passive vibration damper
x_uncontrolled = zeros(p.nx, length(t));
y_uncontrolled = zeros(p.ny, length(t));

% Controlled system / (semi-)active vibration damper
x_controlled = zeros(p.nx, length(t));
y_controlled = zeros(p.ny, length(t));

% Control input
uVector = zeros(p.nu, length(t)-1);

% Computation times
if strcmp(system, 'active')
    cpu_time_quadprog = zeros(1, length(t)-1);
    cpu_time_fmincon = zeros(1, length(t)-1);
elseif strcmp(system, 'semiactive')
    cpu_time = zeros(1, length(t)-1);
end

%% === Road Profile / Disturbance ====================================== %%

% % Generate road profile
h = zeros(1, length(t));                % Ideal plain road

h(10:10+1/p.Ts) = -0.03;                % Road hole with depth 3 cm

% Rate of change of the road profile
dh = [0, diff(h)/p.Ts];

%% === Closed-Loop Simulation ========================================== %%

% Simulation loop
for k = 1 : 1 : length(t)-1
    
    if strcmp(system, 'active')
    
        % === Without control, i.e., passive vibration damper =========== %
        
        % Compute dynamics
        x_uncontrolled(:, k+1) = SuspensionSystem(x_uncontrolled(:, k), 0, dh(k), p);
        
        % Compute performance-relevant quantities
        y_uncontrolled(:, k+1) = ControlledQuantities(x_uncontrolled(:, k+1), 0, p);
        
        % === With control ============================================== %
        
        % Initial guess of the input sequence for the optimization routine      
        if k == 1
            uVector0 = zeros(p.N, 1);
        else
            uVector0 = [uVectorNext(2:end); uVectorNext(end)];
        end
        
        % MPC using quadratic programming
        tic
        uVectorNext = Linear_MPC_quadprog(x_controlled(:, k), uVector0 , p);
        cpu_time_quadprog(k) = toc;
        
        % MPC using fmincon (universal solver)
        
        % Extract control input for the next sampling period
        u(k) = uVectorNext(1);
        
        % Apply optimal input to the system
        x_controlled(:, k+1) = SuspensionSystem(x_controlled(:, k), u(k), dh(k), p);
        
        % Compute performance-relevant quantities
        y_controlled(:, k+1) = ControlledQuantities(x_controlled(:, k+1), u(k), p);
        
    end
    
end

%% === Plotting ======================================================== %%

% === System states ===================================================== %

figure(1);

state_names = {'$$\mathbf{z_s-z_{us}~(m)}$$', ...
    '$$\mathbf{\dot{z}_s~(ms^{-1})}$$', '$$\mathbf{h-z_{us}~(m)}$$', ...
    '$$\mathbf{\dot{z}_{us}~(ms^{-1})}$$'};

for k = 1 : 1 : p.nx
    
    subplot(2, 2, k); hold on; grid on;
    
    % Passive vibration absorber / uncontrolled system
    plot(t, x_uncontrolled(k, :), 'Linewidth', 2);
    
    % Active vibration absorber / controlled system
    plot(t, x_controlled(k, :), 'Linewidth', 2);
    
    xlabel('t (s)'); ylabel(state_names{k}, 'Interpreter', 'Latex');
    legend('Passive', 'Active');
    xlim([t(1), t(end)]);
    
end

sgtitle('States');

% === Performance measures / system outputs ============================= %

figure(2);

output_names = {'$$\mathbf{\ddot{z}_s~(ms^{-2})}$$', ...
    '$$\mathbf{z_s-z_{us}~(m)}$$', '$$\mathbf{h-z_{us}~(m)}$$'};

for k = 1 : 1 : p.ny
    
    subplot(1, 3, k); hold on; grid on;
    
    % Passive viration absorber / uncontrolled system
    plot(t(1:end), y_uncontrolled(k, :), 'Linewidth', 2);
    
    % Active vibration absorber / controlled system
    plot(t(1:end), y_controlled(k, :), 'Linewidth', 2);
    
    xlabel('t (s)'); ylabel(output_names{k}, 'Interpreter', 'Latex');
    legend('Passive', 'Active');
    xlim([t(1), t(end)]);
    
end

sgtitle('Performance Measures');

% === Control input ===================================================== %

figure(3);
hold on; grid on;

plot(t(1:end-1), uVector, 'Linewidth', 2);

xlabel('t (s)'); 
if strcmp(system, 'active')
    ylabel('$$\mathbf{u~(N)}$$', 'Interpreter', 'Latex');
elseif strcmp(system, 'semiactive')
    ylabel('$$\mathbf{u~(\frac{N s}{m})}$$', 'Interpreter', 'Latex');
end
xlim([t(1), t(end-1)]);
title('Control Input');

% === Motions and Road Profile ========================================== %

figure(4);
    
% Road profile
subplot(4, 2, 1); hold on; grid on;
plot(t, h, 'Linewidth', 2);
xlim([t(1), t(end)]); ylim([min(h)-0.01, max(h)+0.01]);
xlabel('t (s)'); ylabel('h');

% Rate of change of road profile
subplot(4, 2, 2); hold on; grid on;
plot(t, dh, 'Linewidth', 2);
xlim([t(1), t(end)]); ylim([min(dh)-0.5, max(dh)+0.5]);
xlabel('t (s)'); ylabel('$$\dot{h}$$', 'Interpreter', 'Latex');

% Displacement of unsprung mass
subplot(4, 2, 3); hold on; grid on;

plot(t, h-x_uncontrolled(3, :), 'Linewidth', 2);
plot(t, h-x_controlled(3, :), 'Linewidth', 2);

xlabel('t (s)'); ylabel('z_u_s');
xlim([t(1), t(end)]);
legend('Passive', 'Active');

% Displacement of sprung mass
subplot(4, 2, 4); hold on; grid on;

plot(t, x_uncontrolled(1, :)+h-x_uncontrolled(3, :), 'Linewidth', 2);
plot(t, x_controlled(1, :)+h-x_controlled(3, :), 'Linewidth', 2);

xlabel('t (s)'); ylabel('z_s');
xlim([t(1), t(end)]);
legend('Passive', 'Active');

% Velocity unsprung mass
subplot(4, 2, 5); hold on; grid on;

plot(t, x_uncontrolled(4, :), 'Linewidth', 2);
plot(t, x_controlled(4, :), 'Linewidth', 2);

xlabel('t (s)'); ylabel('$$\dot{z}_{us}$$', 'Interpreter', 'Latex');
xlim([t(1), t(end)]);
legend('Passive', 'Active');

% Velocity sprung mass
subplot(4, 2, 6); hold on; grid on;

plot(t, x_uncontrolled(2, :), 'Linewidth', 2);
plot(t, x_controlled(2, :), 'Linewidth', 2);

xlabel('t (s)'); ylabel('$$\dot{z}_{s}$$', 'Interpreter', 'Latex');
xlim([t(1), t(end)]);
legend('Passive', 'Active');

% Acceleration unsprung mass

if strcmp(system, 'active')
    ddz_us_passive = 1/p.mus * sum([p.ks, p.cs, p.kt, -p.cs] .* x_uncontrolled', 2);
    ddz_us_active = 1/p.mus * (sum([p.ks, p.cs, p.kt, -p.cs] .* x_controlled', 2) + [0; uVector']);
elseif strcmp(system, 'semiactive')
    ddz_us_passive = 1/p.mus * sum([p.ks, p.cs, p.kt, -p.cs] .* x_uncontrolled', 2);
    ddz_us_active = 1/p.mus * sum([p.ks*ones(length(uVector)+1, 1), [p.cs; uVector'], p.kt*ones(length(uVector)+1, 1), -[p.cs; uVector']] .* x_controlled', 2);
end

subplot(4, 2, 7); hold on; grid on;

plot(t, ddz_us_passive, 'Linewidth', 2);
plot(t, ddz_us_active, 'Linewidth', 2);

xlabel('t (s)'); ylabel('$$\ddot{z}_{us}$$', 'Interpreter', 'Latex');
xlim([t(1), t(end)]);
legend('Passive', 'Active');

% Acceleration sprung mass
subplot(4, 2, 8); hold on; grid on;

plot(t, y_uncontrolled(1, :), 'Linewidth', 2);
plot(t, y_controlled(1, :), 'Linewidth', 2);

xlabel('t (s)'); ylabel('$$\ddot{z}_{s}$$', 'Interpreter', 'Latex');
xlim([t(1), t(end)]);
legend('Passive', 'Active');

% === Computation Time ================================================== %

figure(5); hold on; grid on;

if strcmp(system, 'active')
    plot((1:1:length(t)-1), cpu_time_quadprog*1000, '.', 'Markersize', 15);
    plot((1:1:length(t)-1), cpu_time_fmincon*1000, '.', 'Markersize', 15);
elseif strcmp(system, 'semiactive')
    plot((1:1:length(t)-1), cpu_time*1000, '.', 'Markersize', 15);
end

plot([0, length(t)], [10, 10], 'k-', 'Linewidth', 2);

xlim([0 length(t)]);
xlabel('Number of OCP'); ylabel('Computation Time (ms)');
title('Computation Times');

if strcmp(system, 'active')
    legend('quadprog', 'fmincon', 'Sampling time (10 ms)');
elseif strcmp(system, 'semiactive')
    legend('fmincon', 'Sampling time (10 ms)');
end

% === Quantification ==================================================== %

% Quantifying the oscillations/deviations of the controlled quantities from
% the reference (origin) via approximate integration
error_int_active = p.Ts * sum(abs(y_controlled), 2);
error_int_passive = p.Ts * sum(abs(y_uncontrolled), 2);

disp('Quantification of the deviations of the performance-relevant quantities via approximate error integrals.');
disp('Uncontrolled system:');
disp(['y_1: ', num2str(error_int_passive(1)), ' | y_2: ', num2str(error_int_passive(2)), ' | y_3: ', num2str(error_int_passive(3))]);
disp('Controlled system:');
disp(['y_1: ', num2str(error_int_active(1)), ' | y_2: ', num2str(error_int_active(2)), ' | y_3: ', num2str(error_int_active(3))]);
