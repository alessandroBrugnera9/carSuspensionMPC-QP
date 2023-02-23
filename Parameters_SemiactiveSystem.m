%% === Parameters of the basic quarter-car model ======================= %%

% Number of states, inputs, disturbances, "outputs"
p.nx = 4;
p.nu = 1;
p.ny = 3;

% 1/4 of the vehicle mass (sprung mass)
p.ms = 327; % sprung mass (kg)

% Spring constant of the suspension spring (N/m)
p.ks  = 60186;

% Damping constant of the (passive) shock damper
p.cs = 2801; % (Ns/m)

% Unsprung mass (kg) associated to the tyre and the axle
p.mus = 116;

% Tyre spring constant (N/m)
p.kt = 256740;

%% === Parameters for the controller (and the simulation) ============== %%

% Initial, sampling and final time
p.T0 = 0;
p.Ts = 0.01;
p.Tf = 3;

% Control / Prediction horizon
p.N = 5;

% ======================================================================= %
% Linearization of the nominal model

% Target steady state (reference) and small numerical deviation
% !!! CODE TO BE ADDED !!!

% Dynamics matrix A and output matrix C
% !!! CODE TO BE ADDED !!!

% Input matrix B and feedthrough matrix D
% !!! CODE TO BE ADDED !!!

% Set-up state space system
% !!! CODE TO BE ADDED !!!

% ======================================================================= %
% Cost-function-related computations

% Parameters of a quadratic stage cost function in the outputs and inputs
% !!! CODE TO BE ADDED !!!

% Compute terminal penalty
% !!! CODE TO BE ADDED !!!