%% === Parameters of the linear quarter-car model ====================== %%

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
p.Tf = 3.1;

% Control / Prediction horizon
p.N = 5;

% Matrices of the time-continuous system
A = [0, 1, 0, -1; ...
     -p.ks/p.ms, -p.cs/p.ms, 0, p.cs/p.ms; ...
     0, 0, 0, -1; ...
     p.ks/p.mus, p.cs/p.mus, p.kt/p.mus, -p.cs/p.mus];

B = [0; ...
     -1/p.ms; ...
     0; ...
     1/p.mus];
 
C = [-p.ks/p.ms, -p.cs/p.ms, 0, p.cs/p.ms; ...
     1, 0, 0, 0; ...
     0, 0, 1, 0];
 
D = [-1/p.ms; 0; 0];

% ======================================================================= %
% Model-related computations

% Set up the linear, nominal state space model and discretize
systemContinuos = ss(A,B,C,D);
sys = c2d(systemContinuos,p.Ts); 

% ======================================================================= %
% Cost-function-related computations

% Parameters of a quadratic stage cost function in the outputs and inputs
R = 1E-05;                     % Input penalty: Penalize large input values
Q = diag([1, 5E+04, 5E+03]);

% Weighting matrices for equivalently expressing the cost in terms of the
% states and the inputs
S = sys.C' * Q * sys.D;
R = R + sys.D' * Q * sys.D;
Q = sys.C' * Q * sys.C;

% Compute terminal penalty from unconstrained, infinite horizon LQR
%[K,S,e] = dlqr(sys.A,sys.B,Q,R,S) maybe use this method
%[K,SRicatti,P] = lqr(sys, Q, R, S);
[~, P] = lqr(sys.A,sys.B, Q, R, S);


% Concatenate the weighting matrices for the entire state and input
% sequence in order to represent the cost function in a vectorized form
p.Lx = [repmat({Q},1, p.N), {P}];
p.Lx = blkdiag(p.Lx{:});

p.Lu = repmat({R}, 1, p.N);
p.Lu = blkdiag(p.Lu{:});

p.Lxu = repmat({S}, 1, p.N);
p.Lxu = blkdiag(p.Lxu{:});
p.Lxu = [p.Lxu; zeros(p.nx, p.N*p.nu)];

% ======================================================================= %
% Quadratic programming reformulations

% Initialize cell arrays, representing the block matrices M and W
M = cell(p.N+1, 1);
W = cell(p.N+1, p.N);

% Build up block matrices M and W to directly express the state sequence in
% terms of the initial state and the input sequence

M{1} = eye(p.nx);
W(1, :) = {zeros(p.nx, p.nu)};        
for i = 2 : p.N+1 % starting in 2 because the first line of W is only zeros
    M{i} = sys.A^(i-1);

    % Exploit structure of W: first element of each line needs to be
    % computed while all follow-up elements are the previous line
    % shifted by one to the right
    W(i, :) = [{sys.A^(i-2) * sys.B}, W(i-1, 1:end-1)];
end

% Convert the cell arrays representing the block matrices into numerical
% arrays (matrices)
M = cell2mat(M);
W = cell2mat(W);

% Compute the QP cost matrix H
p.H = 2 * (p.Lu + W'*p.Lxu + p.Lxu' * W + W' * p.Lx * W);

% Due to numeric errors, the matrix is only almost symmetric (deviations in
% late decimal places). As QP requires H to be symmetric, it is symmetrized
% to compensate for those numerical errors
p.H = (p.H + p.H')/2;


% Compute the QP cost matrix f^T as a function of the initial condition
p.fT = @(x)(2 * x' * M' * (p.Lx * W + p.Lxu));