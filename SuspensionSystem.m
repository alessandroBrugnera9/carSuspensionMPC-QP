function x_next = SuspensionSystem(x, u, dh, p)

global system

if strcmp(system, 'active')
    dynamics = @ActiveVibrationDamper_ODE;
elseif strcmp(system, 'semiactive')
    dynamics = @SemiactiveVibrationDamper_ODE;
end

% Fourth-order Runge-Kutta discretization of the continuous dynamics
k1 = dynamics(x, u, dh, p);
k2 = dynamics(x+p.Ts/2*k1, u, dh, p);
k3 = dynamics(x+p.Ts/2*k2, u, dh, p);
k4 = dynamics(x+p.Ts*k3, u, dh, p);

x_next = x + 1/6 * p.Ts * (k1 + 2*k2 + 2*k3 + k4);

end

function dxdt = ActiveVibrationDamper_ODE(x, u, dh, p)
% ODE Model of the active vibration damping system.
% x - state
% u - control input
% dh - disturbance (rate of change of the road profile)
% p - system parameters

% States:
% x1 = z_s - z_us -> Relative displacement of the sprung and unsprung mass
% x2 = dot_z_s    -> Vertical velocity of the sprung mass
% x3 = h - z_us   -> Tyre deflection
% x4 = dot_z_us   -> Vertical velocity of the unsprung mass

% System matrices
A = [0, 1, 0, -1; ...
     -p.ks/p.ms, -p.cs/p.ms, 0, p.cs/p.ms; ...
     0, 0, 0, -1; ...
     p.ks/p.mus, p.cs/p.mus, p.kt/p.mus, -p.cs/p.mus];

B = [0; ...
     -1/p.ms; ...
     0; ...
     1/p.mus];
 
V = [0; 0; 1; 0];
 
% System dynamics
dxdt = A * x + B * u + V * dh;

end