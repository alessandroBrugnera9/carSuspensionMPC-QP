function y = ControlledQuantities(x, u, p)

global system

% The control-relevant quantities / performance characteristics are 
% computed from the states and the input.
 
if strcmp(system, 'active')
    
    C = [-p.ks/p.ms, -p.cs/p.ms, 0, p.cs/p.ms; ...
         1, 0, 0, 0; ...
         0, 0, 1, 0];
    
    D = [-1/p.ms; 0; 0];
    
    y = C * x + D * u;

end

end