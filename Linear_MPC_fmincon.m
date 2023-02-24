function  uVector = Linear_MPC_fmincon (x0, uVector0, p)
    % creating closure with this horizon conditions
    costWithX0Function = @(uVector) Cost_Function(x0,uVector,p);
    
    % defining parameters for optimaztion but just using options
    options = optimoptions('fmincon','Display', 'off');
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    nonlcon = [];
    
    uVector = fmincon(costWithX0Function,uVector0,A,b,Aeq,beq,lb,ub,nonlcon,options);

end

function JCostN = Cost_Function(x0, uVector0, p)

    JCostN = 1/2 * uVector0' * p.H * uVector0 + p.fT(x0) * uVector0;

end