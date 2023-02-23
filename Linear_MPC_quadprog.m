function  uVector = Linear_MPC_quadprog (x0, uVector0, p)

fT = p.fT(x0); %calculate cost dependant on initial condition

uVector = quadprog(p.H, ...     Cost function parameter H
                 fT, ...      Cost function parameter f^T
                 [], ...      Coefficient matrix for linear inequality constraints
                 [], ...      Inhomogeneity for linear inequality constraints                                   
                 [], ...      Coefficient matrix for linear equality constraints
                 [], ...      Inhomogeneity for linear equality constraints
                 [], ...      Lower bound on u_seq
                 [], ...      Upper bound on u_seq
                 uVector0, ... Initial guess
                 [] ...       Solver options
                 );  
             
end