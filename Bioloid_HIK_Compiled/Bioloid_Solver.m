function dq = Bioloid_Solver(J,Dx,P)
%% Priority loop and Convergence loop (Solver)
% Inputs:
%    J = cell with the Jacobians
%    Dx = cell with the Dx
%    P = number of priorities
%
% Outputs:
%    dq = Articular Velocities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Initial Conditions
Pn = eye(24);
Dt = zeros(24,1);
G = 0.8;
%

%Setting up values

%
for i=1:P
Dx_h = Dx{i}-(J{i}*Dt);
J{i} = J{i}*Pn;
[Jtd,Jt] = JacPsInv(J{i});
Dt = Dt+(Jtd*Dx_h);
Pn = Pn-(Jt*J{i});
end

dq=Dt;

end
