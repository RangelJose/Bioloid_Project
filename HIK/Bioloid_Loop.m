function dq = Bioloid_Loop(J1,dX1,J2,dX2,J3,dX3,J4,dX4,J5,dX5)
%% Priority loop and Convergence loop
% Inputs:
%    J1 = First end-effector Jacobian
%    J2 = Second end-effector Jacobian
%    J3 = Third end-effector Jacobian
%    J4 = Fourth end-effector Jacobian
%    J5 = Fifth end-effector Jacobian
%    dX1 = First end-effector delta x
%    dX2 = Second end-effector delta x
%    dX3 = Third end-effector delta x
%    dX4 = Fourth end-effector delta x
%    dX5 = Fifth end-effector delta x
%
% Outputs:
%    dq = Articular Velocities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = 5;
Dx = cell(p,1);
J = cell(p,1);

%Initial Conditions
Pn = eye(24);
Dt = zeros(24,1);
G = .80;
%

%Setting up values
J{1} = G*J1;
J{2} = G*J2;
J{3} = G*J3;
J{4} = G*J4;
J{5} = G*J5;
Dx{1} = dX1;
Dx{2} = dX2;
Dx{3} = dX3;
Dx{4} = dX4;
Dx{5} = dX5;

%
for i=1:p
Dx_h = Dx{i}-(J{i}*Dt);
J{i} = J{i}*Pn;
[Jtd,Jt] = JacPsInv(J{i});
Dt = Dt+(Jtd*Dx_h);
Pn = Pn-(Jt*J{i});
end

dq=Dt;

end
