function  Ja = Bioloid_Ja_SV(T_ee,T,EE,P)
%% Analitycal Jacobian matrix
% Inputs:
%    T_ee = End-effector matrix
%    T = Every Transformation of forward Kinematics
%    EE = The last link where the Jacobian will be obtained
%    P = Priorities
% Outputs:
%    Ja = Analitycal Jacobian matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Bioloidmodel EulerConvention

Ja = cell(1,P);
for i = 1:P
    switch EE{i}
    case 1
        EE{i} = 9;
    case 2
        EE{i} = 12;
    case 3
        EE{i} = 18;
    case 4
        EE{i} = 24;
    case 5
        EE{i} = 6;
    end
%Obtaining Geometric Jacobian
[Jv,Jw] = Jg_EE(T_ee{i}(1:3,4),T,EE{i},Bioloidmodel.parent,Bioloidmodel.axes);
%JG obtained

%Obtaining Euler Angles
[E1,E2,E3] = Euler_Angles(EulerConvention.IndexSaved,T_ee{i}(1:3,1:3));
%Euler Angles Obtained

%Obtaining B Matrix damped
B = BMatrix(EulerConvention.axes,E1,E2,E3);
%B Matrix damped Obtained

%Obtaining Ja
Jwb = B\Jw;
Ja{i} = [Jv;Jwb];
%Ja Obtained
end

end