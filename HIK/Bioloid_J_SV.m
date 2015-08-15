function  [Ja,Dx] = Bioloid_J_SV(task,T_ee,EE,T)
%% Geometric Jacobian matrix
% Inputs:
%    q = Robot configuration, i.e. each element of q is a joint position
% Outputs:
%    Jg = Geometric Jacobian matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Bioloidmodel EulerConvention

switch EE
    case 1
        Last_Link = 9;
    case 2
        Last_Link = 12;
    case 3
        Last_Link = 18;
    case 4
        Last_Link = 24;
    case 5
        Last_Link = 6;
end

%Obtaining Geometric Jacobian
[Jv,Jw] = Jg_EE(T_ee(1:3,4),T,Last_Link,Bioloidmodel.parent,Bioloidmodel.axes);
%JG obtained

%Obtaining Euler Angles
[E1,E2,E3] = Conventions(EulerConvention.IndexSaved,T_ee(1:3,1:3));
%Euler Angles Obtained

%Obtaining B Matrix damped
B = BMatrix(EulerConvention.axes,E1,E2,E3);
%B Matrix damped Obtained

%Obtaining Ja
Jwb = B\Jw;
Ja = [Jv;Jwb];
%Ja Obtained

%Obtaining Delta X
Dx = task - [T_ee(1:3,4);E1;E2;E3];
%Delta X Obtained

end