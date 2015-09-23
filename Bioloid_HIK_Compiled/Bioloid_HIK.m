function dq = Bioloid_HIK (q,Index)
%% Bioloid Hierachical Inverse Kinematics
% Inputs:
%    q     = Robot configuration, i.e. each element of q is a joint position
%    Index = Index for the Euler's Angles
%
% Outputs:
%    dq = Articular Velocities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global srcLoaded Bioloidmodel EulerConvention

if isempty(Bioloidmodel)
    if isempty(srcLoaded)
        addpath(genpath('src'));
        display('--> Folder src and subfolders added to the path')
        srcLoaded = true;
    end
    Bioloid_Model();
    Euler_Convention(Index);
    display('-->Bioloid model loaded')
    display('-->Eulers Angles Convenion loaded')
end %End If isempty BM

if EulerConvention.IndexSaved ~= Index;
    Euler_Convention(Index);
    display('-->Euler Angles convention changed')
end %End if Euler...

%Computing Forward Kinematics...
T = body_LF_Transform (Bioloidmodel.NB, Bioloidmodel.LFinPF, Bioloidmodel.axes, q, Bioloidmodel.parent);
T_ee{1} = T{9}*Bioloidmodel.EEinLF{1};
T_ee{2} = T{12}*Bioloidmodel.EEinLF{2};
T_ee{3} = T{18}*Bioloidmodel.EEinLF{3};
T_ee{4} = T{24}*Bioloidmodel.EEinLF{4};
T_ee{5} = T{6}*Bioloidmodel.EEinLF{5};
%Forward Kinematics Computed

%Computing Poses
[E1,E2,E3] = Euler_Angles(EulerConvention.IndexSaved,T_ee{1}(1:3,1:3));
Pose{1} = [T_ee{1}(1:3,4);E1;E2;E3];
[E1,E2,E3] = Euler_Angles(EulerConvention.IndexSaved,T_ee{2}(1:3,1:3));
Pose{2} = [T_ee{2}(1:3,4);E1;E2;E3];
[E1,E2,E3] = Euler_Angles(EulerConvention.IndexSaved,T_ee{3}(1:3,1:3));
Pose{3} = [T_ee{3}(1:3,4);E1;E2;E3];
[E1,E2,E3] = Euler_Angles(EulerConvention.IndexSaved,T_ee{4}(1:3,1:3));
Pose{4} = [T_ee{4}(1:3,4);E1;E2;E3];
[E1,E2,E3] = Euler_Angles(EulerConvention.IndexSaved,T_ee{5}(1:3,1:3));
Pose{5} = [T_ee{5}(1:3,4);E1;E2;E3];
%Poses Computed

[P,Targets,EE] = SoT();
Dx = cell(P);

%Computing Jacobians
J = Bioloid_Ja_SV(T_ee,T,EE,P);
%Jacobians

for i=1:P
    Dx{i} = Targets{i} - Pose{EE{i}};
end

dq = Bioloid_Solver(J,Dx,P);
end %End Function