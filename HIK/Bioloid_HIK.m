function out = Bioloid_HIK (q,task1,EE1,task2,EE2,task3,EE3,task4,EE4,task5,EE5,Index)
%% Bioloid Hierachical Inverse Kinematics
% Inputs:
%    q     = Robot configuration, i.e. each element of q is a joint position
%    task_ = Task to accomplish in priority _
%    EE_   = End-effector to accomplish task_
%    Index = Index for the Euler's Angles
%
% Outputs:
%    dq = Articular Velocities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global srcLoaded Bioloidmodel EulerConvention Task

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
end

if EulerConvention.IndexSaved ~= Index;
    Euler_Convention(Index);
    display('-->Euler Angles convention changed')
end

Task.task1 = task1;
Task.task2 = task2;
Task.task3 = task3;
Task.task4 = task4;
Task.task5 = task5;

%Forward Kinematics Calculating ...
T = body_LF_Transform (Bioloidmodel.NB, Bioloidmodel.LFinPF, Bioloidmodel.axes, q, Bioloidmodel.parent);
T_ee{1} = T{9}*Bioloidmodel.EEinLF{1};
T_ee{2} = T{12}*Bioloidmodel.EEinLF{2};
T_ee{3} = T{18}*Bioloidmodel.EEinLF{3};
T_ee{4} = T{24}*Bioloidmodel.EEinLF{4};
T_ee{5} = T{6}*Bioloidmodel.EEinLF{5};
%T_ee{1:5}
%Forward Kinematics Calculated

%Calculating Jacobians and Delta X
[J1,dX1] = Bioloid_J_SV(task1,T_ee{EE1},EE1,T);
[J2,dX2] = Bioloid_J_SV(task2,T_ee{EE2},EE2,T);
[J3,dX3] = Bioloid_J_SV(task3,T_ee{EE3},EE3,T);
[J4,dX4] = Bioloid_J_SV(task4,T_ee{EE4},EE4,T);
[J5,dX5] = Bioloid_J_SV(task5,T_ee{EE5},EE5,T);
%Jacobians and Delta X Calculated

dq = Bioloid_Loop(J1,dX1,J2,dX2,J3,dX3,J4,dX4,J5,dX5);


%for i=1:24
%    if dq(i) < -1.57
%        dq(i) = -1.56;
%    elseif dq(i) > 1.57
%        dq(i) = 1.56;
%    end
%end

VRML = Bioloid_Signals2VRML(q,task1,task2,task3,task4,task5,EulerConvention.IndexSaved);

out = [dq;VRML];
end